
#include <sbpl_adaptive/headers.h>

#include <smpl/time.h>

using namespace std;

namespace adim {

AdaptivePlanner::AdaptivePlanner(
    AdaptiveDiscreteSpaceInformation* environment,
    bool bSearchForward)
:
    num_iterations_(0),
    repair_time_(0.0),
    adaptive_environment_(environment),
    planner_(),
    tracker_(),
    stat_(),
    start_state_id_(-1),
    goal_state_id_(-1),
    time_per_retry_plan_(5.0),
    time_per_retry_track_(5.0),
    target_eps_(-1.0),
    planning_eps_(-1.0),
    tracking_eps_(-1.0),
    final_eps_planning_time_(-1.0),
    final_eps_(-1.0),
    forward_search_(bSearchForward),
    search_until_first_solution_(true),
    in_tracking_phase_(false),
    plan_anc_heur_(nullptr),
    plan_heurs_(nullptr),
    track_anc_heur_(nullptr),
    track_heurs_(nullptr),
    num_heur_(0),
    search_expands_(0)
{
    ROS_INFO("Create adaptive planner...");

    stat_.reset(new AdaptivePlannerCSVStat_c());

    ROS_INFO("Initialize planners...");
    planner_.reset(new ARAPlanner(adaptive_environment_, forward_search_));
    planner_->set_search_mode(false);
    tracker_.reset(new ARAPlanner_AD(adaptive_environment_, forward_search_));
    tracker_->set_search_mode(false);
    ROS_INFO("done!");
}

AdaptivePlanner::AdaptivePlanner(
    AdaptiveDiscreteSpaceInformation* environment,
    bool bSearchForward,
    Heuristic* anc_heur,
    Heuristic** heurs,
    int num_heur)
:
    num_iterations_(0),
    repair_time_(0.0),
    adaptive_environment_(environment),
    planner_(),
    tracker_(),
    stat_(),
    start_state_id_(-1),
    goal_state_id_(-1),
    time_per_retry_plan_(5.0),
    time_per_retry_track_(5.0),
    target_eps_(-1.0),
    planning_eps_(100.0),
    tracking_eps_(100.0),
    final_eps_planning_time_(-1.0),
    final_eps_(-1.0),
    forward_search_(bSearchForward),
    search_until_first_solution_(true),
    in_tracking_phase_(false),
    plan_anc_heur_(anc_heur),
    plan_heurs_(heurs),
    track_anc_heur_(anc_heur),
    track_heurs_(heurs),
    num_heur_(num_heur),
    search_expands_(0)
{
    ROS_INFO("Create adaptive planner...");

    stat_.reset(new AdaptivePlannerCSVStat_c());

    ROS_INFO("Initialize planners...");
    planner_.reset(new MHAPlanner_AD(adaptive_environment_, plan_anc_heur_, plan_heurs_, num_heur_));
    planner_->set_search_mode(false);
    tracker_.reset(new MHAPlanner_AD(adaptive_environment_, track_anc_heur_, track_heurs_, num_heur_));
    tracker_->set_search_mode(false);
    ROS_INFO("done!");
}

AdaptivePlanner::~AdaptivePlanner()
{
}

// returns 1 if found a solution, and 0 otherwise
int AdaptivePlanner::replan(
    double allocated_time_secs,
    double allocated_time_per_retry_plan_,
    double allocated_time_per_retry_track_,
    std::vector<int>* solution_stateIDs_V,
    int* psolcost)
{
    ROS_INFO("Begin Adaptive Planning...");

    auto start_t = sbpl::clock::now();

    int round = 0;

    std::chrono::duration<double> allowed_ad_plan_time(allocated_time_per_retry_plan_);
    std::chrono::duration<double> allowed_ad_track_time(allocated_time_per_retry_track_);

    ROS_INFO("Retry time limits: (Planning: %.4f) (Tracking: %.4f)", to_secs(allowed_ad_plan_time), to_secs(allowed_ad_track_time));

    ROS_INFO("Set start and goal for planner and tracker_...");

    std::vector<int> ModifiedStates;
    std::vector<int> TrkModifiedStates;

    std::vector<int> last_tracker_solution;
    bool last_tracker_ret = false;

    stat_->setInitialEps(planning_eps_ * tracking_eps_);

    // set of states to introduce high-dimensional regions around at the start
    // of the next planning phase, initially empty for the first phase
    std::vector<int> new_sphere_locations;

    const std::chrono::duration<double> allowed_time(allocated_time_secs);
    auto time_elapsed = [&](){ return sbpl::clock::now() - start_t; };
    auto time_remaining = [&]() { return allowed_time - time_elapsed(); };
    auto time_expired = [&](){ return time_elapsed() > allowed_time; };

    std::vector<int> planner_solution;
    std::vector<int> tracker_solution;

    if (!in_tracking_phase_)    // some initializations
    {    
        if (planner_->set_start(start_state_id_) == 0) {
            ROS_ERROR("ERROR: planner failed to set start state");
            throw SBPL_Exception();
        }

        if (tracker_->set_start(start_state_id_) == 0) {
            ROS_ERROR("ERROR: tracker failed to set start state");
            throw SBPL_Exception();
        }

        if (planner_->set_goal(goal_state_id_) == 0) {
            ROS_ERROR("ERROR: planner failed to set goal state");
            throw SBPL_Exception();
        }

        if (tracker_->set_goal(goal_state_id_) == 0) {
            ROS_ERROR("ERROR: tracker failed to set goal state");
            throw SBPL_Exception();
        }

        ROS_INFO("Set environment in planning mode...");
        adaptive_environment_->setPlanMode();

        ROS_INFO("Add start (%d) and goal (%d) spheres...", start_state_id_, goal_state_id_);
        adaptive_environment_->addSphere(goal_state_id_);
        adaptive_environment_->addSphere(start_state_id_);

        repair_time_ = 0.0;

        ROS_INFO("Force planning from scratch for planner and tracker_...");
        planner_->force_planning_from_scratch();
        tracker_->force_planning_from_scratch();

    }
    else {
        if (tracker_->set_start(start_state_id_) == 0) {
            ROS_ERROR("ERROR: tracker failed to set start state");
            throw SBPL_Exception();
        }
    }
    do {
        if (time_expired()) {
            ROS_INFO("Search ran out of time!");
            *solution_stateIDs_V = last_tracker_solution;
            ROS_INFO("Done in: %.3f sec", to_secs(time_elapsed()));
            num_iterations_ = round;

            stat_->setTotalPlanningTime(to_secs(time_elapsed()));
            stat_->setFinalEps(-1.0);
            return last_tracker_ret;
        }

        ROS_INFO("=======================================");
        ROS_INFO("||          Iteration %03d            ||", round);
        ROS_INFO("=======================================");

        auto iter_start = sbpl::clock::now();
        auto iter_elapsed = [&](){ return sbpl::clock::now() - iter_start; };

        auto plan_start = sbpl::clock::now();
        auto plan_time = sbpl::clock::now() - plan_start;
        int p_cost;
        if (!in_tracking_phase_)    //skip planning
        {
            ROS_INFO("  =======================================");
            ROS_INFO("  ||          Planning Phase           ||");
            ROS_INFO("  =======================================");

            adaptive_environment_->setPlanMode();
            planner_->force_planning_from_scratch();
            planner_->set_initialsolution_eps(planning_eps_);
            planner_->set_search_mode(false);

            // add pending new spheres
            ROS_INFO("Add %zd pending spheres...", new_sphere_locations.size());
            for (int stateID : new_sphere_locations) {
                adaptive_environment_->addSphere(stateID, &ModifiedStates);
            }
            new_sphere_locations.clear();

            ROS_INFO("Still have time (%.3fs)...planning", to_secs(time_remaining()));
            planner_solution.clear();
            
            int p_ret = planner_->replan(
                    to_secs(allowed_ad_plan_time), &planner_solution, &p_cost);

            
            stat_->addPlanningPhaseTime(to_secs(plan_time));

            ROS_INFO("Planner done in %.3fs...", to_secs(plan_time));
        
            if (!p_ret || planner_solution.empty()) {
                // TODO: an empty solution may be the correct solution and should
                // report success and a correct suboptimality bound
                ROS_ERROR("Solution could not be found within the allowed time (%.3fs.) after %d iterations", to_secs(allowed_ad_plan_time), round);
                adaptive_environment_->visualizeEnvironment();
                num_iterations_ = round;
                stat_->setFinalEps(-1.0);
                stat_->setSuccess(false);
                stat_->setNumIterations(num_iterations_ + 1);
                return p_ret;
            }

            if (adaptive_environment_->isExecutablePath(planner_solution)) {
                *solution_stateIDs_V = planner_solution;

                ROS_INFO("Iteration Time: %.3f sec (avg: %.3f)", to_secs(iter_elapsed()), to_secs(time_elapsed()) / (round + 1.0));
                ROS_INFO("Done in: %.3f sec", to_secs(time_elapsed()));
                num_iterations_ = round;
                stat_->setFinalEps(planner_->get_final_epsilon());
                stat_->setFinalPlanCost(p_cost);
                stat_->setFinalTrackCost(p_cost);
                stat_->setNumIterations(num_iterations_ + 1);
                stat_->setSuccess(true);
                stat_->setTotalPlanningTime(to_secs(sbpl::clock::now() - start_t));
                return true;
            }
            adaptive_environment_->visualizeEnvironment();
            adaptive_environment_->visualizeStatePath(&planner_solution, 0, 120, "planning_path");
        
        // TODO: return false if no time remaining to run the tracker 
            adaptive_environment_->setTrackMode(planner_solution, p_cost, &TrkModifiedStates);
        }

        ROS_INFO("  =======================================");
        ROS_INFO("  ||          Tracking Phase           ||");
        ROS_INFO("  =======================================");
        auto track_start = sbpl::clock::now();
        tracker_->force_planning_from_scratch();
        tracker_->set_initialsolution_eps(tracking_eps_);
        tracker_->set_search_mode(false);

        ROS_INFO("Still have time (%.3fs)...tracking", to_secs(time_remaining()));
        allowed_ad_track_time = time_remaining();
        tracker_solution.clear();
        int t_cost;
        int t_ret = tracker_->replan(to_secs(allowed_ad_track_time), &tracker_solution, &t_cost);

        auto track_time = sbpl::clock::now() - track_start;
        stat_->addTrackingPhaseTime(to_secs(track_time));

        last_tracker_solution = tracker_solution;
        last_tracker_ret = t_ret;

        ROS_INFO("Tracker done in %.3fs...", to_secs(track_time));
    
        ModifiedStates.clear();
        const auto iter_time = plan_time + track_time;
        ROS_INFO("[Planning] Time: %.3fs (%.1f%% of iter time)", to_secs(plan_time), 100.0 * to_secs(plan_time) / to_secs(iter_time));
        ROS_INFO("[Tracking] Time: %.3fs (%.1f%% of iter time)", to_secs(track_time), 100.0 * to_secs(track_time) / to_secs(iter_time));

        adaptive_environment_->visualizeEnvironment();
        adaptive_environment_->visualizeStatePath(&tracker_solution, 240, 300, "tracking_path");

        if (t_ret) {
            const double target_eps = tracking_eps_ * planning_eps_;
            const double cost_ratio = (double)t_cost / (double)p_cost;
            if (cost_ratio <= target_eps) {
                ROS_INFO("Tracking succeeded! - good path found! tCost %d / pCost %d (eps: %.3f, target: %.3f)", t_cost, p_cost, cost_ratio, target_eps);
                adaptive_environment_->visualizeStatePath(&tracker_solution, 0, 240, "tracking_path");
                *solution_stateIDs_V = tracker_solution;
                ROS_INFO("Iteration Time: %.3f sec (avg: %.3f)", to_secs(iter_elapsed()), to_secs(time_elapsed()) / (round + 1.0));
                ROS_INFO("Done in: %.3f sec", to_secs(time_elapsed()));
                num_iterations_ = round;
                stat_->setFinalEps(planner_->get_final_epsilon() * tracker_->get_final_epsilon());
                stat_->setFinalPlanCost(p_cost);
                stat_->setFinalTrackCost(t_cost);
                stat_->setNumIterations(num_iterations_ + 1);
                stat_->setSuccess(true);
                stat_->setTotalPlanningTime(to_secs(time_elapsed()));
                if (tracker_solution.back() != goal_state_id_) {
                    in_tracking_phase_ = true;
                }
                else {
                    in_tracking_phase_ = false;
                }
                return true;
            }

            // introduce new spheres since tracker found a costly path
            ROS_INFO("Tracking succeeded - costly path found! tCost %d / pCost %d (eps: %.3f, target: %.3f)", t_cost, p_cost, cost_ratio, target_eps);
            adaptive_environment_->processCostlyPath(planner_solution, tracker_solution, &new_sphere_locations);
            if (new_sphere_locations.empty()) {
                ROS_ERROR("No new spheres added during this planning episode!!!");
                throw SBPL_Exception();
            }
            
        }
        else {
            ROS_WARN("Tracking Failed!");
            in_tracking_phase_ = false;
            if (tracker_solution.empty()) {
                ROS_ERROR("No new spheres added during this planning episode!!!");
                throw SBPL_Exception();
            }

            // introduce new spheres at the point of failure since tracking
            // failed, note that this requires the planner to return a partial
            // solution rather than an empty path when a complete path to the
            // goal was not found
            int TrackFail_StateID = tracker_solution.back();
            new_sphere_locations.push_back(TrackFail_StateID);
        }
    
        ROS_INFO("Iteration Time: %.3f sec (avg: %.3f)", to_secs(iter_elapsed()), to_secs(time_elapsed()) / (round + 1.0));
        ROS_INFO("Total Time so far: %.3f sec", to_secs(time_elapsed()));
        round++;
    } while (true);

    stat_->setFinalEps(-1);
    stat_->setFinalPlanCost(INFINITECOST);
    stat_->setFinalTrackCost(INFINITECOST);
    stat_->setNumIterations(round + 1);
    stat_->setSuccess(false);
    stat_->setTotalPlanningTime(to_secs(time_elapsed()));

    return false;
}

int AdaptivePlanner::set_goal(int goal_stateID)
{
    goal_state_id_ = goal_stateID;
    ROS_INFO("goal set (StateID: %d)", goal_stateID);
    return 1;
}

int AdaptivePlanner::set_start(int start_stateID)
{
    start_state_id_ = start_stateID;
    ROS_INFO("start set (StateID: %d)", start_stateID);
    return 1;
}

int AdaptivePlanner::force_planning_from_scratch()
{
    if(planner_ != NULL && tracker_ != NULL){
        ROS_INFO("Reset planner and tracker_!");
        planner_->force_planning_from_scratch();
        tracker_->force_planning_from_scratch();
    }
    adaptive_environment_->reset();
    stat_->reset();
    return 1;
}

int AdaptivePlanner::set_search_mode(bool bSearchUntilFirstSolution)
{
    search_until_first_solution_ = bSearchUntilFirstSolution;
    //set search mode
    planner_->set_search_mode(search_until_first_solution_);
    tracker_->set_search_mode(search_until_first_solution_);
    return 1;
}

void AdaptivePlanner::print_searchpath(FILE* fOut)
{
    ROS_WARN("print_searchpath() NOT IMPLEMENTED YET!");
    return;
}

bool AdaptivePlanner::set_time_per_retry(double t_plan, double t_track)
{
    time_per_retry_plan_ = t_plan;
    time_per_retry_track_ = t_track;
    return true;
}

void AdaptivePlanner::set_initialsolution_eps(double initialsolution_eps)
{
    target_eps_ = initialsolution_eps;
    planning_eps_ = sqrt(initialsolution_eps);
    tracking_eps_ = sqrt(initialsolution_eps);
    planner_->set_initialsolution_eps(planning_eps_);
    tracker_->set_initialsolution_eps(tracking_eps_);
}

int AdaptivePlanner::replan(
    vector<int>* solution_stateIDs_V,
    ReplanParams params, int* solcost)
{
    set_initialsolution_eps(params.initial_eps);
    search_until_first_solution_ = params.return_first_solution;
    return replan(params.max_time, params.repair_time, params.repair_time, solution_stateIDs_V, solcost);
}

} // namespace adim
