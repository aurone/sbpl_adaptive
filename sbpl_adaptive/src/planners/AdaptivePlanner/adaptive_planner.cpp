
#include <sbpl_adaptive/headers.h>

#define TRACK_RETRIES 0

using namespace std;

namespace adim {

AdaptivePlanner::AdaptivePlanner(
    AdaptiveDiscreteSpaceInformation* environment,
    bool bSearchForward)
:
    num_iterations_(0),
    repair_time_(0.0),
    track_time_total_s_(0.0),
    plan_time_total_s_(0.0),
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
    plan_anc_heur_(nullptr),
    plan_heurs_(nullptr),
    track_anc_heur_(nullptr),
    track_heurs_(nullptr),
    num_heur_(0),
    search_expands_(0)
{
//    ros::WallTime t = ros::WallTime::now();

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
{
    ROS_INFO("Create adaptive planner...");
    forward_search_ = bSearchForward;
    adaptive_environment_ = environment;

    start_state_id_ = -1;
    goal_state_id_ = -1;

    final_eps_planning_time_ = -1.0;
    final_eps_ = -1.0;

    time_per_retry_plan_ = 5.0;
    time_per_retry_track_ = 5.0;

    planning_eps_ = 100.0;
    tracking_eps_ = 100.0;
    target_eps_ = -1.0;

    num_iterations_ = 0;
    plan_time_total_s_ = 0;
    track_time_total_s_ = 0;
    search_until_first_solution_ = true;
    repair_time_ = 0;
    search_expands_ = 0;

    stat_.reset(new AdaptivePlannerCSVStat_c());
    //adaptive_environment_->setStat(stat_.get());

    ROS_INFO("Initialize planners...");

    plan_anc_heur_ = anc_heur;
    plan_heurs_ = heurs;

    track_anc_heur_ = anc_heur;
    track_heurs_ = heurs;

    num_heur_ = num_heur;

    // planner.reset(new Imp_MHAPlanner_AD(adaptive_environment_, plan_anc_heur_, plan_heurs_, num_heur_, forward_search_));
    planner_.reset(new MHAPlanner_AD(adaptive_environment_, plan_anc_heur_, plan_heurs_, num_heur_));
    planner_->set_search_mode(false);
    // tracker.reset(new Imp_MHAPlanner_AD(adaptive_environment_, track_anc_heur_, track_heurs_, num_heur_, forward_search_));
    tracker_.reset(new MHAPlanner_AD(adaptive_environment_, track_anc_heur_, track_heurs_, num_heur_));
    tracker_->set_search_mode(false);

    ROS_INFO("done!");
}

AdaptivePlanner::~AdaptivePlanner()
{
}

//----------------------------- Interface functions ----------------------------------------------

//returns 1 if found a solution, and 0 otherwise
int AdaptivePlanner::replan(
    double allocated_time_secs,
    double allocated_time_per_retry_plan_,
    double allocated_time_per_retry_track_,
    std::vector<int>* solution_stateIDs_V,
    int* psolcost)
{
    ROS_INFO("Begin Adaptive Planning...");
    adaptive_environment_->setPlanMode();
    //DO THE ADAPTIVE PLANNING LOOP HERE
    std::vector<int> planning_stateV;
    std::vector<int> tracking_stateV;

    MY_TIME_TYPE start_t = MY_TIME_NOW;
    track_time_total_s_ = 0;
    plan_time_total_s_ = 0;

    int round = 0;

    double ad_plan_time_alloc = allocated_time_per_retry_plan_;
    double ad_track_time_alloc = allocated_time_per_retry_track_;

    ROS_INFO("Retry time limits: (Planning: %.4f) (Tracking: %.4f)", ad_plan_time_alloc, ad_track_time_alloc);

    int planning_bRet;
    int tracking_bRet;

    ROS_INFO("Set start and goal for planner and tracker_...");

    if(planner_->set_start(start_state_id_) == 0)
    {
        ROS_ERROR("ERROR: planner failed to set start state");
        throw SBPL_Exception();
    }

    if(tracker_->set_start(start_state_id_) == 0){
        ROS_ERROR("ERROR: tracker failed to set start state");
        throw SBPL_Exception();
    }

    if(planner_->set_goal(goal_state_id_) == 0)
    {
        ROS_ERROR("ERROR: planner failed to set goal state");
        throw SBPL_Exception();
    }

    if(tracker_->set_goal(goal_state_id_) == 0){
        ROS_ERROR("ERROR: tracker failed to set goal state");
        throw SBPL_Exception();
    }

    ROS_INFO("Set environment in planning mode...");
    adaptive_environment_->setPlanMode();

    ROS_INFO("Add start (%d) and goal (%d) spheres...", start_state_id_, goal_state_id_);
    adaptive_environment_->addSphere(goal_state_id_);
    adaptive_environment_->addSphere(start_state_id_);

    std::vector<int> ModifiedStates;
    std::vector<int> TrkModifiedStates;
    repair_time_ = 0.0;

    ROS_INFO("Force planning from scratch for planner and tracker_...");
    planner_->force_planning_from_scratch();
    tracker_->force_planning_from_scratch();

    std::vector<int> LastTrackPath;
    bool LastTrackSuccess = false;
    double t_elapsed_s;

    stat_->setInitialEps(planning_eps_*tracking_eps_);

    std::vector<int> new_sphere_locations; //as stateIDs
    do {
        if(MY_TIME_DIFF_S(MY_TIME_NOW, start_t) > allocated_time_secs){
            ROS_INFO("Search ran out of time!");
            solution_stateIDs_V->clear();
            for(unsigned int i = 0; i < LastTrackPath.size(); i++){
                solution_stateIDs_V->push_back(LastTrackPath[i]);
            }
            ROS_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
            num_iterations_ = round;

            stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
            stat_->setFinalEps(-1.0);
            return LastTrackSuccess;
        }

        ROS_INFO("=======================================");
        ROS_INFO("||          Iteration %03d            ||", round);
        ROS_INFO("=======================================");
        MY_TIME_TYPE iter_start = MY_TIME_NOW;
        MY_TIME_TYPE plan_start = MY_TIME_NOW;
        MY_TIME_TYPE track_start;

        double plan_time_s = 0;
        double track_time_s = 0;

        int p_Cost;		  //planning solution cost
        int t_Cost;		  //tracking solution cost

        //==================================== PLANNING =================================
        ROS_INFO("\t=======================================");
        ROS_INFO("\t||          Planning Phase           ||");
        ROS_INFO("\t=======================================");

        planner_->force_planning_from_scratch();
        planner_->set_initialsolution_eps(planning_eps_);
        planner_->set_search_mode(false);

        adaptive_environment_->setPlanMode();
        //add pending new spheres
        ROS_INFO("Add %zd pending spheres...", new_sphere_locations.size());
        for(int stateID : new_sphere_locations){
            adaptive_environment_->addSphere(stateID, &ModifiedStates);
        }
        new_sphere_locations.clear();
        planning_stateV.clear();
        planning_bRet = 0;

        //PLANNING HERE!
        t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
        while(t_elapsed_s < allocated_time_secs){
            ROS_INFO("Still have time (%.3fs)...planning", allocated_time_secs - t_elapsed_s);
            MY_TIME_TYPE p_start = MY_TIME_NOW;
            planning_bRet = planner_->replan(ad_plan_time_alloc, &planning_stateV, &p_Cost);
            t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
            if(!planning_bRet && MY_TIME_DIFF_S(MY_TIME_NOW, p_start) < 0.1*ad_plan_time_alloc){
                ROS_WARN("Planning too quick! Probably stuck!");
                break;
            }
            if(planning_bRet) break;
        }

        plan_time_s = MY_TIME_DIFF_S(MY_TIME_NOW, plan_start);
        plan_time_total_s_ += plan_time_s;
        stat_->addPlanningPhaseTime(plan_time_s);

        ROS_INFO("Planner done in %.3fs...", plan_time_s);

        if(!planning_bRet || planning_stateV.size() == 0){
            ROS_ERROR("Solution could not be found within the allowed time (%.3fs.) after %d iterations", ad_plan_time_alloc, round);
#ifdef ADP_GRAPHICAL
            adaptive_environment_->visualizeEnvironment();
#endif
            num_iterations_ = round;
            stat_->setFinalEps(-1.0);
            stat_->setSuccess(false);
            stat_->setNumIterations(num_iterations_+1);
            return planning_bRet;
        }
        if(adaptive_environment_->isExecutablePath(planning_stateV)){
            solution_stateIDs_V->clear();
            for(unsigned int i = 0; i < planning_stateV.size(); i++){
                solution_stateIDs_V->push_back(planning_stateV[i]);
            }
            ROS_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
            ROS_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
            num_iterations_ = round;
            stat_->setFinalEps(planner_->get_final_epsilon());
            stat_->setFinalPlanCost(p_Cost);
            stat_->setFinalTrackCost(p_Cost);
            stat_->setNumIterations(num_iterations_+1);
            stat_->setSuccess(true);
            stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
            return true;
        }
#ifdef ADP_GRAPHICAL
        adaptive_environment_->visualizeEnvironment();
        adaptive_environment_->visualizeStatePath(&planning_stateV, 0, 120, "planning_path");
#endif

        //==================================== TRACKING ====================================
        ROS_INFO("\t=======================================");
        ROS_INFO("\t||          Tracking Phase           ||");
        ROS_INFO("\t=======================================");
        track_start = MY_TIME_NOW;
        adaptive_environment_->setTrackMode(planning_stateV, p_Cost, &TrkModifiedStates);
        tracker_->force_planning_from_scratch();
        tracker_->set_initialsolution_eps(tracking_eps_);
        tracker_->set_search_mode(false);
        tracking_stateV.clear();
        tracking_bRet = 0;

        //TRACKING HERE!!!
        t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
        int last_bestTrackedID = -1;
        int retries = 0;
        // while(t_elapsed_s < allocated_time_secs)
        {
            ROS_INFO("Still have time (%.3fs)...tracking", allocated_time_secs - t_elapsed_s);
            MY_TIME_TYPE p_start = MY_TIME_NOW;
            ad_track_time_alloc = allocated_time_secs - t_elapsed_s;
            tracking_bRet = tracker_->replan(ad_track_time_alloc, &tracking_stateV, &t_Cost);
            t_elapsed_s = MY_TIME_DIFF_S(MY_TIME_NOW, start_t);
            if(!tracking_bRet && MY_TIME_DIFF_S(MY_TIME_NOW, p_start) < 0.1*ad_track_time_alloc){
                ROS_WARN("Tracking too quick! Probably stuck!");
                break;
            }

            // if(tracking_bRet) {
            //     break;
            // } else {
                int new_bestTrackedID = adaptive_environment_->getBestSeenState();
                if(new_bestTrackedID != -1 && new_bestTrackedID == last_bestTrackedID){
                    if(retries>=TRACK_RETRIES){
                        ROS_WARN("No progress...giving up this tracking iteration");
                        break;
                    }
                    retries++;
                } else {
                    last_bestTrackedID = new_bestTrackedID;
                    retries=0;
                // }
            }
        }

        track_time_s = MY_TIME_DIFF_S(MY_TIME_NOW, track_start);
        track_time_total_s_ += track_time_s;
        stat_->addTrackingPhaseTime(track_time_s);

        LastTrackPath.clear();
        for(unsigned int i = 0; i < tracking_stateV.size(); i++){
            LastTrackPath.push_back(tracking_stateV[i]);
        }
        LastTrackSuccess = tracking_bRet;

        ROS_INFO("Tracker done in %.3fs...", track_time_s);

        ModifiedStates.clear();
        ROS_INFO("[Planning] Time: %.3fs (%.1f%% of iter time)", plan_time_s, 100*plan_time_s / (plan_time_s + track_time_s));
        ROS_INFO("[Tracking] Time: %.3fs (%.1f%% of iter time)", track_time_s, 100*track_time_s / (plan_time_s + track_time_s));

#ifdef ADP_GRAPHICAL
        adaptive_environment_->visualizeEnvironment();
        adaptive_environment_->visualizeStatePath(&tracking_stateV, 240, 300, "tracking_path");
#endif

        if(tracking_bRet && (t_Cost / (1.0f * p_Cost)) > tracking_eps_ * planning_eps_){
            //tracking found a costly path
            ROS_INFO("Tracking succeeded - costly path found! tCost %d / pCost %d (eps: %.3f, target: %.3f)", t_Cost, p_Cost, (t_Cost/(double)p_Cost), tracking_eps_ * planning_eps_);
            adaptive_environment_->processCostlyPath(planning_stateV, tracking_stateV, &new_sphere_locations);
            if(new_sphere_locations.size() == 0){
                ROS_ERROR("No new spheres added during this planning episode!!!");
                throw SBPL_Exception();
            }
        } else if (tracking_bRet && (t_Cost / (1.0f * p_Cost)) <= tracking_eps_ * planning_eps_ ) {
            ROS_INFO("Tracking succeeded! - good path found! tCost %d / pCost %d (eps: %.3f, target: %.3f)", t_Cost, p_Cost, (t_Cost/(double)p_Cost), tracking_eps_ * planning_eps_);
#ifdef ADP_GRAPHICAL
            adaptive_environment_->visualizeStatePath(&tracking_stateV, 0, 240, "tracking_path");
#endif
            solution_stateIDs_V->clear();
            for(unsigned int i = 0; i < tracking_stateV.size(); i++){
                solution_stateIDs_V->push_back(tracking_stateV[i]);
            }
            ROS_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
            ROS_INFO("Done in: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
            num_iterations_ = round;
            stat_->setFinalEps(planner_->get_final_epsilon() * tracker_->get_final_epsilon());
            stat_->setFinalPlanCost(p_Cost);
            stat_->setFinalTrackCost(t_Cost);
            stat_->setNumIterations(num_iterations_+1);
            stat_->setSuccess(true);
            stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
            return true;
        } else {
            ROS_WARN("Tracking Failed!");
            //since tracking failed -- introduce new spheres
            if(tracking_stateV.size() > 0){
                //get the point of failure
                int TrackFail_StateID = tracking_stateV[tracking_stateV.size()-1];
                new_sphere_locations.push_back(TrackFail_StateID);
            } else {
                ROS_ERROR("No new spheres added during this planning episode!!!");
                throw SBPL_Exception();
            }
        }
        ROS_INFO("Iteration Time: %.3f sec (avg: %.3f)", MY_TIME_DIFF_S(MY_TIME_NOW, iter_start), MY_TIME_DIFF_S(MY_TIME_NOW, start_t) / (round+1.0));
        ROS_INFO("Total Time so far: %.3f sec", MY_TIME_DIFF_S(MY_TIME_NOW, start_t));
        round++;
    } while (true);

    stat_->setFinalEps(-1);
    stat_->setFinalPlanCost(INFINITECOST);
    stat_->setFinalTrackCost(INFINITECOST);
    stat_->setNumIterations(round+1);
    stat_->setSuccess(false);
    stat_->setTotalPlanningTime(MY_TIME_DIFF_S(MY_TIME_NOW, start_t));

    return tracking_bRet;
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
