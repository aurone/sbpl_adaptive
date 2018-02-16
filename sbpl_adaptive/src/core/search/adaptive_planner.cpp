#include <sbpl_adaptive/core/search/adaptive_planner.h>

// standard includes
#include <cmath>
#include <algorithm>

namespace adim {

static const char *LOG = "adaptive_planner";

PlannerAllocator::~PlannerAllocator()
{
}

AdaptivePlanner::AdaptivePlanner(
    AdaptiveDiscreteSpace *environment,
    const PlannerAllocator &plan_search_alloc,
    const PlannerAllocator &track_search_alloc,
    bool forward_search)
:
    adaptive_environment_(environment),
    planner_(),
    tracker_(),
    forward_search_(forward_search),
    time_per_retry_plan_(5.0),
    time_per_retry_track_(5.0),
    target_eps_(-1.0),
    planning_eps_(1.0),
    tracking_eps_(1.0),
    start_state_id_(-1),
    goal_state_id_(-1),
    stat_(),
    final_eps_planning_time_(-1.0),
    final_eps_(-1.0),
    search_expands_(0),
    plan_mode_(PlanPhase::Planning),
    iteration_(-1),
    plan_sol_(),
    plan_cost_(-1),
    track_sol_(),
    track_cost_(-1),
    pending_spheres_(),
    iter_elapsed_(sbpl::clock::duration::zero()),
    plan_elapsed_(sbpl::clock::duration::zero()),
    track_elapsed_(sbpl::clock::duration::zero()),
    time_elapsed_(sbpl::clock::duration::zero()),
    last_start_state_id_(-1),
    last_goal_state_id_(-1),
    last_plan_iter_(-1),
    last_track_iter_(-1)
{
    ROS_INFO("Initialize planners...");
    planner_.reset(plan_search_alloc.make(environment, forward_search));
    planner_->set_search_mode(false);

    tracker_.reset(track_search_alloc.make(environment, forward_search));
    tracker_->set_search_mode(false);
    ROS_INFO("done!");
}

AdaptivePlanner::~AdaptivePlanner()
{
}

/// \brief replan a path within the allocated time
/// \param allocated_time_secs The total amount of time allotted to the
///     planner for this call.
/// \param allocated_time_per_retry_plan The maximum time allotted to the
///     planning phase search. This is modulated by the time remaining for
///     the replan call as a whole.
/// \param allocated_time_per_retry_track The maximum time allotted to the
///     tracking phase search. This is modulated by the time remaining for
///     the replan call as a whole.
/// \param[out] solution The solution vector
/// \return 1 if a solution was found; 0 otherwise
int AdaptivePlanner::replan(
    double allocated_time_secs,
    double allocated_time_per_retry_plan_,
    double allocated_time_per_retry_track_,
    std::vector<int>* solution,
    int* psolcost)
{
    ROS_INFO("Begin Adaptive Planning...");

    auto start_t = sbpl::clock::now();
    time_per_retry_plan_ = allocated_time_per_retry_plan_;
    time_per_retry_track_ = allocated_time_per_retry_track_;

    ROS_INFO("Retry time limits: (Planning: %.4f) (Tracking: %.4f)", allocated_time_per_retry_plan_, allocated_time_per_retry_track_);

    const auto allowed_time = sbpl::to_duration(allocated_time_secs);
    auto time_elapsed = [&](){ return sbpl::clock::now() - start_t; };
    auto time_remaining = [&](){ return allowed_time - time_elapsed(); };
    auto time_expired = [&](){ return time_elapsed() > allowed_time; };

    const bool same_goal = last_goal_state_id_ == goal_state_id_;
    const bool same_start = last_start_state_id_ == start_state_id_;
    auto it = std::find(track_sol_.begin(), track_sol_.end(), start_state_id_);
    const bool start_on_partial_plan = it != end(track_sol_);
    if (same_goal && (same_start || (!same_start && start_on_partial_plan))) {
        if (same_goal && same_start) {
            ROS_INFO("Resume previous planning phase");
        } else {
            ROS_INFO("Skip planning phase and resume tracking from new start state on previous plan solution");
        }
        // in the case where the start has changed, but the goal hasn't, and on
        // a previous tracking iteration we found a (partial) path to the goal
        // through this new start state, then we skip the planning phase and
        // remain in the tracking phase, beginning the tracking search from the
        // new start state
        if (tracker_->set_start(start_state_id_) == 0) {
            throw SBPL_Exception("tracker failed to set the start state");
        }

        // we still need to set the new start state for successive iterations
        if (planner_->set_start(start_state_id_) == 0) {
            throw SBPL_Exception("planner failed to set the start state");
        }
    }
    else {
        ROS_INFO("Begin plan phase on the first iteration");

        // in all other instances, we begin anew with the first planning
        // iteration
        search_expands_ = 0;

        plan_mode_ = PlanPhase::Planning;
        iteration_ = 0;
        plan_cost_ = -1;
        plan_sol_.clear();
        track_cost_ = -1;
        track_sol_.clear();
        pending_spheres_.clear();
        iter_elapsed_ = sbpl::clock::duration::zero();
        plan_elapsed_ = sbpl::clock::duration::zero();
        track_elapsed_ = sbpl::clock::duration::zero();
        time_elapsed_ = sbpl::clock::duration::zero();
        last_plan_iter_ = -1;
        last_track_iter_ = -1;

        adaptive_environment_->reset();
        pending_spheres_.push_back(start_state_id_);
        pending_spheres_.push_back(goal_state_id_);

        if (planner_->set_start(start_state_id_) == 0) {
            throw SBPL_Exception("planner failed to set the start state");
        }
        if (planner_->set_goal(goal_state_id_) == 0) {
            throw SBPL_Exception("planner failed to set the goal state");
        }
        if (tracker_->set_start(start_state_id_) == 0) {
            throw SBPL_Exception("tracker failed to set the start state");
        }
        if (tracker_->set_goal(goal_state_id_) == 0) {
            throw SBPL_Exception("tracker failed to set the goal state");
        }

        stat_.setInitialEps(planning_eps_ * tracking_eps_);
    }

    last_goal_state_id_ = goal_state_id_;
    last_start_state_id_ = start_state_id_;

    do {
        switch (plan_mode_) {
        case PlanPhase::Planning: {
            auto plan_res = onPlanningState(time_remaining(), *solution);
            if (plan_res != ReplanCode::PathFound) {
                return (int)plan_res;
            }
        }   break;
        case PlanPhase::Tracking: {
            auto trak_res = onTrackingState(time_remaining(), *solution);

            switch (trak_res) {
            case ReplanCode::NoPathExists: // move on to planning
                break;
            case ReplanCode::TimeLimitReached: // fallthrough
            case ReplanCode::PathFound:
                *psolcost = track_cost_;
                return (int)trak_res;
            default:
                return (int)trak_res;
            }
        }   break;
        }

        ROS_INFO("Total Time so far: %.3f sec", sbpl::to_seconds(time_elapsed()));
    } while (true);

    stat_.setFinalEps(-1);
    stat_.setFinalPlanCost(INFINITECOST);
    stat_.setFinalTrackCost(INFINITECOST);
    stat_.setNumIterations(iteration_ + 1);
    stat_.setSuccess(false);
    stat_.setTotalPlanningTime(sbpl::to_seconds(time_elapsed()));

    return false;
}

// Run the planning phase for a duration up to time_remaining. If an executable
// solution is found, the result should be stored in \sol and 'true' returned.
// If a non-executable solution was found, this phase should signal to begin
// the tracking phase by setting plan_mode_ appropriately and returning false.
// In that case, the non-executable solution path from the planner should be
// stored in \plan_sol_ and its corresponding cost in plan_cost_.
//
// Additionally, this function also internally handles the transition from a
// previous initial or tracking phase by respecting the variables iteration_
// and pending_spheres_.
//
// Additionally additionally this function is responsible for recording
// statistics such incrementing the time spent in the planning phase, the total
// iteration time, the total time for the current query, among others.
auto AdaptivePlanner::onPlanningState(
    const sbpl::clock::duration time_remaining,
    std::vector<int> &sol)
    -> ReplanCode
{
    if (max_iterations_ != 0 && iteration_ >= max_iterations_) {
        return ReplanCode::TimeLimitReached;
    }

    if (iteration_ != last_plan_iter_) {
        ROS_INFO("=======================================");
        ROS_INFO("||          Iteration %03d            ||", iteration_);
        ROS_INFO("=======================================");
        ROS_INFO("  =======================================");
        ROS_INFO("  ||          Planning Phase           ||");
        ROS_INFO("  =======================================");

        plan_sol_.clear();
        plan_cost_ = -1;

        // reset the timer for the time consumed by this iteration
        iter_elapsed_ = sbpl::clock::duration::zero();
        plan_elapsed_ = sbpl::clock::duration::zero();

        planner_->set_initialsolution_eps(planning_eps_);

        planner_->force_planning_from_scratch(); // updated G^ad

        adaptive_environment_->setPlanMode();

        // add pending new spheres
        ROS_INFO("Add %zd pending spheres...", pending_spheres_.size());
        for (int stateID : pending_spheres_) {
            adaptive_environment_->addSphere(stateID, nullptr);
        }
        pending_spheres_.clear();

        last_plan_iter_ = iteration_;
    }

    ROS_INFO("Still have time (%.3fs)...planning", sbpl::to_seconds(time_remaining));
    auto plan_start = sbpl::clock::now();
    plan_sol_.clear();
    double allowed_plan_time = time_per_retry_plan_;
    allowed_plan_time = std::min(allowed_plan_time, sbpl::to_seconds(time_remaining));
    allowed_plan_time = std::max(allowed_plan_time, 0.0);
    // NOTE: case to ReplanCode...enum values the same as in MRMHAPlannerAD
    ReplanCode p_ret = (ReplanCode)planner_->replan(allowed_plan_time, &plan_sol_, &plan_cost_);
    search_expands_ += planner_->get_n_expands();
    auto plan_time = sbpl::clock::now() - plan_start;
    stat_.addPlanningPhaseTime(sbpl::to_seconds(plan_time));
    plan_elapsed_ += plan_time;
    iter_elapsed_ += plan_time;
    time_elapsed_ += plan_time;
    ROS_INFO("Planner done in %.3fs...", sbpl::to_seconds(plan_time));

    switch (p_ret) {
    case ReplanCode::NoPathExists:
        stat_.setFinalEps(-1.0);
        stat_.setSuccess(false);
        stat_.setNumIterations(iteration_ + 1);
        // fallthrough
    case ReplanCode::TimeLimitReached:
        ROS_ERROR("Solution could not be found within the allowed time (%.3fs.) after %d iterations", allowed_plan_time, iteration_);
        // fallthrough
    case ReplanCode::InvalidStartState:
    case ReplanCode::InvalidGoalState:
    case ReplanCode::InvalidParameters:
    case ReplanCode::InvalidBestState:
        return p_ret;
//    case PathFound:
    }

    adaptive_environment_->visualizeEnvironment();
    adaptive_environment_->visualizeStatePath(&plan_sol_, 0, 120, "planning_path");

    ROS_INFO("Signal tracking phase");
    plan_mode_ = PlanPhase::Tracking;
    return ReplanCode::PathFound;
}

// Run the tracking phase for a duration up to time_remaining. If a solution is
// found and is of acceptable cost, the result should be stored in \sol and
// 'true' returned. If the solution found is only a partial solution to the
// goal, then
//
// If no solution is found to exist or the found solution is unacceptably
// costly, this phase should signal to begin the next planning phase by setting
// plan_mode_ appropriately and returning false. If no solution is found within
// the time limit but the total time allotted to tracking has not expired, this
// function should NOT signal to begin the next planning phase.
//
// Additionally, this function also internally handles the transition from a
// previous planning phase by respecting the variables iteration_, plan_sol_,
// and plan_cost_.
//
// Additionally additionally this function is responsible for recording
// statistics such incrementing the time spent in the tracking phase, the total
// iteration time, the total time for the current query, among others.
auto AdaptivePlanner::onTrackingState(
    const sbpl::clock::duration time_remaining,
    std::vector<int> &sol)
    -> ReplanCode
{
    if (iteration_ != last_track_iter_) {
        if (adaptive_environment_->isExecutablePath(plan_sol_)) {
            sol = plan_sol_;

            ROS_INFO("Iteration Time: %.3f sec (avg: %.3f)", sbpl::to_seconds(iter_elapsed_), sbpl::to_seconds(time_elapsed_) / (iteration_ + 1.0));
            ROS_INFO("Done in: %.3f sec", sbpl::to_seconds(time_elapsed_));

            stat_.setFinalEps(planner_->get_final_epsilon());
            stat_.setFinalPlanCost(plan_cost_);
            stat_.setFinalTrackCost(plan_cost_);
            stat_.setNumIterations(iteration_ + 1);
            stat_.setSuccess(true);
            stat_.setTotalPlanningTime(sbpl::to_seconds(time_elapsed_));
            stat_.setPlanSize(sol.size());
            return ReplanCode::PathFound;
        }

        ROS_INFO("  =======================================");
        ROS_INFO("  ||          Tracking Phase           ||");
        ROS_INFO("  =======================================");

        track_sol_.clear();
        track_cost_ = -1;

        track_elapsed_ = sbpl::clock::duration::zero();

        tracker_->set_initialsolution_eps(tracking_eps_);

        tracker_->force_planning_from_scratch(); // updated G^ad

        adaptive_environment_->setTrackMode(plan_sol_, plan_cost_, nullptr);

        last_track_iter_ = iteration_;
    }

    ROS_INFO("Still have time (%.3fs)...tracking", sbpl::to_seconds(time_remaining));
    auto track_start = sbpl::clock::now();
    track_sol_.clear();
    double allowed_track_time = time_per_retry_track_ - sbpl::to_seconds(track_elapsed_);
    allowed_track_time = std::min(allowed_track_time, sbpl::to_seconds(time_remaining));
    allowed_track_time = std::max(allowed_track_time, 0.0);
    ReplanCode t_ret = (ReplanCode)tracker_->replan(allowed_track_time, &track_sol_, &track_cost_);
    search_expands_ += tracker_->get_n_expands();
    auto track_time = sbpl::clock::now() - track_start;
    stat_.addTrackingPhaseTime(sbpl::to_seconds(track_time));
    track_elapsed_ += track_time;
    iter_elapsed_ += track_time;
    time_elapsed_ += track_time;
    ROS_INFO("Tracker done in %.3fs...", sbpl::to_seconds(track_time));

    adaptive_environment_->visualizeEnvironment();
    adaptive_environment_->visualizeStatePath(&track_sol_, 240, 300, "tracking_path");

    if (t_ret == ReplanCode::PathFound) {
        const double target_eps = tracking_eps_ * planning_eps_;
        const double cost_ratio = (double)track_cost_ / (double)plan_cost_;
        if (cost_ratio <= target_eps) {
            ROS_INFO("Tracking succeeded! - good path found! tCost %d / pCost %d (eps: %.3f, target: %.3f)", track_cost_, plan_cost_, cost_ratio, target_eps);
            sol = track_sol_;
            ROS_INFO("Done in: %.3f sec", sbpl::to_seconds(time_elapsed_));
            stat_.setFinalEps(planner_->get_final_epsilon() * tracker_->get_final_epsilon());
            stat_.setFinalPlanCost(plan_cost_);
            stat_.setFinalTrackCost(track_cost_);
            stat_.setNumIterations(iteration_ + 1);
            stat_.setSuccess(true);
            stat_.setTotalPlanningTime(sbpl::to_seconds(time_elapsed_));
            stat_.setPlanSize(sol.size());
            ROS_INFO("[Planning] Time: %.3fs (%.1f%% of iter time)", sbpl::to_seconds(plan_elapsed_), 100.0 * sbpl::to_seconds(plan_elapsed_) / sbpl::to_seconds(iter_elapsed_));
            ROS_INFO("[Tracking] Time: %.3fs (%.1f%% of iter time)", sbpl::to_seconds(track_elapsed_), 100.0 * sbpl::to_seconds(track_elapsed_) / sbpl::to_seconds(iter_elapsed_));
            ROS_INFO("Iteration Time: %.3f sec (avg: %.3f)", sbpl::to_seconds(iter_elapsed_), sbpl::to_seconds(iter_elapsed_) / (iteration_ + 1.0));
            return ReplanCode::PathFound;
        } else {
            ROS_INFO("Tracking succeeded - costly path found! tCost %d / pCost %d (eps: %.3f, target: %.3f)", track_cost_, plan_cost_, cost_ratio, target_eps);
            // TODO: might want to give more time to the tracking search here to
            // give it a chance to improve the path?
            adaptive_environment_->processCostlyPath(plan_sol_, track_sol_, &pending_spheres_);
            if (pending_spheres_.empty()) {
                ROS_ERROR("No new spheres added during this planning episode!!!");
                throw SBPL_Exception();
            }
            iteration_++;
            ROS_INFO("Signal planning phase");
            plan_mode_ = PlanPhase::Planning;
            ROS_INFO("[Planning] Time: %.3fs (%.1f%% of iter time)", sbpl::to_seconds(plan_elapsed_), 100.0 * sbpl::to_seconds(plan_elapsed_) / sbpl::to_seconds(iter_elapsed_));
            ROS_INFO("[Tracking] Time: %.3fs (%.1f%% of iter time)", sbpl::to_seconds(track_elapsed_), 100.0 * sbpl::to_seconds(track_elapsed_) / sbpl::to_seconds(iter_elapsed_));
            ROS_INFO("Iteration Time: %.3f sec (avg: %.3f)", sbpl::to_seconds(iter_elapsed_), sbpl::to_seconds(iter_elapsed_) / (iteration_ + 1.0));
            return ReplanCode::TimeLimitReached; // to signal that we're not done
        }
    }
    else if (t_ret == ReplanCode::TimeLimitReached) {
        sol = track_sol_;
        ROS_WARN("Solution could not be found within the allowed time (%.3fs.) after %d iterations", allowed_track_time, iteration_);
        if (sbpl::to_seconds(track_elapsed_) >= time_per_retry_track_) {
            // partial solution and time for tracking has expired
            if (track_sol_.empty()) {
                ROS_ERROR("No new spheres added during this planning episode!!!");
                throw SBPL_Exception();
            }

            // introduce new spheres at the point of failure since
            // tracking failed, note that this requires the planner to
            // return a partial solution rather than an empty path when
            // a complete path to the goal was not found
            int TrackFail_StateID = track_sol_.back();
            pending_spheres_.push_back(TrackFail_StateID);

            iteration_++;
            ROS_INFO("Signal planning phase");
            plan_mode_ = PlanPhase::Planning;
            ROS_INFO("[Planning] Time: %.3fs (%.1f%% of iter time)", sbpl::to_seconds(plan_elapsed_), 100.0 * sbpl::to_seconds(plan_elapsed_) / sbpl::to_seconds(iter_elapsed_));
            ROS_INFO("[Tracking] Time: %.3fs (%.1f%% of iter time)", sbpl::to_seconds(track_elapsed_), 100.0 * sbpl::to_seconds(track_elapsed_) / sbpl::to_seconds(iter_elapsed_));
            ROS_INFO("Iteration Time: %.3f sec (avg: %.3f)", sbpl::to_seconds(iter_elapsed_), sbpl::to_seconds(iter_elapsed_) / (iteration_ + 1.0));
        }
        return ReplanCode::TimeLimitReached;
    }
    else if (t_ret == ReplanCode::NoPathExists) {
        stat_.setFinalEps(-1.0);
        stat_.setSuccess(false);
        stat_.setNumIterations(iteration_ + 1);

        if (!track_sol_.empty()) {
            int fail_state_id = track_sol_.back();
            pending_spheres_.push_back(fail_state_id);
        } else {
            ROS_WARN("No spheres added this planning episode!");
        }

        iteration_++;
        plan_mode_ = PlanPhase::Planning;
        return ReplanCode::NoPathExists;
    }
    else {
        ROS_WARN("Tracking Failed!");
        return t_ret;
    }

    return ReplanCode::NoPathExists;
}

/// Set the goal state for the search
/// \return 1 if successful; 0 otherwise
int AdaptivePlanner::set_goal(int goal_stateID)
{
    goal_state_id_ = goal_stateID;
    ROS_INFO("goal set (StateID: %d)", goal_stateID);
    return 1;
}

/// Set the start state for the search
/// \return 1 if successful; 0 otherwise
int AdaptivePlanner::set_start(int start_stateID)
{
    start_state_id_ = start_stateID;
    ROS_INFO("start set (StateID: %d)", start_stateID);
    return 1;
}

/// Force the planner to clear all previous planning efforts and plan from
/// scratch.
/// \return 1 if successful; 0 otherwise
int AdaptivePlanner::force_planning_from_scratch()
{
    ROS_INFO("Reset planner and tracker_!");
    last_start_state_id_ = -1;
    last_goal_state_id_ = -1;
    stat_.reset();
    return 1;
}

/// Set the search mode. true indicates that the search should ignore all time
/// bounds and search until a first solution is found. false indicates that
/// the search should plan up until the allowed time and may improve the
/// solution as time allows.
int AdaptivePlanner::set_search_mode(bool bSearchUntilFirstSolution)
{
    planner_->set_search_mode(bSearchUntilFirstSolution);
    tracker_->set_search_mode(bSearchUntilFirstSolution);
    return 1;
}

/// Set the time limits for the planning and tracking searches.
/// \return true if successful; false otherwise
bool AdaptivePlanner::set_time_per_retry(double t_plan, double t_track)
{
    time_per_retry_plan_ = t_plan;
    time_per_retry_track_ = t_track;
    return true;
}

/// Set the desired suboptimality bound for search as a whole. Each underlying
/// search will have its suboptimality bound set to the sqrt(\p
/// initialsolution_eps)
void AdaptivePlanner::set_initialsolution_eps(double initialsolution_eps)
{
    target_eps_ = initialsolution_eps;
    planning_eps_ = std::sqrt(initialsolution_eps);
    tracking_eps_ = std::sqrt(initialsolution_eps);
    planner_->set_initialsolution_eps(planning_eps_);
    tracker_->set_initialsolution_eps(tracking_eps_);
}

int AdaptivePlanner::replan(
    std::vector<int>* solution,
    ReplanParams params,
    int* solcost)
{
    set_initialsolution_eps(params.initial_eps);
    if (!set_search_mode(params.return_first_solution)) {
        return 0;
    }
    return replan(
            params.max_time,    // total allocated time
            params.repair_time, // planning phase time
            params.repair_time, // tracking phase time
            solution, solcost);
}

} // namespace adim
