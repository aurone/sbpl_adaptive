#ifndef ADIM_ADAPTIVE_PLANNER_H
#define ADIM_ADAPTIVE_PLANNER_H

// standard includes
#include <memory>
#include <chrono>

// system includes
#include <ros/console.h>
#include <sbpl/headers.h>
#include <smpl/forward.h>

// project includes
#include <sbpl_adaptive/SCVStat.h>
#include <sbpl_adaptive/discrete_space_information/adaptive_discrete_space_information.h>

namespace adim {

SBPL_CLASS_FORWARD(AdaptivePlanner)

class PlannerAllocator
{
public:

    virtual ~PlannerAllocator();

    virtual SBPLPlanner *make(
        AdaptiveDiscreteSpaceInformation *space,
        bool forward_search) const = 0;
};

class AdaptivePlanner : public SBPLPlanner
{
public:

    AdaptivePlanner(
        AdaptiveDiscreteSpaceInformation *space,
        const PlannerAllocator &plan_search_alloc,
        const PlannerAllocator &track_search_alloc,
        bool forward_search);

    ~AdaptivePlanner();

    /// \brief replan a path within the allocated time, return the solution in
    ///     the vector
    int replan(
        double allocated_time_secs,
        double allocated_time_per_retry_plan,
        double allocated_time_per_retry_track,
        std::vector<int>* solution_stateIDs_V);

    /// \brief replan a path within the allocated time, return the solution in
    ///     the vector, also returns solution cost
    int replan(
        double allocated_time_sec,
        double allocated_time_per_retry_plan,
        double allocated_time_per_retry_track,
        std::vector<int>* solution_stateIDs_V,
        int* solcost);

    int dynamically_replan(
        double allocated_time_secs,
        void (*Callback)(std::vector<std::vector<double> >*, void*),
        void* obj);

    bool set_time_per_retry(double t_plan, double t_track);

    /// \brief prints out the search path into a file
    void print_searchpath(FILE* fOut);

    void pause();

    bool saveStats(std::string name);

    /// \name Required Public Functions from SBPLPlanner
    ///@{
    int replan(
        double allocated_time_secs,
        std::vector<int>* solution_stateIDs_V) override;

    int replan(
        double allocated_time_secs,
        std::vector<int>* solution_stateIDs_V,
        int* cost) override;

    int set_goal(int goal_stateID) override;

    int set_start(int start_stateID) override;

    int force_planning_from_scratch() override;

    int set_search_mode(bool bSearchUntilFirstSolution) override;

    void costs_changed(const StateChangeQuery& stateChange) override;
    ///@}

    /// \name Reimplemented Public Functions from SBPLPlanner
    ///@{
    int replan(
        std::vector<int>* solution_stateIDs_V,
        ReplanParams params,
        int* solcost) override;

    double get_solution_eps() const override;

    int get_n_expands() const override { return search_expands_; }

    double get_final_eps_planning_time() override { return final_eps_planning_time_; }

    double get_planning_time() { return stat_->getPlanningPhaseTime(); }
    double get_tracking_time() { return stat_->getTrackingPhaseTime(); }
    double get_total_time() { return stat_->getTotalTime(); }
    unsigned long get_planning_cost() { return stat_->getPlanCost(); }
    unsigned long get_tracking_cost() { return stat_->getTrackingCost(); }
    unsigned long get_plan_size() { return stat_->getPlanSize(); }

    double get_final_epsilon() override;

    void set_initialsolution_eps(double initialsolution_eps) override;
    ///@}

private:

    int num_iterations_;
    double repair_time_;

    AdaptiveDiscreteSpaceInformation* adaptive_environment_;

    std::unique_ptr<SBPLPlanner> planner_;
    std::unique_ptr<SBPLPlanner> tracker_;

    AdaptivePlannerCSVStat_cPtr stat_;

    int start_state_id_;
    int goal_state_id_;

    double time_per_retry_plan_;
    double time_per_retry_track_;

    double target_eps_;
    double planning_eps_;
    double tracking_eps_;

    double final_eps_planning_time_;
    double final_eps_;

    // if true, then search proceeds forward, otherwise backward
    bool forward_search_;

    // if true, then search until first solution only (see sbpl/planner.h for
    // search modes)
    bool search_until_first_solution_;

    bool in_tracking_phase_;

    unsigned int search_expands_;
};

inline int AdaptivePlanner::replan(
    double allocated_time_secs,
    std::vector<int>* solution_stateIDs_V)
{
    return replan(allocated_time_secs, time_per_retry_plan_, time_per_retry_track_, solution_stateIDs_V);
}

inline int AdaptivePlanner::replan(
    double allocated_time_secs,
    std::vector<int>* solution_stateIDs_V,
    int* cost)
{
    return replan(allocated_time_secs, time_per_retry_plan_, time_per_retry_track_, solution_stateIDs_V, cost);
}

inline int AdaptivePlanner::replan(
    double allocated_time_secs,
    double allocated_time_per_retry_plan_,
    double allocated_time_per_retry_track_,
    std::vector<int>* solution_stateIDs_V)
{
    int solcost = 0;
    return replan(allocated_time_secs, allocated_time_per_retry_plan_, allocated_time_per_retry_track_, solution_stateIDs_V, &solcost);
}

inline double AdaptivePlanner::get_solution_eps() const
{
    ROS_WARN("get_solution_eps() not implemented for this planner!");
    throw new SBPL_Exception();
    return -1.0;
};

inline double AdaptivePlanner::get_final_epsilon()
{
    return planner_->get_final_epsilon() * tracker_->get_final_epsilon();
}

inline void AdaptivePlanner::costs_changed(StateChangeQuery const & stateChange)
{
    ROS_WARN("costs_changed(...) NOT IMPLEMENTED FOR THIS PLANNER");
}

inline void AdaptivePlanner::pause()
{
    printf("Enter to continue...");
    char inp;
    do {
        inp = getchar();
    }
    while (inp != '\n');
}

inline bool AdaptivePlanner::saveStats(std::string name)
{
    return stat_->writeToFile(name);
}

} // namespace adim

#endif
