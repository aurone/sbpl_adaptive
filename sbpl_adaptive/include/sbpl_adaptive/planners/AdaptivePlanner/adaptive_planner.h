#ifndef __ADAPTIVEPLANNER_H_
#define __ADAPTIVEPLANNER_H_

#include <sbpl_adaptive/macros.h>

#define ADP_DEBUG
#define ADP_VERBOSE //toggles verbose mode
#define ADP_GRAPHICAL //enables calling of visualization functions from the environment

#ifndef ADP_VERBOSE
#define SBPL_INFO(...)
#endif

namespace adim {

SBPL_CLASS_FORWARD(AdaptivePlanner)

class AdaptivePlanner : public SBPLPlanner
{
public:

    /// \brief constructor
    AdaptivePlanner(AdaptiveDiscreteSpaceInformation* environment, bool bforwardsearch);

    /// \brief destructor
    ~AdaptivePlanner();

    int replan(
        double allocated_time_secs,
        std::vector<int>* solution_stateIDs_V)
    {
        return replan(allocated_time_secs, timePerRetryPlan_, timePerRetryTrack_, solution_stateIDs_V);
    }

    int replan(
        double allocated_time_secs,
        std::vector<int>* solution_stateIDs_V,
        int* cost)
    {
        return replan(allocated_time_secs, timePerRetryPlan_, timePerRetryTrack_, solution_stateIDs_V, cost);
    }

    /// \brief replan a path within the allocated time, return the solution in
    ///     the vector
    int replan(
        double allocated_time_secs,
        double allocated_time_per_retry_plan_,
        double allocated_time_per_retry_track_,
        std::vector<int>* solution_stateIDs_V)
    {
        int solcost = 0;
        return replan(allocated_time_secs, allocated_time_per_retry_plan_, allocated_time_per_retry_track_, solution_stateIDs_V, &solcost);
    }

    /// \brief replan a path within the allocated time, return the solution in
    ///     the vector, also returns solution cost
    int replan(
        double allocated_time_sec,
        double allocated_time_per_retry_plan_,
        double allocated_time_per_retry_track_,
        std::vector<int>* solution_stateIDs_V,
        int* solcost);

    int replan(
        std::vector<int>* solution_stateIDs_V,
        ReplanParams params,
        int* solcost);

    int dynamically_replan(
        double allocated_time_secs,
        void (*Callback)(std::vector<std::vector<double> >*, void*),
        void* obj);

    /// \brief set the goal state
    int set_goal(int goal_stateID);

    /// \brief set the start state
    int set_start(int start_stateID);

    /// \brief set a flag to get rid of the previous search efforts, release the
    //     memory and re-initialize the search, when the next replan is called
    int force_planning_from_scratch();

    /// \brief you can either search forwards or backwards
    int set_search_mode(bool bSearchUntilFirstSolution);

    bool set_time_per_retry(double t_plan, double t_track);

    /// \brief returns the suboptimality bound on the currently found solution
    double get_solution_eps() const
    {
        SBPL_WARN("get_solution_eps() not implemented for this planner!");
        throw new SBPL_Exception();
        return -1.0;
    };

    /// \brief returns the number of states expanded so far
    int get_n_expands() const { return searchexpands; }

    /// \brief returns the value of the initial epsilon (suboptimality bound)
    ///     used
    void set_initialsolution_eps(double initialsolution_eps);

    /// \brief prints out the search path into a file
    void print_searchpath(FILE* fOut);

    /// \brief returns the time taken to get the final solution
    double get_final_eps_planning_time(){return final_eps_planning_time;};

    /// \brief returns the final epsilon achieved during the search
    double get_final_epsilon()
    {
        return planner->get_final_epsilon() * tracker->get_final_epsilon();
    }

    void costs_changed(StateChangeQuery const & stateChange)
    {
        SBPL_WARN("costs_changed(...) NOT IMPLEMENTED FOR THIS PLANNER");
    }

    void pause()
    {
        printf("Enter to continue...");
        char inp;
        do {
            inp = getchar();
        }
        while (inp != '\n');
    }

    bool saveStats(std::string name)
    {
        return stat_->writeToFile(name);
    }

    std::shared_ptr<AdaptivePlannerCSVStat_c> getStats() { return stat_; }

    int nIterations;
    double repair_time;
    double track_time_total_s;
    double plan_time_total_s;

protected:

    AdaptiveDiscreteSpaceInformation* adaptive_environment_;
    // std::shared_ptr<AdaptiveDiscreteSpaceInformation> adaptive_environment_;

private:

    std::unique_ptr<SBPLPlanner> planner;
    std::unique_ptr<SBPLPlanner> tracker;

    std::shared_ptr<AdaptivePlannerCSVStat_c> stat_;

    FILE* logstream_;

    int StartStateID;
    int GoalStateID;

    /*double newSphereRad;
	double tunnelWidth;*/
    double timePerRetryPlan_;
    double timePerRetryTrack_;

    double targetEPS;
    double planningEPS;
    double trackingEPS;

    //member variables
    double final_eps_planning_time, final_eps;

    bool bforwardsearch; //if true, then search proceeds forward, otherwise backward
    bool bsearchuntilfirstsolution; //if true, then search until first solution only (see planner.h for search modes)

    unsigned int searchexpands;
    //member functions
};

} // namespace adim

#endif
