#ifndef SBPL_ADAPTIVE_ARAPLANNER_AD_H
#define SBPL_ADAPTIVE_ARAPLANNER_AD_H

// system includes
#include <sbpl/headers.h>
#include <smpl/forward.h>
#include <smpl/time.h>

// project includes
#include <sbpl_adaptive/core/graph/adaptive_discrete_space.h>
#include <sbpl_adaptive/core/search/adaptive_planner.h>

namespace adim {

class ADARAPlannerAllocator : public PlannerAllocator
{
public:

    SBPLPlanner *make(
        AdaptiveDiscreteSpace *space,
        bool forward_search) const override;
};

SBPL_CLASS_FORWARD(ARAPlanner_AD)

/// An implementation of the ARA* algorithm for the adaptive dimensionality
/// framework. Identical to the ARA* implementation in SBPL with two exceptions:
///
/// (1) The environment is explicitly notified (via a call to
///     expandingState(int)) when a state is being expanded, intended to track
///     the "best" expanded state)
///
/// (2) If a path to the goal is not found, the search reconstructs a partial
///     path up to a certain state (dictated by the environment via a call to
///     getBestSeenState()). The call to replan(...) will return false, but the
///     output vector will contain the partial solution.
class ARAPlanner_AD : public SBPLPlanner
{
public:

    ARAPlanner_AD(
        AdaptiveDiscreteSpace* environment,
        bool bforwardsearch);

    ~ARAPlanner_AD();

    /** \brief inform the search about the new edge costs -
	    \note since ARA* is non-incremental, it is sufficient (and more efficient) to just inform ARA* of the fact that some costs changed
     */
    void costs_changed();

    /** \brief prints out the search path into a file
     */
    void print_searchpath(FILE* fOut);

    /// \name Required Public Functions from SBPLPlanner
    ///@{
    int replan(double allocated_time_secs, std::vector<int>* solution_stateIDs_V);
    int replan(double allocated_time_sec, std::vector<int>* solution_stateIDs_V, int* solcost);
    int set_goal(int goal_stateID);
    int set_start(int start_stateID);
    int force_planning_from_scratch();
    int set_search_mode(bool bSearchUntilFirstSolution);
    void costs_changed(StateChangeQuery const & stateChange);
    ///@}

    /// \name Reimplemented Public Functions from SBPLPlanner
    ///@{
    double get_solution_eps() const { return pSearchStateSpace_->eps_satisfied; }
    int get_n_expands() const { return searchexpands; }
    double get_initial_eps(){ return finitial_eps; }
    double get_initial_eps_planning_time() { return finitial_eps_planning_time; }
    double get_final_eps_planning_time() { return final_eps_planning_time; }
    int get_n_expands_init_solution() { return num_of_expands_initial_solution; }
    double get_final_epsilon() { return final_eps; }
    void set_initialsolution_eps(double initialsolution_eps) { finitial_eps = initialsolution_eps; }
    ///@}

private:

    AdaptiveDiscreteSpace* environment_;

    double finitial_eps;
    double finitial_eps_planning_time;
    double final_eps;
    double final_eps_planning_time;

    int num_of_expands_initial_solution;
    unsigned int searchexpands;

    // if true, then search proceeds forward, otherwise backward
    bool bforwardsearch;

    // if true, then search until first solution only (see planner.h for search
    // modes)
    bool bsearchuntilfirstsolution;

    ARASearchStateSpace_t* pSearchStateSpace_;

    int MaxMemoryCounter;
    sbpl::clock::time_point TimeStarted;

    void Initialize_searchinfo(
        CMDPSTATE* state,
        ARASearchStateSpace_t* pSearchStateSpace);

    CMDPSTATE* CreateState(
        int stateID,
        ARASearchStateSpace_t* pSearchStateSpace);

    CMDPSTATE* GetState(int stateID, ARASearchStateSpace_t* pSearchStateSpace);

    int ComputeHeuristic(
        CMDPSTATE* MDPstate,
        ARASearchStateSpace_t* pSearchStateSpace);

    // initialization of a state
    void InitializeSearchStateInfo(
        ARAState* state,
        ARASearchStateSpace_t* pSearchStateSpace);

    // re-initialization of a state
    void ReInitializeSearchStateInfo(
        ARAState* state,
        ARASearchStateSpace_t* pSearchStateSpace);

    void DeleteSearchStateData(ARAState* state);

    // used for backward search
    void UpdatePreds(ARAState* state, ARASearchStateSpace_t* pSearchStateSpace);

    // used for forward search
    void UpdateSuccs(ARAState* state, ARASearchStateSpace_t* pSearchStateSpace);

    int GetGVal(int StateID, ARASearchStateSpace_t* pSearchStateSpace);

    // returns 1 if the solution is found, 0 if the solution does not exist and
    // 2 if it ran out of time
    int ImprovePath(
        ARASearchStateSpace_t* pSearchStateSpace,
        double MaxNumofSecs);

    void BuildNewOPENList(ARASearchStateSpace_t* pSearchStateSpace);

    void Reevaluatefvals(ARASearchStateSpace_t* pSearchStateSpace);

    // creates (allocates memory) search state space
    // does not initialize search statespace
    int CreateSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace);

    // deallocates memory used by SearchStateSpace
    void DeleteSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace);

    // debugging
    void PrintSearchState(ARAState* state, FILE* fOut);

    // reset properly search state space
    // needs to be done before deleting states
    int ResetSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace);

    // initialization before each search
    void ReInitializeSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace);

    // very first initialization
    int InitializeSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace);

    int SetSearchGoalState(
        int SearchGoalStateID,
        ARASearchStateSpace_t* pSearchStateSpace);

    int SetSearchStartState(
        int SearchStartStateID,
        ARASearchStateSpace_t* pSearchStateSpace);

    // reconstruct path functions are only relevant for forward search
    int ReconstructPath(
        ARASearchStateSpace_t* pSearchStateSpace,
        CMDPSTATE* beststate = NULL);

    void PrintSearchPath(ARASearchStateSpace_t* pSearchStateSpace, FILE* fOut);

    int getHeurValue(ARASearchStateSpace_t* pSearchStateSpace, int StateID);

    // get path
    std::vector<int> GetSearchPath(
        ARASearchStateSpace_t* pSearchStateSpace,
        int& solcost,
        CMDPSTATE* beststate = NULL);

    bool Search(
        ARASearchStateSpace_t* pSearchStateSpace,
        std::vector<int>& pathIds,
        int & PathCost,
        bool bFirstSolution,
        bool bOptimalSolution,
        double MaxNumofSecs);
};

} // namespace adim

#endif
