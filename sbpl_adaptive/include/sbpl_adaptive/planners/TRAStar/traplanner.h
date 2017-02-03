/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Pennsylvania nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SBPL_ADAPTIVE_TRAPLANNER_H
#define SBPL_ADAPTIVE_TRAPLANNER_H

// standard includes
#include <stdio.h>
#include <time.h>
#include <vector>

// system includes
#include <sbpl/headers.h>

// control of EPS

// initial suboptimality bound (cost solution <= cost(eps*cost optimal solution)
#define TRA_DEFAULT_INITIAL_EPS     10.0
// as planning time exist, AD* decreases epsilon bound
#define TRA_DECREASE_EPS            0.2
// final epsilon bound
#define TRA_FINAL_EPS               1.0

#define TRA_INCONS_LIST_ID 0
#define TRAMDP_STATEID2IND ARAMDP_STATEID2IND

struct TRAState : public AbstractSearchState
{
    /// \brief the MDP state itself (pointer to the graph represented as MDPstates)
    CMDPSTATE* MDPstate;

    /// \brief TRA* relevant data
    unsigned int E; // expansion time
    unsigned int C; // creation time -- first put on open list
    std::vector<TRAState*> parent_hist; // history of parents

    // history of g-values associated with each of the parents in parent_hist
    std::vector<unsigned int> gval_hist;

    unsigned int v;
    unsigned int g;
    short unsigned int iterationclosed;
    short unsigned int callnumberaccessed;
    short unsigned int numofexpands;

    /// \brief best predecessor and the action from it, used only in forward searches
    CMDPSTATE *bestpredstate;

    /// \brief the next state if executing best action
    CMDPSTATE  *bestnextstate;
    unsigned int costtobestnextstate;
    int h;

    TRAState() {};
    ~TRAState() {};

    int getSize()
    {
        return (int) (
                sizeof(TRAState) +
                sizeof(TRAState*) * parent_hist.size() +    // parent_hist
                sizeof(unsigned int) * gval_hist.size() );  // gval_hist
    }
};

typedef struct TRASEARCHSTATESPACE
{
    double eps;
    double eps_satisfied;

    CHeap* heap;
    CList* inconslist;

    short unsigned int searchiteration;
    short unsigned int callnumber;

    unsigned int expansion_step;

    CMDPSTATE* searchgoalstate;
    CMDPSTATE* searchstartstate;

    CMDP searchMDP;

    bool bReevaluatefvals;
    bool bReinitializeSearchStateSpace;
    bool bRebuildOpenList;

    bool bNewSearchIteration;

    std::vector<TRAState*> seen_states;

} TRASearchStateSpace_t;

SBPL_CLASS_FORWARD(TRAPlanner)

class TRAPlanner : public SBPLPlanner
{
public:

    TRAPlanner(DiscreteSpaceInformation *environment, bool bForwardSearch);

    ~TRAPlanner();

    /// \brief replan a path within the allocated time, return the solution in
    /// the vector
    int replan(
        double allocated_time_secs,
        std::vector<int>* solution_stateIDs_V);

    /// \brief replan a path within the allocated time, return the solution in
    /// the vector, also returns solution cost
    int replan(
        double allocated_time_secs,
        std::vector<int>* solution_stateIDs_V,
        int* solcost);

    /// \brief set the goal state
    int set_goal(int goal_stateID);

    /// \brief set the start state
    int set_start(int start_stateID);

    /// \brief set a flag to get rid of the previous search efforts, release the
    /// memory and re-initialize the search, when the next replan is called
    int force_planning_from_scratch();

    /// \brief you can either search forwards or backwards
    int set_search_mode(bool bSearchUntilFirstSolution);

    /// \brief inform the search about the new edge costs
    void costs_changed(StateChangeQuery const & stateChange);

    /// \brief direct form of informing the search about the new edge costs
    /// \param succsIDV array of successors of changed edges
    /// \note this is used when the search is run forwards
    void update_succs_of_changededges(std::vector<int>* succsIDV);

    /// \brief direct form of informing the search about the new edge costs
    /// \param predsIDV array of predecessors of changed edges
    /// \note this is used when the search is run backwards
    void update_preds_of_changededges(std::vector<int>* predsIDV);

    /// \brief returns the suboptimality bound on the currently found solution
    virtual double get_solution_eps() const { return pSearchStateSpace_->eps_satisfied; };

    /// \brief returns the number of states expanded so far
    virtual int get_n_expands() const { return searchexpands; }

    /// \brief returns the initial epsilon
    double get_initial_eps(){return finitial_eps;};

    /// \brief returns the time taken to find the first solution
    double get_initial_eps_planning_time(){ return finitial_eps_planning_time; }

    /// \brief returns the time taken to get the final solution
    double get_final_eps_planning_time(){ return final_eps_planning_time; };

    /// \brief returns the number of expands to find the first solution
    int get_n_expands_init_solution(){ return num_of_expands_initial_solution; };

    /// \brief returns the final epsilon achieved during the search
    double get_final_epsilon(){return final_eps;};

    /// \brief returns the value of the initial epsilon (suboptimality bound) used
    virtual void set_initialsolution_eps(double initialsolution_eps) {finitial_eps = initialsolution_eps;};

private:

    // member variables
    double finitial_eps;
    double finitial_eps_planning_time;
    double final_eps_planning_time;
    double final_eps;
    int num_of_expands_initial_solution;
    MDPConfig* MDPCfg_;

    bool bforwardsearch;

    // if true, then search until first solution (see planner.h for search
    // modes)
    bool bsearchuntilfirstsolution;

    TRASearchStateSpace_t *pSearchStateSpace_;

    unsigned int searchexpands;
    int MaxMemoryCounter;
    clock_t TimeStarted;
    FILE *fDeb;

    // member functions

    bool RestoreSearchTree(
        TRASearchStateSpace_t* pSearchStateSpace,
        unsigned int expansion_step);

    bool fixParents(
        TRASearchStateSpace_t* pSearchStateSpace,
        TRAState* state,
        unsigned int expansion_step);

    void Recomputegval(TRAState* state);

    void Initialize_searchinfo(
        CMDPSTATE* state,
        TRASearchStateSpace_t* pSearchStateSpace);

    CMDPSTATE* CreateState(
        int stateID,
        TRASearchStateSpace_t* pSearchStateSpace);

    CMDPSTATE* GetState(int stateID, TRASearchStateSpace_t* pSearchStateSpace);

    unsigned int GetStateCreationTime(int stateID)
    {
        CMDPSTATE* mdp_state = GetState(stateID, pSearchStateSpace_);
        TRAState* state = (TRAState*)(mdp_state->PlannerSpecificData);
        return state->C;
    };

    int ComputeHeuristic(
        CMDPSTATE* MDPstate,
        TRASearchStateSpace_t* pSearchStateSpace);

    // initialization of a state
    void InitializeSearchStateInfo(
        TRAState* state,
        TRASearchStateSpace_t* pSearchStateSpace);

    // re-initialization of a state
    void ReInitializeSearchStateInfo(
        TRAState* state,
        TRASearchStateSpace_t* pSearchStateSpace);

    void DeleteSearchStateData(TRAState* state);

    // used for backward search
    void UpdatePreds(TRAState* state, TRASearchStateSpace_t* pSearchStateSpace);

    //used for forward search
    void UpdateSuccs(TRAState* state, TRASearchStateSpace_t* pSearchStateSpace);

    int GetGVal(int StateID, TRASearchStateSpace_t* pSearchStateSpace);

    // returns 1 if the solution is found, 0 if the solution does not exist and
    // 2 if it ran out of time
    int ImprovePath(
        TRASearchStateSpace_t* pSearchStateSpace,
        double MaxNumofSecs);

    void BuildNewOPENList(TRASearchStateSpace_t* pSearchStateSpace);

    void Reevaluatefvals(TRASearchStateSpace_t* pSearchStateSpace);

    // creates (allocates memory) search state space
    // does not initialize search statespace
    int CreateSearchStateSpace(TRASearchStateSpace_t* pSearchStateSpace);

    // deallocates memory used by SearchStateSpace
    void DeleteSearchStateSpace(TRASearchStateSpace_t* pSearchStateSpace);

    // debugging
    void PrintSearchState(TRAState* state, FILE* fOut);

    // reset properly search state space
    // needs to be done before deleting states
    int ResetSearchStateSpace(TRASearchStateSpace_t* pSearchStateSpace);

    // initialization before each search
    void ReInitializeSearchStateSpace(TRASearchStateSpace_t* pSearchStateSpace);

    // very first initialization
    int InitializeSearchStateSpace(TRASearchStateSpace_t* pSearchStateSpace);

    int SetSearchGoalState(
        int SearchGoalStateID,
        TRASearchStateSpace_t* pSearchStateSpace);

    int SetSearchStartState(
        int SearchStartStateID,
        TRASearchStateSpace_t* pSearchStateSpace);

    // reconstruct path functions are only relevant for forward search
    int ReconstructPath(TRASearchStateSpace_t* pSearchStateSpace);

    void PrintSearchPath(TRASearchStateSpace_t* pSearchStateSpace, FILE* fOut);

    int getHeurValue(TRASearchStateSpace_t* pSearchStateSpace, int StateID);

    // get path
    std::vector<int> GetSearchPath(
        TRASearchStateSpace_t* pSearchStateSpace,
        int& solcost);

    bool Search(
        TRASearchStateSpace_t* pSearchStateSpace,
        std::vector<int>& pathIds,
        int & PathCost,
        bool bFirstSolution,
        bool bOptimalSolution,
        double MaxNumofSecs);
};

#endif
