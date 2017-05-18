#include <sbpl_adaptive/core/search/araplanner_ad.h>

// standard includes
#include <assert.h>

// project includes
#include <sbpl_adaptive/common.h>

#define ARAMDP_STATEID2IND_AD STATEID2IND_SLOT1
#define ARA_AD_INCONS_LIST_ID 1

namespace adim {

SBPLPlanner *ADARAPlannerAllocator::make(
    adim::AdaptiveDiscreteSpace *space,
    bool forward_search) const
{
    return new ARAPlanner_AD(space, forward_search);
}

ARAPlanner_AD::ARAPlanner_AD(
    AdaptiveDiscreteSpace* environment,
    bool bSearchForward)
:
    environment_(environment),
    finitial_eps(ARA_DEFAULT_INITIAL_EPS),
    finitial_eps_planning_time(-1.0),
    final_eps(-1.0),
    final_eps_planning_time(-1.0),
    num_of_expands_initial_solution(0),
    searchexpands(0),
    bforwardsearch(bSearchForward),
    bsearchuntilfirstsolution(false),
    pSearchStateSpace_(NULL),
    MaxMemoryCounter(0)
{
    pSearchStateSpace_ = new ARASearchStateSpace_t;

    // create the ARA planner
    SBPL_INFO("Creating search state space...");
    if (CreateSearchStateSpace(pSearchStateSpace_) != 1) {
        SBPL_ERROR("ERROR: failed to create statespace\n");
        return;
    }
    SBPL_INFO("OK");

    // set the start and goal states
    SBPL_INFO("Initializing search state space...");
    if (InitializeSearchStateSpace(pSearchStateSpace_) != 1) {
        SBPL_ERROR("ERROR: failed to create statespace\n");
        return;
    }
    SBPL_INFO("OK");
}

ARAPlanner_AD::~ARAPlanner_AD()
{
    if (pSearchStateSpace_ != NULL) {
        // delete the statespace
        DeleteSearchStateSpace(pSearchStateSpace_);
        delete pSearchStateSpace_;
    }
}

void ARAPlanner_AD::Initialize_searchinfo(
    CMDPSTATE* state,
    ARASearchStateSpace_t* pSearchStateSpace)
{
    ARAState* searchstateinfo = (ARAState*)state->PlannerSpecificData;
    searchstateinfo->MDPstate = state;
    InitializeSearchStateInfo(searchstateinfo, pSearchStateSpace);
}

CMDPSTATE* ARAPlanner_AD::CreateState(
    int stateID,
    ARASearchStateSpace_t* pSearchStateSpace)
{
    CMDPSTATE* state = NULL;

    assert(environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND_AD] == -1);

    // adds to the tail a state
    state = pSearchStateSpace->searchMDP.AddState(stateID);

    // remember the index of the state
    environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND_AD] =
            pSearchStateSpace->searchMDP.StateArray.size() - 1;

    assert(state == pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND_AD]]);

    // create search specific info
    state->PlannerSpecificData = (ARAState*)malloc(sizeof(ARAState));
    Initialize_searchinfo(state, pSearchStateSpace);
    MaxMemoryCounter += sizeof(ARAState);

    return state;
}

CMDPSTATE* ARAPlanner_AD::GetState(
    int stateID,
    ARASearchStateSpace_t* pSearchStateSpace)
{
    if (stateID >= (int)environment_->StateID2IndexMapping.size()) {
        SBPL_ERROR("ERROR int GetState: stateID %d is invalid\n", stateID);
        throw SBPL_Exception();
    }

    if (environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND_AD] == -1) {
        return CreateState(stateID, pSearchStateSpace);
    }
    else {
        int index = environment_->StateID2IndexMapping[stateID][ARAMDP_STATEID2IND_AD];
        return pSearchStateSpace->searchMDP.StateArray[index];
    }
}

int ARAPlanner_AD::ComputeHeuristic(
    CMDPSTATE* MDPstate,
    ARASearchStateSpace_t* pSearchStateSpace)
{
    if (bforwardsearch) {
        return environment_->GetGoalHeuristic(MDPstate->StateID);
    }
    else {
        return environment_->GetStartHeuristic(MDPstate->StateID);
    }
}

// initialization of a state
void ARAPlanner_AD::InitializeSearchStateInfo(
    ARAState* state,
    ARASearchStateSpace_t* pSearchStateSpace)
{
    state->g = INFINITECOST;
    state->v = INFINITECOST;
    state->iterationclosed = 0;
    state->callnumberaccessed = pSearchStateSpace->callnumber;
    state->bestnextstate = NULL;
    state->costtobestnextstate = INFINITECOST;
    state->heapindex = 0;
    state->listelem[ARA_AD_INCONS_LIST_ID] = 0;
    state->bestpredstate = NULL;
    // compute heuristics
    if (pSearchStateSpace->searchgoalstate != NULL) {
        state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace);
    }
    else {
        state->h = 0;
    }
}

// re-initialization of a state
void ARAPlanner_AD::ReInitializeSearchStateInfo(
    ARAState* state,
    ARASearchStateSpace_t* pSearchStateSpace)
{
//    SBPL_WARN("Reinitializing state %d!", state->MDPstate->StateID);
    state->g = INFINITECOST;
    state->v = INFINITECOST;
    state->iterationclosed = 0;
    state->callnumberaccessed = pSearchStateSpace->callnumber;
    state->bestnextstate = NULL;
    state->costtobestnextstate = INFINITECOST;
    state->heapindex = 0;
    state->listelem[ARA_AD_INCONS_LIST_ID] = 0;
    state->bestpredstate = NULL;

    // compute heuristics

    if (pSearchStateSpace->searchgoalstate != NULL) {
        state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace);
    }
    else {
        state->h = 0;
    }
}

void ARAPlanner_AD::DeleteSearchStateData(ARAState* state)
{
    // no memory was allocated
    MaxMemoryCounter = 0;
    return;
}

// used for backward search
void ARAPlanner_AD::UpdatePreds(
    ARAState* state,
    ARASearchStateSpace_t* pSearchStateSpace)
{
    std::vector<int> PredIDV;
    std::vector<int> CostV;
    CKey key;
    ARAState* predstate;

    environment_->GetPreds(state->MDPstate->StateID, &PredIDV, &CostV);

    // iterate through predecessors of s
    for (int pind = 0; pind < (int)PredIDV.size(); pind++) {
        CMDPSTATE* PredMDPState = GetState(PredIDV[pind], pSearchStateSpace);
        predstate = (ARAState*)(PredMDPState->PlannerSpecificData);
        if (predstate->callnumberaccessed != pSearchStateSpace->callnumber) {
            ReInitializeSearchStateInfo(predstate, pSearchStateSpace);
        }

        // see if we can improve the value of predstate
        if (predstate->g > state->v + CostV[pind]) {
            predstate->g = state->v + CostV[pind];
            predstate->bestnextstate = state->MDPstate;
            predstate->costtobestnextstate = CostV[pind];

            // re-insert into heap if not closed yet
            if (predstate->iterationclosed != pSearchStateSpace->searchiteration) {
                key.key[0] = predstate->g + (int)(pSearchStateSpace->eps*predstate->h);
//                key.key[1] = predstate->h;
                if (predstate->heapindex != 0) {
                    pSearchStateSpace->heap->updateheap(predstate,key);
                }
                else {
                    pSearchStateSpace->heap->insertheap(predstate,key);
                }
            }
            else if (predstate->listelem[ARA_AD_INCONS_LIST_ID] == NULL) {
                // take care of incons list
                pSearchStateSpace->inconslist->insert(predstate, ARA_AD_INCONS_LIST_ID);
            }
        }
    } // for predecessors
}

// used for forward search
void ARAPlanner_AD::UpdateSuccs(
    ARAState* state,
    ARASearchStateSpace_t* pSearchStateSpace)
{
    std::vector<int> SuccIDV;
    std::vector<int> CostV;
    CKey key;
    ARAState* succstate;

    environment_->GetSuccs(state->MDPstate->StateID, &SuccIDV, &CostV);

    // iterate through predecessors of s
    for (int sind = 0; sind < (int)SuccIDV.size(); sind++) {
        CMDPSTATE* SuccMDPState = GetState(SuccIDV[sind], pSearchStateSpace);
        int cost = CostV[sind];

        succstate = (ARAState*)(SuccMDPState->PlannerSpecificData);
        if (succstate->callnumberaccessed != pSearchStateSpace->callnumber) {
            ReInitializeSearchStateInfo(succstate, pSearchStateSpace);
        }

        // update generated index

        // see if we can improve the value of succstate
        // taking into account the cost of action
        if (succstate->g > state->v + cost) {
            succstate->g = state->v + cost;
            succstate->bestpredstate = state->MDPstate;

            // re-insert into heap if not closed yet
            if (succstate->iterationclosed != pSearchStateSpace->searchiteration) {
                key.key[0] = succstate->g + (int)(pSearchStateSpace->eps*succstate->h);
//                key.key[1] = succstate->h;

                if (succstate->heapindex != 0) {
                    pSearchStateSpace->heap->updateheap(succstate,key);
                }
                else {
                    pSearchStateSpace->heap->insertheap(succstate,key);
                }
            }
            else if (succstate->listelem[ARA_AD_INCONS_LIST_ID] == NULL) {
                // take care of incons list
                pSearchStateSpace->inconslist->insert(succstate, ARA_AD_INCONS_LIST_ID);
            }
        } // check for cost improvement

    } // for actions
}

// TODO-debugmax - add obsthresh and other thresholds to other environments in 3dkin
int ARAPlanner_AD::GetGVal(int StateID, ARASearchStateSpace_t* pSearchStateSpace)
{
    CMDPSTATE* cmdp_state = GetState(StateID, pSearchStateSpace);
    ARAState* state = (ARAState*)cmdp_state->PlannerSpecificData;
    return state->g;
}

// returns 1 if the solution is found, 0 if the solution does not exist and 2
// if it ran out of time
int ARAPlanner_AD::ImprovePath(
    ARASearchStateSpace_t* pSearchStateSpace,
    double MaxNumofSecs)
{
    int expands;
    ARAState* state;
    ARAState* searchgoalstate;
    CKey key, minkey;
    CKey goalkey;

    expands = 0;

    if (pSearchStateSpace->searchgoalstate == NULL) {
        SBPL_ERROR("ERROR searching: no goal state is set\n");
        throw SBPL_Exception();
    }

    // goal state
    searchgoalstate = (ARAState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);
    if (searchgoalstate->callnumberaccessed != pSearchStateSpace->callnumber) {
        ReInitializeSearchStateInfo(searchgoalstate, pSearchStateSpace);
    }

    // set goal key
    goalkey.key[0] = searchgoalstate->g;
//    goalkey.key[1] = searchgoalstate->h;

    // expand states until done
    minkey = pSearchStateSpace->heap->getminkeyheap();
    CKey oldkey = minkey;
    while (!pSearchStateSpace->heap->emptyheap() &&
        minkey.key[0] < INFINITECOST &&
        goalkey > minkey &&
        sbpl::to_seconds(sbpl::clock::now() - TimeStarted) < MaxNumofSecs)
    {
        //get the state
        state = (ARAState*)pSearchStateSpace->heap->deleteminheap();

        // assert that we're not expanding a state in the incons list
        assert(state->listelem[ARA_AD_INCONS_LIST_ID] == NULL);

#if DEBUG
        if (minkey.key[0] < oldkey.key[0] &&
            fabs(this->finitial_eps - 1.0) < ERR_EPS)
        {
            //SBPL_PRINTF("WARN in search: the sequence of keys decreases\n");
//            throw SBPL_Exception();
        }
        oldkey = minkey;
#endif

        // recompute state value
        state->v = state->g;
        state->iterationclosed = pSearchStateSpace->searchiteration;

        // new expand
        expands++;

        environment_->expandingState(state->MDPstate->StateID);

        if (bforwardsearch == false) {
            UpdatePreds(state, pSearchStateSpace);
        }
        else {
            UpdateSuccs(state, pSearchStateSpace);
        }

        // recompute minkey
        minkey = pSearchStateSpace->heap->getminkeyheap();

        // recompute goalkey if necessary
        if (goalkey.key[0] != (int)searchgoalstate->g) {
            // recompute the goal key (heuristics should be zero)
            goalkey.key[0] = searchgoalstate->g;
//            goalkey.key[1] = searchgoalstate->h;
        }

        if (expands % 100000 == 0 && expands > 0) {
            SBPL_PRINTF("expands so far=%u\n", expands);
        }
    }

    int retv = 1;
    if (searchgoalstate->g == INFINITECOST && pSearchStateSpace->heap->emptyheap()) {
        printf("solution does not exist: search exited because heap is empty\n");
        retv = 0;
    }
    else if (!pSearchStateSpace->heap->emptyheap() && goalkey > minkey) {
        printf("search exited because it ran out of time\n");
        retv = 2;
    }
    else if (searchgoalstate->g == INFINITECOST && !pSearchStateSpace->heap->emptyheap()) {
        printf("solution does not exist: search exited because all candidates for expansion have infinite heuristics\n");
        retv = 0;
    }
    else {
        printf("search exited with a solution for eps=%.3f\n", pSearchStateSpace->eps);
        retv = 1;
    }

    searchexpands += expands;

    return retv;
}

void ARAPlanner_AD::BuildNewOPENList(ARASearchStateSpace_t* pSearchStateSpace)
{
    ARAState* state;
    CKey key;
    CHeap* pheap = pSearchStateSpace->heap;
    CList* pinconslist = pSearchStateSpace->inconslist;

    // move incons into open
    while (pinconslist->firstelement != NULL) {
        state = (ARAState*)pinconslist->firstelement->liststate;

        // compute f-value
        key.key[0] = state->g + (int)(pSearchStateSpace->eps * state->h);
//        key.key[1] = state->h;

        // insert into OPEN
        pheap->insertheap(state, key);
        // remove from INCONS
        pinconslist->remove(state, ARA_AD_INCONS_LIST_ID);
    }
}

void ARAPlanner_AD::Reevaluatefvals(ARASearchStateSpace_t* pSearchStateSpace)
{
    CKey key;
    int i;
    CHeap* pheap = pSearchStateSpace->heap;

    //recompute priorities for states in OPEN and reorder it
    for (i = 1; i <= pheap->currentsize; ++i) {
        ARAState* state = (ARAState*)pheap->heap[i].heapstate;
        pheap->heap[i].key.key[0] = state->g + (int)(pSearchStateSpace->eps*state->h);
//        pheap->heap[i].key.key[1] = state->h;
    }
    pheap->makeheap();

    pSearchStateSpace->bReevaluatefvals = false;
}

// creates (allocates memory) search state space
// does not initialize search statespace
int ARAPlanner_AD::CreateSearchStateSpace(ARASearchStateSpace_t* pSearchStateSpace)
{
    // create a heap
    pSearchStateSpace->heap = new CHeap;
    pSearchStateSpace->inconslist = new CList;
    MaxMemoryCounter += sizeof(CHeap);
    MaxMemoryCounter += sizeof(CList);

    pSearchStateSpace->searchgoalstate = NULL;
    pSearchStateSpace->searchstartstate = NULL;

    searchexpands = 0;

    pSearchStateSpace->bReinitializeSearchStateSpace = false;

    return 1;
}

// deallocates memory used by SearchStateSpace
void ARAPlanner_AD::DeleteSearchStateSpace(
    ARASearchStateSpace_t* pSearchStateSpace)
{
    if (pSearchStateSpace->heap != NULL) {
        pSearchStateSpace->heap->makeemptyheap();
        delete pSearchStateSpace->heap;
        pSearchStateSpace->heap = NULL;
    }

    if (pSearchStateSpace->inconslist != NULL) {
        pSearchStateSpace->inconslist->makeemptylist(ARA_AD_INCONS_LIST_ID);
        delete pSearchStateSpace->inconslist;
        pSearchStateSpace->inconslist = NULL;
    }

    // delete the states themselves
    int iend = (int)pSearchStateSpace->searchMDP.StateArray.size();
    for (int i = 0; i < iend; i++) {
        CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[i];
        if (state != NULL && state->PlannerSpecificData != NULL) {
            DeleteSearchStateData((ARAState*)state->PlannerSpecificData);
            free((ARAState*)state->PlannerSpecificData);
            state->PlannerSpecificData = NULL;
        }
    }
    pSearchStateSpace->searchMDP.Delete();
}

// reset properly search state space
// needs to be done before deleting states
int ARAPlanner_AD::ResetSearchStateSpace(
    ARASearchStateSpace_t* pSearchStateSpace)
{
    pSearchStateSpace->heap->makeemptyheap();
    pSearchStateSpace->inconslist->makeemptylist(ARA_AD_INCONS_LIST_ID);

    return 1;
}

// initialization before each search
void ARAPlanner_AD::ReInitializeSearchStateSpace(
    ARASearchStateSpace_t* pSearchStateSpace)
{
    CKey key;

    // increase callnumber
    pSearchStateSpace->callnumber++;

    // reset iteration
    pSearchStateSpace->searchiteration = 0;
    pSearchStateSpace->bNewSearchIteration = true;

    pSearchStateSpace->heap->makeemptyheap();
    pSearchStateSpace->inconslist->makeemptylist(ARA_AD_INCONS_LIST_ID);

    // reset
    pSearchStateSpace->eps = this->finitial_eps;
    pSearchStateSpace->eps_satisfied = INFINITECOST;

    // initialize start state
    ARAState* startstateinfo = (ARAState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData);
    if (startstateinfo->callnumberaccessed != pSearchStateSpace->callnumber) {
        ReInitializeSearchStateInfo(startstateinfo, pSearchStateSpace);
    }

    startstateinfo->g = 0;

    // insert start state into the heap
    key.key[0] = (long int)(pSearchStateSpace->eps*startstateinfo->h);
//    key.key[1] = startstateinfo->h;
    pSearchStateSpace->heap->insertheap(startstateinfo, key);

    pSearchStateSpace->bReinitializeSearchStateSpace = false;
    pSearchStateSpace->bReevaluatefvals = false;
}

// very first initialization
int ARAPlanner_AD::InitializeSearchStateSpace(
    ARASearchStateSpace_t* pSearchStateSpace)
{
    if (pSearchStateSpace->heap->currentsize != 0 ||
        pSearchStateSpace->inconslist->currentsize != 0)
    {
        SBPL_ERROR("ERROR in InitializeSearchStateSpace: heap or list is not empty\n");
        throw SBPL_Exception();
    }

    pSearchStateSpace->eps = this->finitial_eps;
    pSearchStateSpace->eps_satisfied = INFINITECOST;
    pSearchStateSpace->searchiteration = 0;
    pSearchStateSpace->bNewSearchIteration = true;
    pSearchStateSpace->callnumber = 0;
    pSearchStateSpace->bReevaluatefvals = false;

    // create and set the search start state
    pSearchStateSpace->searchgoalstate = NULL;
//    pSearchStateSpace->searchstartstate = GetState(SearchStartStateID, pSearchStateSpace);
    pSearchStateSpace->searchstartstate = NULL;

    SBPL_ERROR("InitializeSearchStateSpace reInit!");
    pSearchStateSpace->bReinitializeSearchStateSpace = true;

    return 1;
}

int ARAPlanner_AD::SetSearchGoalState(
    int SearchGoalStateID,
    ARASearchStateSpace_t* pSearchStateSpace)
{
    if (pSearchStateSpace->searchgoalstate == NULL ||
        pSearchStateSpace->searchgoalstate->StateID != SearchGoalStateID)
    {
        pSearchStateSpace->searchgoalstate = GetState(SearchGoalStateID, pSearchStateSpace);

        // should be new search iteration
        pSearchStateSpace->eps_satisfied = INFINITECOST;
        pSearchStateSpace->bNewSearchIteration = true;
        pSearchStateSpace_->eps = this->finitial_eps;


        // recompute heuristic for the heap if heuristics is used
        for (int i = 0; i < (int)pSearchStateSpace->searchMDP.StateArray.size(); i++) {
            CMDPSTATE* MDPstate = pSearchStateSpace->searchMDP.StateArray[i];
            ARAState* state = (ARAState*)MDPstate->PlannerSpecificData;
            state->h = ComputeHeuristic(MDPstate, pSearchStateSpace);
        }

        pSearchStateSpace->bReevaluatefvals = true;
    }

    return 1;
}

int ARAPlanner_AD::SetSearchStartState(
    int SearchStartStateID,
    ARASearchStateSpace_t* pSearchStateSpace)
{
    CMDPSTATE* MDPstate = GetState(SearchStartStateID, pSearchStateSpace);

    if (MDPstate != pSearchStateSpace->searchstartstate) {
        pSearchStateSpace->searchstartstate = MDPstate;
        SBPL_ERROR("SetSearchStartState reInit!");
        pSearchStateSpace->bReinitializeSearchStateSpace = true;
    }

    return 1;
}

int ARAPlanner_AD::ReconstructPath(
    ARASearchStateSpace_t* pSearchStateSpace,
    CMDPSTATE* beststate)
{
    // nothing to do, if search is backward
    if (bforwardsearch) {
        CMDPSTATE* MDPstate;
        if (beststate == NULL) {
            MDPstate = pSearchStateSpace->searchgoalstate;
        }
        else {
            MDPstate = beststate;
        }
        CMDPSTATE* PredMDPstate;
        ARAState* predstateinfo;
        ARAState* stateinfo;

        while (MDPstate != pSearchStateSpace->searchstartstate) {
            stateinfo = (ARAState*)MDPstate->PlannerSpecificData;

            if (stateinfo->g == INFINITECOST) {
                SBPL_ERROR("ERROR in ReconstructPath: g of the state on the path is INFINITE\n");
                throw SBPL_Exception();
                //return -1;
            }

            if (stateinfo->bestpredstate == NULL) {
                SBPL_ERROR("ERROR in ReconstructPath: bestpred is NULL\n");
                throw SBPL_Exception();
            }

            // get the parent state
            PredMDPstate = stateinfo->bestpredstate;
            predstateinfo = (ARAState*)PredMDPstate->PlannerSpecificData;

            // set its best next info
            predstateinfo->bestnextstate = MDPstate;

            // check the decrease of g-values along the path
            if (predstateinfo->v >= stateinfo->g) {
                SBPL_ERROR("ERROR in ReconstructPath: g-values are non-decreasing\n");
                throw SBPL_Exception();
            }

            //transition back
            MDPstate = PredMDPstate;
        }
    }

    return 1;
}

void ARAPlanner_AD::PrintSearchPath(
    ARASearchStateSpace_t* pSearchStateSpace,
    FILE* fOut)
{
    ARAState* searchstateinfo;
    CMDPSTATE* state;
    int goalID;
    int PathCost;

    if (bforwardsearch) {
        state  = pSearchStateSpace->searchstartstate;
        goalID = pSearchStateSpace->searchgoalstate->StateID;
    }
    else {
        state = pSearchStateSpace->searchgoalstate;
        goalID = pSearchStateSpace->searchstartstate->StateID;
    }
    if (fOut == NULL) {
        fOut = stdout;
    }

    PathCost = ((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;

    SBPL_FPRINTF(fOut, "Printing a path from state %d to the goal state %d\n", state->StateID, pSearchStateSpace->searchgoalstate->StateID);
    SBPL_FPRINTF(fOut, "Path cost = %d:\n", PathCost);

    environment_->PrintState(state->StateID, false, fOut);

    int costFromStart = 0;
    while (state->StateID != goalID) {
        SBPL_FPRINTF(fOut, "state %d ", state->StateID);

        if (state->PlannerSpecificData == NULL) {
            SBPL_FPRINTF(fOut, "path does not exist since search data does not exist\n");
            break;
        }

        searchstateinfo = (ARAState*)state->PlannerSpecificData;

        if (searchstateinfo->bestnextstate == NULL) {
            SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
            break;
        }
        if (searchstateinfo->g == INFINITECOST) {
            SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
            break;
        }

        int costToGoal = PathCost - costFromStart;
        int transcost = searchstateinfo->g - ((ARAState*)(searchstateinfo->bestnextstate->PlannerSpecificData))->v;
        if (bforwardsearch) {
            transcost = -transcost;
        }

        costFromStart += transcost;

        SBPL_FPRINTF(fOut, "g=%d-->state %d, h = %d ctg = %d  ", searchstateinfo->g, searchstateinfo->bestnextstate->StateID, searchstateinfo->h, costToGoal);

        state = searchstateinfo->bestnextstate;

        environment_->PrintState(state->StateID, false, fOut);
    }
}

void ARAPlanner_AD::PrintSearchState(ARAState* state, FILE* fOut)
{
    SBPL_FPRINTF(fOut, "state %d: h=%d g=%u v=%u iterc=%d callnuma=%d heapind=%d inconslist=%d\n", state->MDPstate->StateID, state->h, state->g, state->v, state->iterationclosed, state->callnumberaccessed, state->heapindex, state->listelem[ARA_AD_INCONS_LIST_ID] ? 1 : 0);
    environment_->PrintState(state->MDPstate->StateID, true, fOut);
}

int ARAPlanner_AD::getHeurValue(
    ARASearchStateSpace_t* pSearchStateSpace,
    int StateID)
{
    CMDPSTATE* MDPstate = GetState(StateID, pSearchStateSpace);
    ARAState* searchstateinfo = (ARAState*)MDPstate->PlannerSpecificData;
    return searchstateinfo->h;
}

std::vector<int> ARAPlanner_AD::GetSearchPath(
    ARASearchStateSpace_t* pSearchStateSpace,
    int& solcost,
    CMDPSTATE* beststate)
{
    std::vector<int> SuccIDV;
    std::vector<int> CostV;
    std::vector<int> wholePathIds;
    ARAState* searchstateinfo;
    CMDPSTATE* state = NULL;
    CMDPSTATE* goalstate = NULL;
    CMDPSTATE* startstate = NULL;

    if (bforwardsearch) {
        startstate = pSearchStateSpace->searchstartstate;
        if (beststate != NULL) {
            SBPL_WARN("Reconstructing partial path");
            goalstate = beststate;
        }
        else {
            goalstate = pSearchStateSpace->searchgoalstate;
        }

        // reconstruct the path by setting bestnextstate pointers appropriately
        ReconstructPath(pSearchStateSpace, goalstate);
    }
    else {
        if (beststate!=NULL) {
            startstate = beststate;
        }
        else {
            startstate = pSearchStateSpace->searchgoalstate;
        }
        goalstate = pSearchStateSpace->searchstartstate;
    }

    state = startstate;

    wholePathIds.push_back(state->StateID);
    solcost = 0;

    FILE* fOut = stdout;
    if (fOut == NULL) {
        SBPL_ERROR("ERROR: could not open file\n");
        throw SBPL_Exception();
    }
    while (state->StateID != goalstate->StateID) {
        if (state->PlannerSpecificData == NULL) {
            SBPL_FPRINTF(fOut, "path does not exist since search data does not exist\n");
            break;
        }

        searchstateinfo = (ARAState*)state->PlannerSpecificData;

        if (searchstateinfo->bestnextstate == NULL) {
            SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
            break;
        }
        if (searchstateinfo->g == INFINITECOST) {
            SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
            break;
        }

        environment_->GetSuccs(state->StateID, &SuccIDV, &CostV);
        int actioncost = INFINITECOST;
        for (int i = 0; i < (int)SuccIDV.size(); i++) {
            if (SuccIDV.at(i) == searchstateinfo->bestnextstate->StateID && CostV.at(i) < actioncost) {
                actioncost = CostV.at(i);
            }
        }
        if (actioncost == INFINITECOST) {
            SBPL_PRINTF("WARNING: actioncost = %d\n", actioncost);
        }

        solcost += actioncost;

        state = searchstateinfo->bestnextstate;

        wholePathIds.push_back(state->StateID);
    }

    return wholePathIds;
}

bool ARAPlanner_AD::Search(
    ARASearchStateSpace_t* pSearchStateSpace,
    std::vector<int>& pathIds,
    int & PathCost,
    bool bFirstSolution,
    bool bOptimalSolution,
    double MaxNumofSecs)
{
    CKey key;
    TimeStarted = sbpl::clock::now();
    searchexpands = 0;

    if (pSearchStateSpace->bReinitializeSearchStateSpace == true ||
        pSearchStateSpace_->heap->emptyheap())
    {
        // re-initialize state space
//        SBPL_ERROR("Reinit search state space!");
//        SBPL_ERROR("Heap size: %d, force reInit = %b", pSearchStateSpace_->heap->currentsize, pSearchStateSpace->bReinitializeSearchStateSpace);
        ReInitializeSearchStateSpace(pSearchStateSpace);
    }

    if (bOptimalSolution) {
        pSearchStateSpace->eps = 1;
        MaxNumofSecs = INFINITECOST;
    }
    else if (bFirstSolution) {
//        MaxNumofSecs = INFINITECOST;
    }

    // ensure heuristics are up-to-date
    environment_->EnsureHeuristicsUpdated((bforwardsearch==true));

    // the main loop of ARA*
    int prevexpands = 0;
    sbpl::clock::time_point loop_time;
    while (pSearchStateSpace->eps_satisfied > ARA_FINAL_EPS &&
        sbpl::to_seconds(sbpl::clock::now() - TimeStarted) < MaxNumofSecs)
    {
        loop_time = sbpl::clock::now();
        // decrease eps for all subsequent iterations
        if (fabs(pSearchStateSpace->eps_satisfied - pSearchStateSpace->eps) < ERR_EPS &&
            !bFirstSolution)
        {
            pSearchStateSpace->eps = pSearchStateSpace->eps - ARA_DECREASE_EPS;
            if (pSearchStateSpace->eps < ARA_FINAL_EPS) {
                pSearchStateSpace->eps = ARA_FINAL_EPS;
            }

            // the priorities need to be updated
            pSearchStateSpace->bReevaluatefvals = true;

            // it will be a new search
            pSearchStateSpace->bNewSearchIteration = true;

            // build a new open list by merging it with incons one
            BuildNewOPENList(pSearchStateSpace);
        }

        if (pSearchStateSpace->bNewSearchIteration) {
            pSearchStateSpace->searchiteration++;
            pSearchStateSpace->bNewSearchIteration = false;
        }

        // re-compute f-values if necessary and reorder the heap
        if (pSearchStateSpace->bReevaluatefvals) {
            Reevaluatefvals(pSearchStateSpace);
        }

        // improve or compute path
        if (ImprovePath(pSearchStateSpace, MaxNumofSecs) == 1) {
            pSearchStateSpace->eps_satisfied = pSearchStateSpace->eps;
        }

        // print the solution cost and eps bound
        SBPL_PRINTF("eps=%f expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands, ((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g, sbpl::to_seconds(sbpl::clock::now() - loop_time));

        if (pSearchStateSpace->eps_satisfied == finitial_eps &&
            pSearchStateSpace->eps == finitial_eps)
        {
            finitial_eps_planning_time = sbpl::to_seconds(sbpl::clock::now() - loop_time);
            num_of_expands_initial_solution = searchexpands - prevexpands;
        }

        prevexpands = searchexpands;

        // if just the first solution then we are done
        if (bFirstSolution) {
            break;
        }

        // no solution exists
        if (((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g == INFINITECOST) {
            break;
        }
    }

    PathCost = ((ARAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;
    MaxMemoryCounter += environment_->StateID2IndexMapping.size() * sizeof(int);

    SBPL_PRINTF("MaxMemoryCounter = %d\n", MaxMemoryCounter);

    int solcost = INFINITECOST;
    bool ret = false;
    if (PathCost == INFINITECOST) {
        SBPL_WARN("Tracking could not reach goal!");
        int BestStateID = environment_->getBestSeenState();
        SBPL_INFO("Best stateID: %d", BestStateID);
        if (BestStateID >= 0) {
            SBPL_WARN("Reconstructing partial path!");
            CMDPSTATE* beststate = GetState(BestStateID, pSearchStateSpace);
            pathIds = GetSearchPath(pSearchStateSpace, solcost, beststate);
        }
        ret = false;
    }
    else {
        SBPL_INFO("Tracking success!");
        pathIds = GetSearchPath(pSearchStateSpace, solcost, NULL);
        ret = true;
    }

    SBPL_PRINTF("total expands this call = %d, planning time = %.3f secs, solution cost=%d\n", searchexpands, sbpl::to_seconds(sbpl::clock::now() - TimeStarted), solcost);
    final_eps_planning_time = sbpl::to_seconds(sbpl::clock::now() - TimeStarted);
    final_eps = pSearchStateSpace->eps_satisfied;

    return ret;
}

// returns 1 if found a solution, and 0 otherwise
int ARAPlanner_AD::replan(
    double allocated_time_secs,
    std::vector<int>* solution_stateIDs_V)
{
    int solcost;
    return replan(allocated_time_secs, solution_stateIDs_V, &solcost);
}

// returns 1 if found a solution, and 0 otherwise
int ARAPlanner_AD::replan(
    double allocated_time_secs,
    std::vector<int>* solution_stateIDs_V,
    int* psolcost)
{
    std::vector<int> pathIds;
    bool bFound = false;
    int PathCost;
    bool bFirstSolution = this->bsearchuntilfirstsolution;
    bool bOptimalSolution = false;
    *psolcost = 0;

    SBPL_PRINTF("planner: replan called (bFirstSol=%d, bOptSol=%d)\n", bFirstSolution, bOptimalSolution);

    // plan
    if ((bFound = Search(pSearchStateSpace_, pathIds, PathCost, bFirstSolution, bOptimalSolution, allocated_time_secs)) == false) {
        SBPL_PRINTF("failed to find a solution\n");
    }

    // copy the solution
    *solution_stateIDs_V = pathIds;
    *psolcost = PathCost;

    return (int)bFound;
}

int ARAPlanner_AD::set_goal(int goal_stateID)
{
    if (bforwardsearch) {
        if (SetSearchGoalState(goal_stateID, pSearchStateSpace_) != 1) {
            SBPL_ERROR("ERROR: failed to set search goal state\n");
            return 0;
        }
    }
    else {
        if (SetSearchStartState(goal_stateID, pSearchStateSpace_) != 1) {
            SBPL_ERROR("ERROR: failed to set search start state\n");
            return 0;
        }
    }

    return 1;
}

int ARAPlanner_AD::set_start(int start_stateID)
{
    if (bforwardsearch) {
        if (SetSearchStartState(start_stateID, pSearchStateSpace_) != 1) {
            SBPL_ERROR("ERROR: failed to set search start state\n");
            return 0;
        }
    }
    else {
        if (SetSearchGoalState(start_stateID, pSearchStateSpace_) != 1) {
            SBPL_ERROR("ERROR: failed to set search goal state\n");
            return 0;
        }
    }

    return 1;
}

void ARAPlanner_AD::costs_changed(const StateChangeQuery& stateChange)
{
    SBPL_ERROR("costs_changed(..) reInit!");
    pSearchStateSpace_->bReinitializeSearchStateSpace = true;
}

void ARAPlanner_AD::costs_changed()
{
    SBPL_ERROR("costs_changed() reInit!");
    pSearchStateSpace_->bReinitializeSearchStateSpace = true;
}

int ARAPlanner_AD::force_planning_from_scratch()
{
    pSearchStateSpace_->bReinitializeSearchStateSpace = true;
    return 1;
}

int ARAPlanner_AD::set_search_mode(bool bSearchUntilFirstSolution)
{
    SBPL_PRINTF("planner: search mode set to %d\n", bSearchUntilFirstSolution);

    bsearchuntilfirstsolution = bSearchUntilFirstSolution;

    return 1;
}

void ARAPlanner_AD::print_searchpath(FILE* fOut)
{
    PrintSearchPath(pSearchStateSpace_, fOut);
}

} // namespace adim

