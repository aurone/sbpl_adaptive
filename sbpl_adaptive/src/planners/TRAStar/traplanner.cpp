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
#include <iostream>
using namespace std;

#include <sbpl_adaptive/headers.h>

//-----------------------------------------------------------------------------------------------------

TRAPlanner::TRAPlanner(DiscreteSpaceInformation* environment, bool bForwardSearch)
{
    environment_ = environment;

    bforwardsearch = bForwardSearch;

    bsearchuntilfirstsolution = false;

    finitial_eps = TRA_DEFAULT_INITIAL_EPS;
    searchexpands = 0;
    MaxMemoryCounter = 0;

#ifndef ROS
    const char* debug = "debug.txt";
#else
    const char* debug = "deb.txt";
#endif

    fDeb = SBPL_FOPEN(debug, "w");
    if(fDeb == NULL){
        SBPL_ERROR("ERROR: could not open planner debug file\n");
        throw new SBPL_Exception();
    }
    SBPL_PRINTF("debug on\n");

    pSearchStateSpace_ = new TRASearchStateSpace_t;

    //create the TRA planner
    if(CreateSearchStateSpace(pSearchStateSpace_) != 1)
    {
        SBPL_ERROR("ERROR: failed to create statespace\n");
        return;
    }

    //set the start and goal states
    if(InitializeSearchStateSpace(pSearchStateSpace_) != 1)
    {
        SBPL_ERROR("ERROR: failed to initialize statespace\n");
        return;
    }

    finitial_eps_planning_time = -1.0;
    final_eps_planning_time = -1.0;
    num_of_expands_initial_solution = 0;
    final_eps = -1.0;
}

TRAPlanner::~TRAPlanner()
{

    //delete the statespace
    DeleteSearchStateSpace(pSearchStateSpace_);
    delete pSearchStateSpace_;

    SBPL_FCLOSE(fDeb);
}


void TRAPlanner::Initialize_searchinfo(CMDPSTATE* state, TRASearchStateSpace_t* pSearchStateSpace)
{
    TRAState* searchstateinfo = (TRAState*)state->PlannerSpecificData;

    searchstateinfo->MDPstate = state;
    InitializeSearchStateInfo(searchstateinfo, pSearchStateSpace);
}


CMDPSTATE* TRAPlanner::CreateState(int stateID, TRASearchStateSpace_t* pSearchStateSpace)
{
    CMDPSTATE* state = NULL;
#if DEBUG
    if(environment_->StateID2IndexMapping[stateID][TRAMDP_STATEID2IND] != -1)
    {
        SBPL_ERROR("ERROR in CreateState: state already created\n");
        throw new SBPL_Exception();
    }
#endif

    //adds to the tail a state
    state = pSearchStateSpace->searchMDP.AddState(stateID);

    //remember the index of the state
    environment_->StateID2IndexMapping[stateID][TRAMDP_STATEID2IND] = pSearchStateSpace->searchMDP.StateArray.size()-1;

#if DEBUG
    if(state != pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][TRAMDP_STATEID2IND]])
    {
        SBPL_ERROR("ERROR in CreateState: invalid state index\n");
        throw new SBPL_Exception();
    }
#endif

    //create search specific info
    state->PlannerSpecificData = (TRAState*)malloc(sizeof(TRAState));
    Initialize_searchinfo(state, pSearchStateSpace);
    MaxMemoryCounter += sizeof(TRAState);

    return state;
}


CMDPSTATE* TRAPlanner::GetState(int stateID, TRASearchStateSpace_t* pSearchStateSpace)
{
    if(stateID >= (int)environment_->StateID2IndexMapping.size())
    {
        SBPL_ERROR("ERROR int GetState: stateID is invalid\n");
        throw new SBPL_Exception();
    }

    if(environment_->StateID2IndexMapping[stateID][TRAMDP_STATEID2IND] == -1)
        return CreateState(stateID, pSearchStateSpace);
    else
        return pSearchStateSpace->searchMDP.StateArray[environment_->StateID2IndexMapping[stateID][TRAMDP_STATEID2IND]];

}


//-----------------------------------------------------------------------------------------------------
int TRAPlanner::ComputeHeuristic(CMDPSTATE* MDPstate, TRASearchStateSpace_t* pSearchStateSpace)
{
    //compute heuristic for search
    if(bforwardsearch)
    {
        //forward search: heur = distance from state to searchgoal which is Goal TRAState
        return environment_->GetGoalHeuristic(MDPstate->StateID);
    }
    else
    {
        //backward search: heur = distance from searchgoal to state
        return environment_->GetStartHeuristic(MDPstate->StateID);
    }
}


//initialization of a state
void TRAPlanner::InitializeSearchStateInfo(TRAState* state, TRASearchStateSpace_t* pSearchStateSpace)
{
    state->g = INFINITECOST;
    state->E = INFINITECOST; //not expanded
    state->C = INFINITECOST; //not created (not yet put on OPEN)
    state->iterationclosed = 0;
    state->callnumberaccessed = pSearchStateSpace->callnumber;
    state->bestnextstate = NULL;
    state->costtobestnextstate = INFINITECOST;
    state->heapindex = 0;
    state->listelem[TRA_INCONS_LIST_ID] = NULL;
    state->numofexpands = 0;
    state->bestpredstate = NULL;

    //compute heuristics
#if USE_HEUR
    if(pSearchStateSpace->searchgoalstate != NULL)
        state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace);
    else
        state->h = 0;
#else
    state->h = 0;
#endif

}


//re-initialization of a state
void TRAPlanner::ReInitializeSearchStateInfo(TRAState* state, TRASearchStateSpace_t* pSearchStateSpace)
{
    state->g = INFINITECOST;

    //TRA* related
    state->E = INFINITECOST; //not expanded
    state->C = INFINITECOST; //not created (not yet put on OPEN)
    state->parent_hist.clear(); //reset the history of parents
    state->gval_hist.clear(); //reset the history of g-values
    //end TRA* related

    state->iterationclosed = 0;
    state->callnumberaccessed = pSearchStateSpace->callnumber;
    state->bestnextstate = NULL;
    state->costtobestnextstate = INFINITECOST;
    state->heapindex = 0;
    state->listelem[TRA_INCONS_LIST_ID] = NULL;
    state->numofexpands = 0;
    state->bestpredstate = NULL;

    //compute heuristics
#if USE_HEUR
    if(pSearchStateSpace->searchgoalstate != NULL)
    {
        state->h = ComputeHeuristic(state->MDPstate, pSearchStateSpace);
    }
    else
        state->h = 0;
#else
    state->h = 0;
#endif

}



void TRAPlanner::DeleteSearchStateData(TRAState* state)
{
    //no memory was allocated
    MaxMemoryCounter = 0;
    return;
}


int TRAPlanner::GetGVal(int StateID, TRASearchStateSpace_t* pSearchStateSpace)
{
    CMDPSTATE* cmdp_state = GetState(StateID, pSearchStateSpace);
    TRAState* state = (TRAState*)cmdp_state->PlannerSpecificData;
    return state->g;
}


//returns 1 if the solution is found, 0 if the solution does not exist and 2 if it ran out of time
int TRAPlanner::ImprovePath(TRASearchStateSpace_t* pSearchStateSpace, double MaxNumofSecs)
{
    SBPL_INFO("Improving path (eps = %.3f)!", pSearchStateSpace->eps);
    int expands;

    TRAState *state, *searchgoalstate;
    CKey key, minkey;
    CKey goalkey;

    expands = 0;

    if(pSearchStateSpace->searchgoalstate == NULL)
    {
        SBPL_ERROR("ERROR searching: no goal state is set\n");
        throw new SBPL_Exception();
    }

    //goal state
    searchgoalstate = (TRAState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);

    if(searchgoalstate->callnumberaccessed != pSearchStateSpace->callnumber)
        ReInitializeSearchStateInfo(searchgoalstate, pSearchStateSpace);

    //set goal key
    goalkey.key[0] = searchgoalstate->g;
    //goalkey.key[1] = searchgoalstate->h;

    //expand states until done
    minkey = pSearchStateSpace->heap->getminkeyheap();
    CKey oldkey = minkey;

    while( !pSearchStateSpace->heap->emptyheap() && //heap not empty
            minkey.key[0] < INFINITECOST && //min key is not infinite
            goalkey > minkey && //goal key is larger than the min key
            (clock()-TimeStarted) < MaxNumofSecs*(double)CLOCKS_PER_SEC ) //still have time
    {

        //get the state
        state = (TRAState*)pSearchStateSpace->heap->deleteminheap();

        //update expanded index
        state->E = pSearchStateSpace->expansion_step;

#if DEBUG
        if(state->listelem[TRA_INCONS_LIST_ID]  != NULL)
        {
            SBPL_FPRINTF(fDeb, "ERROR: expanding a state from inconslist\n");
            SBPL_ERROR("ERROR: expanding a state from inconslist\n");
            throw new SBPL_Exception();
        }
#endif

#if DEBUG
        if(minkey.key[0] < oldkey.key[0] && fabs(this->finitial_eps - 1.0) < ERR_EPS)
        {
            SBPL_PRINTF("WARN in search: the sequence of keys decreases\n");
            throw new SBPL_Exception();
        }
        oldkey = minkey;
#endif

#if DEBUG
        if(state->v == state->g)
        {
            SBPL_ERROR("ERROR: consistent state is being expanded\n");
            SBPL_FPRINTF(fDeb, "ERROR: consistent state is being expanded\n");
            //environment_->visualizeState("expanded-consistent-state", state->MDPstate->StateID, 0);
            //throw new SBPL_Exception();
        }
#endif

        //recompute state value
        state->v = state->g;
        state->iterationclosed = pSearchStateSpace->searchiteration;

        //new expand
        expands++;
        state->numofexpands++;

        if(bforwardsearch == false)
            UpdatePreds(state, pSearchStateSpace);
        else
            UpdateSuccs(state, pSearchStateSpace);

        pSearchStateSpace->expansion_step++; //advance

        //recompute minkey
        minkey = pSearchStateSpace->heap->getminkeyheap();

        //recompute goalkey if necessary
        if(goalkey.key[0] != (int)searchgoalstate->g)
        {
            //recompute the goal key (heuristics should be zero)
            goalkey.key[0] = searchgoalstate->g;
            //goalkey.key[1] = searchgoalstate->h;
        }

        if(expands%100000 == 0 && expands > 0)
        {
            SBPL_PRINTF("expands so far=%u\n", expands);
        }

    } //end main loop (expanding states)

    int retv = 1;
    if(searchgoalstate->g == INFINITECOST && pSearchStateSpace->heap->emptyheap())
    {
        printf("solution does not exist: search exited because heap is empty\n");
        retv = 0;
    }
    else if(!pSearchStateSpace->heap->emptyheap() && goalkey > minkey)
    {
        printf("search exited because it ran out of time\n");
        retv = 2;
    }
    else if(searchgoalstate->g == INFINITECOST && !pSearchStateSpace->heap->emptyheap())
    {
        printf("solution does not exist: search exited because all candidates for expansion have infinite heuristics\n");
        retv = 0;
    }
    else
    {
        printf("search exited with a solution for eps=%.3f\n", pSearchStateSpace->eps);
        retv = 1;
    }

    searchexpands += expands;

    return retv;
}


void TRAPlanner::BuildNewOPENList(TRASearchStateSpace_t* pSearchStateSpace)
{
    TRAState *state;
    CKey key;
    CHeap* pheap = pSearchStateSpace->heap;
    CList* pinconslist = pSearchStateSpace->inconslist;

    //move incons into open
    while(pinconslist->firstelement != NULL)
    {
        state = (TRAState*)pinconslist->firstelement->liststate;

        //compute f-value
        key.key[0] = state->g + (int)(pSearchStateSpace->eps*state->h);
        //key.key[1] = state->h;

        //insert into OPEN
        if(state->heapindex == 0)
            pheap->insertheap(state, key);
        else
            pheap->updateheap(state, key); //should never happen, but sometimes it does - somewhere there is a bug TODO
        //remove from INCONS
        pinconslist->remove(state, TRA_INCONS_LIST_ID);
    }
    pSearchStateSpace->bRebuildOpenList = false;

}


void TRAPlanner::Reevaluatefvals(TRASearchStateSpace_t* pSearchStateSpace)
{
    CKey key;
    int i;
    CHeap* pheap = pSearchStateSpace->heap;

#if DEBUG
    SBPL_FPRINTF(fDeb, "re-computing heap priorities\n");
#endif

    //recompute priorities for states in OPEN and reorder it
    for (i = 1; i <= pheap->currentsize; ++i)
    {
        TRAState* state = (TRAState*)pheap->heap[i].heapstate;
        key.key[0] = state->g + (int)(pSearchStateSpace->eps*state->h);
        pheap->heap[i].key = key;
    }
    pheap->makeheap();

    pSearchStateSpace->bReevaluatefvals = false;
}




//creates (allocates memory) search state space
//does not initialize search statespace
int TRAPlanner::CreateSearchStateSpace(TRASearchStateSpace_t* pSearchStateSpace)
{

    //create a heap
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

//deallocates memory used by SearchStateSpace
void TRAPlanner::DeleteSearchStateSpace(TRASearchStateSpace_t* pSearchStateSpace)
{
    if(pSearchStateSpace->heap != NULL)
    {
        pSearchStateSpace->heap->makeemptyheap();
        delete pSearchStateSpace->heap;
        pSearchStateSpace->heap = NULL;
    }

    if(pSearchStateSpace->inconslist != NULL)
    {
        pSearchStateSpace->inconslist->makeemptylist(TRA_INCONS_LIST_ID);
        delete pSearchStateSpace->inconslist;
        pSearchStateSpace->inconslist = NULL;
    }

    //delete the states themselves
    int iend = (int)pSearchStateSpace->searchMDP.StateArray.size();
    for(int i=0; i < iend; i++)
    {
        CMDPSTATE* state = pSearchStateSpace->searchMDP.StateArray[i];
        DeleteSearchStateData((TRAState*)state->PlannerSpecificData);
        free(state->PlannerSpecificData); // allocated with malloc() on line 199 of revision 19485
        state->PlannerSpecificData = NULL;
    }
    pSearchStateSpace->searchMDP.Delete();
    environment_->StateID2IndexMapping.clear();
}



//reset properly search state space
//needs to be done before deleting states
int TRAPlanner::ResetSearchStateSpace(TRASearchStateSpace_t* pSearchStateSpace)
{
    pSearchStateSpace->heap->makeemptyheap();
    pSearchStateSpace->inconslist->makeemptylist(TRA_INCONS_LIST_ID);

    return 1;
}

//initialization before each search
void TRAPlanner::ReInitializeSearchStateSpace(TRASearchStateSpace_t* pSearchStateSpace)
{
    CKey key;

    //increase callnumber
    pSearchStateSpace->callnumber++;

    //reset iteration
    pSearchStateSpace->searchiteration = 0;


#if DEBUG
    SBPL_FPRINTF(fDeb, "reinitializing search state-space (new call number=%d search iter=%d)\n",
        pSearchStateSpace->callnumber,pSearchStateSpace->searchiteration );
#endif



    pSearchStateSpace->heap->makeemptyheap();
    pSearchStateSpace->inconslist->makeemptylist(TRA_INCONS_LIST_ID);

    //reset
    pSearchStateSpace->eps = this->finitial_eps;
    pSearchStateSpace->eps_satisfied = INFINITECOST;

    //initialize start state
    TRAState* startstateinfo = (TRAState*)(pSearchStateSpace->searchstartstate->PlannerSpecificData);
    if(startstateinfo->callnumberaccessed != pSearchStateSpace->callnumber)
        ReInitializeSearchStateInfo(startstateinfo, pSearchStateSpace);

    startstateinfo->g = 0;

    //insert start state into the heap
    key.key[0] = startstateinfo->g + (int)(pSearchStateSpace->eps*startstateinfo->h);
    pSearchStateSpace->heap->insertheap(startstateinfo, key);

    pSearchStateSpace->bReinitializeSearchStateSpace = false;
    pSearchStateSpace->bReevaluatefvals = false;
    pSearchStateSpace->bRebuildOpenList = false;
}

//very first initialization
int TRAPlanner::InitializeSearchStateSpace(TRASearchStateSpace_t* pSearchStateSpace)
{

    if(pSearchStateSpace->heap->currentsize != 0 ||
            pSearchStateSpace->inconslist->currentsize != 0)
    {
        SBPL_ERROR("ERROR in InitializeSearchStateSpace: heap or list is not empty\n");
        throw new SBPL_Exception();
    }

    pSearchStateSpace->eps = this->finitial_eps;
    pSearchStateSpace->eps_satisfied = INFINITECOST;
    pSearchStateSpace->searchiteration = 0;
    pSearchStateSpace->callnumber = 0;
    pSearchStateSpace->bReevaluatefvals = false;
    pSearchStateSpace->bRebuildOpenList = false;


    //create and set the search start state
    pSearchStateSpace->searchgoalstate = NULL;
    //pSearchStateSpace->searchstartstate = GetState(SearchStartStateID, pSearchStateSpace);
    pSearchStateSpace->searchstartstate = NULL;


    pSearchStateSpace->bReinitializeSearchStateSpace = true;

    return 1;

}


int TRAPlanner::SetSearchGoalState(int SearchGoalStateID, TRASearchStateSpace_t* pSearchStateSpace)
{

    if(pSearchStateSpace->searchgoalstate == NULL ||
            pSearchStateSpace->searchgoalstate->StateID != SearchGoalStateID)
    {
        pSearchStateSpace->searchgoalstate = GetState(SearchGoalStateID, pSearchStateSpace);

        //current solution may be invalid
        pSearchStateSpace->eps_satisfied = INFINITECOST;
        pSearchStateSpace_->eps = this->finitial_eps;

        //recompute heuristic for the heap if heuristics is used
#if USE_HEUR
        int i;
        //TODO - should get rid of and instead use iteration to re-compute h-values online as needed
        for(i = 0; i < (int)pSearchStateSpace->searchMDP.StateArray.size(); i++)
        {
            CMDPSTATE* MDPstate = pSearchStateSpace->searchMDP.StateArray[i];
            TRAState* state = (TRAState*)MDPstate->PlannerSpecificData;
            state->h = ComputeHeuristic(MDPstate, pSearchStateSpace);
        }
#if DEBUG
        SBPL_PRINTF("re-evaluated heuristic values for %d states\n", i);
#endif

        pSearchStateSpace->bReevaluatefvals = true;
#endif
    }


    return 1;

}


int TRAPlanner::SetSearchStartState(int SearchStartStateID, TRASearchStateSpace_t* pSearchStateSpace)
{
    CMDPSTATE* MDPstate = GetState(SearchStartStateID, pSearchStateSpace);

    if(MDPstate !=  pSearchStateSpace->searchstartstate)
    {
        pSearchStateSpace->searchstartstate = MDPstate;
        pSearchStateSpace->bReinitializeSearchStateSpace = true;
        pSearchStateSpace->bRebuildOpenList = true;
    }

    return 1;

}



int TRAPlanner::ReconstructPath(TRASearchStateSpace_t* pSearchStateSpace)
{

    //nothing to do, if search is backward
    if(bforwardsearch)
    {

        CMDPSTATE* MDPstate = pSearchStateSpace->searchgoalstate;
        CMDPSTATE* PredMDPstate;
        TRAState *predstateinfo, *stateinfo;

        int steps = 0;
        const int max_steps = 100000;
        while(MDPstate != pSearchStateSpace->searchstartstate && steps < max_steps)
        {
            steps++;

            stateinfo = (TRAState*)MDPstate->PlannerSpecificData;

            if(stateinfo->g == INFINITECOST)
            {
                //SBPL_ERROR("ERROR in ReconstructPath: g of the state on the path is INFINITE\n");
                //throw new SBPL_Exception();
                return -1;
            }

            if(stateinfo->bestpredstate == NULL)
            {
                SBPL_ERROR("ERROR in ReconstructPath: bestpred is NULL\n");
                throw new SBPL_Exception();
            }

            //get the parent state
            PredMDPstate = stateinfo->bestpredstate;
            predstateinfo = (TRAState*)PredMDPstate->PlannerSpecificData;

            //set its best next info
            predstateinfo->bestnextstate = MDPstate;

            //check the decrease of g-values along the path
            if(predstateinfo->v >= stateinfo->g)
            {
                SBPL_WARN("ERROR in ReconstructPath: g-values are non-decreasing\n");
                //throw new SBPL_Exception();
            }

            //transition back
            MDPstate = PredMDPstate;
        }

        if(MDPstate != pSearchStateSpace->searchstartstate){
            SBPL_ERROR("ERROR: Failed to reconstruct path (compute bestnextstate pointers): steps processed=%d\n", steps);
            return 0;
        }
    }

    return 1;
}


void TRAPlanner::PrintSearchState(TRAState* searchstateinfo, FILE* fOut)
{

    CKey key;
    key.key[0] = searchstateinfo->g + (int)(pSearchStateSpace_->eps*searchstateinfo->h);
    SBPL_FPRINTF(fOut, "g=%d v=%d h = %d heapindex=%d inconslist=%d key=[%d %d] iterc=%d callnuma=%d expands=%d (current callnum=%d iter=%d)",
        searchstateinfo->g, searchstateinfo->v, searchstateinfo->h, searchstateinfo->heapindex, (searchstateinfo->listelem[TRA_INCONS_LIST_ID] != NULL),
        (int)key[0], (int)key[1], searchstateinfo->iterationclosed, searchstateinfo->callnumberaccessed, searchstateinfo->numofexpands,
        this->pSearchStateSpace_->callnumber, this->pSearchStateSpace_->searchiteration);

}

void TRAPlanner::PrintSearchPath(TRASearchStateSpace_t* pSearchStateSpace, FILE* fOut)
{
    TRAState* searchstateinfo;
    CMDPSTATE* state = pSearchStateSpace->searchgoalstate;
    CMDPSTATE* nextstate = NULL;

    if(fOut == NULL)
        fOut = stdout;

    int PathCost = ((TRAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;

    SBPL_FPRINTF(fOut, "Printing a path from state %d to the search start state %d\n",
        state->StateID, pSearchStateSpace->searchstartstate->StateID);
    SBPL_FPRINTF(fOut, "Path cost = %d:\n", PathCost);

    environment_->PrintState(state->StateID, true, fOut);

    int costFromStart = 0;
    int steps = 0;
    const int max_steps = 100000;
    while(state->StateID != pSearchStateSpace->searchstartstate->StateID && steps < max_steps)
    {
        steps++;

        SBPL_FPRINTF(fOut, "state %d ", state->StateID);

        if(state->PlannerSpecificData == NULL)
        {
            SBPL_FPRINTF(fOut, "path does not exist since search data does not exist\n");
            break;
        }

        searchstateinfo = (TRAState*)state->PlannerSpecificData;

        if(bforwardsearch)
            nextstate = searchstateinfo->bestpredstate;
        else
            nextstate = searchstateinfo->bestnextstate;

        if(nextstate == NULL)
        {
            SBPL_FPRINTF(fOut, "path does not exist since nextstate == NULL\n");
            break;
        }
        if(searchstateinfo->g == INFINITECOST)
        {
            SBPL_FPRINTF(fOut, "path does not exist since state->g == NULL\n");
            break;
        }

        int costToGoal = PathCost - costFromStart;
        if(!bforwardsearch)
        {
            //otherwise this cost is not even set
            costFromStart += searchstateinfo->costtobestnextstate;
        }


#if DEBUG
        if(searchstateinfo->g > searchstateinfo->v){
            SBPL_FPRINTF(fOut, "ERROR: underconsistent state %d is encountered\n", state->StateID);
            throw new SBPL_Exception();
        }

        if(!bforwardsearch) //otherwise this cost is not even set
        {
            if(nextstate->PlannerSpecificData != NULL && searchstateinfo->g < searchstateinfo->costtobestnextstate + ((TRAState*)(nextstate->PlannerSpecificData))->g)
            {
                SBPL_FPRINTF(fOut, "ERROR: g(source) < c(source,target) + g(target)\n");
                throw new SBPL_Exception();
            }
        }

#endif

        //PrintSearchState(searchstateinfo, fOut);
        SBPL_FPRINTF(fOut, "-->state %d ctg = %d  ",
            nextstate->StateID, costToGoal);

        state = nextstate;

        environment_->PrintState(state->StateID, true, fOut);

    }

    if(state->StateID != pSearchStateSpace->searchstartstate->StateID){
        SBPL_ERROR("ERROR: Failed to printsearchpath, max_steps reached\n");
        return;
    }

}

int TRAPlanner::getHeurValue(TRASearchStateSpace_t* pSearchStateSpace, int StateID)
{
    CMDPSTATE* MDPstate = GetState(StateID, pSearchStateSpace);
    TRAState* searchstateinfo = (TRAState*)MDPstate->PlannerSpecificData;
    return searchstateinfo->h;
}


vector<int> TRAPlanner::GetSearchPath(TRASearchStateSpace_t* pSearchStateSpace, int& solcost)
{
    std::vector<int> SuccIDV;
    std::vector<int> CostV;
    std::vector<int> wholePathIds;

    TRAState* searchstateinfo;
    CMDPSTATE* state = NULL;
    CMDPSTATE* goalstate = NULL;
    CMDPSTATE* startstate=NULL;

    if(bforwardsearch)
    {
        startstate = pSearchStateSpace->searchstartstate;
        goalstate = pSearchStateSpace->searchgoalstate;

        //reconstruct the path by setting bestnextstate pointers appropriately
        ReconstructPath(pSearchStateSpace);
    }
    else
    {
        startstate = pSearchStateSpace->searchgoalstate;
        goalstate = pSearchStateSpace->searchstartstate;
    }

    state = startstate;

    wholePathIds.push_back(state->StateID);
    solcost = 0;

    FILE* fOut = stdout;
    if(fOut == NULL){
        SBPL_ERROR("ERROR: could not open file\n");
        throw new SBPL_Exception();
    }
    while(state->StateID != goalstate->StateID)
    {
        if(state->PlannerSpecificData == NULL)
        {
            SBPL_FPRINTF(fOut, "path does not exist since search data does not exist\n");
            break;
        }

        searchstateinfo = (TRAState*)state->PlannerSpecificData;

        if(searchstateinfo->bestnextstate == NULL)
        {
            SBPL_FPRINTF(fOut, "path does not exist since bestnextstate == NULL\n");
            break;
        }
        if(searchstateinfo->g == INFINITECOST)
        {
            SBPL_FPRINTF(fOut, "path does not exist since g-value is infinite\n");
            break;
        }

        environment_->GetSuccs(state->StateID, &SuccIDV, &CostV);
        int actioncost = INFINITECOST;
        for(int i = 0; i < (int)SuccIDV.size(); i++)
        {
            if(SuccIDV.at(i) == searchstateinfo->bestnextstate->StateID && CostV.at(i) < actioncost)
                actioncost = CostV.at(i);
        }
        if(actioncost == INFINITECOST)
            SBPL_PRINTF("WARNING: infinite actioncost = %d on path\n", actioncost);

        solcost += actioncost;

#if DEBUG
        TRAState* nextstateinfo = (TRAState*)(searchstateinfo->bestnextstate->PlannerSpecificData);
        if(actioncost != abs((int)(searchstateinfo->g - nextstateinfo->g)) && pSearchStateSpace->eps_satisfied <= 1.001)
        {
            SBPL_FPRINTF(fDeb, "ERROR: actioncost=%d is not matching the difference in g-values of %d\n",
                    actioncost, abs((int)(searchstateinfo->g - nextstateinfo->g)));
            SBPL_ERROR("ERROR: actioncost=%d is not matching the difference in g-values of %d\n",
                actioncost, abs((int)(searchstateinfo->g - nextstateinfo->g)));
            PrintSearchState(searchstateinfo, fDeb);
            PrintSearchState(nextstateinfo, fDeb);
        }
#endif
        state = searchstateinfo->bestnextstate;
        wholePathIds.push_back(state->StateID);
    }
    return wholePathIds;
}



bool TRAPlanner::Search(TRASearchStateSpace_t* pSearchStateSpace, vector<int>& pathIds, int & PathCost, bool bFirstSolution, bool bOptimalSolution, double MaxNumofSecs)
{
    CKey key;
    TimeStarted = clock();
    searchexpands = 0;

#if DEBUG
    SBPL_FPRINTF(fDeb, "TRAplanner: new search call (call number=%d)\n", pSearchStateSpace->callnumber);
#endif

    if(pSearchStateSpace->bReinitializeSearchStateSpace == true){
        //re-initialize state space
        ReInitializeSearchStateSpace(pSearchStateSpace);
    }


    if(bOptimalSolution)
    {
        pSearchStateSpace->eps = 1;
        MaxNumofSecs = INFINITECOST;
    }
    else if(bFirstSolution)
    {
        //MaxNumofSecs = INFINITECOST;
    }

    //ensure heuristics are up-to-date
    environment_->EnsureHeuristicsUpdated((bforwardsearch==true));

    //the main loop of TRA*
    int prevexpands = 0;
    clock_t loop_time;

    while(  pSearchStateSpace->eps_satisfied > TRA_FINAL_EPS &&
            (clock()- TimeStarted) < MaxNumofSecs * (double)CLOCKS_PER_SEC   )
    {
        loop_time = clock();
        //decrease eps for all subsequent iterations
        if(fabs(pSearchStateSpace->eps_satisfied - pSearchStateSpace->eps) < ERR_EPS && !bFirstSolution)
        {
            //keep decreasing eps if not looking for first solution
            pSearchStateSpace->eps = pSearchStateSpace->eps - TRA_DECREASE_EPS;
            if(pSearchStateSpace->eps < TRA_FINAL_EPS)
                pSearchStateSpace->eps = TRA_FINAL_EPS;

            //the priorities need to be updated
            pSearchStateSpace->bReevaluatefvals = true;

            //it will be a new search
            pSearchStateSpace->bNewSearchIteration = true;

            //build a new open list by merging it with incons one
            BuildNewOPENList(pSearchStateSpace);
        }

        if(pSearchStateSpace->bNewSearchIteration)
        {
            pSearchStateSpace->searchiteration++;
            pSearchStateSpace->bNewSearchIteration = false;
        }

        //re-compute f-values if necessary and reorder the heap
        if(pSearchStateSpace->bReevaluatefvals)
            Reevaluatefvals(pSearchStateSpace);

        //improve or compute path
        if(ImprovePath(pSearchStateSpace, MaxNumofSecs) == 1){
            pSearchStateSpace->eps_satisfied = pSearchStateSpace->eps;
        }

        //print the solution cost and eps bound
        SBPL_PRINTF("eps=%f expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands, ((TRAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g, double(clock()-loop_time) / (double)CLOCKS_PER_SEC);

        if(pSearchStateSpace->eps_satisfied == finitial_eps && pSearchStateSpace->eps == finitial_eps)
        {
            finitial_eps_planning_time = double(clock() - loop_time) / (double)CLOCKS_PER_SEC;
            num_of_expands_initial_solution = searchexpands - prevexpands;
        }

#if DEBUG
        SBPL_FPRINTF(fDeb, "eps=%f expands=%d g(searchgoal)=%d time=%.3f\n", pSearchStateSpace->eps_satisfied, searchexpands - prevexpands, ((TRAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g,double(clock() - loop_time)/ (double) CLOCKS_PER_SEC);
        PrintSearchState((TRAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData, fDeb);
#endif
        prevexpands = searchexpands;

        //if just the first solution then we are done
        if(bFirstSolution)
            break;

        //no solution exists
        if(((TRAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g == INFINITECOST)
            break;

    } //main loop (decreasing epsilon)

#if DEBUG
    SBPL_FFLUSH(fDeb);
#endif

    PathCost = ((TRAState*)pSearchStateSpace->searchgoalstate->PlannerSpecificData)->g;
    MaxMemoryCounter += environment_->StateID2IndexMapping.size()*sizeof(int);

    SBPL_PRINTF("MaxMemoryCounter = %d\n", MaxMemoryCounter);

    int solcost = INFINITECOST;
    bool ret = false;
    if(PathCost == INFINITECOST)
    {
        SBPL_PRINTF("could not find a solution\n");
        ret = false;
    }
    else
    {
        SBPL_PRINTF("solution is found\n");
        pathIds = GetSearchPath(pSearchStateSpace, solcost);
        ret = true;
    }

    SBPL_PRINTF("total expands this call = %d, planning time = %.3f secs, solution cost=%d\n",
        searchexpands, (clock()-TimeStarted)/((double)CLOCKS_PER_SEC), solcost);

    final_eps_planning_time = (clock()-TimeStarted)/((double)CLOCKS_PER_SEC);
    final_eps = pSearchStateSpace->eps_satisfied;

    return ret;
}


//-----------------------------Interface function-----------------------------------------------------
//returns 1 if found a solution, and 0 otherwise
int TRAPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V)
{
    int solcost;
    return replan(allocated_time_secs, solution_stateIDs_V, &solcost);
}


//returns 1 if found a solution, and 0 otherwise
int TRAPlanner::replan(double allocated_time_secs, vector<int>* solution_stateIDs_V, int* psolcost)
{
    std::vector<int> pathIds;
    bool bFound = false;
    int PathCost;
    bool bFirstSolution = this->bsearchuntilfirstsolution;
    bool bOptimalSolution = false;
    *psolcost = 0;

    SBPL_PRINTF("TRAplanner: replan called (bFirstSol=%d, bOptSol=%d)\n", bFirstSolution, bOptimalSolution);

    //plan
    if((bFound = Search(pSearchStateSpace_, pathIds, PathCost, bFirstSolution, bOptimalSolution, allocated_time_secs)) == false)
    {
        SBPL_PRINTF("TRAplanner: failed to find a solution\n");
    }

    //copy the solution
    *solution_stateIDs_V = pathIds;
    *psolcost = PathCost;

    return (int)bFound;
}

int TRAPlanner::set_goal(int goal_stateID)
{

    SBPL_PRINTF("TRAplanner: setting goal to %d\n", goal_stateID);
    environment_->PrintState(goal_stateID, true, stdout);

    //it will be a new search iteration
    pSearchStateSpace_->searchiteration++;
    pSearchStateSpace_->bRebuildOpenList = true; //is not really necessary for search goal changes

    if(bforwardsearch)
    {
        if(SetSearchGoalState(goal_stateID, pSearchStateSpace_) != 1)
        {
            SBPL_ERROR("TRAplanner ERROR: failed to set search goal state\n");
            return 0;
        }
    }
    else
    {
        if(SetSearchStartState(goal_stateID, pSearchStateSpace_) != 1)
        {
            SBPL_ERROR("TRAplanner ERROR: failed to set search start state\n");
            return 0;
        }
        //if the search start state changes we need to start from scratch (restore search to time 0)
        force_planning_from_scratch();
    }

    return 1;
}


int TRAPlanner::set_start(int start_stateID)
{
    SBPL_PRINTF("TRAplanner: setting start to %d\n", start_stateID);
    environment_->PrintState(start_stateID, true, stdout);

    //it will be a new search iteration
    pSearchStateSpace_->searchiteration++;
    pSearchStateSpace_->bRebuildOpenList = true;

    if(bforwardsearch)
    {
        if(SetSearchStartState(start_stateID, pSearchStateSpace_) != 1)
        {
            SBPL_ERROR("TRAplanner ERROR: failed to set search start state\n");
            return 0;
        }
        //if the search start state changes we need to start from scratch (restore search to time 0)
        force_planning_from_scratch();
    }
    else
    {
        if(SetSearchGoalState(start_stateID, pSearchStateSpace_) != 1)
        {
            SBPL_ERROR("TRAplanner ERROR: failed to set search goal state\n");
            return 0;
        }
    }
    return 1;
}


void TRAPlanner::update_succs_of_changededges(vector<int>* succstatesIDV)
{
    //TODO:: fix
    SBPL_PRINTF("TRAplanner: UpdateSuccs called on %d succs\n", (int)succstatesIDV->size());

    if(succstatesIDV->size() == 0) return;

    unsigned int earliest_creation = GetStateCreationTime(succstatesIDV->at(0));
    for(size_t i = 1; i < succstatesIDV->size(); i++){
        unsigned int C_ = GetStateCreationTime(succstatesIDV->at(i));
        if(C_ < earliest_creation){
            earliest_creation = C_;
        }
    }
    if(!RestoreSearchTree(pSearchStateSpace_, earliest_creation)){
        SBPL_ERROR("Tree-restoring failed!");
        throw new SBPL_Exception();
    } else {
        SBPL_INFO("Tree-restoring succeeded! Restored to time: %d", (int)earliest_creation);
    }
}


void TRAPlanner::update_preds_of_changededges(vector<int>* predstatesIDV)
{
    //TODO:: fix
    SBPL_PRINTF("TRAplanner: UpdatePreds called on %d preds\n", (int) predstatesIDV->size());
    if(predstatesIDV->size() == 0) return;

    unsigned int earliest_creation = GetStateCreationTime(predstatesIDV->at(0));
    for(size_t i = 1; i < predstatesIDV->size(); i++){
        unsigned int C_ = GetStateCreationTime(predstatesIDV->at(i));
        if(C_ < earliest_creation){
            earliest_creation = C_;
        }
    }
    if(!RestoreSearchTree(pSearchStateSpace_, earliest_creation)){
        SBPL_ERROR("Tree-restoring failed!");
        throw new SBPL_Exception();
    } else {
        SBPL_INFO("Tree-restoring succeeded! Restored to time: %d", (int)earliest_creation);
    }
}


int TRAPlanner::force_planning_from_scratch()
{
    SBPL_PRINTF("TRAplanner: forceplanfromscratch set\n");
    pSearchStateSpace_->bReinitializeSearchStateSpace = true;
    return 1;
}


int TRAPlanner::set_search_mode(bool bSearchUntilFirstSolution)
{
    std::string mode = (bSearchUntilFirstSolution)?std::string("FIRST SOLUTION"):std::string("BEST SOLUTION");
    SBPL_PRINTF("TRAplanner: search mode set to %d (%s)\n", bSearchUntilFirstSolution, mode.c_str());
    bsearchuntilfirstsolution = bSearchUntilFirstSolution;
    return 1;
}


void TRAPlanner::costs_changed(StateChangeQuery const & stateChange)
{
    //TODO: fix!
    if(pSearchStateSpace_->bReinitializeSearchStateSpace == true || pSearchStateSpace_->searchiteration == 0)
        return; //no processing if no search efforts anyway

    if (bforwardsearch){
        //Update_SearchSuccs_of_ChangedEdges(stateChange.getSuccessors());
    }
    else {
        //Update_SearchSuccs_of_ChangedEdges(stateChange.getPredecessors());
    }
}


//used for backward search
void TRAPlanner::UpdatePreds(TRAState* state, TRASearchStateSpace_t* pSearchStateSpace)
{
    std::vector<int> PredIDV;
    std::vector<int> CostV(1,state->E);
    CKey key;
    TRAState *predstate;

    environment_->GetPreds(state->MDPstate->StateID, &PredIDV, &CostV);

    //iterate through predecessors of s
    for(int pind = 0; pind < (int)PredIDV.size(); pind++)
    {
        CMDPSTATE* PredMDPState = GetState(PredIDV[pind], pSearchStateSpace);
        predstate = (TRAState*)(PredMDPState->PlannerSpecificData);
        if(predstate->callnumberaccessed != pSearchStateSpace->callnumber)
            ReInitializeSearchStateInfo(predstate, pSearchStateSpace);

        //see if we can improve the value of predstate
        if(predstate->g > state->v + CostV[pind])
        {
            predstate->g = state->v + CostV[pind];
            predstate->bestnextstate = state->MDPstate;
            predstate->costtobestnextstate = CostV[pind];

            //re-insert into heap if not closed yet
            if(predstate->iterationclosed != pSearchStateSpace->searchiteration)
            {
                key.key[0] = predstate->g + (int)(pSearchStateSpace->eps*predstate->h);
                //key.key[1] = predstate->h;
                if(predstate->heapindex != 0) {
                    //already in the heap
                    pSearchStateSpace->heap->updateheap(predstate,key);
                }
                else {
                    //inserting in the heap for the first time
                    pSearchStateSpace->heap->insertheap(predstate,key);
                    predstate->C = pSearchStateSpace->expansion_step;
                    if(predstate->E != INFINITECOST){
                        SBPL_ERROR("Inserting new state in the heap, but the state has non-infinite expansion index!");
                        throw new SBPL_Exception();
                    }
                }
            }
            //take care of incons list
            else if(predstate->listelem[TRA_INCONS_LIST_ID] == NULL)
            {
                pSearchStateSpace->inconslist->insert(predstate, TRA_INCONS_LIST_ID);
            }
        }
    } //for predecessors
}

//used for forward search
void TRAPlanner::UpdateSuccs(TRAState* state, TRASearchStateSpace_t* pSearchStateSpace)
{
    std::vector<int> SuccIDV;
    std::vector<int> CostV(1,state->E);
    CKey key;
    TRAState *succstate;

    environment_->GetSuccs(state->MDPstate->StateID, &SuccIDV, &CostV);

    //iterate through predecessors of s
    for(int sind = 0; sind < (int)SuccIDV.size(); sind++)
    {
        CMDPSTATE* SuccMDPState = GetState(SuccIDV[sind], pSearchStateSpace);
        int cost = CostV[sind];

        succstate = (TRAState*)(SuccMDPState->PlannerSpecificData);
        if(succstate->callnumberaccessed != pSearchStateSpace->callnumber)
            ReInitializeSearchStateInfo(succstate, pSearchStateSpace);

        //see if we can improve the value of succstate
        //taking into account the cost of action
        if(succstate->g > state->v + cost)
        {
            succstate->g = state->v + cost;
            succstate->bestpredstate = state->MDPstate;

            //re-insert into heap if not closed yet
            if(succstate->iterationclosed != pSearchStateSpace->searchiteration)
            {
                key.key[0] = succstate->g + (int)(pSearchStateSpace->eps*succstate->h);
                //key.key[1] = succstate->h;

                if(succstate->heapindex != 0) {
                    //already in the heap
                    pSearchStateSpace->heap->updateheap(succstate,key);
                }
                else {
                    //inserting in the heap for the first time
                    pSearchStateSpace->heap->insertheap(succstate,key);
                    succstate->C = pSearchStateSpace->expansion_step;
                    if(succstate->E != INFINITECOST){
                        SBPL_ERROR("Inserting new state in the heap, but the state has non-infinite expansion index!");
                        throw new SBPL_Exception();
                    }
                }
            }
            //take care of incons list
            else if(succstate->listelem[TRA_INCONS_LIST_ID] == NULL)
            {
                pSearchStateSpace->inconslist->insert(succstate, TRA_INCONS_LIST_ID);
            }
        } //check for cost improvement

    } //for actions
}

//restores the search tree / state to right before expansion # (expansion_step)
bool TRAPlanner::RestoreSearchTree(TRASearchStateSpace_t* pSearchStateSpace, unsigned int expansion_step){
    //TODO: add the code for decreasing edge costs
    clock_t start_t = clock();

    pSearchStateSpace_->heap->makeemptyheap();
    pSearchStateSpace_->inconslist->makeemptylist(TRA_INCONS_LIST_ID);
    std::vector<int> heapStates;
    std::vector<int> heapKeys;

    int Gind_idx = 0;
    for(int i = (int) pSearchStateSpace->seen_states.size() - 1; i >= 0; i--){
        if(pSearchStateSpace->seen_states[i]->C >= expansion_step) {
            //State should be UNSEEN
            //1. remove from incons list
            if(pSearchStateSpace->inconslist->in(pSearchStateSpace->seen_states[i], TRA_INCONS_LIST_ID)){
                pSearchStateSpace_->inconslist->remove(pSearchStateSpace->seen_states[i], TRA_INCONS_LIST_ID);
            }
            //2. remove from heap
            if(pSearchStateSpace->seen_states[i]->heapindex != 0){
                pSearchStateSpace_->heap->deleteheap(pSearchStateSpace->seen_states[i]);
            }
            ReInitializeSearchStateInfo(pSearchStateSpace->seen_states[i], pSearchStateSpace); //this will also reset the parent history
            Gind_idx = i;
            continue;
        }
        //state is not UNSEEN (either in OPEN or in CLOSED)
        //fix the parents (and g-values)
        if(!fixParents(pSearchStateSpace, pSearchStateSpace->seen_states[i], expansion_step)){
            SBPL_ERROR("TRAplanner: error when fixing parents!");
            throw new SBPL_Exception();
            return false;
        }

        if(pSearchStateSpace->seen_states[i]->E >= expansion_step){
            //State should be on OPEN
            //1. remove from closed
            pSearchStateSpace->seen_states[i]->iterationclosed = 0;
            //2. insert into heap
            CKey key;
            key.key[0] = pSearchStateSpace->seen_states[i]->g + (int)(pSearchStateSpace->eps*pSearchStateSpace->seen_states[i]->h);
            if(pSearchStateSpace->seen_states[i]->heapindex != 0){
                SBPL_WARN("Hmm.... State already in HEAP! -- updating");
                pSearchStateSpace->heap->updateheap(pSearchStateSpace->seen_states[i], key);
            } else {
                pSearchStateSpace->heap->insertheap(pSearchStateSpace->seen_states[i], key);
            }
            //3. remove from incons list
            if(pSearchStateSpace->inconslist->in(pSearchStateSpace->seen_states[i], TRA_INCONS_LIST_ID)){
                pSearchStateSpace->inconslist->remove(pSearchStateSpace->seen_states[i], TRA_INCONS_LIST_ID);
            }
            //4. mark as not yet expanded
            pSearchStateSpace->seen_states[i]->E = INFINITECOST;
        } else {
            //State should be on CLOSED
            //1. remove from heap
            if(pSearchStateSpace->seen_states[i]->heapindex != 0){
                pSearchStateSpace->heap->deleteheap(pSearchStateSpace->seen_states[i]);
            }
            //2. remove from incons list
            if(pSearchStateSpace->inconslist->in(pSearchStateSpace->seen_states[i], TRA_INCONS_LIST_ID)){
                pSearchStateSpace->inconslist->remove(pSearchStateSpace->seen_states[i], TRA_INCONS_LIST_ID);
            }
            pSearchStateSpace->seen_states[i]->iterationclosed = pSearchStateSpace_->searchiteration;
        }
    }

    pSearchStateSpace->seen_states.resize(Gind_idx);
    pSearchStateSpace->expansion_step = expansion_step - 1;

    pSearchStateSpace->eps_satisfied = INFINITECOST;

    TRAState* searchgoalstate = (TRAState*)(pSearchStateSpace->searchgoalstate->PlannerSpecificData);
    searchgoalstate->g = INFINITECOST;

    double elapsed_t = (clock() - start_t) / (double) CLOCKS_PER_SEC;
    SBPL_ERROR("TRAplanner: restored search state to step %d in %.3f sec. (%d seen states in the state-space)", expansion_step, elapsed_t, (int)pSearchStateSpace->seen_states.size());
    return true;
}


bool TRAPlanner::fixParents(TRASearchStateSpace_t* pSearchStateSpace, TRAState* state, unsigned int expansion_step) {
    if(expansion_step > pSearchStateSpace->expansion_step) {
        SBPL_WARN("TRAplanner: the restore step is in the future! cannot fix parent histories!");
        return true;
    }
    //check if the state was created at step (expansion_step)
    if(state->C >= expansion_step){
        //the state has not yet been created! -- reset state and parents history
        ReInitializeSearchStateInfo(state, pSearchStateSpace);
        return true;
    }

    if(state->parent_hist.size() == 0) {
        if(state->MDPstate->StateID != pSearchStateSpace->searchstartstate->StateID){
            SBPL_WARN("The state has no parents! (StateID: %d)", state->MDPstate->StateID);
            SBPL_WARN("How/why is it in the CREATED list???");
            state->g = INFINITECOST;
            state->bestpredstate = NULL;
            throw new SBPL_Exception();
            return false;
        }
        return true;
    }

    //go backwards through the parent history and find the last valid parent
    for(int i = (int)state->parent_hist.size() - 1; i >= 0; i--){
        //check if the parent is valid -- expanded before (expansion_step)
        if(state->parent_hist[i]->E < expansion_step)
        {
            //first valid parent
            state->bestpredstate = state->parent_hist[i]->MDPstate;
            state->g = state->gval_hist[i];
            SBPL_INFO("Valid parent found!\nHistory len before %d", (int)state->parent_hist.size());
            state->parent_hist.resize(i+1);
            state->gval_hist.resize(i+1);
            SBPL_INFO("History len after %d\n-----", (int)state->parent_hist.size());
            return true;
        }
    }
    SBPL_ERROR("No valid parents found for state -- hmm... this should not happen (I think) -- for now just resetting state!");
    ReInitializeSearchStateInfo(state, pSearchStateSpace);
    throw new SBPL_Exception();
    return false;
}
