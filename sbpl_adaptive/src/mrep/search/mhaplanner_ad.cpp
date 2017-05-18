#include <sbpl_adaptive/mrep/search/mhaplanner_ad.h>

// standard includes
#include <assert.h>
#include <stdlib.h>
#include <time.h>
#include <algorithm>

// system includes
#include <sbpl/utils/key.h>

// project includes
#include <sbpl_adaptive/common.h>
#include <sbpl_adaptive/mrep/graph/multirep_adaptive_discrete_space.h>

namespace adim {

static const char *SLOG = "mrmha";
static const char *SELOG = "mrmha.expansion";
static const char *SSLOG = "mrmha.successors";

ADMHAPlannerAllocator::ADMHAPlannerAllocator(
    MultiRepHeuristic *aheur,
    MultiRepHeuristic **heurs,
    int h_count)
:
    aheur_(aheur),
    heurs_(heurs),
    h_count_(h_count)
{
}

SBPLPlanner *ADMHAPlannerAllocator::make(
    AdaptiveDiscreteSpace*space,
    bool forward_search) const
{
    MultiRepAdaptiveDiscreteSpace* mrep_space =
            dynamic_cast<MultiRepAdaptiveDiscreteSpace*>(space);
    if (!mrep_space) {
        ROS_ERROR("AdaptiveDiscreteSpaceInformation must be a MultiRepAdaptiveDiscreteSpace");
        return nullptr;
    }

    return new MHAPlanner_AD(mrep_space, aheur_, heurs_, h_count_);
}

MHAPlanner_AD::MHAPlanner_AD(
    MultiRepAdaptiveDiscreteSpace* space,
    MultiRepHeuristic* hanchor,
    MultiRepHeuristic** heurs,
    int hcount)
:
    SBPLPlanner(),
//    environment_(environment),
    space_(space),
    m_hanchor(hanchor),
    m_heurs(heurs),
    m_hcount(hcount),
    m_params(0.0),
    m_initial_eps_mha(100.0),
    m_max_expansions(0),
    m_eps(1.0),
    m_eps_mha(1.0),
    m_eps_satisfied((double)INFINITECOST),
    m_num_expansions(0),
    m_elapsed(sbpl::clock::duration::zero()),
    m_call_number(0), // uninitialized
    m_start_state(NULL),
    m_goal_state(NULL),
    m_search_states(),
    m_open(NULL),
    set_heur_(false),
    m_last_start_state_id(-1),
    m_last_goal_state_id(-1)
{
    environment_ = space;

    m_open = new CHeap[hcount + 1];

    // Overwrite default members for ReplanParams to represent a single optimal
    // search
    m_params.initial_eps = 100.0;
    m_params.final_eps = 100.0;
    m_params.dec_eps = 1.0;
    m_params.return_first_solution = false;
    m_params.max_time = 0.0;
    m_params.repair_time = 0.0;

    // map from representation id to the indices of heuristics that apply to it
    m_heuristic_list[-1] = { 0 };

    for (int i = 0; i < space->NumRepresentations(); ++i) {
        // anchor heuristic should apply to every representation
        m_heuristic_list[i].push_back(0);
        for (int j = 0; j < hcount; ++j) {
            if (heurs[j]->IsDefinedForRepresentation(i)) {
                m_heuristic_list[i].push_back(j + 1);
            }
        }
    }

    ROS_DEBUG_NAMED(SLOG, "Representation -> Heuristic Mapping:");
    for (const auto &entry : m_heuristic_list) {
        std::stringstream ss;
        ss << entry.first << ": [ ";
        for (size_t i = 0; i < entry.second.size(); ++i) {
            ss << entry.second[i];
            if (i != entry.second.size() - 1) {
                ss << ", ";
            }
            else {
                ss << ' ';
            }
        }
        ss << ']';
        ROS_DEBUG_NAMED(SLOG, "  %s", ss.str().c_str());
    }

    /// Four Modes:
    ///     Search Until Solution Bounded
    ///     Search Until Solution Unbounded
    ///     Improve Solution Bounded
    ///     Improve Solution Unbounded
}

MHAPlanner_AD::~MHAPlanner_AD()
{
    clear();

    delete[] m_open;
}

int MHAPlanner_AD::set_start(int start_stateID)
{
    m_start_state = get_state(start_stateID);
    if (!m_start_state) {
        return 0;
    }
    else {
        return 1;
    }
}

int MHAPlanner_AD::set_goal(int goal_stateID)
{
    m_goal_state = get_state(goal_stateID);
    if (!m_goal_state) {
        return 0;
    }
    else {
        return 1;
    }
}

int MHAPlanner_AD::replan(
    double allocated_time_sec,
    std::vector<int>* solution_stateIDs_V)
{
    int solcost;
    return replan(allocated_time_sec, solution_stateIDs_V, &solcost);
}

int MHAPlanner_AD::replan(
    double allocated_time_sec,
    std::vector<int>* solution_stateIDs_V,
    int* solcost)
{
    ReplanParams params = m_params;
    params.max_time = allocated_time_sec;
    return replan(solution_stateIDs_V, params, solcost);
}

int MHAPlanner_AD::replan(
    std::vector<int>* solution_stateIDs_V,
    ReplanParams params)
{
    int solcost;
    return replan(solution_stateIDs_V, params, &solcost);
}

int MHAPlanner_AD::replan(
    std::vector<int>* solution_stateIDs_V,
    ReplanParams params,
    int* solcost)
{
    if (!m_start_state) {
        ROS_ERROR("Start state is not set");
        return 0;
    }
    if (!m_goal_state) {
        ROS_ERROR("Goal state is not set");
        return 0;
    }

    if (!check_params(m_params)) { // errors printed within
        return 0;
    }

    if (m_start_state->state_id != m_last_start_state_id ||
        m_goal_state->state_id != m_last_goal_state_id)
    {
        // TODO: the search doesn't need to reinitialize when the goal state
        // changes
        reinit_search();

        ++m_call_number;

        space_->EnsureHeuristicsUpdated(true); // TODO: support backwards search

        reinit_state(m_goal_state);
        reinit_state(m_start_state);
        m_start_state->g = 0;

        // insert start state into all heaps with key(start, i) as priority
        int dimID = space_->GetDimID(m_start_state->state_id);
        for (int hidx : m_heuristic_list[dimID]) {
            //    for (int hidx = 0; hidx < num_heuristics(); ++hidx) {
            CKey key;
            key.key[0] = compute_key(m_start_state, hidx);
            m_open[hidx].insertheap(&m_start_state->od[hidx].open_state, key);
            ROS_DEBUG_NAMED(SLOG, "Inserted start state %d into search %d with f = %ld", m_start_state->state_id, hidx, key.key[0]);
        }

        m_eps = m_params.initial_eps;
        m_eps_mha = m_initial_eps_mha;
        m_eps_satisfied = (double)INFINITECOST;

        m_last_start_state_id = m_start_state->state_id;
        m_last_goal_state_id = m_goal_state->state_id;
    }

    // m_params = params;
    m_params.max_time = params.max_time;

    ROS_DEBUG_NAMED(SLOG, "Generic Search parameters:");
    ROS_DEBUG_NAMED(SLOG, "  Initial Epsilon: %0.3f", m_params.initial_eps);
    ROS_DEBUG_NAMED(SLOG, "  Final Epsilon: %0.3f", m_params.final_eps);
    ROS_DEBUG_NAMED(SLOG, "  Delta Epsilon: %0.3f", m_params.dec_eps);
    ROS_DEBUG_NAMED(SLOG, "  Return First Solution: %s", m_params.return_first_solution ? "true" : "false");
    ROS_DEBUG_NAMED(SLOG, "  Max Time: %0.3f", m_params.max_time);
    ROS_DEBUG_NAMED(SLOG, "  Repair Time: %0.3f", m_params.repair_time);
    ROS_DEBUG_NAMED(SLOG, "MHA Search parameters:");
    ROS_DEBUG_NAMED(SLOG, "  MHA Epsilon: %0.3f", m_initial_eps_mha);
    ROS_DEBUG_NAMED(SLOG, "  Max Expansions: %d", m_max_expansions);

    // reset time limits
    m_num_expansions = 0;
    m_elapsed = sbpl::clock::duration::zero();

    sbpl::clock::time_point start_time, end_time;

    start_time = sbpl::clock::now();

    end_time = sbpl::clock::now();
    m_elapsed += end_time - start_time;

    while (!m_open[0].emptyheap() && !time_limit_reached()) {
        start_time = sbpl::clock::now();
        // special case for mha* without additional heuristics
        if (num_heuristics() == 1) {
            if (m_goal_state->g <= get_minf(m_open[0])) {
                m_eps_satisfied = m_eps * m_eps_mha;
                extract_path(solution_stateIDs_V, solcost);
                return 1;
            }
            else {
                MHASearchState* s = state_from_open_state(m_open[0].getminheap());
                expand(s, 0);
            }
        }

        bool all_empty = true;
        for (int hidx = 1; hidx < num_heuristics(); ++hidx) {
            if (m_open[0].emptyheap()) {
                ROS_INFO_NAMED(SLOG, "Anchor empty");
                break;
            }

            all_empty &= m_open[hidx].emptyheap();
            if (m_open[hidx].emptyheap()) {
                continue;
            }

            if (get_minf(m_open[hidx]) <= m_eps_mha * get_minf(m_open[0])) {
                if (m_goal_state->g <= get_minf(m_open[hidx])) {
                    m_eps_satisfied = m_eps * m_eps_mha;
                    extract_path(solution_stateIDs_V, solcost);
                    return 1;
                }
                else {
                    MHASearchState* s =
                            state_from_open_state(m_open[hidx].getminheap());
                    expand(s, hidx);
                }
            }
            else {
                if (m_goal_state->g <= get_minf(m_open[0])) {
                    m_eps_satisfied = m_eps * m_eps_mha;
                    extract_path(solution_stateIDs_V, solcost);
                    return 1;
                }
                else {
                    MHASearchState* s =
                            state_from_open_state(m_open[0].getminheap());
                    expand(s, 0);
                }
            }
        }

        if (all_empty && !m_open[0].emptyheap()) {
            if (m_goal_state->g <= get_minf(m_open[0])) {
                m_eps_satisfied = m_eps * m_eps_mha;
                extract_path(solution_stateIDs_V, solcost);
                return 1;
            }
            else {
                MHASearchState* s =
                        state_from_open_state(m_open[0].getminheap());
                expand(s, 0);
            }
        }

        end_time = sbpl::clock::now();
        m_elapsed += end_time - start_time;
    }

    if (m_open[0].emptyheap()) {
        ROS_DEBUG_NAMED(SLOG, "Anchor search exhausted");
    }
    if (time_limit_reached()) {
        ROS_DEBUG_NAMED(SLOG, "Time limit reached");

        int best_state_id = space_->getBestSeenState();
        ROS_DEBUG_NAMED(SLOG, "Best stateID: %d", best_state_id);
        if (best_state_id >= 0) {
            ROS_WARN("Reconstructing partial path!");
            MHASearchState* best_seen_state = get_state(best_state_id);
            extract_partial_path(solution_stateIDs_V, solcost, best_seen_state);
            if (best_state_id == m_start_state->state_id) {
                return 0;
            }
            return 1;
        }

    }

    return 0;
}

int MHAPlanner_AD::force_planning_from_scratch()
{
    return 0;
}

int MHAPlanner_AD::force_planning_from_scratch_and_free_memory()
{
    return 0;
}

void MHAPlanner_AD::costs_changed(StateChangeQuery const & stateChange)
{
}

void MHAPlanner_AD::costs_changed()
{
}

int MHAPlanner_AD::set_search_mode(bool bSearchUntilFirstSolution)
{
    return m_params.return_first_solution = bSearchUntilFirstSolution;
}

void MHAPlanner_AD::set_initialsolution_eps(double eps)
{
    m_params.initial_eps = eps;
}

double MHAPlanner_AD::get_initial_eps()
{
    return m_params.initial_eps;
}

double MHAPlanner_AD::get_solution_eps() const
{
    return m_eps_satisfied;
}

double MHAPlanner_AD::get_final_epsilon()
{
    return m_eps_satisfied;
}

double MHAPlanner_AD::get_final_eps_planning_time()
{
    return sbpl::to_seconds(m_elapsed);
}

double MHAPlanner_AD::get_initial_eps_planning_time()
{
    return sbpl::to_seconds(m_elapsed);
}

int MHAPlanner_AD::get_n_expands() const
{
    return m_num_expansions;
}

int MHAPlanner_AD::get_n_expands_init_solution()
{
    return m_num_expansions;
}

void MHAPlanner_AD::get_search_stats(std::vector<PlannerStats>* s)
{
}

void MHAPlanner_AD::set_initial_mha_eps(double eps)
{
    m_initial_eps_mha = eps;
}

void MHAPlanner_AD::set_final_eps(double eps)
{
    m_params.final_eps = eps;
}

void MHAPlanner_AD::set_dec_eps(double eps)
{
    m_params.dec_eps = eps;
}

void MHAPlanner_AD::set_max_expansions(int expansion_count)
{
    m_max_expansions = expansion_count;
}

void MHAPlanner_AD::set_max_time(double max_time)
{
    m_params.max_time = max_time;
}

double MHAPlanner_AD::get_initial_mha_eps() const
{
    return m_initial_eps_mha;
}

double MHAPlanner_AD::get_final_eps() const
{
    return m_params.final_eps;
}

double MHAPlanner_AD::get_dec_eps() const
{
    return m_params.dec_eps;
}

int MHAPlanner_AD::get_max_expansions() const
{
    return m_max_expansions;
}

double MHAPlanner_AD::get_max_time() const
{
    return m_params.max_time;
}

bool MHAPlanner_AD::check_params(const ReplanParams& params)
{
    if (params.initial_eps < 1.0) {
        ROS_ERROR("Initial Epsilon must be greater than or equal to 1");
        return false;
    }

    if (params.final_eps > params.initial_eps) {
        ROS_ERROR("Final Epsilon must be less than or equal to initial epsilon %f %f", params.final_eps, params.initial_eps);
        return false;
    }

    if (params.dec_eps <= 0.0) {
        ROS_ERROR("Delta epsilon must be strictly positive");
        return false;
    }

    if (m_initial_eps_mha < 1.0) {
        ROS_ERROR("MHA Epsilon must be greater than or equal to 1");
        return false;
    }

    if (params.return_first_solution &&
        params.max_time <= 0.0 &&
        m_max_expansions <= 0)
    {
        ROS_ERROR("Max Time or Max Expansions must be positive");
        return false;
    }

    return true;
}

bool MHAPlanner_AD::time_limit_reached() const
{
    if (m_params.return_first_solution) {
        return false;
    }
    else if (m_params.max_time > 0.0 && sbpl::to_seconds(m_elapsed) >= m_params.max_time) {
        return true;
    }
    else if (m_max_expansions > 0 && m_num_expansions >= m_max_expansions) {
        return true;
    }
    else {
        return false;
    }
}

MHASearchState* MHAPlanner_AD::get_state(int state_id)
{
    if (m_graph_to_search_state.size() <= state_id) {
        m_graph_to_search_state.resize(state_id + 1, -1);
    }

    if (m_graph_to_search_state[state_id] == -1) {
        // map graph state to search state
        m_graph_to_search_state[state_id] = (int)m_search_states.size();

        // create new search state
        const size_t state_size =
                sizeof(MHASearchState) +
                sizeof(MHASearchState::HeapData) * (m_hcount);
        MHASearchState* s = (MHASearchState*)malloc(state_size);

        const size_t mha_state_idx = m_search_states.size();
        init_state(s, mha_state_idx, state_id);
        m_search_states.push_back(s);

        return s;
    }
    else {
        int ssidx = m_graph_to_search_state[state_id];
        return m_search_states[ssidx];
    }
}

void MHAPlanner_AD::clear()
{
    clear_open_lists();

    // free states
    for (size_t i = 0; i < m_search_states.size(); ++i) {
        // unmap graph to search state
        MHASearchState* search_state = m_search_states[i];
        const int state_id = m_search_states[i]->state_id;
        int* idxs = space_->StateID2IndexMapping[state_id];
        idxs[MHAMDP_STATEID2IND] = -1;

        // free search state
        free(m_search_states[i]);
    }

    // empty state table
    m_search_states.clear();

    m_start_state = NULL;
    m_goal_state = NULL;
}

void MHAPlanner_AD::init_state(
    MHASearchState* state,
    size_t mha_state_idx,
    int state_id)
{
    state->call_number = 0; // not initialized for any iteration
    state->state_id = state_id;
    state->closed_in_anc = false;
    state->closed_in_add = false;
    int dimID = space_->GetDimID(state_id);
    for (int i : m_heuristic_list[dimID]) {
    // for (int i = 0; i < num_heuristics(); i++) {
        state->od[i].open_state.heapindex = 0;
        state->od[i].h = compute_heuristic(state->state_id, i);
        // hijack list element pointers to map back to mha search state
        assert(sizeof(state->od[i].open_state.listelem) >= sizeof(struct listelement*));
        reinterpret_cast<size_t&>(state->od[i].open_state.listelem[0]) = mha_state_idx;
    }
}

void MHAPlanner_AD::reinit_state(MHASearchState* state)
{
    if (state->call_number != m_call_number) {
        state->call_number = m_call_number;
        state->g = INFINITECOST;
        state->bp = NULL;

        state->closed_in_anc = false;
        state->closed_in_add = false;

        int dimID = space_->GetDimID(state->state_id);
        for (int i : m_heuristic_list[dimID]) {
        // for (int i = 0; i < num_heuristics(); i++) {
            state->od[i].open_state.heapindex = 0;
            state->od[i].h = compute_heuristic(state->state_id, i);
        }
    }
}

void MHAPlanner_AD::reinit_search()
{
    clear_open_lists();
}

void MHAPlanner_AD::clear_open_lists()
{
    for (int i = 0; i < num_heuristics(); ++i) {
        m_open[i].makeemptyheap();
    }
}

long int MHAPlanner_AD::compute_key(MHASearchState* state, int hidx)
{
    return (long int)state->g + (long int)(m_eps * (long int)state->od[hidx].h);
}

void MHAPlanner_AD::expand(MHASearchState* state, int hidx)
{
    int dimID = space_->GetDimID(state->state_id);
    ROS_DEBUG_NAMED(SELOG, "Expanding state %d (dim = %d) in search %d { g = %d, h(0) = %d, h(%d) = %d, f = %ld }", state->state_id, dimID, hidx, state->g, state->od[0].h, hidx, state->od[hidx].h, compute_key(state, hidx));
    space_->expandingState(state->state_id);

    assert(!closed_in_add_search(state) || !closed_in_anc_search(state));

    if (hidx == 0) {
        state->closed_in_anc = true;
    }
    else {
        state->closed_in_add = true;
    }
    ++m_num_expansions;

    // // remove s from all open lists
    // for (int hidx = 0; hidx < num_heuristics(); ++hidx) {
    //     if (m_open[hidx].inheap(&state->od[hidx].open_state)) {
    //         m_open[hidx].deleteheap(&state->od[hidx].open_state);
    //     }
    // }

    // remove s from all open lists based on dimID
    for (int i : m_heuristic_list[dimID]){
    // for (int i = 0; i < num_heuristics(); ++i){
        if (m_open[i].inheap(&state->od[i].open_state)) {
            m_open[i].deleteheap(&state->od[i].open_state);
        }
    }

    std::vector<int> succ_ids;
    std::vector<int> costs;
    space_->GetSuccs(state->state_id, &succ_ids, &costs);
    assert(succ_ids.size() == costs.size());

    for (size_t sidx = 0; sidx < succ_ids.size(); ++sidx)  {
        const int cost = costs[sidx];
        MHASearchState* succ_state = get_state(succ_ids[sidx]);
        reinit_state(succ_state);

        ROS_DEBUG_NAMED(SSLOG, " Successor %d (dim = %d)", succ_state->state_id, space_->GetDimID(succ_ids[sidx]));

        int new_g = state->g + costs[sidx];
        if (new_g < succ_state->g) {
            succ_state->g = new_g;
            succ_state->bp = state;
            if (!closed_in_anc_search(succ_state)) {
                const long int fanchor = compute_key(succ_state, 0);
                insert_or_update(succ_state, 0, fanchor);
                ROS_DEBUG_NAMED(SSLOG, "  Update in search %d with f = %d + %0.3f * %d = %ld", 0, succ_state->g, m_eps, succ_state->od[0].h, fanchor);

                if (!closed_in_add_search(succ_state)) {
                    int dimID = space_->GetDimID(succ_state->state_id);
                    for (int hidx : m_heuristic_list[dimID]){
                    // for (int hidx = 0; hidx < num_heuristics(); ++hidx){
                        if(hidx == 0)
                            continue;
                        long int fn = compute_key(succ_state, hidx);
                        if (fn <= (long int)(m_eps_mha * fanchor)) {
                            insert_or_update(succ_state, hidx, fn);
                            ROS_DEBUG_NAMED(SSLOG, "  Update in search %d with f = %d + %0.3f * %d = %ld", hidx, succ_state->g, m_eps, succ_state->od[hidx].h, fn);
                        }
                        else {
                            ROS_DEBUG_NAMED(SSLOG, "  Skip update in search %d with f = %d + %0.3f * %d = %ld (> %0.3f * %ld = %ld)",
                                    hidx,
                                    succ_state->g, m_eps, succ_state->od[hidx].h, fn,
                                    m_eps_mha, fanchor, (long int)(m_eps * fanchor));
                        }
                    }
                }
            }
        }
    }

    assert(closed_in_any_search(state));
}

MHASearchState* MHAPlanner_AD::state_from_open_state(
    AbstractSearchState* open_state)
{
    const size_t ssidx = reinterpret_cast<size_t>(open_state->listelem[0]);
    return m_search_states[ssidx];
}

int MHAPlanner_AD::compute_heuristic(int state_id, int hidx)
{
    if (hidx == 0) {
        return m_hanchor->GetGoalHeuristic(state_id);
    }
    else {
        return m_heurs[hidx - 1]->GetGoalHeuristic(state_id);
    }
}

long int MHAPlanner_AD::get_minf(CHeap& pq) const
{
    return pq.getminkeyheap().key[0];
}

void MHAPlanner_AD::insert_or_update(MHASearchState* state, int hidx, int f)
{
    CKey new_key;
    new_key.key[0] = f;

    if (state->od[hidx].open_state.heapindex != 0) {
        m_open[hidx].updateheap(&state->od[hidx].open_state, new_key);
    }
    else {
        m_open[hidx].insertheap(&state->od[hidx].open_state, new_key);
    }
}

void MHAPlanner_AD::extract_path(std::vector<int>* solution_path, int* solcost)
{
    ROS_DEBUG_NAMED(SLOG, "Extracting path");
    solution_path->clear();
    *solcost = 0;
    for (MHASearchState* state = m_goal_state; state; state = state->bp) {
        solution_path->push_back(state->state_id);
        if (state->bp) {
            *solcost += (state->g - state->bp->g);
        }
    }

    // TODO: special cases for backward search
    std::reverse(solution_path->begin(), solution_path->end());
}

void MHAPlanner_AD::extract_partial_path(
    std::vector<int>* solution_path,
    int* solcost,
    MHASearchState* best_seen_state)
{
    ROS_DEBUG_NAMED(SLOG, "Extracting path");
    solution_path->clear();
    *solcost = 0;
    for (MHASearchState* state = best_seen_state; state; state = state->bp) {
        solution_path->push_back(state->state_id);
        if (state->bp) {
            *solcost += (state->g - state->bp->g);
        }
    }

    // TODO: special cases for backward search
    std::reverse(solution_path->begin(), solution_path->end());
}

bool MHAPlanner_AD::closed_in_anc_search(MHASearchState* state) const
{
    return state->closed_in_anc;
}

bool MHAPlanner_AD::closed_in_add_search(MHASearchState* state) const
{
    return state->closed_in_add;
}

bool MHAPlanner_AD::closed_in_any_search(MHASearchState* state) const
{
    return state->closed_in_anc || state->closed_in_add;
}

} // namespace adim
