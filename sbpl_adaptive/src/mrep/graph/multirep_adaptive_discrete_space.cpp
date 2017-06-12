#include <sbpl_adaptive/mrep/graph/multirep_adaptive_discrete_space.h>

namespace adim {

static const char *GLOG = "mrep";

/// \class MultiRepAdaptiveDiscreteSpace
///
/// An implementation of a discrete space composed of multiple state space
/// representations.
///
/// This class maintains the set of generated representation-aware states and
/// dispatches calls to retrieve the successor/predecessor states of a given
/// state to the appropriate representation.
///
/// Transitions between states of the same representations are maintained by the
/// the representations themselves. Transitions between states of differing
/// representations are managed by additional Projection instances linked to
/// this combined space.
///
/// One state, from any representation, may be specified as the start state.
/// Multiple states may be specified as being goal states, via an abstract goal
/// condition. The abstract goal condition is implemented as a single state,
/// with no associated repressentation, whose ID can be queried through
/// GetGoalStateID(). The goal state should never be expanded by the search
/// routine.
///
/// The set of representations may be arranged to form an abstraction hierarchy,
/// such that less abstract representations are parents of more abstract
/// representations. For two representations (r1, r2) where r1 is a parent (less
/// abstract) representation to r2, states from r1 may project to a single state
/// in r2, and states from r2 should project to a set of states in r1. This
/// relationship forms a many-to-(one-or-none) mapping from r1 to r2 and a 1-to-
/// many mapping from r2 to r1.
///
/// For successful operation, one representation must be chosen as the least
/// abstract representation, which acts as the most ancient descendant of any
/// class in the hierarchy. Every state should be able to be recursively
/// projected to a set of states in this least abstract representation. This
/// representation is typically the representation containing states with the
/// highest dimensionality, and is referred to as the full-dimensional
/// representation.
///
/// This class manages states at the abstract level, but is not aware of the
/// concrete implementation of a state for any of its representations. For this
/// reason, representations are expected, in most cases, to downcast states to
/// its internal state representation type.
///
/// Similarly, subclasses of this class and subclasses of
/// AdaptiveStateRepresentation may need to downcast to explicit instances of
/// AdaptiveStateRepresentation. They will also likely need to be aware of the
/// concrete implementation of AbstractGoal used to specify the goal condition.
/// Convenience functions are provided to cleanly perform this (unsafe)
/// downcasting.
///
/// This class implements the AdaptiveDiscreteSpace interface so that it may be
/// used in the Planning with Adaptive Dimensionality framework. It is aware of
/// the current planning mode (plan mode vs. track mode), and calls the
/// appropriate methods on each of its associated representations to return the
/// correct successors for the given mode. This class makes no attempt to
/// dictate what successors are available in which each representation for a
/// given planning mode/iteration. This state must be implemented by a subclass
/// of this class and shared between concrete AdaptiveStateRepresentation
/// instances.

/// Constructor
MultiRepAdaptiveDiscreteSpace::MultiRepAdaptiveDiscreteSpace() :
    full_representation_(),
    representations_(),
    goal_(nullptr),
    goal_hash_entry_(nullptr),
    start_hash_entry_(nullptr),
    hash_table_size_(32 * 1024),
    hash_tables_(),
    state_id_to_hash_entry_(),
    proj_matrix_()
{
}

/// Destructor
MultiRepAdaptiveDiscreteSpace::~MultiRepAdaptiveDiscreteSpace()
{
    for (size_t i = 0; i < state_id_to_hash_entry_.size(); i++) {
        adim::AdaptiveHashEntry *entry = state_id_to_hash_entry_[i];
        if (IsValidRepID(entry->dimID)) { // skip the goal
            // tell the representation to delete its state data (the void *)
            representations_[entry->dimID]->deleteStateData(entry->stateID);
        }
        delete entry;
        state_id_to_hash_entry_[i] = nullptr;
    }
}

/// Insert a new state into the state table and assigns a state id to the
/// inserted state. The state should not currently exist in the state table as
/// this function will not check for prior existence and may insert duplicate
/// entries. The dimID of the state should correctly initialized to refer to
/// an existing representation registered with this multi-representation space.
///
/// \param The state to be inserted
/// \param The hash value of the state to be inserted
/// \return The id assigned to the state, or -1 if the state's dimID is invalid
int MultiRepAdaptiveDiscreteSpace::InsertHashEntry(
    AdaptiveHashEntry *entry,
    size_t binID)
{
    if (!IsValidRepID(entry->dimID)) { // TODO(Andrew): assertion material
        ROS_ERROR_NAMED(GLOG, "dimID %d does not have a hash table!", entry->dimID);
        return -1;
    }

    binID &= (hash_table_size_ - 1);

    // get corresponding state ID
    entry->stateID = state_id_to_hash_entry_.size();

    // insert into list of states
    state_id_to_hash_entry_.push_back(entry);

    // insert into hash table in corresponding bin
    hash_tables_[entry->dimID][binID].push_back(entry);

    // make room to map and insert planner data
    int *planner_data = new int[NUMOFINDICES_STATEID2IND];
    std::fill(planner_data, planner_data + NUMOFINDICES_STATEID2IND, -1);
    StateID2IndexMapping.push_back(planner_data);

    return entry->stateID;
}

/// Lookup a state by its id.
///
/// \param stateID The id of a previously inserted state
/// \return A pointer to the referenced state, or nullptr if the id is invalid
AdaptiveHashEntry *MultiRepAdaptiveDiscreteSpace::GetState(int stateID) const
{
    if (!IsValidStateID(stateID)) { // TODO(Andrew): assertion material
        ROS_ERROR_NAMED(GLOG, "stateID [%d] out of range [0,%zu)", stateID, state_id_to_hash_entry_.size());
        return nullptr;
    }
    return state_id_to_hash_entry_[stateID];
}

/// Lookup the representation of a state by its id.
///
/// \param stateID The id of a previously inserted state.
/// \return The id of the state's associated representation.
int MultiRepAdaptiveDiscreteSpace::GetDimID(int stateID)
{
    if (!IsValidStateID(stateID)) { // TODO(Andrew): assertion material
        ROS_ERROR_NAMED(GLOG, "stateID [%d] out of range [0,%zu)", stateID, state_id_to_hash_entry_.size());
        return -1;
    }
    return state_id_to_hash_entry_[stateID]->dimID;
}

/// Register a new representation and mark it as the full-dimensional
/// representation.
///
/// \param The full-dimensional representation to be registered.
/// \return true if registration was successful; see RegisterRepresentation for
///     details on successful registration.
bool MultiRepAdaptiveDiscreteSpace::RegisterFullDRepresentation(
    const AdaptiveStateRepresentationPtr &rep)
{
    bool res = RegisterRepresentation(rep);
    if (res) {
        full_representation_ = rep;
    }
    return res;
}

/// Register a new representation. If successfully registered, the
/// representation will be assigned a unique id to refer to it. Registration
/// will fail if the space already has a registered representation with the same
/// id.
///
/// \param rep The representation to be registered
/// \return true if registration was successful
bool MultiRepAdaptiveDiscreteSpace::RegisterRepresentation(
    const AdaptiveStateRepresentationPtr &rep)
{
    for (size_t i = 0; i < representations_.size(); i++) {
        if (representations_[i]->getID() == rep->getID()) {
            ROS_ERROR_NAMED(GLOG, "Failed to register new representation (%s) with ID (%d) -- duplicate ID registered already", rep->getName().c_str(), rep->getID());
            return false;
        }
    }

    rep->setID(representations_.size());
    representations_.push_back(rep);

    // create a new hash table
    std::vector<std::vector<AdaptiveHashEntry*>> hash_table(hash_table_size_ + 1);
    hash_tables_.push_back(std::move(hash_table));

    // update the projection matrix
    std::vector<ProjectionPtr> new_proj_matrix;
    new_proj_matrix.resize(representations_.size() * representations_.size());

    // copy over projection matrix
    int prev_rep_count = NumRepresentations() - 1;
    ROS_INFO_NAMED(GLOG, "Resize projection matrix to %d^2 -> %d^2", prev_rep_count, NumRepresentations());
    for (int i = 0; i < prev_rep_count; ++i) {
        for (int j = 0; j < prev_rep_count; ++j) {
            int prev_rep_idx = i * prev_rep_count + j;
            int curr_rep_idx = i * NumRepresentations() + j;
            new_proj_matrix[curr_rep_idx] = proj_matrix_[prev_rep_idx];
        }
    }
    proj_matrix_ = std::move(new_proj_matrix);

    ROS_INFO_NAMED(GLOG, "Registered representation %d '%s'", rep->getID(), rep->getName().c_str());
    return true;
}

/// Register a new projection between representations. If successfully
/// registered, the projection will be linked to this space. Registration will
/// fail if the source or target representation of the projection is not yet
/// registered to this space.
///
/// \param proj The projection to be registered
/// \return true if registration was successful
bool MultiRepAdaptiveDiscreteSpace::RegisterProjection(
    const ProjectionPtr &proj)
{
    // TODO(Andrew): assertion material?
    if (!IsValidRepID(proj->sourceRepID()) ||
        !IsValidRepID(proj->targetRepID()))
    {
        return false;
    }

    int proj_idx = proj->sourceRepID() * NumRepresentations() + proj->targetRepID();
    proj_matrix_[proj_idx] = proj;
    proj->setPlanningSpace(this);
    return true;
}

/// Set the start state from a discrete state.
///
/// In the event of a failure, the start state is not set and -1 is returned as
/// the state id. Failure may occur if the representation id is invalid or if
/// the representation could not set the start state.
///
/// \param dimID The id of the start state representation
/// \param state The state to use as the start state
/// \return The id of the start state or -1 in the event of failure
int MultiRepAdaptiveDiscreteSpace::SetStartCoords(
    int dimID,
    const AdaptiveState *state)
{
    ROS_INFO_NAMED(GLOG, "Set start coordinates");

    if (!IsValidRepID(dimID)) { // TODO(Andrew): assertion material
        return -1;
    }

    int start_id = representations_[dimID]->SetStartCoords(state);
    if (start_id == -1) {
        return -1;
    }

    start_hash_entry_ = GetState(start_id);
    ROS_INFO_NAMED(GLOG, "start set %d", start_id);
    return start_id;
}

/// Set the start state from a continuous state.
///
/// In the event of a failure, the start state is not set and -1 is returned as
/// the state id. Failure may occur if the representation id is invalid or if
/// the representation could not set the start state.
///
/// \param dimID The id of the start state representation
/// \param state The state to use as the start state
/// \return The id of the start state or -1 in the event of failure
int MultiRepAdaptiveDiscreteSpace::SetStartConfig(
    int dimID,
    const ModelCoords *coords)
{
    ROS_INFO_NAMED(GLOG, "Set start configuration");

    if (!IsValidRepID(dimID)) { // TODO(Andrew): assertion material
        return -1;
    }

    int start_id = representations_[dimID]->SetStartConfig(coords);
    if (start_id == -1) {
        return -1;
    }

    start_hash_entry_ = GetState(start_id);
    ROS_INFO_NAMED(GLOG, "start set %d", start_id);
    return start_id;
}

/// Set the goal state from a discrete state.
///
/// In the event of a failure, the goal state is not set and -1 is returned as
/// the state id. Failure may occur if the representation id is invalid or if
/// the representation could not set the goal state.
///
/// \param dimID The id of the goal state representation
/// \param state The state to use as the goal state
/// \return The id of the goal state or -1 in the event of failure
int MultiRepAdaptiveDiscreteSpace::SetGoalCoords(
    int dimID,
    const AdaptiveState *state)
{
    ROS_INFO_NAMED(GLOG, "Set goal coordinates");

    if (!IsValidRepID(dimID)) { // TODO(Andrew): assertion material
        return -1;
    }

    int goal_id = representations_[dimID]->SetGoalCoords(state);
    if (goal_id == -1) {
        return -1;
    }

    goal_hash_entry_ = GetState(goal_id);
    ROS_INFO_NAMED(GLOG, "start set %d", goal_id);
    return goal_id;
}

/// Set the goal state from a continuous state.
///
/// In the event of a failure, the goal state is not set and -1 is returned as
/// the state id. Failure may occur if the representation id is invalid or if
/// the representation could not set the goal state.
///
/// \param dimID The id of the goal state representation
/// \param state The state to use as the goal state
/// \return The id of the goal state or -1 in the event of failure
int MultiRepAdaptiveDiscreteSpace::SetGoalConfig(
    int dimID,
    const ModelCoords *coords)
{
    ROS_INFO_NAMED(GLOG, "Set goal configuration");

    if (!IsValidRepID(dimID)) { // TODO(Andrew): assertion material
        return -1;
    }

    int GoalID = representations_[dimID]->SetGoalConfig(coords);
    if (GoalID == -1) {
        return -1;
    }

    goal_hash_entry_ = GetState(GoalID);
    ROS_INFO_NAMED(GLOG, "start set %d", GoalID);
    return GoalID;
}

int MultiRepAdaptiveDiscreteSpace::GetStartStateID() const
{
    return start_hash_entry_ ? start_hash_entry_->stateID : -1;
}

int MultiRepAdaptiveDiscreteSpace::GetGoalStateID() const
{
    return goal_hash_entry_ ? goal_hash_entry_->stateID : -1;
}

/// Set an abstract goal condition.
///
/// The abstract goal is represented as a discrete state, with a valid state id,
/// but belonging to no representation (its representation id is -1) and
/// containing no state data.
///
/// \param The abstract goal condition
/// \return The id of the abstract goal state
int MultiRepAdaptiveDiscreteSpace::SetAbstractGoal(AbstractGoal *goal)
{
    ROS_INFO_NAMED(GLOG, "Setting abstract goal...");
    goal_ = goal;

    // create a fake metagoal
    // TODO(Andrew): probably shouldn't do this multiple times?
    AdaptiveHashEntry *entry = new AdaptiveHashEntry;
    entry->dimID = -1;
    entry->stateData = nullptr;
    InsertMetaGoalHashEntry(entry);
    goal_hash_entry_ = entry;
    ROS_INFO_NAMED(GLOG, "Metagoal ID: %d --> %d", entry->stateID, entry->dimID);
    return entry->stateID;
}

/// Project a state to a different representation. The semantics depend on the
/// relationship between the two representations:
///
/// For projections from a low-dimensional representation to the high-
/// dimensional representation, this should return the states from the higher-
/// dimensional representation whose projections to the lower-dimensional
/// representation map back to this state.
///
/// For projections from the high-dimensional representation to a lower-
/// dimensional representation, this should return either a single state in the
/// lower-dimensional representation or no states at all.
///
/// For projections between representations not immediately associated, this
/// should return a subset of states in the target representation that are
/// reachable via a projection from the source state to the high-dimensional
/// representation, followed by a path through the high-dimensional
/// representation, and eventually a down-projection to the target
/// representation.
bool MultiRepAdaptiveDiscreteSpace::Project(
    const AdaptiveState *state,
    int fromID,
    int toID,
    std::vector<int> &proj_state_ids,
    std::vector<int> &proj_costs,
    int adPathIdx)
{
    if (!IsValidRepID(toID)) { // TODO(Andrew): assertion material
        ROS_WARN_NAMED(GLOG, "Dimensionality ID %d is out of bounds!", toID);
        return false;
    }

    if (!IsValidRepID(fromID)) { // TODO(Andrew): assertion material
        ROS_WARN_NAMED(GLOG, "Dimensionality ID %d is out of bounds!", fromID);
        return false;
    }

    int proj_idx = GetProjectionIndex(fromID, toID);
    if (proj_matrix_[proj_idx]) {
        proj_matrix_[proj_idx]->project(state, proj_state_ids, proj_costs, adPathIdx);
        return true;
    }
    else {
        return false;
    }
}

/// Test whether a projection transition is executable.
bool MultiRepAdaptiveDiscreteSpace::IsProjectionExecutable(
    int fromID,
    int toID) const
{
    int proj_idx = GetProjectionIndex(fromID, toID);
    if (!proj_matrix_[proj_idx]) {
        return false;
    }
    else {
        return proj_matrix_[proj_idx]->executable();
    }
}

/// Test whether a path is executable, i.e., an executable transition exists
/// between all consecutive state pairs.
///
/// An error will be encountered in the following cases:
/// * Any of the state ids in the path do not refer to an existing state in the
///   state table.
/// * Any of the states, other than the final state, listed in the path do not
///   belong to a registered representation
///
/// \param stateIDV A path of states represented by their ids
/// \return true if the path is executable, false otherwise or as result of an
///     error.
bool MultiRepAdaptiveDiscreteSpace::isExecutablePath(
    const std::vector<int> &stateIDV)
{
    if (stateIDV.size() < 2) {
        ROS_WARN_NAMED(GLOG, "Path is trivially executable");
        return true;
    }

    AdaptiveHashEntry *prev_entry = GetState(stateIDV.front());

    if (!prev_entry) {
        ROS_ERROR_NAMED(GLOG, "No hash entry for state id %d", stateIDV.front());
        return false;
    }

    if (prev_entry->dimID < 0 ||
        prev_entry->dimID >= (int)representations_.size())
    {
        ROS_ERROR_NAMED(GLOG, "Invalid representation %d for state %d", prev_entry->dimID, stateIDV.front());
        return false;
    }

    for (size_t i = 1; i < stateIDV.size(); ++i) {
        int prev_id = stateIDV[i - 1];
        int curr_id = stateIDV[i];

        AdaptiveHashEntry *curr_entry = GetState(curr_id);
        if (!curr_entry) {
            ROS_ERROR_NAMED(GLOG, "No hash entry for state id %d", curr_id);
            return false;
        }

        if (curr_entry->dimID == -1) {
            ROS_INFO_NAMED(GLOG, "State %d is the metagoal", curr_id);
            continue;
        }

        if (curr_entry->dimID < 0 ||
            curr_entry->dimID >= (int)representations_.size())
        {
            ROS_ERROR_NAMED(GLOG, "Invalid representation %d for state %d", curr_entry->dimID, curr_id);
            return false;
        }

        if (curr_entry->dimID == prev_entry->dimID) {
            const auto &rep = representations_[curr_entry->dimID];
            if (!rep->IsExecutableAction(prev_id, curr_id)) {
                return false;
            }
        }
        else {
            if (IsProjectionExecutable(prev_entry->dimID, curr_entry->dimID)) {
                return false;
            }
        }

        prev_entry = curr_entry;
    }

    return true;
}

void MultiRepAdaptiveDiscreteSpace::GetSuccs_Plan(
    int state_id,
    std::vector<int> *succs,
    std::vector<int> *costs)
{
    succs->clear();
    costs->clear();
    AdaptiveHashEntry *entry = GetState(state_id);
    representations_[entry->dimID]->GetSuccs(state_id, succs, costs);
}

void MultiRepAdaptiveDiscreteSpace::GetSuccs_Track(
    int state_id,
    std::vector<int> *succs,
    std::vector<int> *costs)
{
    succs->clear();
    costs->clear();
    AdaptiveHashEntry *entry = GetState(state_id);
    representations_[entry->dimID]->GetTrackSuccs(state_id, succs, costs);
}

void MultiRepAdaptiveDiscreteSpace::GetPreds_Plan(
    int state_id,
    std::vector<int> *preds,
    std::vector<int> *costs)
{
    preds->clear();
    costs->clear();
    AdaptiveHashEntry *entry = GetState(state_id);
    representations_[entry->dimID]->GetSuccs(state_id, preds, costs);
}

void MultiRepAdaptiveDiscreteSpace::GetPreds_Track(
    int state_id,
    std::vector<int> *preds,
    std::vector<int> *costs)
{
    preds->clear();
    costs->clear();
    AdaptiveHashEntry *entry = GetState(state_id);
    representations_[entry->dimID]->GetPreds(state_id, preds, costs);
}

void MultiRepAdaptiveDiscreteSpace::GetSuccs_Plan(
    int SourceStateID,
    int expansion_step,
    std::vector<int> *SuccIDV,
    std::vector<int> *costs)
{
    ROS_ERROR_NAMED(GLOG, "GetSuccs_Plan with exp_step not implemented. Defaulting");
    GetSuccs_Plan(SourceStateID, SuccIDV, costs);
}

void MultiRepAdaptiveDiscreteSpace::GetSuccs_Track(
    int state_id,
    int expansion_step,
    std::vector<int> *succs,
    std::vector<int> *costs)
{
    // TODO(Andrew): once
    ROS_ERROR_NAMED(GLOG, "GetSuccs_Track with exp_step not implemented. Defaulting");
    GetSuccs_Track(state_id, succs, costs);
}

void MultiRepAdaptiveDiscreteSpace::GetPreds_Plan(
    int state_id,
    int expansion_step,
    std::vector<int> *preds,
    std::vector<int> *costs)
{
    ROS_ERROR_NAMED(GLOG, "GetPreds_Plan with exp_step not implemented. Defaulting");
    GetPreds_Plan(state_id, preds, costs);
}

void MultiRepAdaptiveDiscreteSpace::GetPreds_Track(
    int state_id,
    int expansion_step,
    std::vector<int> *preds,
    std::vector<int> *costs)
{
    ROS_ERROR_NAMED(GLOG, "GetPreds_Track with exp_step not implemented. Defaulting");
    GetPreds_Track(state_id, preds, costs);
}

int MultiRepAdaptiveDiscreteSpace::InsertMetaGoalHashEntry(
    AdaptiveHashEntry *entry)
{
    entry->stateID = state_id_to_hash_entry_.size(); // assign state id
    state_id_to_hash_entry_.push_back(entry); // insert into state table

    // initialize mapping from search state to graph state
    int *planner_data = new int[NUMOFINDICES_STATEID2IND];
    std::fill(planner_data, planner_data + NUMOFINDICES_STATEID2IND, -1);
    StateID2IndexMapping.push_back(planner_data);

    goal_hash_entry_ = entry;
    return entry->stateID;
}

bool MultiRepAdaptiveDiscreteSpace::IsValidStateID(int stateID) const
{
    return stateID >= 0 && stateID < (int)state_id_to_hash_entry_.size();
}

bool MultiRepAdaptiveDiscreteSpace::IsValidRepID(int dimID) const
{
    return dimID >= 0 && dimID < representations_.size();
}

int MultiRepAdaptiveDiscreteSpace::GetProjectionIndex(int srep, int trep) const
{
    return srep * NumRepresentations() + trep;
}

} // namespace adim
