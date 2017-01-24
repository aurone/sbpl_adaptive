/*
 * multirep_adaptive_environment.cpp
 *
 *  Created on: Mar 15, 2016
 *      Author: kalin
 */

#include <sbpl_adaptive/headers.h>

namespace adim {

static const char *GLOG = "mrep";
static const char *GPLOG = "mrep.projection";

/// \brief Return the HD states up-projected from a LD state
bool MultiRepAdaptiveDiscreteSpaceInformation::ProjectToFullD(
    const void *local_state_data,
    int fromID,
    std::vector<int> &proj_stateIDs,
    int adPathIdx)
{
    if (fromID >= representations_.size()) {
        ROS_ERROR("AdaptiveEnvironment_t::ProjectToFullD - Dimensionality ID %d is out of bounds!", fromID);
        throw SBPL_Exception();
        return false;
    }
    return representations_[fromID]->ProjectToFullD(local_state_data, proj_stateIDs, adPathIdx);
}

/// \brief Project a state to a set of states in another dimension
bool MultiRepAdaptiveDiscreteSpaceInformation::Project(
    const void *state_data,
    int fromID,
    int toID,
    std::vector<int> &proj_stateIDs,
    int adPathIdx)
{
    if (toID >= (int)representations_.size() || toID < 0) {
        ROS_WARN("AdaptiveEnvironment_t::Project - Dimensionality ID %d is out of bounds!", toID);
        return false;
    }
    if (fromID >= (int)representations_.size() || fromID < 0) {
        ROS_WARN("AdaptiveEnvironment_t::Project - Dimensionality ID %d is out of bounds!", toID);
        return false;
    }

//    if (isInTrackingMode() && !representations_[toID]->isExecutable()) {
//        ROS_WARN("Can't project to non-executable type in tracking mode!");
//        return false; //no projections to non-executable types in trackmode
//    }

    // optimization when toID is fullD: directly up-project to HD
    if (toID == fulld_representation_->getID()) {
        //fromID understands state_data
        bool bRes = representations_[fromID]->ProjectToFullD(state_data, proj_stateIDs, adPathIdx);
        ROS_DEBUG_NAMED(GPLOG, "Got %lu projections when projecting from [%s] to [%s]", proj_stateIDs.size(), representations_[fromID]->getDescription().c_str(), representations_[toID]->getDescription().c_str());
        if (!bRes) {
            ROS_ERROR("Failed to project from [%s] to [%s]", representations_[fromID]->getDescription().c_str(), representations_[toID]->getDescription().c_str());
        }
        return bRes;
    }

    // optimization when fromID is fullD: directly down-project to LD
    if (fromID == fulld_representation_->getID()) {
        //state_data is hd toID understands it
        bool bRes = representations_[toID]->ProjectFromFullD(state_data, proj_stateIDs, adPathIdx);
        ROS_DEBUG_NAMED(GPLOG, "Got %lu projections when projecting from [%s] to [%s]", proj_stateIDs.size(), representations_[fromID]->getDescription().c_str(), representations_[toID]->getDescription().c_str());
        if (!bRes) {
            ROS_ERROR("Failed to project from [%s] to [%s]", representations_[fromID]->getDescription().c_str(), representations_[toID]->getDescription().c_str());
        }
        return bRes;
    }

    // project state to HD and project the hd-projections to a different LD
    std::vector<int> hd_proj_stateIDs;
    ROS_DEBUG_NAMED(GPLOG, "Projecting %d [%s] to FullD first! (adPathIdx=%d)", fromID, representations_[fromID]->getDescription().c_str(), adPathIdx);
    if (!ProjectToFullD(state_data, fromID, hd_proj_stateIDs, adPathIdx)) {
        ROS_ERROR("Failed to project state data from representation %d [%s] to fullD representation", fromID, representations_[fromID]->getDescription().c_str());
        return false;
    }
    ROS_DEBUG_NAMED(GPLOG, "Got %lu FullD projections", hd_proj_stateIDs.size());
    ROS_DEBUG_NAMED(GPLOG, "Now projecting to %d [%s] (adPathIdx=%d)", toID, representations_[toID]->getDescription().c_str(), adPathIdx);
    for (int hd_stateID : hd_proj_stateIDs) {
        AdaptiveHashEntry *entry = GetState(hd_stateID);
        if (entry != NULL) {
            if (!representations_[toID]->ProjectFromFullD(entry->stateData, proj_stateIDs, adPathIdx)) {
                ROS_ERROR("Failed to project HD state data to representation %d [%s]", toID, representations_[toID]->getDescription().c_str());
                return false;
            }
        }
        else {
            ROS_ERROR("Hmm... something is wrong here!");
            ROS_ERROR("Could not get hash entry for HD state stateID %d", (int)hd_stateID);
            pause();
            return false;
        }
    }
    ROS_DEBUG_NAMED(GPLOG, "Got %lu projections when projecting from [%s] to [%s]", proj_stateIDs.size(), representations_[fromID]->getDescription().c_str(), representations_[toID]->getDescription().c_str());
    return true;
}

int MultiRepAdaptiveDiscreteSpaceInformation::SetGoalCoords(
    int dimID,
    const void *representation_specific_disc_data)
{
    ROS_INFO("[AdaptiveEnvironment_t] setting goal coordinates");
    int GoalID = representations_[dimID]->SetGoalCoords(representation_specific_disc_data);
    if (!representations_[dimID]->isExecutable()) {
        ROS_WARN("[AdaptiveEnvironment_t] the start representation is of non-executable type!");
    }
    if (GoalID == -1) {
        return -1;
    }
    this->data_.goalHashEntry = GetState(GoalID);
    ROS_INFO("[AdaptiveEnvironment_t] start set %d", GoalID);
    return GoalID;
}

int MultiRepAdaptiveDiscreteSpaceInformation::SetGoalConfig(
    int dimID,
    const void *representation_specific_cont_data)
{
    ROS_INFO("[AdaptiveEnvironment_t] setting goal configuration");
    int GoalID = representations_[dimID]->SetGoalConfig(
            representation_specific_cont_data);
    if (!representations_[dimID]->isExecutable()) {
        ROS_WARN("[AdaptiveEnvironment_t] the start representation is of non-executable type!");
    }
    if (GoalID == -1) {
        return -1;
    }
    this->data_.goalHashEntry = GetState(GoalID);
    ROS_INFO("[AdaptiveEnvironment_t] start set %d", GoalID);
    return GoalID;
}

int MultiRepAdaptiveDiscreteSpaceInformation::SetStartCoords(
    int dimID,
    const void *representation_specific_disc_data)
{
    ROS_INFO("[AdaptiveEnvironment_t] setting start coordinates");
    int StartID = representations_[dimID]->SetStartCoords(
            representation_specific_disc_data);
    if (!representations_[dimID]->isExecutable()) {
        ROS_WARN("[AdaptiveEnvironment_t] the start representation is of non-executable type!");
    }
    if (StartID == -1) {
        return -1;
    }
    this->data_.startHashEntry = GetState(StartID);
    ROS_INFO("[AdaptiveEnvironment_t] start set %d", StartID);
    return StartID;
}

int MultiRepAdaptiveDiscreteSpaceInformation::SetStartConfig(
    int dimID,
    const void *representation_specific_cont_data)
{
    ROS_INFO("[AdaptiveEnvironment_t] setting start configuration");
    int StartID = representations_[dimID]->SetStartConfig(
            representation_specific_cont_data);
    if (!representations_[dimID]->isExecutable()) {
        ROS_WARN("[AdaptiveEnvironment_t] the start representation is of non-executable type!");
    }
    if (StartID == -1) {
        return -1;
    }
    this->data_.startHashEntry = GetState(StartID);
    ROS_INFO("[AdaptiveEnvironment_t] start set %d", StartID);
    return StartID;
}

void MultiRepAdaptiveDiscreteSpaceInformation::InsertMetaGoalHashEntry(
    AdaptiveHashEntry *entry)
{
    int i;
    /* get corresponding state ID */
    entry->stateID = data_.StateID2HashEntry.size();
    /* insert into list of states */
    data_.StateID2HashEntry.push_back(entry);
    /* insert into hash table in corresponding bin */
    //don't insert it into hash table because it does not fit any dimensionality ID -- only way to get to it is by ID or by data_.goalHashEntry ptr
    /* check if everything ok */
    if (entry->stateID != StateID2IndexMapping.size()) {
        ROS_ERROR("ERROR in AdaptiveEnvironment_t::InsertHashEntry function: last state has incorrect stateID");
        throw SBPL_Exception();
    }
    /* make room and insert planner data */
    int *planner_data = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(planner_data);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[entry->stateID][i] = -1;
    }
    data_.goalHashEntry = entry;
    return;
}

int MultiRepAdaptiveDiscreteSpaceInformation::SetAbstractGoal(
    AbstractGoal *goal)
{
    ROS_INFO("Setting abstract goal...");
    data_.goaldata = goal;
    // create a fake metagoal
    AdaptiveHashEntry *entry = new AdaptiveHashEntry;
    entry->dimID = -1;
    entry->stateData = NULL;
    InsertMetaGoalHashEntry(entry);
    data_.goalHashEntry = entry;
    ROS_INFO("Metagoal ID: %zu --> %d", entry->stateID, entry->dimID);
    return entry->stateID;
}

MultiRepAdaptiveDiscreteSpaceInformation::MultiRepAdaptiveDiscreteSpaceInformation()
{
    data_.HashTableSize = 32 * 1024; //should be power of two
    data_.StateID2HashEntry.clear();
    data_.goalHashEntry = NULL;
    data_.startHashEntry = NULL;
    env_data_.reset();
    BestTracked_StateID = -1;
    BestTracked_Cost = INFINITECOST;
}

MultiRepAdaptiveDiscreteSpaceInformation::~MultiRepAdaptiveDiscreteSpaceInformation()
{
    for (size_t i = 0; i < data_.StateID2HashEntry.size(); i++) {
        adim::AdaptiveHashEntry *entry = data_.StateID2HashEntry[i];
        representations_[entry->dimID]->deleteStateData(entry->stateID); //tell the representation to delete its state data (the void*)
        delete entry;
        data_.StateID2HashEntry[i] = NULL;
    }
    data_.StateID2HashEntry.clear();
    data_.HashTables.clear();
}

bool MultiRepAdaptiveDiscreteSpaceInformation::RegisterFullDRepresentation(
    const AdaptiveStateRepresentationPtr &rep)
{
    fulld_representation_ = rep;
    return RegisterRepresentation(rep);
}

bool MultiRepAdaptiveDiscreteSpaceInformation::RegisterRepresentation(
    const AdaptiveStateRepresentationPtr &rep)
{
    for (int i = 0; i < representations_.size(); i++) {
        if (representations_[i]->getID() == rep->getID()) {
            ROS_ERROR("Failed to register new representation (%s) with ID (%d) -- duplicate ID registered already", rep->getDescription().c_str(), rep->getID());
            ROS_ERROR("Duplicate (%d)(%s)", representations_[i]->getID(), representations_[i]->getDescription().c_str());
            return false;
        }
    }

    rep->setID(representations_.size());
    representations_.push_back(rep);

    // make room in hash table
    std::vector<std::vector<AdaptiveHashEntry*>> HashTable(data_.HashTableSize + 1);

    data_.HashTables.push_back(HashTable);

    ROS_INFO("Registered representation %d '%s'", rep->getID(), rep->getDescription().c_str());

    return true;
}

size_t MultiRepAdaptiveDiscreteSpaceInformation::InsertHashEntry(
    AdaptiveHashEntry *entry,
    size_t binID)
{
    int i;

    /* get corresponding state ID */
    entry->stateID = data_.StateID2HashEntry.size();

    /* insert into list of states */
    data_.StateID2HashEntry.push_back(entry);

    /* insert into hash table in corresponding bin */

    if (entry->dimID >= data_.HashTables.size()) {
        ROS_ERROR("ERROR in AdaptiveEnvironment_t::InsertHashEntry function: dimID %d does not have a hash table!", entry->dimID);
        throw SBPL_Exception();
    }

    data_.HashTables[entry->dimID][binID].push_back(entry);

    /* check if everything ok */
    if (entry->stateID != StateID2IndexMapping.size()) {
        ROS_ERROR("ERROR in AdaptiveEnvironment_t::InsertHashEntry function: last state has incorrect stateID");
        throw SBPL_Exception();
    }

    /* make room and insert planner data */
    int *planner_data = new int[NUMOFINDICES_STATEID2IND];
    StateID2IndexMapping.push_back(planner_data);
    for (i = 0; i < NUMOFINDICES_STATEID2IND; i++) {
        StateID2IndexMapping[entry->stateID][i] = -1;
    }

    return entry->stateID;
}

AdaptiveHashEntry *MultiRepAdaptiveDiscreteSpaceInformation::GetState(
    size_t stateID)
{
    if (stateID >= data_.StateID2HashEntry.size()) {
        ROS_ERROR("stateID [%zu] out of range [%zu]", stateID, data_.StateID2HashEntry.size());
        throw SBPL_Exception();
    }
    return data_.StateID2HashEntry[stateID];
}

bool MultiRepAdaptiveDiscreteSpaceInformation::isExecutablePath(
    const std::vector<int> &stateIDV)
{
    if (stateIDV.size() < 2) {
        ROS_WARN("Path is trivially executable");
        return true;
    }

    AdaptiveHashEntry *prev_entry = GetState(stateIDV.front());

    if (!prev_entry) {
        std::stringstream ss;
        ss << __FUNCTION__ << ": no hash entry for state id " << stateIDV.front();
        throw SBPL_Exception(ss.str());
    }

    if (prev_entry->dimID < 0 ||
        prev_entry->dimID >= (int)representations_.size())
    {
        std::stringstream ss;
        ss << __FUNCTION__ << ": dimID " << (int)prev_entry->dimID << " is out of range";
        throw SBPL_Exception(ss.str());
    }

    for (size_t i = 1; i < stateIDV.size(); ++i) {
        int prev_id = stateIDV[i - 1];
        int curr_id = stateIDV[i];

        AdaptiveHashEntry *curr_entry = GetState(curr_id);
        if (!curr_entry) {
            std::stringstream ss;
            ss << __FUNCTION__ << ": no hash entry for state id " << curr_id;
            throw SBPL_Exception(ss.str());
        }

        if (curr_entry->dimID == -1) {
            ROS_INFO("State %d is the metagoal", curr_id);
            continue;
        }

        if (curr_entry->dimID < 0 || curr_entry->dimID >= (int)representations_.size()) {
            std::stringstream ss;
            ss << __FUNCTION__ << ": dimID " << (int)curr_entry->dimID << " is out of range";
            throw SBPL_Exception(ss.str());
        }

        if (curr_entry->dimID == prev_entry->dimID) {
            const auto &rep = representations_[curr_entry->dimID];
            if (!rep->IsExecutableAction(prev_id, curr_id)) {
                return false;
            }
        }
        else {
            const auto &prev_rep = representations_[prev_entry->dimID];
            const auto &curr_rep = representations_[curr_entry->dimID];
            ROS_WARN("Skip checking for executable projection from '%s' to '%s' for now", prev_rep->getDescription().c_str(), curr_rep->getDescription().c_str());
        }

        prev_entry = curr_entry;
    }

    return true;
}

void MultiRepAdaptiveDiscreteSpaceInformation::GetSuccs_Track(
    int SourceStateID,
    std::vector<int> *SuccIDV,
    std::vector<int> *CostV)
{
    SuccIDV->clear();
    CostV->clear();
    AdaptiveHashEntry *entry = GetState(SourceStateID);
//    if (!representations_[entry->dimID]->isExecutable()) {
//        std::stringstream ss;
//        ss << "stateID " << SourceStateID << " has representation ID " <<
//                entry->dimID << " [" <<
//                representations_[entry->dimID]->getDescription() <<
//                "] which is not executable. Cannot get tracking successors";
//        throw SBPL_Exception(ss.str());
//    }
    representations_[entry->dimID]->GetTrackSuccs(SourceStateID, SuccIDV, CostV, env_data_.get());
}

void MultiRepAdaptiveDiscreteSpaceInformation::GetSuccs_Track(
    int SourceStateID,
    int expansion_step,
    std::vector<int> *SuccIDV,
    std::vector<int> *CostV)
{
    ROS_ERROR("GetSuccs_Track with exp_step not implemented---defaulting");
    GetSuccs_Track(SourceStateID, SuccIDV, CostV);
}

void MultiRepAdaptiveDiscreteSpaceInformation::GetSuccs_Plan(
    int SourceStateID,
    std::vector<int> *SuccIDV,
    std::vector<int> *CostV)
{
    SuccIDV->clear();
    CostV->clear();
    AdaptiveHashEntry *entry = GetState(SourceStateID);
    representations_[entry->dimID]->GetSuccs(SourceStateID, SuccIDV, CostV, env_data_.get());
    //ROS_INFO("%d -> Got %zu [%zu] successors", SourceStateID, SuccIDV->size(), CostV->size());
}

void MultiRepAdaptiveDiscreteSpaceInformation::GetSuccs_Plan(
    int SourceStateID,
    int expansion_step,
    std::vector<int> *SuccIDV,
    std::vector<int> *CostV)
{
    //TODO GetSuccs_Plan
    ROS_ERROR("GetSuccs_Plan with exp_step not implemented---defaulting");
    GetSuccs_Plan(SourceStateID, SuccIDV, CostV);
}

void MultiRepAdaptiveDiscreteSpaceInformation::GetPreds_Track(
    int TargetStateID,
    std::vector<int> *PredIDV,
    std::vector<int> *CostV)
{
    AdaptiveHashEntry *entry = GetState(TargetStateID);
    if (!representations_[entry->dimID]->isExecutable()) {
        ROS_ERROR("stateID [%d] has representation ID %d [%s], which is not executable. Cannot get tracking successors!", TargetStateID, entry->dimID, representations_[entry->dimID]->getDescription().c_str());
        throw SBPL_Exception();
    }
    representations_[entry->dimID]->GetPreds(TargetStateID, PredIDV, CostV, env_data_.get());
}

void MultiRepAdaptiveDiscreteSpaceInformation::GetPreds_Track(
    int TargetStateID,
    int expansion_step,
    std::vector<int> *PredIDV,
    std::vector<int> *CostV)
{
    ROS_ERROR("GetPreds_Track with exp_step not implemented---defaulting");
    GetPreds_Track(TargetStateID, PredIDV, CostV);
}

void MultiRepAdaptiveDiscreteSpaceInformation::GetPreds_Plan(
    int TargetStateID,
    std::vector<int> *PredIDV,
    std::vector<int> *CostV)
{
    AdaptiveHashEntry *entry = GetState(TargetStateID);
    representations_[entry->dimID]->GetSuccs(TargetStateID, PredIDV, CostV, env_data_.get());
}

void MultiRepAdaptiveDiscreteSpaceInformation::GetPreds_Plan(
    int TargetStateID,
    int expansion_step,
    std::vector<int> *PredIDV,
    std::vector<int> *CostV)
{
    ROS_ERROR("GetPreds_Plan with exp_step not implemented---defaulting");
    GetPreds_Plan(TargetStateID, PredIDV, CostV);
}

} // namespace adim
