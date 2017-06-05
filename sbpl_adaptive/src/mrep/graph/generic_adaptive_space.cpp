#include <sbpl_adaptive/core/graph/generic_adaptive_space.h>

namespace adim {

/////////////////////////////
// GenericCollisionChecker //
/////////////////////////////

GenericCollisionChecker::GenericCollisionChecker(DiscreteSpaceInformation* env) :
    env_(env)
{
}

GenericCollisionChecker::~GenericCollisionChecker()
{
}

//////////////////////////
// GenericAdaptiveSpace //
//////////////////////////

/// \class GenericAdaptiveSpace

/// constructor - specify the high-dim size and low-dim size
GenericAdaptiveSpace::GenericAdaptiveSpace(
    GenericCollisionChecker *lo_cspace,
    GenericCollisionchecker *hi_cspace)
:
    l_cspace_(lo_cspace),
    h_cspace_(hi_cspace)
{
}

GenericAdaptiveSpace::~GenericAdaptiveSpace()
{
    spheres_.clear();
}

void GenericAdaptiveSpace::GetSuccs_HighD(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    EnvHashEntry_t *SourceState = GetHashEntry(state_id);
    if (!SourceState) {
        return;
    }

    std::vector<double> coords(SourceState->coord.size(), 0);
    coords_disc2cont_h(&(SourceState->coord), &coords);
    std::vector<tAction*>* h_actions_ = getActionsForState(SourceState);

    // loop through all high-d actions
    for (size_t aid = 0; aid < h_actions_->size(); ++aid) {
        // collision check action
        if (!h_cspace_->isValidTransition(coords, (*h_actions_)[aidx])) {
            continue;
        }

        // compute successor state coords
        std::vector<short int> succ_coords(SourceState->coord);
        for (size_t cid = 0; cid < h_actions_->at(aid)->d_.size(); cid++) {
            succ_coords[cid] = SourceState->coord[cid] + h_actions_->at(aid)->d_[cid];
            // the following takes care of circular angular values (modulo arithmetic)
            if (h_angle_mask_[cid] > 0) {
                while (succ_coords[cid] < 0) {
                    succ_coords[cid] += h_angle_mask_[cid];
                }
                succ_coords[cid] = succ_coords[cid] % h_angle_mask_[cid];
            }
        }

        // check if this is a valid high-D transition
        bool project_to_lowD = false;
        if (!isValidHighDState(&succ_coords)) {
            if (trackMode) {
                continue;
            }
            else {
                project_to_lowD = true;
            }
        }

        // generate new state hash entry if needed
        if (project_to_lowD) {
            // make succ_coords the coordinates of the
            projectToLowDCoords(&succ_coords);
        }

        EnvHashEntry_t* SuccState = GetHashEntryFromCoord(succ_coords);
        if (!SuccState) {
            SuccState = CreateHashEntry(succ_coords, h_actions_->at(aid));
        }
        succs->push_back(SuccState->stateID);
        costs->push_back(h_actions_->at(aid)->cost);
    }
}

/// Get the low-dimensional successors of a state.
void GenericAdaptiveSpace::GetSuccs_LowD(
    int state_id,
    vector<int>* succs,
    vector<int>* costs)
{
    EnvHashEntry_t* state = GetHashEntry(state_id);
    if (!state){
        return;
    }

    std::vector<double> coords(state->coord.size(), 0);
    coords_disc2cont_l(&state->coord, &coords);

    // loop through all low-d actions
    std::vector<tAction*> *l_actions_ = getActionsForState(state);
    for (unsigned int aid = 0; aid < l_actions_->size(); aid++) {
        // compute new state coords
        std::vector<short int> succ_coords(state->coord);
        for (unsigned int cid = 0; cid < l_actions_->at(aid)->d_.size(); cid++) {
            succ_coords[cid] = state->coord[cid] + l_actions_->at(aid)->d_[cid];
            // the following takes care of circular angular values (modulo arithmetic)
            if (l_angle_mask_[cid] > 0) {
                while (succ_coords[cid] < 0) {
                    succ_coords[cid] += l_angle_mask_[cid];
                }
                succ_coords[cid] = succ_coords[cid] % l_angle_mask_[cid];
            }
        }
        // check if this is a valid tracking transition
        if (!isValidLowDState(&succ_coords))
            continue;
        //collision check action
        if (!l_cspace_->isValidTransition(coords, (*l_actions_)[aid])) {
            continue;
        }
        //generate new state hash entry if needed
        EnvHashEntry_t* SuccState = GetHashEntryFromCoord(succ_coords);
        if (SuccState == NULL) {
            SuccState = CreateHashEntry(succ_coords, l_actions_->at(aid));
        }
        succs->push_back(SuccState->stateID);
        costs->push_back(l_actions_->at(aid)->cost);
    }
}

/// Get the valid low- and high-dimensional successors of a state that is near a
/// sphere
void GenericAdaptiveSpace::GetSuccs_LowDNear(
    int state_id,
    vector<int>* succs,
    vector<int>* costs)
{
    // get the valid low-d successors
    GetSuccs_LowD(state_id, succs, costs);

    // then get the valid high-d successors of all high-d projections of state_id
    std::vector<int> HighDImages;
    projectToHighD(state_id, &HighDImages);
    for (size_t i = 0; i < HighDImages.size(); i++) {
        GetSuccs_HighD(HighDImages[i], succs, costs);
    }
}

/// Get successors for tracking mode
void GenericAdaptiveSpace::GetSuccs_Track(
    int state_id,
    vector<int>* succs,
    vector<int>* costs)
{
    if (!trackMode) {
        ROS_ERROR("GetSuccs_Track called while environment in planning mode");
        throw SBPL_Exception();
    }
    GetSuccs_HighD(state_id, succs, costs);
}

/// Get successors for planning mode.
void GenericAdaptiveSpace::GetSuccs_Plan(
    int state_id,
    vector<int>* succs,
    vector<int>* costs)
{
    if (trackMode) {
        ROS_ERROR_ONCE("GetSuccs_Plan called while environment in tracking mode");
        return;
    }

    EnvHashEntry_t *SourceState = GetHashEntry(state_id);
    std::vector<double> SourceStateCoords(SourceState->coord.size(), 0);
    coords_disc2cont_h(&(SourceState->coord), &SourceStateCoords);

    char stategrid = lookup_stategrid(&SourceState->coord);

    if (stategrid == STATEGRID_FAR) {
        // state is far from all spheres
        GetSuccs_LowD(state_id, succs, costs);
    }
    else if (stategrid == STATEGRID_NEAR) {
        // state is near a sphere
        GetSuccs_LowDNear(state_id, succs, costs);
    }
    else if (stategrid == STATEGRID_IN) {
        // state is inside a sphere
        GetSuccs_HighD(state_id, succs, costs);
    }
}

} // namespace adim
