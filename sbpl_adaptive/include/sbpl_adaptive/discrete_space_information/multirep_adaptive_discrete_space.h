/// \author Kalin Gochev

#ifndef SBPL_ADAPTIVE_MULTIREP_ADAPTIVE_DISCRETE_SPACE_H
#define SBPL_ADAPTIVE_MULTIREP_ADAPTIVE_DISCRETE_SPACE_H

// standard includes
#include <stdlib.h>
#include <memory>
#include <vector>

// system includes
#include <ros/console.h>
#include <smpl/forward.h>

// project includes
#include <sbpl_adaptive/common.h>
#include <sbpl_adaptive/state.h>
#include <sbpl_adaptive/adaptive_state_representation.h>
#include <sbpl_adaptive/discrete_space_information/adaptive_discrete_space.h>
#include <sbpl_adaptive/discrete_space_information/projection.h>

namespace adim {

struct EnvStateData
{
    EnvStateData()
    {
        goaldata = NULL;
        HashTableSize = 32 * 1024;
        goalHashEntry = NULL;
        startHashEntry = NULL;
    }

    AbstractGoal *goaldata;

    // start and goal entries
    AdaptiveHashEntry *goalHashEntry;
    AdaptiveHashEntry *startHashEntry;

    // hash tables
    size_t HashTableSize;
    std::vector<std::vector<std::vector<AdaptiveHashEntry *>>> HashTables;

    // vector that maps from stateID to coords
    std::vector<AdaptiveHashEntry *> StateID2HashEntry;
};

SBPL_CLASS_FORWARD(MultiRepAdaptiveDiscreteSpace);

class MultiRepAdaptiveDiscreteSpace : public AdaptiveDiscreteSpace
{
public:

    MultiRepAdaptiveDiscreteSpace();

    ~MultiRepAdaptiveDiscreteSpace();

    /// inserts a new hash entry into the hash table and updates its stateID
    /// accordingly returns the stateID
    size_t InsertHashEntry(AdaptiveHashEntry *entry, size_t binID);

    AdaptiveHashEntry *GetState(size_t stateID) const;
    int GetDimID(size_t stateID);

    const EnvStateData *getEnvStateDataPtr() { return &data_; }

    /// \name Representation Management
    ///@{
    bool RegisterFullDRepresentation(
        const AdaptiveStateRepresentationPtr &fullD_representation);

    bool RegisterRepresentation(const AdaptiveStateRepresentationPtr &rep);

    int GetFullDRepresentationID() { return fulld_representation_->getID(); }

    int NumRepresentations() const { return (int)representations_.size(); }

    AdaptiveStateRepresentation *GetFullDRepresentation();
    const AdaptiveStateRepresentation *GetFullDRepresentation() const;

    AdaptiveStateRepresentation *GetRepresentation(int dimID);
    const AdaptiveStateRepresentation *GetRepresentation(int dimID) const;

    // convenient wrappers that (unsafely) downwcast to a specified type for the
    // representation
    template <typename T> T *GetFullDRepresentation();
    template <typename T> const T *GetFullDRepresentation() const;
    template <typename T> T *GetRepresentation(int dimID);
    template <typename T> const T *GetRepresentation(int dimID) const;
    ///@}

    /// \name Projection Management
    ///@{
    bool RegisterProjection(const ProjectionPtr &proj);
    ///@}

    int SetStartCoords(int dimID, const adim::AdaptiveState *state);
    int SetStartConfig(int dimID, const void *representation_specific_data);
    int SetGoalCoords(int dimID, const adim::AdaptiveState *state);
    int SetGoalConfig(int dimID, const void *goal_representation_specific_continuous_data);

    int SetAbstractGoal(AbstractGoal *abstract_goal);

    void updateBestTracked(int StateID, int costToGoal);

    bool ProjectToFullD(
        const adim::AdaptiveState *local_state_data,
        int fromID,
        std::vector<int> &proj_stateIDs,
        int adPathIdx = 0);

    bool Project(
        const adim::AdaptiveState *hd_state_data,
        int fromID,
        int toID,
        std::vector<int> &proj_stateIDs,
        int adPathIdx = 0);

    bool IsProjectionExecutable(int fromID, int toID) const;

    /// \name Required Public Functions From AdaptiveDiscreteSpaceInformation
    ///@{

    bool isExecutablePath(const std::vector<int> &path) override;

    void GetSuccs_Plan(
        int state_id,
        std::vector<int> *succs,
        std::vector<int> *costs) override;

    void GetSuccs_Track(
        int state_id,
        std::vector<int> *succs,
        std::vector<int> *costs) override;

    void GetPreds_Plan(
        int state_id,
        std::vector<int> *preds,
        std::vector<int> *costs) override;

    void GetPreds_Track(
        int state_id,
        std::vector<int> *preds,
        std::vector<int> *costs) override;

    void GetSuccs_Plan(
        int state_id,
        int expansion_step,
        std::vector<int> *succs,
        std::vector<int> *costs) override;

    void GetSuccs_Track(
        int state_id,
        int expansion_step,
        std::vector<int> *succs,
        std::vector<int> *costs) override;

    void GetPreds_Plan(
        int state_id,
        int expansion_step,
        std::vector<int> *preds,
        std::vector<int> *costs) override;

    void GetPreds_Track(
        int state_id,
        int expansion_step,
        std::vector<int> *preds,
        std::vector<int> *costs) override;

    ///@}

protected:

    int BestTracked_StateID;
    int BestTracked_Cost;

    void InsertMetaGoalHashEntry(AdaptiveHashEntry *entry);

    AdaptiveStateRepresentationPtr fulld_representation_;
    std::vector<AdaptiveStateRepresentationPtr> representations_;

    EnvStateData data_;

    std::vector<ProjectionPtr> proj_matrix_;
};

inline
AdaptiveStateRepresentation *
MultiRepAdaptiveDiscreteSpace::GetFullDRepresentation()
{
    return fulld_representation_.get();
}

inline
const AdaptiveStateRepresentation *
MultiRepAdaptiveDiscreteSpace::GetFullDRepresentation() const
{
    return fulld_representation_.get();
}

inline
AdaptiveStateRepresentation *
MultiRepAdaptiveDiscreteSpace::GetRepresentation(int dimID)
{
    if (dimID < 0 || dimID >= representations_.size()) {
        return NULL;
    }
    return representations_[dimID].get();
}

inline
const AdaptiveStateRepresentation *
MultiRepAdaptiveDiscreteSpace::GetRepresentation(int dimID) const
{
    if (dimID < 0 || dimID >= representations_.size()) {
        return NULL;
    }
    return representations_[dimID].get();
}

template <typename T> T *
MultiRepAdaptiveDiscreteSpace::GetFullDRepresentation()
{
    return (T *)fulld_representation_.get();
}

template <typename T> const T *
MultiRepAdaptiveDiscreteSpace::GetFullDRepresentation() const
{
    return (const T *)fulld_representation_.get();
}

template <typename T> T *
MultiRepAdaptiveDiscreteSpace::GetRepresentation(int dimID)
{
    if (dimID < 0 || dimID >= representations_.size()) {
        return nullptr;
    }
    return (T *)representations_[dimID].get();
}

template <typename T> const T *
MultiRepAdaptiveDiscreteSpace::GetRepresentation(int dimID) const
{
    if (dimID < 0 || dimID >= representations_.size()) {
        return nullptr;
    }
    return (const T *)representations_[dimID].get();
}

inline
void MultiRepAdaptiveDiscreteSpace::updateBestTracked(
    int StateID,
    int costToGoal)
{
    if (costToGoal < BestTracked_Cost) {
        BestTracked_Cost = costToGoal;
        BestTracked_StateID = StateID;
        ROS_INFO("BestSeenState: %d @ %d", BestTracked_StateID, BestTracked_Cost);
    }
}

} // namespace adim

#endif
