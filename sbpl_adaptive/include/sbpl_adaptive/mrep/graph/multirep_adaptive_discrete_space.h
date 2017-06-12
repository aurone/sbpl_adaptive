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
#include <sbpl_adaptive/core/graph/adaptive_discrete_space.h>
#include <sbpl_adaptive/mrep/graph/state.h>
#include <sbpl_adaptive/mrep/graph/adaptive_state_representation.h>
#include <sbpl_adaptive/mrep/graph/projection.h>

namespace adim {

SBPL_CLASS_FORWARD(MultiRepAdaptiveDiscreteSpace);

class MultiRepAdaptiveDiscreteSpace : public AdaptiveDiscreteSpace
{
public:

    MultiRepAdaptiveDiscreteSpace();

    ~MultiRepAdaptiveDiscreteSpace();

    /// \name Representation Management
    ///@{

    bool RegisterFullDRepresentation(const AdaptiveStateRepresentationPtr &rep);
    bool RegisterRepresentation(const AdaptiveStateRepresentationPtr &rep);

    int GetFullDRepresentationID() { return full_representation_->getID(); }

    AdaptiveStateRepresentation *GetFullDRepresentation();
    const AdaptiveStateRepresentation *GetFullDRepresentation() const;
    AdaptiveStateRepresentation *GetRepresentation(int dimID);
    const AdaptiveStateRepresentation *GetRepresentation(int dimID) const;

    template <typename T> T *GetFullDRepresentation();
    template <typename T> const T *GetFullDRepresentation() const;
    template <typename T> T *GetRepresentation(int dimID);
    template <typename T> const T *GetRepresentation(int dimID) const;

    int NumRepresentations() const { return (int)representations_.size(); }
    ///@}

    /// \name State Management
    ///@{
    int InsertHashEntry(AdaptiveHashEntry *entry, size_t binID);

    template <typename Equal>
    adim::AdaptiveHashEntry *FindHashEntry(size_t binID, int dimID, Equal eq);

    AdaptiveHashEntry *GetState(int stateID) const;
    int GetDimID(int stateID);
    ///@}

    /// \name Start State and Goal Condition
    ///@{
    int SetStartCoords(int dimID, const AdaptiveState *state);
    int SetStartConfig(int dimID, const ModelCoords *coords);
    int SetGoalCoords(int dimID, const AdaptiveState *state);
    int SetGoalConfig(int dimID, const ModelCoords *coords);

    int GetStartStateID() const;
    int GetGoalStateID() const;

    int SetAbstractGoal(AbstractGoal *abstract_goal);

    const AbstractGoal *getAbstractGoal() const { return goal_; }
    AbstractGoal *getAbstractGoal() { return goal_; }

    template <typename T> const T *getAbstractGoal() const {
        return static_cast<const T *>(goal_);
    }
    template <typename T> T *getAbstractGoal() {
        return static_cast<T *>(goal_);
    }
    ///@}

    /// \name Projections
    ///@{
    bool RegisterProjection(const ProjectionPtr &proj);

    bool Project(
        const adim::AdaptiveState *hd_state_data,
        int src_rep,
        int dst_rep,
        std::vector<int> &proj_state_ids,
        std::vector<int> &proj_costs,
        int adPathIdx = 0);

    bool IsProjectionExecutable(int fromID, int toID) const;
    ///@}

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

    AdaptiveStateRepresentationPtr full_representation_;
    std::vector<AdaptiveStateRepresentationPtr> representations_;

    AbstractGoal *goal_;

    // start and goal entries
    AdaptiveHashEntry *goal_hash_entry_;
    AdaptiveHashEntry *start_hash_entry_;

    // hash tables
    size_t hash_table_size_;
    std::vector<std::vector<std::vector<AdaptiveHashEntry *>>> hash_tables_;

    // vector that maps from stateID to coords
    std::vector<AdaptiveHashEntry *> state_id_to_hash_entry_;

    std::vector<ProjectionPtr> proj_matrix_;

    int InsertMetaGoalHashEntry(AdaptiveHashEntry *entry);

    bool IsValidStateID(int stateID) const;
    bool IsValidRepID(int dimID) const;
    int GetProjectionIndex(int srep, int trep) const;
};

inline
AdaptiveStateRepresentation *
MultiRepAdaptiveDiscreteSpace::GetFullDRepresentation()
{
    return full_representation_.get();
}

inline
const AdaptiveStateRepresentation *
MultiRepAdaptiveDiscreteSpace::GetFullDRepresentation() const
{
    return full_representation_.get();
}

inline
AdaptiveStateRepresentation *
MultiRepAdaptiveDiscreteSpace::GetRepresentation(int dimID)
{
    if (dimID < 0 || dimID >= representations_.size()) {
        return nullptr;
    }
    return representations_[dimID].get();
}

inline
const AdaptiveStateRepresentation *
MultiRepAdaptiveDiscreteSpace::GetRepresentation(int dimID) const
{
    if (dimID < 0 || dimID >= representations_.size()) {
        return nullptr;
    }
    return representations_[dimID].get();
}

template <typename T> T *
MultiRepAdaptiveDiscreteSpace::GetFullDRepresentation()
{
    return (T *)full_representation_.get();
}

template <typename T> const T *
MultiRepAdaptiveDiscreteSpace::GetFullDRepresentation() const
{
    return (const T *)full_representation_.get();
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

/// Lookup a hash entry for a state
/// \param binID The hash value of the state being looked up
/// \param dimID The representation id of the state being looked up
/// \param eq The equivalence condition for a state
/// \return A pointer to an equivalent state, or nullptr if none is found
template <typename Equal>
adim::AdaptiveHashEntry *MultiRepAdaptiveDiscreteSpace::FindHashEntry(
    size_t binID,
    int dimID,
    Equal eq)
{
    size_t bin = binID & (hash_table_size_ - 1);
    const auto &state_table = hash_tables_[dimID];
    for (size_t i = 0; i < state_table[bin].size(); ++i) {
        adim::AdaptiveHashEntry *entry = state_table[bin][i];
        if (eq(entry)) {
            return entry;
        }
    }

    return nullptr;
}

} // namespace adim

#endif
