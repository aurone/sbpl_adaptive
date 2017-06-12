#ifndef SBPL_ADAPTIVE_PROJECTION_H
#define SBPL_ADAPTIVE_PROJECTION_H

// standard includes
#include <vector>

// system includes
#include <smpl/forward.h>

// project includes
#include <sbpl_adaptive/mrep/graph/adaptive_state_representation.h>
#include <sbpl_adaptive/mrep/graph/state.h>

namespace adim {

SBPL_CLASS_FORWARD(Projection);

class Projection
{
public:

    Projection(
        const AdaptiveStateRepresentationPtr &src_rep,
        const AdaptiveStateRepresentationPtr &tgt_rep,
        bool executable = false);

    virtual ~Projection();

    /// Return projection successors from a state in the source representation
    /// to states in the target representation and their associated costs. A
    /// cost of -1 implies that the cost should be overridden by the caller,
    /// often the source representation.
    ///
    /// Successors (and costs) should only be appended to the output array of
    /// successors and the previous contents should not be modified or cleared.
    ///
    /// \param state A state in the source representation
    /// \param proj_state_ids The state ids of the successor states in the
    ///     target representation
    /// \param proj_costs Associated costs of projection transitions, -1 for
    ///     each transition whose cost should be determined by the caller
    /// \param ad_path_idx The index of the nearest state on the adaptive path
    virtual void project(
        const adim::AdaptiveState *state,
        std::vector<int> &proj_state_ids,
        std::vector<int> &proj_costs,
        int ad_path_idx = 0) = 0;

    /// Associate a multi-representation space with this projection. Need only
    /// be called by the multi-representation space upon registration.
    void setPlanningSpace(MultiRepAdaptiveDiscreteSpace *space)
    { space_ = space; }

    const AdaptiveStateRepresentationPtr &sourceRep() const
    { return src_rep_; }

    const AdaptiveStateRepresentationPtr &targetRep() const
    { return tgt_rep_; }

    template <class T> T *sourceRep()
    { return static_cast<T*>(src_rep_.get()); }

    template <class T> const T *sourceRep() const
    { return static_cast<const T*>(src_rep_.get()); }

    template <class T> T *targetRep()
    { return static_cast<T*>(tgt_rep_.get()); }

    template <class T> const T *targetRep() const
    { return static_cast<const T*>(tgt_rep_.get()); }

    int sourceRepID() const { return src_rep_->getID(); }
    int targetRepID() const { return tgt_rep_->getID(); }

    bool executable() const { return executable_; }

    MultiRepAdaptiveDiscreteSpace *space()
    { return space_; }

    const MultiRepAdaptiveDiscreteSpace *space() const
    { return space_; }

    template <class T> T *space()
    { return static_cast<T*>(space_); }

    template <class T> const T *space() const
    { return static_cast<const T*>(space_); }

private:

    MultiRepAdaptiveDiscreteSpace *space_;
    AdaptiveStateRepresentationPtr src_rep_;
    AdaptiveStateRepresentationPtr tgt_rep_;
    bool executable_;
};

} // namespace adim

#endif
