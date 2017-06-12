#ifndef SBPL_ADAPTIVE_STATE_REPRESENTATION_H
#define SBPL_ADAPTIVE_STATE_REPRESENTATION_H

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <smpl/forward.h>

// project includes
#include <sbpl_adaptive/core/graph/state.h>

namespace adim {

// breaks cyclic dependency
SBPL_CLASS_FORWARD(MultiRepAdaptiveDiscreteSpace)

SBPL_CLASS_FORWARD(AdaptiveStateRepresentation)

class AdaptiveStateRepresentation :
    public std::enable_shared_from_this<AdaptiveStateRepresentation>
{
public:

    AdaptiveStateRepresentation(
        const MultiRepAdaptiveDiscreteSpacePtr &env,
        bool executable,
        const std::string &name);

    virtual ~AdaptiveStateRepresentation();

    /// \name Start and Goal States
    ///@{

    // return the state ID of the created start state
    virtual int SetStartCoords(const AdaptiveState *state) = 0;
    virtual int SetStartConfig(const ModelCoords *coords) = 0;

    // return the state ID of the created goal state
    virtual int SetGoalCoords(const AdaptiveState *state) = 0;
    virtual int SetGoalConfig(const ModelCoords *coords) = 0;

    virtual bool isGoalState(int StateID) const = 0;

    ///@}

    /// \name Transitions
    ///@{

    virtual void GetSuccs(
        int stateID,
        std::vector<int> *succs,
        std::vector<int> *costs) = 0;

    virtual void GetTrackSuccs(
        int state_id,
        std::vector<int> *succs,
        std::vector<int> *costs) = 0;

    virtual void GetPreds(
        int stateID,
        std::vector<int> *preds,
        std::vector<int> *costs) = 0;

    virtual bool IsExecutableAction(int src_id, int dst_id) {
        return isExecutable(); }

    ///@}

    virtual bool IsValidStateData(const AdaptiveState *state) const = 0;
    virtual bool IsValidConfig(const ModelCoords *coords) const = 0;

    virtual int GetGoalHeuristic(int stateID) const = 0;

    /// \name Debug Information
    ///@{

    virtual void PrintState(
        int stateID,
        bool bVerbose,
        FILE* fOut = stdout) const = 0;

    virtual void PrintStateData(
        const AdaptiveState *state_data,
        bool bVerbose,
        FILE* fOut = stdout) const = 0;

    virtual void VisualizeState(
        int stateID,
        int hue,
        const std::string &ns,
        int &viz_idx) const = 0;

    ///@}

    /// \name Lossless Projections
    ///@{

    virtual bool ProjectToFullD(
        const AdaptiveState *ld_state_data,
        std::vector<int> &hd_projStateIDs,
        int adPathIdx = 0) = 0;

    virtual bool ProjectFromFullD(
        const AdaptiveState *hd_state_data,
        std::vector<int> &ld_projStateIDs,
        int adPathIdx = 0) = 0;

    ///@}

    virtual void deleteStateData(int stateID) = 0;

    virtual void toCont(
        const AdaptiveState *state,
        ModelCoords *coords) const = 0;

    virtual void toDisc(
        const ModelCoords *coords,
        AdaptiveState *state) const = 0;

    const MultiRepAdaptiveDiscreteSpace *mrepSpace() const { return env_.get(); }
    MultiRepAdaptiveDiscreteSpace *mrepSpace() { return env_.get(); }

    template <typename T> const T *mrepSpace() const
    { return static_cast<const T *>(env_.get()); }

    template <typename T> T *mrepSpace()
    { return static_cast<T *>(env_.get()); }

    bool isExecutable() const { return executable_; }
    const std::string &getName() const { return name_; }
    int getID() const { return dimID_; }
    void setID(int id) { dimID_ = id; }

    /// \name Abstraction Hierarchy
    ///@{

    void setFullDRepresentation(const AdaptiveStateRepresentationPtr &fullD_rep)
    { fullD_rep_ = fullD_rep; }

    void addParentRepresentation(const AdaptiveStateRepresentationPtr &parent);

    void addChildRepresentation(const AdaptiveStateRepresentationPtr &child);

    void GetExecutableParents(
        std::vector<const AdaptiveStateRepresentation*> &executableParents) const;

    const std::vector<AdaptiveStateRepresentationPtr> &parents() const
    { return parents_; }

    const std::vector<AdaptiveStateRepresentationPtr> &children() const
    { return children_; }

    ///@}

    /// \name Adaptive Dimensionality Parameters
    ///@{
    double getSphereRadius() const { return sphere_radius_; }
    double getNearRadius() const { return near_radius_; }
    double getTunnelRadius() const { return tunnel_radius_; }
    ///@}

protected:

    double sphere_radius_;
    double near_radius_;
    double tunnel_radius_;

    MultiRepAdaptiveDiscreteSpacePtr env_;
    int dimID_;

private:

    bool executable_;
    std::string name_;

    AdaptiveStateRepresentationPtr fullD_rep_;

    // less abstract representations
    std::vector<AdaptiveStateRepresentationPtr> parents_;

    // more abstract representations
    std::vector<AdaptiveStateRepresentationPtr> children_;
};

} // namespace adim

#endif
