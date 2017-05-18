#ifndef SBPL_ADAPTIVE_ADAPTIVE_DISCRETE_SPACE_H
#define SBPL_ADAPTIVE_ADAPTIVE_DISCRETE_SPACE_H

// standard includes
#include <string>
#include <vector>

// system includes
#include <sbpl/headers.h>
#include <smpl/forward.h>

namespace adim {

SBPL_CLASS_FORWARD(AdaptiveDiscreteSpace)

/// \brief base class for environments defining planning graphs
///
/// It is independent of the graph search used. The main means of communication
/// between environment and graph search is through stateID. Each state is
/// uniquely defined by stateID and graph search is ONLY aware of stateIDs. It
/// doesn't know anything about the actual state variables. Environment, on the
/// other hand, maintains a mapping from stateID to actual state variables
/// (coordinates) using StateID2IndexMapping array
class AdaptiveDiscreteSpace : public DiscreteSpaceInformation
{
public:

    AdaptiveDiscreteSpace();

    virtual ~AdaptiveDiscreteSpace() { }

    /// \brief used by the tracker to tell the environment which state is being
    /// expanded
    virtual void expandingState(int StateID) = 0;

    /// \brief gets the state ID of the 'best' state encountered during tracking
    virtual int getBestSeenState() = 0;

    /// \brief checks if the given stateID path is executable
    virtual bool isExecutablePath(const std::vector<int> &stateIDV) = 0;

    /// Adds a new sphere of radius rad at the state coordinates specified by
    /// StateID a list of modified stateIDs is returned in modifiedStates if not
    /// NULL
    virtual void addSphere(
        int StateID,
        std::vector<int> *modifiedStates = NULL) = 0;

    virtual void processCostlyPath(
        const std::vector<int> &planning_path,
        const std::vector<int> &tracking_path,
        std::vector<int> *new_sphere_locations) = 0;

    /// \brief resets the environment to its original state - no spheres, etc.
    virtual void reset() = 0;

    /// \name Interface Functions for TRAPlanner
    ///@{

    /// \brief adds a new sphere of radius rad at the state coordinates
    /// specified by StateID returns the earliest expansion step of the modified
    /// states
    virtual void addSphere(int StateID, int &first_mod_step) = 0;

    void GetPreds(
        int TargetStateID,
        int expansionStep,
        std::vector<int> *PredIDV,
        std::vector<int> *CostV);

    void GetSuccs(
        int SourceStateID,
        int expansionStep,
        std::vector<int> *SuccIDV,
        std::vector<int> *CostV);

    ///@}

    virtual void visualizeState(int sID, int scolor, std::string name);

    virtual void visualizeStatePath(
        std::vector<int> *path,
        int scolor,
        int ecolor,
        std::string name);

    virtual void visualizeEnvironment() { }

    /// \brief sets the environment in adaptive planning mode
    void setPlanMode();

    /// \brief constructs a tunnel of width tunnelWidth around the path
    /// specified by stateIDs_V and sets the environment in tracking mode
    void setTrackMode(
        const std::vector<int> &stateIDs_V,
        int cost,
        std::vector<int> *ModStates = NULL);

    bool isInTrackingMode() const { return trackMode; }

    const std::vector<int> &getLastAdaptivePath() { return lastAdaptivePath_; }

    /// \name Reimplemented Public Functions from DiscreteSpaceInformation
    ///@{

    void GetSuccs(
        int SourceStateID,
        std::vector<int>* SuccIDV,
        std::vector<int>* CostV);

    void GetPreds(
        int TargetStateID,
        std::vector<int>* PredIDV,
        std::vector<int>* CostV);

    ///@}

protected:

    /// NOTES:
    ///
    /// * The environment is always in tracking or planning mode. use
    ///   setTrackMode and setPlanMode to change the mode of the environment
    /// * The SBPL AdaptivePlanner will use setTrackMode and setPlanMode to
    ///   switch between environment modes
    /// * getSuccs and getPreds functions should take into account the
    ///   environment mode when generating successor or predecessor states for
    ///   the planner

    /// \brief gets successors for tracking mode
    virtual void GetSuccs_Track(
        int SourceStateID,
        std::vector<int>* SuccIDV,
        std::vector<int>* CostV) = 0;

    /// \brief gets successors for planning mode
    virtual void GetSuccs_Plan(
        int SourceStateID,
        std::vector<int>* SuccIDV,
        std::vector<int>* CostV) = 0;

    /// \brief gets predecessors for tracking mode
    virtual void GetPreds_Track(
        int TargetStateID,
        std::vector<int>* PredIDV,
        std::vector<int>* CostV) = 0;

    /// \brief gets predecessors for planning mode
    virtual void GetPreds_Plan(
        int TargetStateID,
        std::vector<int>* PredIDV,
        std::vector<int>* CostV) = 0;

    /// \brief gets successors for tracking mode
    virtual void GetSuccs_Track(
        int SourceStateID,
        int expansion_step,
        std::vector<int>* SuccIDV,
        std::vector<int>* CostV) = 0;

    /// \brief gets successors for planning mode
    virtual void GetSuccs_Plan(
        int SourceStateID,
        int expansion_step,
        std::vector<int>* SuccIDV,
        std::vector<int>* CostV) = 0;

    /// \brief gets predecessors for tracking mode
    virtual void GetPreds_Track(
        int TargetStateID,
        int expansion_step,
        std::vector<int>* PredIDV,
        std::vector<int>* CostV) = 0;

    /// \brief gets predecessors for planning mode
    virtual void GetPreds_Plan(
        int TargetStateID,
        int expansion_step,
        std::vector<int>* PredIDV,
        std::vector<int>* CostV) = 0;

    virtual void onSetPlanMode() { }

    virtual void onSetTrackMode(
        const std::vector<int> &stateIDs_V,
        int cost,
        std::vector<int> *ModStates) { }

private:

    bool trackMode; ///< true - tracking, false - planning
    std::vector<int> lastAdaptivePath_;
};

inline
AdaptiveDiscreteSpace::AdaptiveDiscreteSpace() :
    trackMode(false),
    lastAdaptivePath_()
{
}

inline
void AdaptiveDiscreteSpace::GetPreds(
    int TargetStateID,
    int expansionStep,
    std::vector<int>* PredIDV,
    std::vector<int>* CostV)
{
    if (trackMode) {
        GetPreds_Track(TargetStateID, expansionStep, PredIDV, CostV);
    }
    else {
        GetPreds_Plan(TargetStateID, expansionStep, PredIDV, CostV);
    }
};

inline
void AdaptiveDiscreteSpace::GetSuccs(
    int SourceStateID,
    int expansionStep,
    std::vector<int>* SuccIDV,
    std::vector<int>* CostV)
{
    if (trackMode) {
        GetSuccs_Track(SourceStateID, expansionStep, SuccIDV, CostV);
    }
    else {
        GetSuccs_Plan(SourceStateID, expansionStep, SuccIDV, CostV);
    }
};

inline
void AdaptiveDiscreteSpace::GetPreds(
    int TargetStateID,
    std::vector<int>* PredIDV,
    std::vector<int>* CostV)
{
    if (trackMode) {
        GetPreds_Track(TargetStateID, PredIDV, CostV);
    }
    else {
        GetPreds_Plan(TargetStateID, PredIDV, CostV);
    }
}

inline
void AdaptiveDiscreteSpace::GetSuccs(
    int SourceStateID,
    std::vector<int>* SuccIDV,
    std::vector<int>* CostV)
{
    if (trackMode) {
        GetSuccs_Track(SourceStateID, SuccIDV, CostV);
    }
    else {
        GetSuccs_Plan(SourceStateID, SuccIDV, CostV);
    }
}

inline
void AdaptiveDiscreteSpace::visualizeStatePath(
    std::vector<int> *path,
    int scolor,
    int ecolor,
    std::string name)
{
    SBPL_ERROR("visualizeStatePath not implemented!");
}

inline
void AdaptiveDiscreteSpace::visualizeState(
    int sID,
    int scolor,
    std::string name)
{
    SBPL_ERROR("visualizeState not implemented!");
}

inline
void AdaptiveDiscreteSpace::setPlanMode()
{
    trackMode = false;
    onSetPlanMode();
}

inline
void AdaptiveDiscreteSpace::setTrackMode(
    const std::vector<int> &stateIDs_V,
    int cost,
    std::vector<int> *ModStates)
{
    onSetTrackMode(stateIDs_V, cost, ModStates);
    trackMode = true;
    lastAdaptivePath_ = stateIDs_V;
}

} // namespace adim

#endif
