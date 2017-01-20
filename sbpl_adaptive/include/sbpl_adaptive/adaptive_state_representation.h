/*
 * representation.h
 *
 *  Created on: Feb 8, 2016
 *      Author: kalin
 */

#ifndef sbpl_adim_representation_h
#define sbpl_adim_representation_h

// standard includes
#include <string>
#include <vector>

// project includes
#include <sbpl_adaptive/macros.h>

namespace adim {

// breaks cyclic dependency
SBPL_CLASS_FORWARD(MultiRepAdaptiveDiscreteSpaceInformation)

SBPL_CLASS_FORWARD(AdaptiveStateRepresentation)

class AdaptiveStateRepresentation :
    public std::enable_shared_from_this<AdaptiveStateRepresentation>
{
public:

    AdaptiveStateRepresentation(
        MultiRepAdaptiveDiscreteSpaceInformationPtr env,
        bool executable,
        std::string description);

    virtual ~AdaptiveStateRepresentation() { };

    // return the state ID of the created start state
    virtual int SetStartCoords(const void *disc_data) = 0;

    virtual int SetStartConfig(const void *cont_data) = 0;

    // return the state ID of the created start state
    virtual int SetGoalCoords(const void *disc_data) = 0;

    virtual int SetGoalConfig(const void *cont_data) = 0;

    virtual bool isGoalState(int StateID) const = 0;

    virtual void GetSuccs(
        int stateID,
        std::vector<int> *SuccV,
        std::vector<int> *CostV,
        const void *env_data) = 0;

    virtual void GetTrackSuccs(
        int state_id,
        std::vector<int> *succs,
        std::vector<int> *costs,
        const void *env_data) = 0;

    virtual void GetPreds(
        int stateID,
        std::vector<int> *PredV,
        std::vector<int> *CostV,
        const void *env_data) = 0;

    virtual void PrintState(
        int stateID,
        bool bVerbose,
        FILE* fOut = stdout) const = 0;

    virtual void PrintStateData(
        const void *state_data,
        bool bVerbose,
        FILE* fOut = stdout) const = 0;

    virtual int GetGoalHeuristic(int stateID) const = 0;

    virtual void VisualizeState(
        int stateID,
        int hue,
        std::string ns,
        int &viz_idx) const = 0;

    virtual bool IsValidStateData(const void *disc_data) const = 0;

    virtual bool IsValidConfig(const void *cont_data) const = 0;

    virtual bool ProjectToFullD(
        const void *ld_state_data,
        std::vector<int> &hd_projStateIDs,
        int adPathIdx = 0) = 0;

    virtual bool ProjectFromFullD(
        const void *hd_state_data,
        std::vector<int> &ld_projStateIDs,
        int adPathIdx = 0) = 0;

    // free the stateData ptr in the AdaptiveHashEntry
    virtual void deleteStateData(int stateID) = 0;

    virtual void toCont(const void *disc_data, void *cont_data) const = 0;

    virtual void toDisc(const void *cont_data, void *disc_data) const = 0;

    virtual bool IsExecutableAction(int src_id, int dst_id) const
    {
        return isExecutable();
    }

    void setFullDRepresentation(const AdaptiveStateRepresentationPtr &fullD_rep)
    {
        fullD_rep_ = fullD_rep;
    }

    bool isExecutable() const { return executable_; }

    int getID() const { return dimID_; }

    void setID(int id){ dimID_ = id; }

    const std::string getDescription() const { return description_; }

    void addParentRepresentation(const AdaptiveStateRepresentationPtr &parent);

    void addChildRepresentation(const AdaptiveStateRepresentationPtr &child);

    void GetExecutableParents(
        std::vector<const AdaptiveStateRepresentation*> &executableParents) const;

    void getParentIDs(int stateID, std::vector<int> &IDs) const;

    void getChildIDs(int stateID, std::vector<int> &IDs) const;

    double getSphereRadius() const { return sphere_radius_; }

    double getNearRadius() const { return near_radius_; }

    double getTunnelRadius() const { return tunnel_radius_; }

protected:

    double sphere_radius_;
    double near_radius_;
    double tunnel_radius_;

    MultiRepAdaptiveDiscreteSpaceInformationPtr env_;
    int dimID_;
    bool executable_;
    std::string description_;
    AdaptiveStateRepresentationPtr fullD_rep_;

    // these form the abstraction hierarchy
    std::vector<AdaptiveStateRepresentationPtr> parents_; // less abstract representations
    std::vector<AdaptiveStateRepresentationPtr> children_; // more abstract representations
};

inline
AdaptiveStateRepresentation::AdaptiveStateRepresentation(
    MultiRepAdaptiveDiscreteSpaceInformationPtr env,
    bool executable,
    std::string description)
:
    env_(env),
    dimID_(-1),
    executable_(executable),
    description_(description)
{
    sphere_radius_ = 1.0;
    near_radius_ = 1.0;
    tunnel_radius_ = 1.0;
}

inline
void AdaptiveStateRepresentation::addParentRepresentation(
    const AdaptiveStateRepresentationPtr &parent)
{
    bool found = false;
    for (const auto &rep : parents_) {
        if (rep == parent) {
            found = true;
            break;
        }
    }
    if (!found) {
        SBPL_INFO("Added %s as parent representation to %s!", parent->getDescription().c_str(), getDescription().c_str());
        parents_.push_back(parent);
        parent->addChildRepresentation(shared_from_this());
    }
}

inline
void AdaptiveStateRepresentation::addChildRepresentation(
    const AdaptiveStateRepresentationPtr &child)
{
    bool found = false;
    for (const auto &rep : children_) {
        if (rep == child) {
            found = true;
            break;
        }
    }
    if (!found) {
        SBPL_INFO("Added %s as child representation to %s!", child->getDescription().c_str(), getDescription().c_str());
        children_.push_back(child);
        child->addParentRepresentation(shared_from_this());
    }
}

/// If executable, returns this representation, otherwise return the executable
/// parents of its parents.
inline
void AdaptiveStateRepresentation::GetExecutableParents(
    std::vector<const AdaptiveStateRepresentation *> &executableParents) const
{
    if (isExecutable()) {
        executableParents.push_back(this);
    }
    else {
        for (const auto &parent : parents_) {
            parent->GetExecutableParents(executableParents);
        }
    }
}

inline
void AdaptiveStateRepresentation::getParentIDs(
    int stateID,
    std::vector<int> &IDs) const
{
    for (const auto &parent : parents_) {
        IDs.push_back(parent->getID());
    }
}

inline
void AdaptiveStateRepresentation::getChildIDs(
    int stateID,
    std::vector<int> &IDs) const
{
    for (const auto &child : children_) {
        IDs.push_back(child->getID());
    }
}

} // namespace adim

#endif
