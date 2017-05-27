#include <sbpl_adaptive/mrep/graph/adaptive_state_representation.h>

// system includes
#include <ros/console.h>

namespace adim {

/// \class AdaptiveStateRepresentation
///
////Base class defining the interface to a discrete graph representation used in
/// the multi-representation adaptive dimensionality planning framework.
///
/// Each subclass of AdaptiveStateRepresentation is responsible for creating
/// states on demand and inserting them into the combined multi-representation
/// graph representation. A child class may choose to instantiate subclasses
/// of AdaptiveState and safely assume that input states and states
/// corresponding to input state ids are of the same type as instantiated by the
/// representation, and may be safely downcasted. However, to support debugging,
/// the function state_cast should be used to allow run-time type checking of
/// states in debug mode. Since no run-time type information is present in
/// release mode, the representation is responsible for destroying each state
/// it constructs.
///
/// The edge transitions between states in an AdaptiveStateRepresentation have
/// an implicit property labeling their executability. This corresponds to
/// whether an action may be executed be a corresponding controller. Both
/// executable and non-executable actions may be used to generate successors in
/// calls to GetSuccs (and GetPreds), but only executable actions may be used to
/// generate successors in GetTrackSuccs.
///
/// Each AdaptiveStateRepresentation may contain multiple parent and child
/// representations, forming a representation graph.

AdaptiveStateRepresentation::AdaptiveStateRepresentation(
    const MultiRepAdaptiveDiscreteSpacePtr &env,
    bool executable,
    const std::string &description)
:
    env_(env),
    dimID_(-1),
    executable_(executable),
    name_(description)
{
    sphere_radius_ = 1.0;
    near_radius_ = 1.0;
    tunnel_radius_ = 1.0;
}

AdaptiveStateRepresentation::~AdaptiveStateRepresentation()
{
}

void AdaptiveStateRepresentation::addParentRepresentation(
    const AdaptiveStateRepresentationPtr &parent)
{
    auto it = std::find(parents_.begin(), parents_.end(), parent);
    if (it == parents_.end()) {
        ROS_INFO("Added %s as parent representation to %s!", parent->getName().c_str(), getName().c_str());
        parents_.push_back(parent);
        parent->addChildRepresentation(shared_from_this());
    }
}

void AdaptiveStateRepresentation::addChildRepresentation(
    const AdaptiveStateRepresentationPtr &child)
{
    auto it = std::find(children_.begin(), children_.end(), child);
    if (it == children_.end()) {
        ROS_INFO("Added %s as child representation to %s!", child->getName().c_str(), getName().c_str());
        children_.push_back(child);
        child->addParentRepresentation(shared_from_this());
    }
}

/// If executable, returns this representation, otherwise return the executable
/// parents of its parents.
void AdaptiveStateRepresentation::GetExecutableParents(
    std::vector<const AdaptiveStateRepresentation *> &executableParents) const
{
    if (isExecutable()) {
        executableParents.push_back(this);
    }
    for (const auto &parent : parents_) {
        parent->GetExecutableParents(executableParents);
    }
}


} // namespace adim
