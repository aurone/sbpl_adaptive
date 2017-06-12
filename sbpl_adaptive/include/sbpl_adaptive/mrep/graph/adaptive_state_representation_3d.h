#ifndef SBPL_ADAPTIVE_ADAPTIVE_STATE_REPRESENTATION_3D_H
#define SBPL_ADAPTIVE_ADAPTIVE_STATE_REPRESENTATION_3D_H

// standard includes
#include <string>
#include <vector>

// system includes
#include <smpl/forward.h>

// project includes
#include <sbpl_adaptive/common.h>
#include <sbpl_adaptive/mrep/graph/adaptive_state_representation.h>
#include <sbpl_adaptive/mrep/graph/multirep_adaptive_discrete_space_3d.h>

namespace adim {

SBPL_CLASS_FORWARD(AdaptiveStateRepresentation3D)

class AdaptiveStateRepresentation3D : public AdaptiveStateRepresentation
{
public:

    AdaptiveStateRepresentation3D(
        const MultiRepAdaptiveDiscreteSpace3DPtr &env,
        bool executable,
        const std::string &description);

    virtual ~AdaptiveStateRepresentation3D() { }

    virtual std::vector<AdaptiveSphere3D> getSpheresForState(int StateID) = 0;

    virtual std::vector<AdaptiveSphere3D> getUpgradeSpheresForState(
        int StateID) = 0;

    virtual void getTunnelSpheresForState(
        int StateID,
        int costToGoal,
        std::vector<AdaptiveSphere3D> &spheres) = 0;

    virtual void getTunnelSpheresForAction(
        int state_id,
        int succ_id,
        int cost_to_goal,
        std::vector<AdaptiveSphere3D> &spheres)
    {
        return getTunnelSpheresForState(state_id, cost_to_goal, spheres);
    }

    virtual std::vector<Position3D> getSpherePositionsForState(int StateID) = 0;

    const MultiRepAdaptiveDiscreteSpace3D *mrepSpace3D() const
    { return mrepSpace<MultiRepAdaptiveDiscreteSpace3D>(); }

    MultiRepAdaptiveDiscreteSpace3D *mrepSpace3D()
    { return mrepSpace<MultiRepAdaptiveDiscreteSpace3D>(); }
};

inline
AdaptiveStateRepresentation3D::AdaptiveStateRepresentation3D(
    const MultiRepAdaptiveDiscreteSpace3DPtr &env,
    bool executable,
    const std::string &description)
:
    AdaptiveStateRepresentation(env, executable, description)
{
}

} // namespace adim

#endif
