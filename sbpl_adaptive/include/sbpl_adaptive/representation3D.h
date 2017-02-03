/*
 * representation.h
 *
 *  Created on: Feb 8, 2016
 *      Author: kalin
 */

#ifndef SBPL_ADAPTIVE_ADAPTIVE_STATE_REPRESENTATION_3D_H
#define SBPL_ADAPTIVE_ADAPTIVE_STATE_REPRESENTATION_3D_H

// standard includes
#include <string>
#include <vector>

// system includes
#include <smpl/forward.h>

// project includes
#include <sbpl_adaptive/adaptive_state_representation.h>
#include <sbpl_adaptive/common.h>
#include <sbpl_adaptive/discrete_space_information/multirep_adaptive_3d/multirep_adaptive_3d.h>

namespace adim {

SBPL_CLASS_FORWARD(AdaptiveStateRepresentation3D)

class AdaptiveStateRepresentation3D : public AdaptiveStateRepresentation
{
public:

    AdaptiveStateRepresentation3D(
        const MultiRepAdaptiveDiscreteSpaceInformation3DPtr &env,
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

    virtual std::vector<Position3D> getSpherePositionsForState(int StateID) = 0;

protected:

    MultiRepAdaptiveDiscreteSpaceInformation3DPtr env3d_;
};

inline
AdaptiveStateRepresentation3D::AdaptiveStateRepresentation3D(
    const MultiRepAdaptiveDiscreteSpaceInformation3DPtr &env,
    bool executable,
    const std::string &description)
:
    AdaptiveStateRepresentation(env, executable, description),
    env3d_(env)
{

}

} // namespace adim

#endif
