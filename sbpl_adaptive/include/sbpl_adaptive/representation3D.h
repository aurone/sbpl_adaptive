/*
 * representation.h
 *
 *  Created on: Feb 8, 2016
 *      Author: kalin
 */

#ifndef _ADAPTIVE_REPRESENTATION3D_H_
#define _ADAPTIVE_REPRESENTATION3D_H_

namespace sbpl_adaptive {

class MultiRepAdaptiveDiscreteSpaceInformation3D;

class AdaptiveStateRepresentation3D_t : public sbpl_adaptive::AdaptiveStateRepresentation_t {
public:
    inline AdaptiveStateRepresentation3D_t(std::shared_ptr<MultiRepAdaptiveDiscreteSpaceInformation3D> env, bool executable, std::string description) :
        AdaptiveStateRepresentation_t(std::shared_ptr<MultiRepAdaptiveDiscreteSpaceInformation>(env.get()), executable, description),
        env3d_(env)
    {

    }

    virtual inline ~AdaptiveStateRepresentation3D_t() { };

    virtual std::vector<AdaptiveSphere3D_t> getSpheresForState(int StateID) = 0;
    virtual std::vector<AdaptiveSphere3D_t> getUpgradeSpheresForState(int StateID) = 0;
    virtual std::vector<AdaptiveSphere3D_t> getTunnelSpheresForState(int StateID, int costToGoal) = 0;

    virtual std::vector<Position3D_t> getSpherePositionsForState(int StateID) = 0;

protected:
    std::shared_ptr<MultiRepAdaptiveDiscreteSpaceInformation3D> env3d_;
};

}


#endif /* ADAPTIVE_PLANNING_SBPL_HUMANOID_PLANNER_INCLUDE_REPRESENTATION_H_ */
