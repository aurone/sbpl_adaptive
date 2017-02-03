/*
 * multirep_adaptive_3d.h
 *
 *  Created on: Mar 15, 2016
 *      Author: kalin
 */

#ifndef SBPL_ADAPTIVE_MULTIREP_ADAPTIVE_3D_H
#define SBPL_ADAPTIVE_MULTIREP_ADAPTIVE_3D_H

// standard includes
#include <vector>

// system includes
#include <smpl/forward.h>

// projects includes
#include <sbpl_adaptive/common.h>
#include <sbpl_adaptive/discrete_space_information/multirep_adaptive_environment.h>

namespace adim {

SBPL_CLASS_FORWARD(MultiRepAdaptiveDiscreteSpaceInformation3D)

class MultiRepAdaptiveDiscreteSpaceInformation3D :
    public MultiRepAdaptiveDiscreteSpaceInformation
{
public:

    virtual bool IsDimEnabledAtPosition(const Position3D &p, int dim) const = 0;

    virtual void GetEnabledDimsAtPosition(
        const Position3D &p,
        std::vector<int> &dims) const = 0;

    virtual int GetTrackingCostToGoalForPosition(Position3D p) = 0;

    virtual void addSphere(const AdaptiveSphere3D &sphere) = 0;
};

} // namespace adim

#endif
