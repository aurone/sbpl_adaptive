/*
 * multirep_adaptive_3d.h
 *
 *  Created on: Mar 15, 2016
 *      Author: kalin
 */

#ifndef SRC_SBPL_ADAPTIVE_INCLUDE_SBPL_ADAPTIVE_DISCRETE_SPACE_INFORMATION_MULTIREP_ADAPTIVE_3D_MULTIREP_ADAPTIVE_3D_H_
#define SRC_SBPL_ADAPTIVE_INCLUDE_SBPL_ADAPTIVE_DISCRETE_SPACE_INFORMATION_MULTIREP_ADAPTIVE_3D_MULTIREP_ADAPTIVE_3D_H_

#include <sbpl_adaptive/headers.h>
#include <sbpl_adaptive/macros.h>

namespace adim {

SBPL_CLASS_FORWARD(MultiRepAdaptiveDiscreteSpaceInformation3D)

class MultiRepAdaptiveDiscreteSpaceInformation3D :
    public MultiRepAdaptiveDiscreteSpaceInformation
{
public:

    virtual int GetDimIDForPosition(Position3D p) = 0;

    virtual int GetTrackingCostToGoalForPosition(Position3D p) = 0;

    virtual void addSphere(const AdaptiveSphere3D_t &sphere) = 0;
};

} // namespace adim


#endif
