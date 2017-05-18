#ifndef SBPL_ADAPTIVE_MULTIREP_ADAPTIVE_DISCRETE_SPACE_3D_H
#define SBPL_ADAPTIVE_MULTIREP_ADAPTIVE_DISCRETE_SPACE_3D_H

// standard includes
#include <vector>

// system includes
#include <smpl/forward.h>

// projects includes
#include <sbpl_adaptive/common.h>
#include <sbpl_adaptive/mrep/graph/multirep_adaptive_discrete_space.h>

namespace adim {

SBPL_CLASS_FORWARD(MultiRepAdaptiveDiscreteSpace3D)

class MultiRepAdaptiveDiscreteSpace3D : public MultiRepAdaptiveDiscreteSpace
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
