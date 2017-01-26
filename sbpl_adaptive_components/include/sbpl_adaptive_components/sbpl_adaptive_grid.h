/// \author Kalin Gochev

#ifndef SBPL_ADAPTIVE_COMPONENTS_INCLUDE_SBPL_ADAPTIVE_COMPONENTS_SBPL_ADAPTIVE_GRID_H_
#define SBPL_ADAPTIVE_COMPONENTS_INCLUDE_SBPL_ADAPTIVE_COMPONENTS_SBPL_ADAPTIVE_GRID_H_

#include <vector>

#include <sbpl_adaptive/headers.h>
#include <sbpl_adaptive/macros.h>

namespace adim {

struct AdaptiveGridCell
{
    int pDefaultDimID;
    int pDimID;
    int pNearDimID;
    int tDimID;
    int tNearDimID;
    unsigned int costToGoal;
};

} // namespace adim

#endif
