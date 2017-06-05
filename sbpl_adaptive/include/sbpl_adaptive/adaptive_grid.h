#ifndef SBPL_ADAPTIVE_ADAPTIVE_GRID_H
#define SBPL_ADAPTIVE_ADAPTIVE_GRID_H

#include <tuple>

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

inline
bool operator==(const AdaptiveGridCell &a, const AdaptiveGridCell &b)
{
    return
        std::tie(a.pDefaultDimID, a.pDimID, a.pNearDimID, a.tDimID, a.tNearDimID, a.costToGoal) ==
        std::tie(b.pDefaultDimID, b.pDimID, b.pNearDimID, b.tDimID, b.tNearDimID, b.costToGoal);
}

} // namespace adim

#endif
