/// \author Kalin Gochev

#ifndef SBPL_ADAPTIVE_ADAPTIVE_GRID_H
#define SBPL_ADAPTIVE_ADAPTIVE_GRID_H

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
