/*
 * sbpl_adaptive_grid.h
 *
 *  Created on: Feb 23, 2016
 *      Author: kalin
 */

#ifndef SBPL_ADAPTIVE_COMPONENTS_INCLUDE_SBPL_ADAPTIVE_COMPONENTS_SBPL_ADAPTIVE_GRID_H_
#define SBPL_ADAPTIVE_COMPONENTS_INCLUDE_SBPL_ADAPTIVE_COMPONENTS_SBPL_ADAPTIVE_GRID_H_

/*
 * sbpl_adaptive_grid3D.h
 *
 *  Created on: Nov 4, 2014
 *      Author: Kalin Gochev
 */

#include <vector>
#include <sbpl_adaptive/headers.h>

namespace sbpl_adaptive_components {

struct AdaptiveGridCell_t {
    int pDefaultDimID;
    int pDimID;
    int pNearDimID;
    int tDimID;
    int tNearDimID;
    unsigned int costToGoal;
};

class AdaptiveGrid
{
  protected:

    //double near_rad_;
    bool trackMode_;

    int ldID_;

    std::vector<int> grid_sizes_;

    std::vector<std::vector<int>> spheres_;

    virtual void resetTrackingGrid()=0;

    virtual void addTrackingSphere(const std::vector<int> &coords, int dimID, int rad, int near_rad, int costToGoal) = 0;

    virtual void setCellPlanningDim(const std::vector<int> &coord, int dimID) = 0;

    virtual void setCellTrackingDim(const std::vector<int> &coord, int dimID) = 0;

    virtual void setCellCostToGoal(const std::vector<int> &coord, unsigned int costToGoal) = 0;

    virtual void setDefaultDimID(const std::vector<int> &coord, int dimID) = 0;

  public:

    inline AdaptiveGrid(int ldID) { ldID_ = ldID; trackMode_ = false; };

    virtual ~AdaptiveGrid() {};

    inline bool isInPlanningMode(){
        return !trackMode_;
    }

    inline bool isInTrackingMode(){
        return trackMode_;
    }

    virtual void reset() = 0;

    virtual void init() = 0;

    virtual void clearAllSpheres() = 0;

    virtual void setPlanningMode() = 0;

    virtual void setTrackingMode(const std::vector<std::vector<int>> &tunnel_centers, const std::vector<int> &costsToGoal) = 0;

    virtual void addPlanningSphere(const std::vector<int> &coord, int dimID, int rad, int near_rad) = 0;

    virtual bool isInBounds(const std::vector<int> &coord) const = 0;

    virtual AdaptiveGridCell_t getCell(const std::vector<int> &coord) const = 0;

    virtual int getCellPlanningDim(const std::vector<int> &coord) const = 0;

    virtual int getCellTrackingDim(const std::vector<int> &coord) const = 0;

    virtual unsigned int getCellCostToGoal(const std::vector<int> &coord) const = 0;

};

}

#endif /* SBPL_ADAPTIVE_COMPONENTS_INCLUDE_SBPL_ADAPTIVE_COMPONENTS_SBPL_ADAPTIVE_GRID_H_ */
