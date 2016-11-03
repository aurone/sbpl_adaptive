/// \author Kalin Gochev

#ifndef SBPL_ADAPTIVE_COMPONENTS_INCLUDE_SBPL_ADAPTIVE_COMPONENTS_SBPL_ADAPTIVE_GRID_H_
#define SBPL_ADAPTIVE_COMPONENTS_INCLUDE_SBPL_ADAPTIVE_COMPONENTS_SBPL_ADAPTIVE_GRID_H_

#include <vector>

#include <sbpl_adaptive/headers.h>
#include <sbpl_adaptive/macros.h>

namespace adim {

struct AdaptiveGridCell_t
{
    int pDefaultDimID;
    int pDimID;
    int pNearDimID;
    int tDimID;
    int tNearDimID;
    unsigned int costToGoal;
};

SBPL_CLASS_FORWARD(AdaptiveGrid)

class AdaptiveGrid
{
public:

    AdaptiveGrid(int ldID) : ldID_(ldID), trackMode_(false) { }

    virtual ~AdaptiveGrid() { }

    bool isInPlanningMode() { return !trackMode_; }

    bool isInTrackingMode() { return trackMode_; }

    virtual void reset() = 0;

    virtual void clearAllSpheres() = 0;

    virtual void setPlanningMode() = 0;

    virtual void setTrackingMode(
        const std::vector<std::vector<int>> &tunnel_centers,
        const std::vector<int> &costsToGoal) = 0;

    virtual void addPlanningSphere(
        const std::vector<int> &coord,
        int dimID,
        int rad,
        int near_rad) = 0;

    virtual bool isInBounds(const std::vector<int> &coord) const = 0;

    virtual AdaptiveGridCell_t getCell(const std::vector<int> &coord) const = 0;

    virtual int getCellPlanningDim(const std::vector<int> &coord) const = 0;

    virtual int getCellTrackingDim(const std::vector<int> &coord) const = 0;

    virtual unsigned int getCellCostToGoal(
        const std::vector<int> &coord) const = 0;

protected:

    //double near_rad_;
    bool trackMode_;

    int ldID_;

    virtual void resetTrackingGrid() = 0;

    virtual void addTrackingSphere(
        const std::vector<int> &coords,
        int dimID,
        int rad,
        int near_rad,
        int costToGoal) = 0;

    virtual void setCellPlanningDim(
        const std::vector<int> &coord,
        int dimID) = 0;

    virtual void setCellTrackingDim(
        const std::vector<int> &coord,
        int dimID) = 0;

    virtual void setCellCostToGoal(
        const std::vector<int> &coord,
        unsigned int costToGoal) = 0;

    virtual void setDefaultDimID(
        const std::vector<int> &coord,
        int dimID) = 0;
};

} // namespace adim

#endif
