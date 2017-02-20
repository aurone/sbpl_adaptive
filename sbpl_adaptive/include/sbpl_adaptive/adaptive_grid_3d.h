/*
 * adaptive_grid_3d.h
 *
 *  Created on: Nov 4, 2014
 *      Author: Kalin Gochev
 */

#ifndef SBPL_ADAPTIVE_ADAPTIVE_GRID_3D_H
#define SBPL_ADAPTIVE_ADAPTIVE_GRID_3D_H

// standard includes
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <string>
#include <vector>

// system includes
#include <geometry_msgs/Point.h>
#include <sbpl/sbpl_exception.h>
#include <sbpl/utils/key.h>
#include <smpl/occupancy_grid.h>
#include <smpl/forward.h>
#include <smpl/grid.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_adaptive/common.h>
#include <sbpl_adaptive/adaptive_grid.h>

namespace adim {

#define ADAPTIVE_GRID_EXCLUSIVE 0

SBPL_CLASS_FORWARD(AdaptiveGrid3D)

/// A representation of the current state of an adaptive graph with respect to
/// enabled regions mapped to three-dimensional grid cells. Each grid cell
/// models three concepts:
///
/// (1) Whether a state from some representation, mapped to this cell, with no
///     explicit regions added, is enabled during the search.
///
/// (2) Whether a state from some representation, mapped to this cell, with
///     regions added as the result of previously failed tracking iterations,
///     is enabled during the search
///
/// (3) Whether a state from some representation, mapped to this cell, with
///     regions added as the result of previously failed tracking iterations
///     or constructed as a tunnel around a path found during a previous
///     planning iteration, is enabled during the search
///
/// A state enabled by default is always enabled during both planning and
/// tracking iterations.
///
/// A state not enabled by default, may be enabled during a planning iteration
/// and will then also be enabled during a succeeding tracking iteration.
///
/// A state not enabled by default and not enabled during a previous planning
/// iteration, may be enabled during a tracking iteration.
class AdaptiveGrid3D
{
public:

    static const int InvalidDim = 0;

    AdaptiveGrid3D(const sbpl::OccupancyGridPtr& grid);

    bool isInPlanningMode() const { return !trackMode_; }
    bool isInTrackingMode() const { return trackMode_; }

    /// \name (Voxel) Grid Functionality
    ///@{
    void getDimensions(int &sizeX, int &sizeY, int &sizeZ) const;

    bool isInBounds(int gx, int gy, int gz) const;
    bool isInBounds(double wx, double wy, double wz) const;

    const AdaptiveGridCell &getCell(int gx, int gy, int gz) const;
    const AdaptiveGridCell &getCell(double wx, double wy, double wz) const;

    double resolution() const { return oc_grid_->getResolution(); }

    void world2grid(
        double wx, double wy, double wz,
        size_t &gx, size_t &gy, size_t &gz) const;

    void grid2world(
        size_t gx, size_t gy, size_t gz,
        double& wx, double& wy, double& wz) const;
    ///@}

    /// \name Base functionality
    ///@{
    bool enableDimDefault(int gx, int gy, int gz, int dimID);
    bool disableDimDefault(int gx, int gy, int gz, int dimID);

    bool enableDimPlanning(int gx, int gy, int gz, int dimID);
    bool disableDimPlanning(int gx, int gy, int gz, int dimID);

    bool enableDimTracking(int gx, int gy, int gz, int dimID);
    bool disableDimTracking(int gx, int gy, int gz, int dimID);

    bool enableNearDimPlanning(int gx, int gy, int gz, int dimID);
    bool disableNearDimPlanning(int gx, int gy, int gz, int dimID);

    bool enableNearDimTracking(int gx, int gy, int gz, int dimID);
    bool disableNearDimTracking(int gx, int gy, int gz, int dimID);

    bool dimEnabledPlanning(int gx, int gy, int gz, int dimID) const;
    bool dimEnabledTracking(int gx, int gy, int gz, int dimID) const;
    ///@}

    /// \name Wrappers around base functionality
    ///@{
    bool enableDim(int gx, int gy, int gz, int dimID, bool tracking);
    bool disableDim(int gx, int gy, int gz, int dimID, bool tracking);

    bool enableNearDim(int gx, int gy, int gz, int dimID, bool tracking);
    bool disableNearDim(int gx, int gy, int gz, int dimID, bool tracking);

    bool dimEnabled(int gx, int gy, int gz, int dimID, bool tracking) const;
    ///@}

    void addPlanningSphere(
        const AdaptiveSphere3D &sphere,
        std::vector<Position3D> &modCells);

    void setPlanningMode();

    void setTrackingMode(
        const std::vector<AdaptiveSphere3D> &tunnel,
        std::vector<Position3D> &modCells);

    unsigned int getCellCostToGoal(double wx, double wy, double wz) const;

    void reset();

    unsigned int getCellCostToGoal(int gx, int gy, int gz) const;

    /// \name Visualization
    ///@{
    visualization_msgs::MarkerArray getVisualizations(
        std::string ns_prefix,
        int throttle = 1,
        double scale = 1.0);

    visualization_msgs::Marker getAdaptiveGridVisualization(
        std::string ns_prefix,
        int throttle = 1,
        double scale = 1.0);

    visualization_msgs::Marker getCostToGoalGridVisualization(
        std::string ns_prefix,
        int throttle = 1,
        double scale = 1.0);
    ///@}

private:

    bool trackMode_;

    std::vector<int> grid_sizes_;
    std::vector<std::vector<int>> spheres_;

    // used to keep track of state type (LD, NearLD, HD)
    sbpl::Grid3<AdaptiveGridCell> grid_;
    AdaptiveGridCell invalid_cell_;

    int max_dimID_;
    unsigned int max_costToGoal_;

    sbpl::OccupancyGridPtr oc_grid_;

    void clearAllSpheres();

    void getBoundaryVisualizationPoints(
        std::vector<geometry_msgs::Point> &points,
        std::vector<std_msgs::ColorRGBA> &colors) const;

    void addSphere(
        bool tracking,
        size_t x,
        size_t y,
        size_t z,
        int rad,
        int near_rad,
        int dimID,
        unsigned int costToGoal,
        std::vector<Position3D> &modCells);

    bool setCellNearDim(
        bool tracking,
        size_t x,
        size_t y,
        size_t z,
        int dimID);

    void addTrackingSphere(
        const AdaptiveSphere3D &sphere,
        std::vector<Position3D> &modCells);

    void getOverlappingSpheres(
        size_t x, size_t y, size_t z,
        int dimID,
        std::vector<std::vector<int>> &spheres);

    void resetTrackingGrid();

    void setCellCostToGoal(int gx, int gy, int gz, unsigned int costToGoal);
};

/// Also enables the planning bit for the representation.
inline
bool AdaptiveGrid3D::enableDimDefault(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

#if ADAPTIVE_GRID_EXCLUSIVE
    grid_(gx, gy, gz).pDefaultDimID = InvalidDim;
#endif

    int prev_dims = grid_(gx, gy, gz).pDefaultDimID;
    grid_(gx, gy, gz).pDefaultDimID |= (1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);

    return (grid_(gx, gy, gz).pDefaultDimID != prev_dims) |
            enableDimPlanning(gx, gy, gz, dimID);
}

/// Also disables the planning bit for the representation.
inline
bool AdaptiveGrid3D::disableDimDefault(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_(gx, gy, gz).pDefaultDimID;
    grid_(gx, gy, gz).pDefaultDimID &= ~(1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return (grid_(gx, gy, gz).pDefaultDimID != prev_dims) |
            disableDimDefault(gx, gy, gz, dimID);
}

/// Also enables the tracking bit for the representation.
inline
bool AdaptiveGrid3D::enableDimPlanning(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

#if ADAPTIVE_GRID_EXCLUSIVE
    grid_(gx, gy, gz).pDimID = InvalidDim;
#endif

    int prev_dims = grid_(gx, gy, gz).pDimID;
    grid_(gx, gy, gz).pDimID |= (1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return grid_(gx, gy, gz).pDimID != prev_dims |
            enableDimTracking(gx, gy, gz, dimID);
}

/// Also disables the tracking bit for the representation.
inline
bool AdaptiveGrid3D::disableDimPlanning(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_(gx, gy, gz).pDimID;
    grid_(gx, gy, gz).pDimID &= ~(1 << dimID);
    return grid_(gx, gy, gz).pDimID != prev_dims |
            disableDimTracking(gx, gy, gz, dimID);
}

inline
bool AdaptiveGrid3D::enableDimTracking(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

#if ADAPTIVE_GRID_EXCLUSIVE
    grid_(gx, gy, gz).tDimID = InvalidDim;
#endif

    int prev_dims = grid_(gx, gy, gz).tDimID;
    grid_(gx, gy, gz).tDimID |= (1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return grid_(gx, gy, gz).tDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::disableDimTracking(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_(gx, gy, gz).tDimID;
    grid_(gx, gy, gz).tDimID &= ~(1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return grid_(gx, gy, gz).tDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::enableNearDimPlanning(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_(gx, gy, gz).pNearDimID;
    grid_(gx, gy, gz).pNearDimID |= (1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return grid_(gx, gy, gz).pNearDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::disableNearDimPlanning(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_(gx, gy, gz).pNearDimID;
    grid_(gx, gy, gz).pNearDimID &= ~(1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return grid_(gx, gy, gz).pNearDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::enableNearDimTracking(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_(gx, gy, gz).tNearDimID;
    grid_(gx, gy, gz).tNearDimID |= (1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return grid_(gx, gy, gz).tNearDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::disableNearDimTracking(int gx, int gy, int gz, int dimID)
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }

    int prev_dims = grid_(gx, gy, gz).tNearDimID;
    grid_(gx, gy, gz).tNearDimID &= ~(1 << dimID);
    max_dimID_ = std::max(max_dimID_, dimID);
    return grid_(gx, gy, gz).tNearDimID != prev_dims;
}

inline
bool AdaptiveGrid3D::enableNearDim(
    int gx,
    int gy,
    int gz,
    int dimID,
    bool tracking)
{
    if (tracking) {
        return enableNearDimTracking(gx, gy, gz, dimID);
    }
    else {
        return enableNearDimPlanning(gx, gy, gz, dimID);
    }
}

inline
bool AdaptiveGrid3D::disableNearDim(
    int gx,
    int gy,
    int gz,
    int dimID,
    bool tracking)
{
    if (tracking) {
        return disableNearDimTracking(gx, gy, gz, dimID);
    }
    else {
        return disableNearDimPlanning(gx, gy, gz, dimID);
    }
}

inline
bool AdaptiveGrid3D::enableDim(int gx, int gy, int gz, int dimID, bool tracking)
{
    if (tracking) {
        return enableDimTracking(gx, gy, gz, dimID);
    }
    else {
        return enableDimPlanning(gx, gy, gz, dimID);
    }
}

inline
bool AdaptiveGrid3D::disableDim(
    int gx,
    int gy,
    int gz,
    int dimID,
    bool tracking)
{
    if (tracking) {
        return disableDimTracking(gx, gy, gz, dimID);
    }
    else {
        return disableDimPlanning(gx, gy, gz, dimID);
    }
}

inline
bool AdaptiveGrid3D::dimEnabledPlanning(int gx, int gy, int gz, int dimID) const
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }
    return grid_(gx, gy, gz).pDimID & (1 << dimID);
}

inline
bool AdaptiveGrid3D::dimEnabledTracking(int gx, int gy, int gz, int dimID) const
{
    if (!isInBounds(gx, gy, gz)) {
        return false;
    }
    return grid_(gx, gy, gz).tDimID & (1 << dimID);
}

inline
bool AdaptiveGrid3D::dimEnabled(
    int gx,
    int gy,
    int gz,
    int dimID,
    bool tracking) const
{
    if (tracking) {
        return dimEnabledTracking(gx, gy, gz, dimID);
    }
    else {
        return dimEnabledPlanning(gx, gy, gz, dimID);
    }
}

inline
void AdaptiveGrid3D::addPlanningSphere(
    const AdaptiveSphere3D &sphere,
    std::vector<Position3D> &modCells)
{
    size_t gx, gy, gz;
    world2grid(sphere.x,sphere.y,sphere.z,gx,gy,gz);
    int r = round(sphere.rad / oc_grid_->getResolution());
    int nr = round(sphere.near_rad / oc_grid_->getResolution());
    addSphere(false, gx, gy, gz, r, nr, sphere.dimID, INFINITECOST, modCells);
}

inline
void AdaptiveGrid3D::getDimensions(int &sizeX, int &sizeY, int &sizeZ) const
{
    sizeX = grid_sizes_[0];
    sizeY = grid_sizes_[1];
    sizeZ = grid_sizes_[2];
}

inline
const AdaptiveGridCell &AdaptiveGrid3D::getCell(int gx, int gy, int gz) const
{
    if (!isInBounds(gx, gy, gz)) {
        return invalid_cell_;
    }
    else {
        return grid_(gx, gy, gz);
    }
}

inline
const AdaptiveGridCell &AdaptiveGrid3D::getCell(double wx, double wy, double wz) const
{
    size_t gcoordx, gcoordy, gcoordz;
    world2grid(wx, wy, wz, gcoordx, gcoordy, gcoordz);
    return getCell((int)gcoordx, (int)gcoordy, (int)gcoordz);
}

inline
unsigned int AdaptiveGrid3D::getCellCostToGoal(double wx, double wy, double wz) const
{
    size_t gcoordx, gcoordy, gcoordz;
    world2grid(wx, wy, wz, gcoordx, gcoordy, gcoordz);
    return getCellCostToGoal((int)gcoordx, (int)gcoordy, (int)gcoordz);
}

inline
bool AdaptiveGrid3D::isInBounds(int gx, int gy, int gz) const
{
    return oc_grid_->isInBounds(gx, gy, gz);
}

inline
bool AdaptiveGrid3D::isInBounds(double wx, double wy, double wz) const
{
    size_t gx, gy, gz;
    world2grid(wx, wy, wz, gx, gy, gz);
    return isInBounds((int)gx, (int)gy, (int)gz);
}

inline
void AdaptiveGrid3D::addTrackingSphere(
    const AdaptiveSphere3D &sphere,
    std::vector<Position3D> &modCells)
{
    size_t gx, gy, gz;
    world2grid(sphere.x, sphere.y, sphere.z, gx, gy, gz);
    int r = round(sphere.rad / oc_grid_->getResolution());
    int nr = round(sphere.near_rad / oc_grid_->getResolution());
    addSphere(true, gx, gy, gz, r, nr, sphere.dimID, sphere.costToGoal, modCells);
}

} // namespace adim

#endif
