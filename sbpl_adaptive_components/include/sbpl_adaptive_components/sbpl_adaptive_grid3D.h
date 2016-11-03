/*
 * sbpl_adaptive_grid3D.h
 *
 *  Created on: Nov 4, 2014
 *      Author: Kalin Gochev
 */

#ifndef _SBPL_ADAPTIVE_GRID3D_H_
#define _SBPL_ADAPTIVE_GRID3D_H_

// system includes
#include <leatherman/utils.h>
#include <ros/ros.h>
#include <sbpl_adaptive/macros.h>
#include <smpl/occupancy_grid.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_adaptive_components/sbpl_adaptive_grid.h>

namespace adim {

SBPL_CLASS_FORWARD(AdaptiveGrid3D)

class AdaptiveGrid3D : public AdaptiveGrid
{
public:

    AdaptiveGrid3D(const sbpl::OccupancyGridPtr& grid, int ldID);

    ~AdaptiveGrid3D();

    void setDefaultDimID(const std::vector<int> &coords, int dimID);

    void setTrackingMode(
        const std::vector<std::vector<int>> &tunnel_centers,
        const std::vector<int> &costsToGoal,
        std::vector<adim::Position3D> &modCells);

    void setTrackingMode(
        const std::vector<adim::AdaptiveSphere3D_t> &tunnel,
        std::vector<adim::Position3D> &modCells);

    void addPlanningSphere(
        const std::vector<int> &coord,
        int dimID,
        int rad,
        int near_rad,
        std::vector<adim::Position3D> &modCells);

    void addPlanningSphere(
        adim::AdaptiveSphere3D_t sphere,
        std::vector<adim::Position3D> &modCells);

    AdaptiveGridCell_t getCell(double wx, double wy, double wz) const;

    int getCellPlanningDim(double wx, double wy, double wz) const;

    unsigned int getCellCostToGoal(double wx, double wy, double wz) const;

    int getCellTrackingDim(double wx, double wy, double wz) const;

    int getCellDim(bool bTrackMode, size_t x, size_t y, size_t z) const;

    void setVisualizationReferenceFrame(std::string frm);

    void getDimensions(int &sizeX, int &sizeY, int &sizeZ) const;

    double getResolution() const { return oc_grid_->getResolution(); }

    void world2grid(
        double wx, double wy, double wz,
        size_t& gx, size_t& gy, size_t& gz) const;

    void grid2world(
        size_t gx, size_t gy, size_t gz,
        double& wx, double& wy, double& wz) const;

    visualization_msgs::MarkerArray getVisualizations(
        std::string ns_prefix,
        int throttle = 1,
        double scale = -1);

    visualization_msgs::Marker getAdaptiveGridVisualization(
        std::string ns_prefix,
        int throttle = 1,
        double scale = -1);

    visualization_msgs::Marker getCostToGoalGridVisualization(
        std::string ns_prefix,
        int throttle = 1,
        double scale = 1);

    void visualize(std::string ns_prefix);

    /// \name Required Public Functions From AdaptiveGrid
    ///@{

    void reset();

    void init();

    void clearAllSpheres();

    void setPlanningMode();

    void setTrackingMode(
        const std::vector<std::vector<int>> &tunnel_centers,
        const std::vector<int> &costsToGoal);

    void addPlanningSphere(
        const std::vector<int> &coord,
        int dimID,
        int rad,
        int near_rad);

    bool isInBounds(const std::vector<int> &coord) const;

    AdaptiveGridCell_t getCell(const std::vector<int> &gcoord) const;

    int getCellPlanningDim(const std::vector<int> &gcoord) const;

    int getCellTrackingDim(const std::vector<int> &gcoord) const;

    unsigned int getCellCostToGoal(const std::vector<int> &coord) const;

    ///@}

private:

    std::vector<int> grid_sizes_;
    std::vector<std::vector<int>> spheres_;

    // used to keep track of state type (LD, NearLD, HD)
    std::vector<std::vector<std::vector<AdaptiveGridCell_t>>> grid_;

    std::string frame_;

    int max_dimID_;
    unsigned int max_costToGoal_;

    sbpl::OccupancyGridPtr oc_grid_;

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Publisher marker_array_publisher_;
    ros::Publisher marker_publisher_;

    static double getDist2(int x1, int y1, int z1, int x2, int y2, int z2);

    static double getDist(int x1, int y1, int z1, int x2, int y2, int z2);

    void addSphere(bool bTrackMode, size_t x, size_t y, size_t z, int rad, int near_rad, int dimID, unsigned int costToGoal, std::vector<adim::Position3D> &modCells);

    bool setCellDim(bool bTrackMode, size_t x, size_t y, size_t z, int dimID);

    bool setCellNearDim(bool bTrackMode, size_t x, size_t y, size_t z, int dimID);

    void addTrackingSphere(
        const std::vector<int> &coords,
        int dimID,
        int rad,
        int near_rad,
        int costToGoal,
        std::vector<adim::Position3D> &modCells);

    void addTrackingSphere(
        adim::AdaptiveSphere3D_t sphere,
        std::vector<adim::Position3D> &modCells);

    void getOverlappingSpheres(
        size_t x, size_t y, size_t z,
        int dimID,
        std::vector<std::vector<int>> &spheres);

    /// \name Required Protected Functions From AdaptiveGrid
    ///@{

    void resetTrackingGrid();

    void addTrackingSphere(
        const std::vector<int> &coords,
        int dimID,
        int rad,
        int near_rad,
        int costToGoal);

    void setCellPlanningDim(const std::vector<int> &coord, int dimID);

    void setCellTrackingDim(const std::vector<int> &coord, int dimID);

    void setCellCostToGoal(
        const std::vector<int> &coord,
        unsigned int costToGoal);

    ///@}
};

inline
void AdaptiveGrid3D::setDefaultDimID(const std::vector<int> &coords, int dimID)
{
    if (isInBounds({coords[0], coords[1], coords[2]})) {
        grid_[coords[0]][coords[1]][coords[2]].pDefaultDimID = dimID;
        grid_[coords[0]][coords[1]][coords[2]].pDimID = dimID;
        max_dimID_ = std::max(max_dimID_, dimID);
    }
}

inline
void AdaptiveGrid3D::addPlanningSphere(
    const std::vector<int> &coord,
    int dimID,
    int rad,
    int near_rad,
    std::vector<adim::Position3D> &modCells)
{
    addSphere(false, coord[0], coord[1], coord[2], rad, near_rad, dimID, INFINITECOST, modCells);
}

inline
void AdaptiveGrid3D::addPlanningSphere(
    adim::AdaptiveSphere3D_t sphere,
    std::vector<adim::Position3D> &modCells)
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
AdaptiveGridCell_t AdaptiveGrid3D::getCell(double wx, double wy, double wz) const
{
    size_t gcoordx, gcoordy, gcoordz;
    world2grid(wx,wy,wz,gcoordx,gcoordy,gcoordz);
    return this->getCell({(int)gcoordx, (int)gcoordy, (int)gcoordz});
}

inline
int AdaptiveGrid3D::getCellPlanningDim(double wx, double wy, double wz) const
{
    size_t gcoordx, gcoordy, gcoordz;
    world2grid(wx,wy,wz,gcoordx,gcoordy,gcoordz);
    return this->getCellPlanningDim({(int)gcoordx, (int)gcoordy, (int)gcoordz});
}

inline
unsigned int AdaptiveGrid3D::getCellCostToGoal(double wx, double wy, double wz) const
{
    size_t gcoordx, gcoordy, gcoordz;
    world2grid(wx,wy,wz,gcoordx,gcoordy,gcoordz);
    return this->getCellCostToGoal({(int)gcoordx, (int)gcoordy, (int)gcoordz});
}

inline
int AdaptiveGrid3D::getCellTrackingDim(double wx, double wy, double wz) const
{
    size_t gcoordx, gcoordy, gcoordz;
    world2grid(wx,wy,wz,gcoordx,gcoordy,gcoordz);
    return this->getCellTrackingDim({(int)gcoordx, (int)gcoordy, (int)gcoordz});
}

inline
void AdaptiveGrid3D::setTrackingMode(
    const std::vector<std::vector<int>> &tunnel_centers,
    const std::vector<int> &costsToGoal)
{
    std::vector<adim::Position3D> modCells;
    setTrackingMode(tunnel_centers, costsToGoal, modCells);
}

inline
void AdaptiveGrid3D::addPlanningSphere(
    const std::vector<int> &coord,
    int dimID,
    int rad,
    int near_rad)
{
    std::vector<adim::Position3D> modCells;
    addSphere(false, coord[0], coord[1], coord[2], rad, near_rad, dimID, INFINITECOST, modCells);
}

inline
bool AdaptiveGrid3D::isInBounds(const std::vector<int> &coord) const
{
    return oc_grid_->isInBounds(coord[0], coord[1], coord[2]);
}

inline
void AdaptiveGrid3D::visualize(std::string ns_prefix)
{
    marker_array_publisher_.publish(getVisualizations(ns_prefix));
}

inline
void AdaptiveGrid3D::addTrackingSphere(
    const std::vector<int> &coords,
    int dimID,
    int rad,
    int near_rad,
    int costToGoal,
    std::vector<adim::Position3D> &modCells)
{
    addSphere(true, coords[0], coords[1], coords[2], rad, near_rad, dimID, costToGoal, modCells);
}

inline
void AdaptiveGrid3D::addTrackingSphere(
    adim::AdaptiveSphere3D_t sphere,
    std::vector<adim::Position3D> &modCells)
{
    size_t gx, gy, gz;
    world2grid(sphere.x,sphere.y,sphere.z,gx,gy,gz);
    int r = round(sphere.rad / oc_grid_->getResolution());
    int nr = round(sphere.near_rad / oc_grid_->getResolution());
    addSphere(true, gx, gy, gz, r, nr, sphere.dimID, sphere.costToGoal, modCells);
}

inline
void AdaptiveGrid3D::addTrackingSphere(
    const std::vector<int> &coords,
    int dimID,
    int rad,
    int near_rad,
    int costToGoal)
{
    std::vector<adim::Position3D> modCells;
    addSphere(true, coords[0], coords[1], coords[2], rad, near_rad, dimID, costToGoal, modCells);
}

} // namespace adim

#endif
