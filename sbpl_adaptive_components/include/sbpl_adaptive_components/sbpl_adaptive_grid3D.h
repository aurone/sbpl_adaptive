/*
 * sbpl_adaptive_grid3D.h
 *
 *  Created on: Nov 4, 2014
 *      Author: Kalin Gochev
 */

#ifndef _SBPL_ADAPTIVE_GRID3D_H_
#define _SBPL_ADAPTIVE_GRID3D_H_

#include <sbpl_adaptive_components/sbpl_adaptive_grid.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <sbpl_adaptive_collision_checking/occupancy_grid.h>

namespace sbpl_adaptive_components
{

inline static void HSVtoRGB( double *r, double *g, double *b, double h, double s, double v ){
    int i;
    double f, p, q, t;
    if( s == 0 ) {
        // achromatic (grey)
        *r = *g = *b = v;
        return;
    }
    h /= 60;        // sector 0 to 5
    i = floor(h);
    f = h - i;          // factorial part of h
    p = v * ( 1 - s );
    q = v * ( 1 - s * f );
    t = v * ( 1 - s * ( 1 - f ) );
    switch( i ) {
        case 0:
            *r = v;
            *g = t;
            *b = p;
            break;
        case 1:
            *r = q;
            *g = v;
            *b = p;
            break;
        case 2:
            *r = p;
            *g = v;
            *b = t;
            break;
        case 3:
            *r = p;
            *g = q;
            *b = v;
            break;
        case 4:
            *r = t;
            *g = p;
            *b = v;
            break;
                default:
            *r = v;
            *g = p;
            *b = q;
            break;
    }
}

inline static std_msgs::ColorRGBA fromHSV(double h, double s, double v){
    double r, g, b;
    HSVtoRGB(&r, &g, &b, h, s, v);
    std_msgs::ColorRGBA col;
    col.r = r;
    col.g = g;
    col.b = b;
    col.a = 1.0;
    return col;
}

class AdaptiveGrid3D_t : public AdaptiveGrid_t
{
  private:

    std::vector< std::vector< std::vector<AdaptiveGridCell_t> > > grid_; //used to keep track of state type (LD, NearLD, HD)

    std::string frame_;

    int max_dimID_;
    unsigned int max_costToGoal_;

    std::shared_ptr<sbpl_adaptive_collision_checking::OccupancyGrid> oc_grid_;

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Publisher marker_array_publisher_;
    ros::Publisher marker_publisher_;

    static double getDist2(int x1, int y1, int z1, int x2, int y2, int z2);

    static double getDist(int x1, int y1, int z1, int x2, int y2, int z2);

    void addSphere(bool bTrackMode, size_t x, size_t y, size_t z, int rad, int near_rad, int dimID, unsigned int costToGoal, std::vector<sbpl_adaptive::Position3D_t> &modCells);

    bool setCellDim(bool bTrackMode, size_t x, size_t y, size_t z, int dimID);

    bool setCellNearDim(bool bTrackMode, size_t x, size_t y, size_t z, int dimID);

    /* pure virtual functions from AdaptiveGrid_t */

    void resetTrackingGrid();

    inline void addTrackingSphere(const std::vector<int> &coords, int dimID, int rad, int near_rad, int costToGoal){
        std::vector<sbpl_adaptive::Position3D_t> modCells;
        addSphere(true, coords[0], coords[1], coords[2], rad, near_rad, dimID, costToGoal, modCells);
    }

    inline void addTrackingSphere(const std::vector<int> &coords, int dimID, int rad, int near_rad, int costToGoal, std::vector<sbpl_adaptive::Position3D_t> &modCells){
        addSphere(true, coords[0], coords[1], coords[2], rad, near_rad, dimID, costToGoal, modCells);
    }

    inline void addTrackingSphere(sbpl_adaptive::AdaptiveSphere3D_t sphere, std::vector<sbpl_adaptive::Position3D_t> &modCells){
        size_t gx, gy, gz;
        world2grid(sphere.x,sphere.y,sphere.z,gx,gy,gz);
        int r = round(sphere.rad / oc_grid_->getResolution());
        int nr = round(sphere.near_rad / oc_grid_->getResolution());
        addSphere(true, gx, gy, gz, r, nr, sphere.dimID, sphere.costToGoal, modCells);
    }

    void setCellPlanningDim(const std::vector<int> &coord, int dimID);

    void setCellTrackingDim(const std::vector<int> &coord, int dimID);

    void setCellCostToGoal(const std::vector<int> &coord, unsigned int costToGoal);

    /* end pure virtual */

    void getOverlappingSpheres(size_t x, size_t y, size_t z, int dimID, std::vector<std::vector<int>> &spheres);

  public:

    inline void setDefaultDimID(const std::vector<int> &coords, int dimID)
    {
        if(isInBounds({coords[0], coords[1], coords[2]})){
            grid_[coords[0]][coords[1]][coords[2]].pDefaultDimID = dimID;
            grid_[coords[0]][coords[1]][coords[2]].pDimID = dimID;
            max_dimID_ = std::max(max_dimID_, dimID);
        }
    }

    /* pure virtual functions from AdaptiveGrid_t */
    void reset();

    void init();

    void clearAllSpheres();

    void setPlanningMode();

    inline void setTrackingMode(
        const std::vector<std::vector<int>> &tunnel_centers,
        const std::vector<int> &costsToGoal)
    {
        std::vector<sbpl_adaptive::Position3D_t> modCells;
        setTrackingMode(tunnel_centers, costsToGoal, modCells);
    }

    void setTrackingMode(
        const std::vector<std::vector<int>> &tunnel_centers,
        const std::vector<int> &costsToGoal,
        std::vector<sbpl_adaptive::Position3D_t> &modCells);

    void setTrackingMode(
        const std::vector<sbpl_adaptive::AdaptiveSphere3D_t> &tunnel,
        std::vector<sbpl_adaptive::Position3D_t> &modCells);

    inline void addPlanningSphere(const std::vector<int> &coord, int dimID, int rad, int near_rad)
    {
        std::vector<sbpl_adaptive::Position3D_t> modCells;
        addSphere(false, coord[0], coord[1], coord[2], rad, near_rad, dimID, INFINITECOST, modCells);
    }

    inline void addPlanningSphere(
        const std::vector<int> &coord,
        int dimID,
        int rad,
        int near_rad,
        std::vector<sbpl_adaptive::Position3D_t> &modCells)
    {
        addSphere(false, coord[0], coord[1], coord[2], rad, near_rad, dimID, INFINITECOST, modCells);
    }

    inline bool isInBounds(const std::vector<int> &coord) const {
        return oc_grid_->isInBounds(coord[0], coord[1], coord[2]);
    }

    AdaptiveGridCell_t getCell(const std::vector<int> &gcoord) const;

    int getCellPlanningDim(const std::vector<int> &gcoord) const;

    int getCellTrackingDim(const std::vector<int> &gcoord) const;

    unsigned int getCellCostToGoal(const std::vector<int> &coord) const;

    /* end pure virtual functions from AdaptiveGrid_t */

    inline void addPlanningSphere(sbpl_adaptive::AdaptiveSphere3D_t sphere, std::vector<sbpl_adaptive::Position3D_t> &modCells){
        size_t gx, gy, gz;
        world2grid(sphere.x,sphere.y,sphere.z,gx,gy,gz);
        int r = round(sphere.rad / oc_grid_->getResolution());
        int nr = round(sphere.near_rad / oc_grid_->getResolution());
        addSphere(false, gx, gy, gz, r, nr, sphere.dimID, INFINITECOST, modCells);
    }

    inline AdaptiveGridCell_t getCell(double wx, double wy, double wz) const {
        size_t gcoordx, gcoordy, gcoordz;
        world2grid(wx,wy,wz,gcoordx,gcoordy,gcoordz);
        return this->getCell({(int)gcoordx, (int)gcoordy, (int)gcoordz});
    }

    inline int getCellPlanningDim(double wx, double wy, double wz) const {
        size_t gcoordx, gcoordy, gcoordz;
        world2grid(wx,wy,wz,gcoordx,gcoordy,gcoordz);
        return this->getCellPlanningDim({(int)gcoordx, (int)gcoordy, (int)gcoordz});
    }

    inline unsigned int getCellCostToGoal(double wx, double wy, double wz) const {
        size_t gcoordx, gcoordy, gcoordz;
        world2grid(wx,wy,wz,gcoordx,gcoordy,gcoordz);
        return this->getCellCostToGoal({(int)gcoordx, (int)gcoordy, (int)gcoordz});
    }

    inline int getCellTrackingDim(double wx, double wy, double wz) const {
        size_t gcoordx, gcoordy, gcoordz;
        world2grid(wx,wy,wz,gcoordx,gcoordy,gcoordz);
        return this->getCellTrackingDim({(int)gcoordx, (int)gcoordy, (int)gcoordz});
    }

    AdaptiveGrid3D_t(std::shared_ptr<sbpl_adaptive_collision_checking::OccupancyGrid> grid, int ldID);

    ~AdaptiveGrid3D_t();

    int getCellDim(bool bTrackMode, size_t x, size_t y, size_t z) const;

    void setVisualizationReferenceFrame(std::string frm);

    inline void getDimensions(int &sizeX, int &sizeY, int &sizeZ) const {
        sizeX = grid_sizes_[0];
        sizeY = grid_sizes_[1];
        sizeZ = grid_sizes_[2];
    }

    inline double getResolution() const {
        return oc_grid_->getResolution();
    }

    void world2grid(double wx, double wy, double wz, size_t& gx, size_t& gy, size_t& gz) const;

    void grid2world(size_t gx, size_t gy, size_t gz, double& wx, double& wy, double& wz) const;

    visualization_msgs::MarkerArray getVisualizations(std::string ns_prefix, int throttle=1, double scale=-1);
    visualization_msgs::Marker getAdaptiveGridVisualization(std::string ns_prefix, int throttle=1, double scale=-1);
    visualization_msgs::Marker getCostToGoalGridVisualization(std::string ns_prefix, int throttle=1, double scale=-1);

    inline void visualize(std::string ns_prefix){
    	marker_array_publisher_.publish(getVisualizations(ns_prefix));
    }

};

}

#endif /* SBPL_ADAPTIVE_MANIPULATION_SBPL_ADAPTIVE_ARM_PLANNER_INCLUDE_SBPL_ADAPTIVE_GRID3D_H_ */
