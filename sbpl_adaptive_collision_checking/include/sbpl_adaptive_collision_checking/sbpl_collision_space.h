#ifndef _SBPL_COLLISION_SPACE_
#define _SBPL_COLLISION_SPACE_

// standard includes
#include <cmath>
#include <vector>

// system includes
#include <angles/angles.h>
#include <geometry_msgs/Point.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <ros/ros.h>
#include <smpl/forward.h>
#include <smpl/occupancy_grid.h>
#include <sbpl_geometry_utils/bounding_spheres.h>
#include <sbpl_geometry_utils/interpolate.h>
#include <sbpl_geometry_utils/utils.h>
#include <sbpl_geometry_utils/voxelize.h>
#include <tf_conversions/tf_kdl.h>

// project includes
#include <sbpl_adaptive_collision_checking/sbpl_collision_model.h>

namespace adim {

SBPL_CLASS_FORWARD(SBPLCollisionSpace)

class SBPLCollisionSpace
{
public:

    SBPLCollisionSpace(
        SBPLCollisionModelPtr model,
        const sbpl::OccupancyGridPtr& grid);

    ~SBPLCollisionSpace();

    void setPadding(double padding);
    void setContactPadding(double padding);

    /// \name Collision Detection
    ///@{
    bool checkCollision(const ModelCoords &coords, double &dist);

    bool checkCollision(
        const ModelCoords &coords0,
        const ModelCoords &coords1,
        int steps,
        double &dist);
    ///@}

    /// \name Contact Detection
    ///@{
    bool checkContact(const ModelCoords &coords, double &dist);

    bool checkContact(
        const ModelCoords &coords,
        const std::string &link_name,
        double &dist);

    bool checkContact(
        const ModelCoords &coords0,
        const ModelCoords &coords1,
        int steps,
        double &dist);
    ///@}

    double getResolution();

    std::string getReferenceFrame();

    void getSize(int &dim_x, int &dim_y, int &dim_z);

    bool getModelVoxelsInGrid(
        const ModelCoords &coords,
        std::vector<Eigen::Vector3i> &voxels);

    SBPLCollisionModelConstPtr getModelPtr() const;

    bool isValidPoint(double x, double y, double z) const;

private:

    SBPLCollisionModelPtr model_;
    sbpl::OccupancyGridPtr grid_;

    double padding_;
    double contact_padding_;

    double isValidLineSegment(
        const std::vector<int>& a,
        const std::vector<int>& b,
        const int radius);
    bool getClearance(
        const ModelCoords &coords,
        int num_spheres,
        double &avg_dist,
        double &min_dist);
};

inline
double SBPLCollisionSpace::getResolution()
{
    return grid_->resolution();
}

inline
std::string SBPLCollisionSpace::getReferenceFrame()
{
    return grid_->getReferenceFrame();
}

inline
void SBPLCollisionSpace::getSize(int &dim_x, int &dim_y, int &dim_z)
{
    dim_x = grid_->numCellsX();
    dim_y = grid_->numCellsY();
    dim_z = grid_->numCellsZ();
}

inline
SBPLCollisionModelConstPtr SBPLCollisionSpace::getModelPtr() const
{
    SBPLCollisionModelConstPtr ptr = model_;
    return ptr;
}

inline
bool SBPLCollisionSpace::isValidPoint(double x, double y, double z) const
{
    int gx, gy, gz;
    grid_->worldToGrid(x, y, z, gx, gy, gz);
    if (grid_->isInBounds(gx, gy, gz)) {
        if (grid_->getDistance(gx, gy, gz) > grid_->resolution()) {
            return true;
        }
    }
    return false;
}

} // namespace adim

#endif

