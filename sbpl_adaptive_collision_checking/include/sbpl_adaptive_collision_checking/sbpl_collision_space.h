/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** \author Benjamin Cohen */

#ifndef _SBPL_COLLISION_SPACE_
#define _SBPL_COLLISION_SPACE_

#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sbpl_adaptive_collision_checking/sbpl_collision_model.h>
#include <sbpl_geometry_utils/Interpolator.h>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <sbpl_geometry_utils/SphereEncloser.h>

#include <tf_conversions/tf_kdl.h>
#include <angles/angles.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <geometry_msgs/Point.h>
#include <angles/angles.h>

#include <sbpl_geometry_utils/interpolation.h>
#include <sbpl_geometry_utils/utils.h>
#include <sbpl_adaptive_collision_checking/occupancy_grid.h>
#include <sbpl_adaptive_collision_checking/bresenham.h>

namespace sbpl_adaptive_collision_checking {

class SBPLCollisionSpace
{
public:

    SBPLCollisionSpace(
        std::shared_ptr<SBPLCollisionModel> model,
        std::shared_ptr<OccupancyGrid> grid);

    ~SBPLCollisionSpace();

    void setPadding(double padding);
    void setContactPadding(double padding);

    /** --------------- Collision Checking ----------- */
    bool checkCollision(const ModelCoords_t &coords, double &dist);
    bool checkCollision(
        const ModelCoords_t &coords0,
        const ModelCoords_t &coords1,
        int steps,
        double &dist);

    bool checkContact(const ModelCoords_t &coords, double &dist);
    bool checkContact(
        const ModelCoords_t &coords,
        const std::string &link_name,
        double &dist);
    bool checkContact(
        const ModelCoords_t &coords0,
        const ModelCoords_t &coords1,
        int steps,
        double &dist);
    bool getContacts(
        const ModelCoords_t &coords,
        std::vector<std::string> &link_names);

    double getResolution();

    std::string getReferenceFrame();

    void getSize(int &dim_x, int &dim_y, int &dim_z);

    bool getModelVoxelsInGrid(
        const ModelCoords_t &coords,
        std::vector<Eigen::Vector3i> &voxels);

    std::shared_ptr<const SBPLCollisionModel> getModelPtr();

    bool isValidPoint(double x, double y, double z) const;

private:

    std::shared_ptr<SBPLCollisionModel> model_;
    std::shared_ptr<OccupancyGrid> grid_;

    double padding_;
    double contact_padding_;

    inline bool isValidCell(
        const int x,
        const int y,
        const int z,
        const int radius);
    double isValidLineSegment(
        const std::vector<int> a,
        const std::vector<int> b,
        const int radius);
    bool getClearance(
        const ModelCoords_t &coords,
        int num_spheres,
        double &avg_dist,
        double &min_dist);
};

inline
double SBPLCollisionSpace::getResolution()
{
    return grid_->getResolution();
}

inline
std::string SBPLCollisionSpace::getReferenceFrame()
{
    return grid_->getReferenceFrame();
}

inline
void SBPLCollisionSpace::getSize(int &dim_x, int &dim_y, int &dim_z)
{
    grid_->getGridSize(dim_x, dim_y, dim_z);
}

inline
bool SBPLCollisionSpace::getModelVoxelsInGrid(
    const ModelCoords_t &coords,
    std::vector<Eigen::Vector3i> &voxels)
{
    std::vector<Sphere> spheres;
    if (!model_->getModelCollisionSpheres(coords, spheres)) {
        return false;
    }
    if (!model_->getModelContactSpheres(coords, spheres)) {
        return false;
    }
    //1. find the extents of the spheres (axis-aligned bounding box)
    int min_x = 0, max_x = 0, dim_x = 0;
    int min_y = 0, max_y = 0, dim_y = 0;
    int min_z = 0, max_z = 0, dim_z = 0;
    grid_->getGridSize(dim_x, dim_y, dim_z);
    min_x = dim_x;
    min_y = dim_y;
    min_z = dim_z;
    for (Sphere s : spheres) {
        double mn_x = s.v.x() - s.radius;
        double mx_x = s.v.x() + s.radius;
        double mn_y = s.v.y() - s.radius;
        double mx_y = s.v.y() + s.radius;
        double mn_z = s.v.z() - s.radius;
        double mx_z = s.v.z() + s.radius;
        int mins[3];
        int maxs[3];
        grid_->worldToGrid(mn_x, mn_y, mn_z, mins[0], mins[1], mins[2]);
        grid_->worldToGrid(mx_x, mx_y, mx_z, maxs[0], maxs[1], maxs[2]);
        if (mins[0] < min_x) {
            min_x = mins[0];
        }
        if (mins[1] < min_y) {
            min_y = mins[1];
        }
        if (mins[2] < min_z) {
            min_z = mins[2];
        }
        if (maxs[0] > max_x) {
            min_x = mins[0];
        }
        if (maxs[1] > max_y) {
            min_y = mins[1];
        }
        if (maxs[2] > max_z) {
            min_z = mins[2];
        }
    }
    min_x = std::max(0, min_x);
    min_y = std::max(0, min_y);
    min_z = std::max(0, min_z);
    max_x = std::min(dim_x, max_x);
    max_y = std::min(dim_y, max_y);
    max_z = std::min(dim_z, max_z);
    //2. loop through all the voxels in the bounding box and check if in sphere
    for (int x = min_x; x <= max_x; x++) {
        for (int y = min_y; y <= max_y; y++) {
            for (int z = min_z; z <= max_z; z++) {
                bool bIn = false;
                double wx, wy, wz;
                grid_->gridToWorld(x, y, z, wx, wy, wz);
                Eigen::Vector3d v(wx, wy, wz);
                Eigen::Vector3i vi(x, y, z);
                for (Sphere s : spheres) {
                    Eigen::Vector3d d = s.v - v;
                    if (d.norm() <= s.radius) {
                        bIn = true;
                        break;
                    }
                }
                if (bIn) {
                    voxels.push_back(vi);
                }
            }
        }
    }
    return true;
}

inline
std::shared_ptr<const SBPLCollisionModel> SBPLCollisionSpace::getModelPtr()
{
    std::shared_ptr<const SBPLCollisionModel> ptr = model_;
    return ptr;
}

inline
bool SBPLCollisionSpace::isValidPoint(double x, double y, double z) const
{
    int gx, gy, gz;
    grid_->worldToGrid(x, y, z, gx, gy, gz);
    if (grid_->isInBounds(gx, gy, gz)) {
        if (grid_->getDistance(gx, gy, gz) > grid_->getResolution()) {
            return true;
        }
    }
    return false;
}

inline
bool SBPLCollisionSpace::isValidCell(
    const int x,
    const int y,
    const int z,
    const int radius)
{
    if (grid_->getCell(x, y, z) <= radius)
        return false;
    return true;
}

} // namespace sbpl_adaptive_collision_checking

#endif

