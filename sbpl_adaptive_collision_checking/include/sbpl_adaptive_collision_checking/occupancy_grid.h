/*
 * Copyright (c) 2010, Maxim Likhachev
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
 *     * Neither the name of the University of Pennsylvania nor the names of its
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

/* \author Benjamin Cohen */

#ifndef _OCCUPANCY_GRID_
#define _OCCUPANCY_GRID_

#include <cmath>
#include <fstream>
#include <vector>

#include <eigen_conversions/eigen_kdl.h>
#include <moveit/distance_field/voxel_grid.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <ros/console.h>
#include <sys/stat.h>
#include <tf/LinearMath/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <sbpl_adaptive_collision_checking/common.h>

namespace sbpl_adaptive_collision_checking {

/**
 * @brief At this point, this is a very lightweight layer on top of the
 * PropagationDistanceField class. I'll eventually get rid of it once the
 * PDF class has a couple of more things in it.
 */
class OccupancyGrid
{
public:

    /**
     * @brief Constructor
     * @param dim_x Dimension of the grid along the X axis, in meters
     * @param dim_y Dimension of the grid along the Y axis, in meters
     * @param dim_z Dimension of the grid along the Z axis, in meters
     * @param resolution Resolution of the grid, in meters
     * @param origin_x X Coordinate of origin, in meters
     * @param origin_y Y Coordinate of origin, in meters
     * @param origin_z Z Coordinate of origin, in meters
     * @param max_dist The maximum distance away from obstacles to propogate the distance field, in meters
     */
    OccupancyGrid(
        double dim_x, double dim_y, double dim_z,
        double resolution,
        double origin_x, double origin_y, double origin_z,
        double max_dist);

    /**
     * @brief Construct an OccupancyGrid with an externally managed distance field.
     * @param df A pointer to the externally managed distance field
     */
    OccupancyGrid(distance_field::PropagationDistanceField* df);

    ~OccupancyGrid();

    /** @brief Return a pointer to the distance field */
    inline distance_field::PropagationDistanceField* getDistanceFieldPtr() const { return grid_.get(); }

    /** @name Attributes */
    /**@{*/

    /** @brief Get the dimensions of the grid, in cells */
    void getGridSize(int &dim_x, int &dim_y, int &dim_z) const;

    /** @brief Get the dimensions of the world, in meters */
    void getWorldSize(double &dim_x, double &dim_y, double &dim_z) const;

    /** @brief Get the origin of the world, in meters */
    void getOrigin(double &wx, double &wy, double &wz) const;

    /** @brief Get the resolution of the world, in meters */
    double getResolution() const { return grid_->getResolution(); }
    double getMaxDistance() const { return grid_->getUninitializedDistance(); }

    /** @brief Convert grid cell coords into world coords*/
    void gridToWorld(int x, int y, int z, double &wx, double &wy, double &wz) const
    {
        grid_->gridToWorld(x, y, z, wx, wy, wz);
    }

    /** @brief Convert world coords into grid cell coords*/
    void worldToGrid(double wx, double wy, double wz, int &x, int &y, int &z) const
    {
        grid_->worldToGrid(wx, wy, wz, x, y, z);

        /*if ((x > 10000) || (y > 10000) || (z > 10000) || (x < 0) || (y < 0) || (z < 0)) {
            ROS_ERROR("[grid] worldToGrid converted %0.5f %0.5f %0.5f to %d %d %d", wx, wy, wz, x, y, z);
            fflush(stdout);
        }*/
    }

    const std::string& getReferenceFrame() const { return reference_frame_; }
    void setReferenceFrame(const std::string &frame) { reference_frame_ = frame; }

    /**@}*/

    /** @name Grid Cell Accessors */
    /**@{*/

    /** @brief Get the cell's distance to the nearest obstacle in cells*/
    unsigned char getCell(int x, int y, int z) const
    {
        return (unsigned char)(grid_->getDistance(x,y,z) / grid_->getResolution());
    }

    /** @brief Get the cell's distance to the nearest obstacle in meters*/
    double getCell(int *xyz) const
    {
        return grid_->getDistance(xyz[0],xyz[1],xyz[2]);
    }

    /** @sa PropagationDistanceField::getDistance(int, int, int) */
    inline double getDistanceToBorder(int x, int y, int z) const {
        if(!isInBounds(x,y,z)) return 0;
        int dx = std::min(grid_->getXNumCells() - x, x);
        int dy = std::min(grid_->getYNumCells() - y, y);
        int dz = std::min(grid_->getZNumCells() - z, z);
        return grid_->getResolution() * std::min(dx, std::min(dy, dz));
    }

    double getDistance(int x, int y, int z) const { return grid_->getDistance(x,y,z); }

    /** @sa PropagationDistanceField::getDistance(double, double, double) */
    double getDistanceFromPoint(double x, double y, double z) const
    {
        int gx, gy, gz;
        worldToGrid(x, y, z, gx, gy, gz);
        if(!isInBounds(gx, gy, gz)) return 0;
        return grid_->getDistance(gx, gy, gz);
    }

    /** @brief Return whether the grid(x,y,z) is in bounds of the grid */
    bool isInBounds(int x, int y, int z) const
    {
        return (x>=0 && x<grid_->getXNumCells() && y>=0 && y<grid_->getYNumCells() && z>=0 && z<grid_->getZNumCells());
    }

    void getOccupiedVoxels(std::vector<geometry_msgs::Point> &voxels) const;

    void getOccupiedVoxels(
            const geometry_msgs::Pose &pose,
            const std::vector<double> &dim,
            std::vector<Eigen::Vector3d> &voxels) const;

    void getOccupiedVoxels(
            double x_center,
            double y_center,
            double z_center,
            double radius,
            std::string text,
            std::vector<geometry_msgs::Point> &voxels) const;

    /**@}*/

    /** @name Modifiers */
    /**@{*/

    void addPointsToField(const std::vector<Eigen::Vector3d> &points);
    void updatePointsInField(const std::vector<Eigen::Vector3d> &points, bool iterative = false);

    /** @sa PropagationDistanceField::reset */
    void reset();

    /**@}*/

    /**
     * @brief Return a visualization of the distance field.
     *
     * The visualization_msgs::MarkerArray's contents vary depending on the argument:
     *     "bounds": line markers for the bounding box of the distance field in the namespace "collision_space_bounds"
     *     "distance_field": cube markers for all voxels nearby occupied voxels in the namespace "distance_field"
     *     "occupied_voxels" list of points representing all occupied voxels in the namespace "occupied voxels"
     *
     * @param type "bounds", "distance_field", "occupied_voxels"
     */
    visualization_msgs::MarkerArray getVisualization(std::string type);

private:

    std::string reference_frame_;
    boost::shared_ptr<distance_field::PropagationDistanceField> grid_;
};

} // namespace sbpl_adaptive_collision_checking

#endif
