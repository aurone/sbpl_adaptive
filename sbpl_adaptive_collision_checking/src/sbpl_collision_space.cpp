/*
 * Copyright (c) 2011, Maxim Likhachev
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

/** \author Benjamin Cohen */

#include <sbpl_adaptive_collision_checking/sbpl_collision_space.h>

namespace sbpl_adaptive_collision_checking {

SBPLCollisionSpace::SBPLCollisionSpace(
    std::shared_ptr<sbpl_adaptive_collision_checking::SBPLCollisionModel> model,
    std::shared_ptr<sbpl_adaptive_collision_checking::OccupancyGrid> grid)
:
    model_(model), grid_(grid), padding_(0.0), contact_padding_(0.0)
{

}

SBPLCollisionSpace::~SBPLCollisionSpace()
{
}

void SBPLCollisionSpace::setPadding(double padding)
{
    padding_ = padding;
}

void SBPLCollisionSpace::setContactPadding(double padding)
{
    contact_padding_ = padding;
}

double SBPLCollisionSpace::isValidLineSegment(
    const std::vector<int> a,
    const std::vector<int> b,
    const int radius)
{
    bresenham3d_param_t params;
    int nXYZ[3], retvalue = 1;
    double cell_val, min_dist = 100.0;
    CELL3V tempcell;
    std::vector<CELL3V>* pTestedCells = NULL;

    //iterate through the points on the segment
    get_bresenham3d_parameters(a[0], a[1], a[2], b[0], b[1], b[2], &params);
    do {
        get_current_point3d(&params, &(nXYZ[0]), &(nXYZ[1]), &(nXYZ[2]));

        if (!grid_->isInBounds(nXYZ[0], nXYZ[1], nXYZ[2]))
            return 0;

        cell_val = grid_->getDistance(nXYZ[0], nXYZ[1], nXYZ[2]);
        if (cell_val <= radius) {
            if (pTestedCells == NULL)
                return cell_val;   //return 0
            else
                retvalue = 0;
        }

        if (cell_val < min_dist)
            min_dist = cell_val;

        //insert the tested point
        if (pTestedCells) {
            if (cell_val <= radius) {
                tempcell.bIsObstacle = true;
            }
            else {
                tempcell.bIsObstacle = false;
            }
            tempcell.x = nXYZ[0];
            tempcell.y = nXYZ[1];
            tempcell.z = nXYZ[2];
            pTestedCells->push_back(tempcell);
        }
    }
    while (get_next_point3d(&params));

    if (retvalue) {
        return min_dist;
    }
    else {
        return 0;
    }
}

bool SBPLCollisionSpace::checkCollision(
    const ModelCoords_t &coords,
    double &dist)
{
    int x, y, z;
    double sum = 0, dist_temp = 100;
    std::vector<Sphere> collision_spheres;

    if (!model_->getModelCollisionSpheres(coords, collision_spheres)) {
        ROS_ERROR("[cspace] Failed to get model spheres");
        return false;
    }

    for (Sphere s : collision_spheres) {
        grid_->worldToGrid(s.v.x(), s.v.y(), s.v.z(), x, y, z);
        if (!grid_->isInBounds(x, y, z)) {
            //ROS_WARN("Sphere %s out of bounds!", s.name_.c_str());
            dist = 0;
            return false;
        }
        double dist_bounds = grid_->getDistanceToBorder(x, y, z);
        dist_temp = std::min(grid_->getDistance(x, y, z), dist_bounds);
        if (dist_temp < dist) {
            dist = dist_temp;
        }
        if (dist_temp < s.radius + padding_) {
            //ROS_WARN("Cell: [%d %d %d] [%.3f %.3f %.3f]", x, y, z, s.v.x(), s.v.y(), s.v.z());
            //ROS_WARN("Sphere %s in collision! r=%.3f+p=%.3f > d=%.3f", s.name_.c_str(), s.radius, padding_, dist_temp);
            return false;
        }
    }
    return true;
}

bool SBPLCollisionSpace::checkContact(
    const ModelCoords_t &coords,
    const std::string &link_name,
    double &dist)
{
    std::vector<Sphere> contact_spheres;
    int x, y, z;
    double dist_temp = 0;
    if (!model_->getModelContactSpheres(coords, link_name, contact_spheres)) {
        ROS_ERROR("[cspace] Failed to get model spheres");
        return false;
    }

    for (Sphere s : contact_spheres) {
        grid_->worldToGrid(s.v.x(), s.v.y(), s.v.z(), x, y, z);
        double dist_bounds = grid_->getDistanceToBorder(x, y, z);
        dist_temp = std::min(grid_->getDistance(x, y, z), dist_bounds);
        if (dist_temp > dist) {
            dist = dist_temp;
        }
        if (dist_temp > s.radius + contact_padding_) {
            //ROS_WARN("Sphere %s for link %s not in contact! r=%.3f+p=%.3f < d=%.3f", s.name_.c_str(), link_name.c_str(), s.radius, contact_padding_, dist_temp);
            return false;
        }
    }
    //all contact spheres in contact!
    return true;
}

bool SBPLCollisionSpace::checkContact(const ModelCoords_t &coords, double &dist)
{
    int x, y, z;
    double sum = 0, dist_temp = 0;
    std::vector<Sphere> contact_spheres;

    if (!model_->getModelContactSpheres(coords, contact_spheres)) {
        ROS_ERROR("[cspace] Failed to get model spheres");
        return false;
    }

    for (Sphere s : contact_spheres) {
        grid_->worldToGrid(s.v.x(), s.v.y(), s.v.z(), x, y, z);
        dist_temp = grid_->getDistance(x, y, z);
        if (dist_temp > dist) {
            dist = dist_temp;
        }
        if (dist_temp > s.radius + contact_padding_) {
            //ROS_WARN("Sphere %s not in contact! r=%.3f+p=%.3f < d=%.3f", s.name_.c_str(), s.radius, contact_padding_, dist_temp);
            return false;
        }
    }
    return true;
}

bool SBPLCollisionSpace::checkCollision(
    const ModelCoords_t &coords0,
    const ModelCoords_t &coords1,
    int steps,
    double &dist)
{
    //TODO:
    int x, y, z;
    double sum = 0, dist_temp = 100;
    std::vector<Sphere> collision_spheres;

    if (!model_->getModelPathCollisionSpheres(coords0, coords1, steps,
            collision_spheres)) {
        ROS_ERROR("[cspace] Failed to get model spheres");
        return false;
    }

    for (Sphere s : collision_spheres) {
        grid_->worldToGrid(s.v.x(), s.v.y(), s.v.z(), x, y, z);
        dist_temp = grid_->getDistance(x, y, z);
        if (dist_temp < dist) {
            dist = dist_temp;
        }
        if (dist_temp < s.radius + padding_) {
            //ROS_WARN("Cell: [%d %d %d] [%.3f %.3f %.3f]", x, y, z, s.v.x(), s.v.y(), s.v.z());
            //ROS_WARN("Sphere %s in collision! r=%.3f+p=%.3f > d=%.3f", s.name_.c_str(), s.radius, padding_, dist_temp);
            return false;
        }
    }
    return true;
}

bool SBPLCollisionSpace::checkContact(
    const ModelCoords_t &coords0,
    const ModelCoords_t &coords1,
    int steps,
    double &dist)
{
    int x, y, z;
    double sum = 0, dist_temp = 0;
    std::vector<Sphere> contact_spheres;

    if (!model_->getModelPathContactSpheres(coords0, coords1, steps,
            contact_spheres)) {
        ROS_ERROR("[cspace] Failed to get model spheres");
        return false;
    }

    for (Sphere s : contact_spheres) {
        grid_->worldToGrid(s.v.x(), s.v.y(), s.v.z(), x, y, z);
        dist_temp = grid_->getDistance(x, y, z);
        if (dist_temp > dist) {
            dist = dist_temp;
        }
        if (dist_temp > s.radius + contact_padding_) {
            //ROS_WARN("Sphere %s not in contact! r=%.3f+p=%.3f < d=%.3f", s.name_.c_str(), s.radius, contact_padding_, dist_temp);
            return false;
        }
    }
    return true;
}

} // namespace sbpl_adaptive_collision_checking
