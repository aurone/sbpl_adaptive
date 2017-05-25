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
/** \author Kalin Gochev */

#ifndef _SBPL_COLLISION_MODEL_
#define _SBPL_COLLISION_MODEL_

#include <kdl/kdl.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotState.h>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_kdl.h>
#include <sbpl_adaptive_collision_checking/interpolation.h>

// standard includes
#include <vector>
#include <string>

// system includes
#include <smpl/forward.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sbpl_adaptive/core/graph/state.h>

// project includes
#include <sbpl_adaptive_collision_checking/common.h>

namespace adim {

SBPL_CLASS_FORWARD(SBPLCollisionModel);

/// \brief Represents the collision model of the robot used for planning.
class SBPLCollisionModel
{
public:

    virtual ~SBPLCollisionModel() { }

    virtual bool checkLimits(const ModelCoords &coords) const = 0;

    virtual bool getModelCollisionSpheres(
        const ModelCoords &coords,
        std::vector<Sphere> &spheres) const = 0;

    virtual bool getModelContactSpheres(
        const ModelCoords &coords,
        std::vector<Sphere> &spheres) const = 0;

    virtual bool getModelContactSpheres(
        const ModelCoords &coords,
        const std::string &link_name,
        std::vector<Sphere> &spheres) const = 0;

    virtual bool getModelPathCollisionSpheres(
        const ModelCoords &coords0,
        const ModelCoords &coords1,
        int steps,
        std::vector<Sphere> &spheres) const = 0;

    virtual bool getModelPathContactSpheres(
        const ModelCoords &coords0,
        const ModelCoords &coords1,
        int steps,
        std::vector<Sphere> &spheres) const = 0;

    // get just spheres visualization
    virtual visualization_msgs::MarkerArray getModelBasicVisualization(
        const ModelCoords &coords,
        std::string frame_id,
        std::string ns,
        std_msgs::ColorRGBA col,
        int &idx) const;

    visualization_msgs::Marker getSphereMarker(
        const Sphere &s,
        std::string ns,
        std::string frame_id,
        std_msgs::ColorRGBA col,
        int &id) const;

    // get more advanced mesh visualization when available
    virtual visualization_msgs::MarkerArray getModelVisualization(
        const ModelCoords &coords,
        const std::string &frame_id,
        const std::string &ns,
        const std_msgs::ColorRGBA &col,
        int &idx) const = 0;

protected:

    std::vector<Sphere> collision_spheres_;
    std::vector<Sphere> contact_spheres_;
};

} // namespace adim

#endif
