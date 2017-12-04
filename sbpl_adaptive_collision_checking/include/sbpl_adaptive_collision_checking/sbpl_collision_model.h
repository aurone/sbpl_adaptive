/// \author Kalin Gochev

#ifndef SBPL_ADAPTIVE_COLLISION_CHECKING_SBPL_COLLISION_MODEL_H
#define SBPL_ADAPTIVE_COLLISION_CHECKING_SBPL_COLLISION_MODEL_H

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
#include <sbpl_adaptive_collision_checking/interpolation.h>

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
