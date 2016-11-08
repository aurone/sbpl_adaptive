/*
 * spherical.h
 *
 *  Created on: Mar 15, 2016
 *      Author: kalin
 */

#ifndef SRC_SBPL_COLLISION_CHECKING_INCLUDE_SBPL_COLLISION_CHECKING_SIMPLE_SPHERICAL_H_
#define SRC_SBPL_COLLISION_CHECKING_INCLUDE_SBPL_COLLISION_CHECKING_SIMPLE_SPHERICAL_H_

#include <sbpl_adaptive_collision_checking/interpolation.h>
#include <sbpl_adaptive_collision_checking/sbpl_collision_model.h>

namespace adim {

struct SphericalModelCoords_t : public ModelCoords
{
    double x, y, z;
};

class SBPLSphericalCollisionModel : public SBPLCollisionModel
{
public:

    SBPLSphericalCollisionModel(double rad);

    ~SBPLSphericalCollisionModel();

    bool getModelCollisionSpheres(
        const SphericalModelCoords_t &c,
        std::vector<Sphere> &spheres) const;

    bool getModelPathCollisionSpheres(
        const SphericalModelCoords_t &c0,
        const SphericalModelCoords_t &c1,
        int steps,
        std::vector<Sphere> &spheres) const;


    /// \name Reimplemented Public Functions
    ///@{
    bool checkLimits(const ModelCoords &coords) const override;

    bool getModelCollisionSpheres(
        const ModelCoords &coords,
        std::vector<Sphere> &spheres) const override;

    bool getModelContactSpheres(
        const ModelCoords &coords,
        std::vector<Sphere> &spheres) const override;

    bool getModelPathCollisionSpheres(
        const ModelCoords &coords0,
        const ModelCoords &coords1,
        int steps,
        std::vector<Sphere> &spheres) const override;

    bool getModelPathContactSpheres(
        const ModelCoords &coords0,
        const ModelCoords &coords1,
        int steps,
        std::vector<Sphere> &spheres) const override;

    visualization_msgs::MarkerArray getModelVisualization(
        const ModelCoords &coords,
        const std::string &frame_id,
        const std::string &ns,
        const std_msgs::ColorRGBA &col) const override;
    ///@}

private:

    double myrad_;
};

inline
SBPLSphericalCollisionModel::SBPLSphericalCollisionModel(double rad)
{
    myrad_ = rad;
}

inline
SBPLSphericalCollisionModel::~SBPLSphericalCollisionModel()
{

}

inline
bool SBPLSphericalCollisionModel::checkLimits(const ModelCoords &coords) const
{
    return true;
}

inline
bool SBPLSphericalCollisionModel::getModelCollisionSpheres(
    const ModelCoords &coords,
    std::vector<Sphere> &spheres) const
{
    const SphericalModelCoords_t &c =
            dynamic_cast<const SphericalModelCoords_t&>(coords);
    return getModelCollisionSpheres(c, spheres);
}

inline
bool SBPLSphericalCollisionModel::getModelCollisionSpheres(
    const SphericalModelCoords_t &c,
    std::vector<Sphere> &spheres) const
{
    Sphere s;
    s.v[0] = c.x;
    s.v[1] = c.y;
    s.v[2] = c.z;
    s.radius = myrad_;
    s.link_name_ = "root";
    s.name_ = "spherical-model-main-sphere";
    spheres.push_back(s);
    return true;
}

inline
bool SBPLSphericalCollisionModel::getModelContactSpheres(
    const ModelCoords &coords,
    std::vector<Sphere> &spheres) const
{
    return true;
}

inline
bool SBPLSphericalCollisionModel::getModelPathCollisionSpheres(
    const ModelCoords &coords0,
    const ModelCoords &coords1,
    int steps,
    std::vector<Sphere> &spheres) const
{
    const SphericalModelCoords_t &c0 =
            dynamic_cast<const SphericalModelCoords_t&>(coords0);
    const SphericalModelCoords_t &c1 =
            dynamic_cast<const SphericalModelCoords_t&>(coords1);
    return getModelPathCollisionSpheres(c0, c1, steps, spheres);
}

inline
bool SBPLSphericalCollisionModel::getModelPathCollisionSpheres(
    const SphericalModelCoords_t &c0,
    const SphericalModelCoords_t &c1,
    int steps,
    std::vector<Sphere> &spheres) const
{
    for (int i = 0; i < steps; i++) {
        double t = i / (double)(steps - 1);
        Sphere s;
        s.v[0] = LERP(c0.x, c1.x, t);
        s.v[1] = LERP(c0.y, c1.y, t);
        s.v[2] = LERP(c0.z, c1.z, t);
        s.radius = myrad_;
        s.link_name_ = "root";
        s.name_ = "spherical-model-main-sphere";
        spheres.push_back(s);
    }
    return true;
}

inline
bool SBPLSphericalCollisionModel::getModelPathContactSpheres(
    const ModelCoords &coords0,
    const ModelCoords &coords1,
    int steps,
    std::vector<Sphere> &spheres) const
{
    return true;
}

inline
visualization_msgs::MarkerArray
SBPLSphericalCollisionModel::getModelVisualization(
    const ModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    const std_msgs::ColorRGBA &col) const
{
    return getModelBasicVisualization(coords, frame_id, ns, col);
}

} // namespace adim

#endif /* SRC_SBPL_COLLISION_CHECKING_INCLUDE_SBPL_COLLISION_CHECKING_SIMPLE_SPHERICAL_H_ */
