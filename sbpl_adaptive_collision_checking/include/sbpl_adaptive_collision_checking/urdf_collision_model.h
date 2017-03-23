/*
 * Copyright (c) 2011, Kalin Gochev
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
 *     * Neither the name of the copyright holder nor the names of its
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

#ifndef SBPL_ADAPTIVE_COLLISION_CHECKING_URDF_COLLISION_MODEL_H
#define SBPL_ADAPTIVE_COLLISION_CHECKING_URDF_COLLISION_MODEL_H

// standard includes
#include <map>
#include <memory>
#include <utility>

// system includes
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <sbpl/sbpl_exception.h>
#include <urdf/model.h>

#include <sbpl_adaptive_collision_checking/sbpl_collision_model.h>

namespace adim {

struct URDFModelCoords : ModelCoords
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Affine3d root;
    std::map<std::string, std::vector<double>> coordmap;
    std::vector<std::string> collision_links;
    std::vector<std::string> contact_links;

    bool getCoords(
        const std::string &name,
        std::vector<double> &p_coords) const;

    bool getCoord(const std::string &name, double &p_coord, int idx = 0) const;

    double getCoord(const std::string &name, int idx = 0) const;

    void set(const std::string &name, const std::vector<double> &p_coords);

    static void updateWith(
        URDFModelCoords &target,
        const URDFModelCoords &source);

    static double getMaxJointDistance(
        const URDFModelCoords &from,
        const URDFModelCoords &to);
};

std::ostream &operator<<(std::ostream &o, const URDFModelCoords &coords);

struct AttachedObject
{
    std::string name;
    std::vector<Sphere> spheres;
};

class URDFCollisionModel : public SBPLCollisionModel
{
public:

    URDFCollisionModel();
    ~URDFCollisionModel();

    URDFModelCoords getRandomCoordinates() const;

    URDFModelCoords getDefaultCoordinates() const;

    virtual bool initFromURDF(
        const std::string &urdf_string,
        const std::string &srdf_string);

    virtual bool initFromParam(const std::string &robot_desc_param_name);

    virtual bool computeSpheresFromURDFModel(
        double res,
        const std::vector<std::string> &ignore_collision_links,
        const std::vector<std::string> &contact_links);

    virtual void autoIgnoreSelfCollisions();
    virtual void autoIgnoreSelfCollisions(const URDFModelCoords &coords);

    void printIgnoreSelfCollisionLinkPairs();

    bool checkLimits(const URDFModelCoords &coords) const;

    bool checkSelfCollisions(const URDFModelCoords &coords) const;

    std::vector<std::pair<std::string, std::string>> getSelfCollisions(
        const URDFModelCoords &coords) const;

    bool getModelCollisionSpheres(
        const URDFModelCoords &coords,
        std::vector<Sphere> &spheres) const;

    bool getModelContactSpheres(
        const URDFModelCoords &coords,
        std::vector<Sphere> &spheres) const;

    bool getModelPathCollisionSpheres(
        const URDFModelCoords &coords0,
        const URDFModelCoords &coords1,
        int steps,
        std::vector<Sphere> &spheres) const;

    bool getModelPathContactSpheres(
        const URDFModelCoords &coords0,
        const URDFModelCoords &coords1,
        int steps,
        std::vector<Sphere> &spheres) const;

    virtual bool getInterpolatedCoordinates(
        const URDFModelCoords &coords0,
        const URDFModelCoords &coords1,
        double t,
        URDFModelCoords &interp) const;

    virtual bool getInterpolatedPath(
        const URDFModelCoords &coords0,
        const URDFModelCoords &coords1,
        double resolution,
        std::vector<URDFModelCoords> &path) const;

    virtual bool getInterpolatedPath(
        const URDFModelCoords &coords0,
        const URDFModelCoords &coords1,
        double resolution,
        std::vector<URDFModelCoords> &path,
        int max_depth) const;

    virtual bool getInterpolatedPath(
        const URDFModelCoords &coords0,
        const URDFModelCoords &coords1,
        int steps,
        std::vector<URDFModelCoords> &path) const;

    void addContactSpheres(
        const std::string &link_name,
        const std::vector<Sphere> &s);

    void addCollisionSpheres(
        const std::string &link_name,
        const std::vector<Sphere> &s);

    void addContactSphere(const std::string &link_name, Sphere s);

    void addCollisionSphere(const std::string &link_name, Sphere s);

    // get more advanced mesh visualization when available
    visualization_msgs::MarkerArray getModelSelfCollisionVisualization(
        const URDFModelCoords &coords,
        const std::string &frame_id,
        const std::string &ns,
        const std_msgs::ColorRGBA &col,
        int &idx) const;

    visualization_msgs::MarkerArray getModelBasicVisualizationByLink(
        const URDFModelCoords &coords,
        const std::string &frame_id,
        const std::string &ns,
        int &idx) const;
    visualization_msgs::MarkerArray getModelBasicVisualization(
        const URDFModelCoords &coords,
        const std::string &frame_id,
        const std::string &ns,
        std_msgs::ColorRGBA col,
        int &idx) const;
    visualization_msgs::MarkerArray getModelVisualization(
        const URDFModelCoords &coords,
        const std::string &frame_id,
        const std::string &ns,
        const std_msgs::ColorRGBA &col,
        int &idx) const;
    visualization_msgs::MarkerArray getAttachedObjectsVisualization(
        const URDFModelCoords &coords,
        const std::string &frame_id,
        const std::string &ns,
        const std_msgs::ColorRGBA &col,
        int &idx) const;

    bool computeGroupIK(
        const std::string &group_name,
        const Eigen::Affine3d &ee_pose_map,
        const URDFModelCoords &seed,
        URDFModelCoords &sol,
        bool bAllowApproxSolutions = false,
        int n_attempts = 0,
        double time_s = 0);

    bool computeCOM(
        const URDFModelCoords &coords,
        KDL::Vector &COM,
        double &mass) const;

    bool getLinkGlobalTransform(
        const URDFModelCoords &coords,
        const std::string &link_name,
        Eigen::Affine3d &tfm) const;

    bool attachObjectToLink(
        const std::string &link_name,
        const Eigen::Affine3d &pose,
        const shapes::Shape &object,
        const std::string &object_name,
        double res);

    void PrintModelInfo(std::ostream &o) const;

    void addIgnoreSelfCollisionLinkPair(
        const std::pair<std::string, std::string> pair);
    void addIgnoreSelfCollisionLinkPairs(
        const std::vector<std::pair<std::string, std::string>> pairs);

    boost::shared_ptr<const urdf::ModelInterface> getURDF() const;
    boost::shared_ptr<const srdf::Model> getSRDF() const;
    moveit::core::RobotModelConstPtr getRobotModel() const;

    moveit::core::RobotStatePtr getStateAt(const URDFModelCoords &coords) const;

    /// \name Required Public Functions from SBPLCollisionModel
    ///@{
    bool checkLimits(const ModelCoords &coord) const override;

    bool getModelCollisionSpheres(
        const ModelCoords &coords,
        std::vector<Sphere> &spheres) const override;

    bool getModelContactSpheres(
        const ModelCoords &coords,
        std::vector<Sphere> &spheres) const override;

    bool getModelContactSpheres(
        const ModelCoords &coords,
        const std::string &link_name,
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
        const std_msgs::ColorRGBA &col,
        int &idx) const override;
    ///@}

protected:

    boost::shared_ptr<const urdf::ModelInterface> urdf_;
    boost::shared_ptr<const srdf::Model> srdf_;

    std::vector<std::string> links_with_collision_spheres_;
    std::vector<std::string> links_with_contact_spheres_;

    std::vector<std::pair<std::string, std::string>> self_collision_ignore_pairs_;

    std::map<std::string, std::vector<Sphere>> collision_spheres_;
    std::map<std::string, std::vector<Sphere>> contact_spheres_;

    std::map<std::string, std::vector<AttachedObject>> attached_objects_;

    robot_model_loader::RobotModelLoaderPtr rm_loader_;
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr robot_state_;

    KDL::Tree kdl_tree_;

    bool updateFK(const URDFModelCoords &coords) const;

    bool updateFK(
        moveit::core::RobotState &state,
        const URDFModelCoords &coords) const;

    bool hasIgnoreSelfPair(std::string link1, std::string link2) const;

    bool getLinkCollisionSpheres(
        const URDFModelCoords &coords,
        const std::string &link_name,
        std::vector<Sphere> &spheres) const;
    bool getLinkCollisionSpheres_CurrentState(
        const std::string &link_name,
        std::vector<Sphere> &spheres) const;

    bool getLinkContactSpheres(
        const URDFModelCoords &coords,
        const std::string &link_name,
        std::vector<Sphere> &spheres) const;
    bool getLinkContactSpheres_CurrentState(
        const std::string &link_name,
        std::vector<Sphere> &spheres) const;

    bool getLinkAttachedObjectsSpheres(
        const std::string &link_name,
        const Eigen::Affine3d link_tfm,
        std::vector<Sphere> &spheres) const;

    bool getInterpolatedPath(
        const URDFModelCoords &coords0,
        const URDFModelCoords &coords1,
        double resolution,
        std::vector<URDFModelCoords> &path,
        int depth,
        int max_depth) const;

    bool computeCOMRecursURDF(
        const boost::shared_ptr<const urdf::Link> &link,
        const URDFModelCoords &coords,
        const Eigen::Affine3d &tf,
        double &m,
        Eigen::Vector3d &com) const;

    bool initRobotModelFromURDF(
        const std::string &urdf_string,
        const std::string &srdf_string);

    bool hasAttachedObject(
        const std::string &link_name,
        const std::string &object_name) const;
    bool hasAttachedObjects(const std::string &link_name) const;
    const std::vector<AttachedObject> getAttachedObjects(
            const std::string &link_name) const;

    bool attachObject(
        const std::string &link_name,
        const AttachedObject &obj);

    bool computeShapeBoundingSpheres(
        const shapes::Shape &shape,
        double res,
        std::vector<Sphere> &spheres);
};

////////////////////////////////////
// URDFModelCoords Implementation //
////////////////////////////////////

inline
bool URDFModelCoords::getCoords(
    const std::string &name,
    std::vector<double> &p_coords) const
{
    auto jnt = coordmap.find(name);
    if (jnt == coordmap.end()) {
        return false;
    }
    p_coords = jnt->second;
    return true;
}

inline
bool URDFModelCoords::getCoord(
    const std::string &name,
    double &p_coord,
    int idx) const
{
    auto jnt = coordmap.find(name);
    if (jnt == coordmap.end()) {
        return false;
    }
    if (idx >= jnt->second.size()) {
        return false;
    }
    p_coord = jnt->second[idx];
    return true;
}

inline
double URDFModelCoords::getCoord(const std::string &name, int idx) const
{
    auto jnt = coordmap.find(name);
    if (jnt == coordmap.end()) {
        ROS_ERROR("name %s not found in coordmap", name.c_str());
        throw SBPL_Exception();
        return 0;
    }
    if (idx >= jnt->second.size()) {
        ROS_ERROR("index %d out of bounds [0,%zu)", idx, jnt->second.size());
        throw SBPL_Exception();
        return false;
    }
    return jnt->second[idx];
}

inline
void URDFModelCoords::set(
    const std::string &name,
    const std::vector<double> &p_coords)
{
    coordmap[name] = p_coords;
}

inline
std::ostream &operator<<(std::ostream &o, const URDFModelCoords &c)
{
    o << "root: [" << std::setprecision(3) << c.root(0, 0) << " " << std::setprecision(3) << c.root(0, 1) << " " << std::setprecision(3) << c.root(0, 2) << " " << std::setprecision(3) << c.root(0, 3) << "]\n";
    o << "root: [" << std::setprecision(3) << c.root(1, 0) << " " << std::setprecision(3) << c.root(1, 1) << " " << std::setprecision(3) << c.root(1, 2) << " " << std::setprecision(3) << c.root(1, 3) << "]\n";
    o << "root: [" << std::setprecision(3) << c.root(2, 0) << " " << std::setprecision(3) << c.root(2, 1) << " " << std::setprecision(3) << c.root(2, 2) << " " << std::setprecision(3) << c.root(2, 3) << "]\n";
    o << "root: [" << std::setprecision(3) << c.root(3, 0) << " " << std::setprecision(3) << c.root(3, 1) << " " << std::setprecision(3) << c.root(3, 2) << " " << std::setprecision(3) << c.root(3, 3) << "]\n";

    for (const auto &entry : c.coordmap) {
        o << entry.first << " : { ";
        for (const double v : entry.second) {
            o << v << ' ';
        }
        o << "}\n";
    }
    return o;
}

inline
void URDFModelCoords::updateWith(
    URDFModelCoords &target,
    const URDFModelCoords &source)
{
    for (auto jnt = source.coordmap.begin(); jnt != source.coordmap.end();
            jnt++)
    {
        target.set(jnt->first, jnt->second);
    }
}

inline
double URDFModelCoords::getMaxJointDistance(
    const URDFModelCoords &from,
    const URDFModelCoords &to)
{
    double max_dist = 0.0;
    for (auto it = to.coordmap.begin(); it != to.coordmap.end(); ++it) {
        std::vector<double> seed_val;
        if (!from.getCoords(it->first, seed_val)) {
            ROS_ERROR_THROTTLE(1.0, "%s not found in [from]!", it->first.c_str());
            continue;
        }
        for (int v = 0; v < seed_val.size(); v++) {
            double diff = fabs(angles::shortest_angular_distance(
                    seed_val[v], it->second[v]));
            if (diff > max_dist) {
                //ROS_WARN("MaxDiff: %.3f deg @ %s", angles::to_degrees(diff), it->first.c_str());
                max_dist = diff;
            }
        }
    }
    return max_dist;
}

///////////////////////////////////////
// URDFCollisionModel Implementation //
///////////////////////////////////////

/// \brief Add a set of contact spheres to a link
inline
void URDFCollisionModel::addContactSpheres(
    const std::string &link_name,
    const std::vector<Sphere> &s)
{
    for (auto sp : s) {
        addContactSphere(link_name, sp);
    }
}

/// \brief Add a set of collision spheres to a link
inline
void URDFCollisionModel::addCollisionSpheres(
    const std::string &link_name,
    const std::vector<Sphere> &s)
{
    for (auto sp : s) {
        addCollisionSphere(link_name, sp);
    }
}

/// \brief Add a contact sphere to a link
///
/// There sphere position relative to the link is specified in the link frame.
inline
void URDFCollisionModel::addContactSphere(
    const std::string &link_name,
    Sphere s)
{
    auto it = contact_spheres_.find(link_name);
    if (it == contact_spheres_.end()) {
        s.link_name_ = link_name;
        s.name_ = link_name + "_0";
        contact_spheres_[link_name] = {s};
        links_with_contact_spheres_.push_back(link_name);
        ROS_INFO("Added contact sphere 1 for link %s", link_name.c_str());
    }
    else {
        s.link_name_ = link_name;
        s.name_ = link_name + "_" + std::to_string(it->second.size());
        it->second.push_back(s);
        ROS_INFO("Added contact sphere %d for link %s", (int )it->second.size(), link_name.c_str());
    }
}

/// \brief Add a collision sphere to a link
///
/// The sphere position relative to the link is specified in the link frame.
inline
void URDFCollisionModel::addCollisionSphere(const std::string &link_name, Sphere s)
{
    auto it = collision_spheres_.find(link_name);
    if (it == collision_spheres_.end()) {
        s.link_name_ = link_name;
        s.name_ = link_name + "_0";
        collision_spheres_[link_name] = {s};
        links_with_collision_spheres_.push_back(link_name);
    }
    else {
        s.link_name_ = link_name;
        s.name_ = link_name + "_" + std::to_string(it->second.size());
        it->second.push_back(s);
    }
}

inline
void URDFCollisionModel::addIgnoreSelfCollisionLinkPair(
    const std::pair<std::string, std::string> pair)
{
    if (!hasIgnoreSelfPair(pair.first, pair.second)) {
        self_collision_ignore_pairs_.push_back(pair);
    }
}

inline
void URDFCollisionModel::addIgnoreSelfCollisionLinkPairs(
    const std::vector<std::pair<std::string, std::string>> pairs)
{
    for (auto pair : pairs) {
        addIgnoreSelfCollisionLinkPair(pair);
    }
}

inline
bool URDFCollisionModel::getModelCollisionSpheres(
    const ModelCoords &coords,
    std::vector<Sphere> &spheres) const
{
    const URDFModelCoords &c = static_cast<const URDFModelCoords&>(coords);
    return getModelCollisionSpheres(c, spheres);
}

inline
bool URDFCollisionModel::getModelContactSpheres(
    const ModelCoords &coords,
    std::vector<Sphere> &spheres) const
{
    const URDFModelCoords &c = static_cast<const URDFModelCoords&>(coords);
    return getModelContactSpheres(c, spheres);
}

inline
bool URDFCollisionModel::getModelContactSpheres(
    const ModelCoords &coords,
    const std::string &link_name,
    std::vector<Sphere> &spheres) const
{
    const URDFModelCoords &c = static_cast<const URDFModelCoords&>(coords);
    return getLinkContactSpheres(c, link_name, spheres);
}

inline
bool URDFCollisionModel::getModelPathCollisionSpheres(
    const ModelCoords &coords0,
    const ModelCoords &coords1,
    int steps,
    std::vector<Sphere> &spheres) const
{
    const URDFModelCoords &c0 = static_cast<const URDFModelCoords&>(coords0);
    const URDFModelCoords &c1 = static_cast<const URDFModelCoords&>(coords1);
    return getModelPathCollisionSpheres(c0, c1, steps, spheres);
}

inline
bool URDFCollisionModel::getModelPathContactSpheres(
    const ModelCoords &coords0,
    const ModelCoords &coords1,
    int steps,
    std::vector<Sphere> &spheres) const
{
    const URDFModelCoords &c0 = static_cast<const URDFModelCoords&>(coords0);
    const URDFModelCoords &c1 = static_cast<const URDFModelCoords&>(coords1);
    return getModelPathContactSpheres(c0, c1, steps, spheres);
}

inline
boost::shared_ptr<const urdf::ModelInterface>
URDFCollisionModel::getURDF() const
{
    return urdf_;
}

inline
boost::shared_ptr<const srdf::Model>
URDFCollisionModel::getSRDF() const
{
    return srdf_;
}

inline
moveit::core::RobotModelConstPtr URDFCollisionModel::getRobotModel() const
{
    return robot_model_;
}

inline
bool URDFCollisionModel::checkLimits(const ModelCoords &coord) const
{
    const URDFModelCoords &c = static_cast<const URDFModelCoords&>(coord);
    return checkLimits(c);
}

inline
visualization_msgs::MarkerArray URDFCollisionModel::getModelVisualization(
    const ModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    const std_msgs::ColorRGBA &col,
    int &idx) const
{
    const URDFModelCoords &c = static_cast<const URDFModelCoords&>(coords);
    return getModelVisualization(c, frame_id, ns, col, idx);
}

inline
bool URDFCollisionModel::hasIgnoreSelfPair(
    std::string link1,
    std::string link2) const
{
    for (int k = 0; k < self_collision_ignore_pairs_.size(); k++) {
        std::pair<std::string, std::string> p = self_collision_ignore_pairs_[k];
        if (link1 == p.first && link2 == p.second) {
            return true;
        }
        else if (link1 == p.second && link2 == p.first) {
            return true;
        }
    }
    return false;
}

} // namespace adim

#endif
