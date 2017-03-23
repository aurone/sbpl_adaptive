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

#include <sbpl_adaptive_collision_checking/urdf_collision_model.h>

// system includes
#include <leatherman/utils.h>
#include <leatherman/print.h>
#include <sbpl_geometry_utils/bounding_spheres.h>
#include <sbpl_geometry_utils/voxelize.h>

namespace adim {

URDFCollisionModel::URDFCollisionModel() :
    urdf_(),
    srdf_(),
    links_with_collision_spheres_(),
    links_with_contact_spheres_(),
    self_collision_ignore_pairs_(),
    collision_spheres_(),
    contact_spheres_(),
    attached_objects_(),
    rm_loader_(),
    robot_model_(),
    robot_state_()
{
}

URDFCollisionModel::~URDFCollisionModel()
{
}

URDFModelCoords URDFCollisionModel::getDefaultCoordinates() const
{
    URDFModelCoords c;

    c.root = Eigen::Affine3d::Identity();
    c.collision_links = links_with_collision_spheres_;
    c.contact_links = links_with_contact_spheres_;

    for (const moveit::core::JointModel *j : robot_model_->getJointModels()) {
        if (j->getVariableCount() > 0) {
            std::vector<double> vals(j->getVariableCount());
            j->getVariableDefaultPositions(&vals[0]);
            c.coordmap[j->getName()] = vals;
        }
    }

    return c;
}

URDFModelCoords URDFCollisionModel::getRandomCoordinates() const
{
    URDFModelCoords c;

    c.root = Eigen::Affine3d::Identity();
    c.collision_links = links_with_collision_spheres_;
    c.contact_links = links_with_contact_spheres_;

    for (moveit::core::JointModel* j : robot_model_->getJointModels()) {
        if (j->getVariableCount() > 0) {
            std::vector<double> vals(j->getVariableCount());
            do {
                j->getVariableRandomPositions(
                        robot_state_->getRandomNumberGenerator(), &vals[0]);
            }
            while (!j->enforcePositionBounds(&vals[0]));
            c.coordmap[j->getName()] = vals;
        }
    }

    return c;
}

bool URDFCollisionModel::initFromURDF(
    const std::string& urdf_string,
    const std::string& srdf_string)
{
    urdf::Model* urdf_model = new urdf::Model;
    if (!urdf_model->initString(urdf_string)) {
        ROS_WARN("Failed to parse the URDF");
        return false;
    }
    urdf_.reset(urdf_model);
    if (!initRobotModelFromURDF(urdf_string, srdf_string)) {
        ROS_WARN("Failed to load robot model from URDF/SRDF");
        return false;
    }

    robot_state_->setToDefaultValues();

    return true;
}

void URDFCollisionModel::printIgnoreSelfCollisionLinkPairs()
{
    printf("{\n");
    for (auto pair : self_collision_ignore_pairs_) {
        printf("std::make_pair(\"%s\",\"%s\"),", pair.first.c_str(),
                pair.second.c_str());
    }
    printf("}\n");
}

bool URDFCollisionModel::computeSpheresFromURDFModel(
    double res,
    const std::vector<std::string> &ignore_collision_links,
    const std::vector<std::string> &contact_links)
{
    const std::vector<robot_model::LinkModel*> links = robot_model_->getLinkModels();

    for (const robot_model::LinkModel* link : links) {
        std::string link_name = link->getName();
        bool bIgnoreCollision = std::find(
                ignore_collision_links.begin(),
                ignore_collision_links.end(),
                link_name) != ignore_collision_links.end();
        bool bIsContact = std::find(
                contact_links.begin(),
                contact_links.end(),
                link_name) != contact_links.end();

        if (bIgnoreCollision && !bIsContact) {
            continue;
        }

        std::vector<Sphere> link_spheres;

        // multiple shapes after than groovy
        std::vector<shapes::ShapeConstPtr> objects = link->getShapes();
        for (shapes::ShapeConstPtr object : objects) {
            if (!object) {
                continue;
            }
            Eigen::Affine3d pose = Eigen::Affine3d::Identity(); //link->getJointOriginTransform();

            size_t prev_size = link_spheres.size();

            computeShapeBoundingSpheres(*object, res, link_spheres);

            // transform the spheres by the pose
            for (size_t i = prev_size; i < link_spheres.size(); ++i) {
                link_spheres[i].v = pose * link_spheres[i].v;
            }

            // give unique names to the spheres and reference the attached link
            for (size_t i = 0; i < link_spheres.size(); ++i) {
                Sphere& s = link_spheres[i];
                s.name_ = link->getName() + "_" + std::to_string(i);
                s.link_name_ = link->getName();
            }

            if (!bIgnoreCollision) {
                collision_spheres_[link->getName()] = link_spheres;
                links_with_collision_spheres_.push_back(link->getName());
                ROS_INFO("Adding %d collision spheres for link %s", (int )link_spheres.size(), link->getName().c_str());
            }

            if (bIsContact) {
                contact_spheres_[link->getName()] = link_spheres;
                links_with_contact_spheres_.push_back(link->getName());
                ROS_INFO("Adding %d contact spheres for link %s", (int )link_spheres.size(), link->getName().c_str());
            }
        }
    }
    return true;
}

bool URDFCollisionModel::initFromParam(const std::string &robot_desc_param_name)
{
    robot_model_loader::RobotModelLoader::Options ops;
    ops.robot_description_ = robot_desc_param_name;
    ops.urdf_string_ = "";
    ops.srdf_string_ = "";
    ops.urdf_doc_ = nullptr;
    ops.srdf_doc_ = nullptr;
    ops.load_kinematics_solvers_ = true;

    rm_loader_.reset(new robot_model_loader::RobotModelLoader(ops));
    if (!rm_loader_) {
        ROS_ERROR("Failed to instantiate Robot Model Loader from description");
        ROS_ERROR("%s", robot_desc_param_name.c_str());
        return false;
    }

    robot_model_ = rm_loader_->getModel();
    if (!robot_model_) {
        ROS_ERROR("Failed to retrieve valid Robot Model");
        ROS_ERROR("%s", robot_desc_param_name.c_str());
        return false;
    }

    urdf_ = robot_model_->getURDF();
    srdf_ = robot_model_->getSRDF();

    robot_state_.reset(new robot_state::RobotState(robot_model_));
    if (!robot_state_) {
        ROS_ERROR("Failed to instantiate Robot State");
        return false;
    }

    const robot_model::JointModel* root = robot_model_->getJointModel(robot_model_->getRootJointName());

    robot_state_->setToDefaultValues();

    return true;
}

bool URDFCollisionModel::initRobotModelFromURDF(
    const std::string &urdf_string,
    const std::string& srdf_string)
{
    robot_model_loader::RobotModelLoader::Options ops;
    ops.robot_description_ = "";
    ops.urdf_string_ = urdf_string;
    ops.srdf_string_ = srdf_string;
    ops.urdf_doc_ = nullptr;
    ops.srdf_doc_ = nullptr;
    ops.load_kinematics_solvers_ = true;

    rm_loader_.reset(new robot_model_loader::RobotModelLoader(ops));
    if (!rm_loader_) {
        ROS_ERROR("Failed to instantiate Robot Model Loader");
        return false;
    }

    robot_model_ = rm_loader_->getModel();
    if (!robot_model_) {
        ROS_ERROR("Failed to retrieve valid Robot Model");
        return false;
    }

    srdf_ = robot_model_->getSRDF();

    robot_state_.reset(new robot_state::RobotState(robot_model_));
    if (!robot_state_) {
        ROS_ERROR("Failed to instantiate Robot State");
        return false;
    }

    const robot_model::JointModel* root = robot_model_->getRootJoint();

    return true;
}

bool URDFCollisionModel::getLinkCollisionSpheres_CurrentState(
    const std::string &link_name,
    std::vector<Sphere> &spheres) const
{
    Eigen::Affine3d tfm = robot_state_->getGlobalLinkTransform(link_name);

    auto it = collision_spheres_.find(link_name);
    if (it == collision_spheres_.end()) {
        //no spheres on this link
        return true;
    }
    for (const Sphere &s : it->second) {
        Sphere s_;
        s_.name_ = s.name_;
        s_.link_name_ = s.link_name_;
        s_.v = tfm * s.v;
        s_.radius = s.radius;
        //Sphere::print(s_);
        spheres.push_back(s_);
    }

    if (hasAttachedObjects(link_name)) {
        ROS_INFO("Found attahed objects for link %s", link_name.c_str());
        if (!getLinkAttachedObjectsSpheres(link_name, tfm, spheres)) {
            return false;
        }
    }

    return true;
}

bool URDFCollisionModel::getLinkCollisionSpheres(
    const URDFModelCoords &coords,
    const std::string &link_name,
    std::vector<Sphere> &spheres) const
{
    if (!updateFK(coords)) {
        ROS_ERROR("URDFCollisionModel::getLinkCollisionSpheres - failed to update FK!");
        return false;
    }
    return getLinkCollisionSpheres_CurrentState(link_name, spheres);
}

bool URDFCollisionModel::getLinkContactSpheres(
    const URDFModelCoords &coords,
    const std::string &link_name,
    std::vector<Sphere> &spheres) const
{
    if (!updateFK(coords)) {
        ROS_ERROR("URDFCollisionModel::getLinkContactSpheres - failed to update FK!");
        return false;
    }
    return getLinkContactSpheres_CurrentState(link_name, spheres);
}

/// \brief Return the contact spheres for a link in the current state
bool URDFCollisionModel::getLinkContactSpheres_CurrentState(
    const std::string &link_name,
    std::vector<Sphere> &spheres) const
{
    Eigen::Affine3d tfm = robot_state_->getGlobalLinkTransform(link_name);

    auto it = contact_spheres_.find(link_name);
    if (it == contact_spheres_.end()) {
        //no spheres on this link
        return true;
    }

    for (const Sphere &s : it->second) {
        Sphere s_;
        s_.v = tfm * s.v;
        s_.radius = s.radius;
        spheres.push_back(s_);
    }
    return true;
}

void URDFCollisionModel::autoIgnoreSelfCollisions()
{
    URDFModelCoords defaultCoords = getDefaultCoordinates();
    autoIgnoreSelfCollisions(defaultCoords);
}

void URDFCollisionModel::autoIgnoreSelfCollisions(
    const URDFModelCoords &coords)
{
    std::vector<std::pair<std::string, std::string>> colliding_links;
    do {
        colliding_links = getSelfCollisions(coords);
        for (auto colliding_pair : colliding_links) {
            if (!hasIgnoreSelfPair(colliding_pair.first, colliding_pair.second)) {
                addIgnoreSelfCollisionLinkPair(colliding_pair);
                ROS_WARN("Adding ignored self collision pair (%s, %s)", colliding_pair.first.c_str(), colliding_pair.second.c_str());
            }
        }
    }
    while (colliding_links.size() > 0);
}

std::vector<std::pair<std::string, std::string>>
URDFCollisionModel::getSelfCollisions(const URDFModelCoords &coords) const
{
    std::vector<std::pair<std::string, std::string>> colliding_links;
    if (!updateFK(coords)) {
        ROS_ERROR("URDFCollisionModel::getSelfCollisions could not update FK!");
        throw SBPL_Exception();
    }

    for (int i = 0; i < links_with_collision_spheres_.size(); i++) {
        for (int j = i + 1; j < links_with_collision_spheres_.size(); j++) {
            std::string link1 = links_with_collision_spheres_[i];
            std::string link2 = links_with_collision_spheres_[j];

            if (hasIgnoreSelfPair(link1, link2))
                continue;

            std::vector<Sphere> l1;
            std::vector<Sphere> l2;

            if (!getLinkCollisionSpheres_CurrentState(link1, l1)) {
                ROS_ERROR("URDFCollisionModel::getSelfCollisions could not get link %s collision spheres!", link1.c_str());
                throw SBPL_Exception();
            }
            if (!getLinkCollisionSpheres_CurrentState(link2, l2)) {
                ROS_ERROR("URDFCollisionModel::getSelfCollisions could not get link %s collision spheres!", link2.c_str());
                throw SBPL_Exception();
            }
            for (Sphere s1 : l1) {
                for (Sphere s2 : l2) {
                    Eigen::Vector3d d = s1.v - s2.v; //distance between sphere centers
                    if (d.norm() < s1.radius + s2.radius) { //if less than the sum of the radii, then self collision
                        ROS_WARN("Sphere %s [link: %s] in collision with sphere %s [link: %s]", s1.name_.c_str(), link1.c_str(), s2.name_.c_str(), link2.c_str());
                        colliding_links.push_back(std::make_pair(link1, link2));
                    }
                }
            }
        }
    }
    return colliding_links;
}

bool URDFCollisionModel::checkSelfCollisions(
    const URDFModelCoords &coords) const
{
    if (!updateFK(coords)) {
        return false;
    }

    for (int i = 0; i < links_with_collision_spheres_.size(); i++) {
        for (int j = i + 1; j < links_with_collision_spheres_.size(); j++) {
            std::string link1 = links_with_collision_spheres_[i];
            std::string link2 = links_with_collision_spheres_[j];

            if (hasIgnoreSelfPair(link1, link2)) {
                continue;
            }

            std::vector<Sphere> l1;
            std::vector<Sphere> l2;

            if (!getLinkCollisionSpheres_CurrentState(link1, l1)) {
                return false;
            }
            if (!getLinkCollisionSpheres_CurrentState(link2, l2)) {
                return false;
            }
            for (Sphere s1 : l1) {
                for (Sphere s2 : l2) {
                    Eigen::Vector3d d = s1.v - s2.v; //distance between sphere centers
                    if (d.norm() < s1.radius + s2.radius) { //if less than the sum of the radii, then self collision
//                        ROS_WARN("Sphere %s [link: %s] in collision with sphere %s [link: %s]", s1.name_.c_str(), link1.c_str(), s2.name_.c_str(), link2.c_str());
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

bool URDFCollisionModel::getLinkGlobalTransform(
    const URDFModelCoords &coords,
    const std::string &link_name,
    Eigen::Affine3d &tfm) const
{
    if (!updateFK(coords)) {
        return false;
    }
    tfm = robot_state_->getGlobalLinkTransform(link_name);
    return true;
}

moveit::core::RobotStatePtr URDFCollisionModel::getStateAt(
    const URDFModelCoords &coords) const
{
    auto state = boost::make_shared<moveit::core::RobotState>(robot_model_);
    updateFK(*state, coords);
    return state;
}

bool URDFCollisionModel::checkLimits(const URDFModelCoords &coords) const
{
    if (!robot_model_) {
        ROS_ERROR("robot_model_ not initialized!");
        throw SBPL_Exception();
    }
    for (auto it = coords.coordmap.begin(); it != coords.coordmap.end(); it++) {
        std::string joint_name = it->first;
        const robot_model::JointModel* jm = robot_model_->getJointModel(joint_name);

        if (!jm) {
            ROS_ERROR("Could not get joint model for joint %s", joint_name.c_str());
            throw SBPL_Exception();
        }

        int var_count = jm->getVariableCount();
        robot_model::JointModel::Bounds bounds = jm->getVariableBounds();

        if (var_count != it->second.size()) {
            ROS_ERROR("URDFCollisionModel::getModelCollisionSpheres -- coords.size() (%zu) != joint #vars (%d) for joint %s!", it->second.size(), var_count, joint_name.c_str());
            throw SBPL_Exception();
            return false;
        }
        for (int j = 0; j < var_count; j++) {
            if (bounds[j].position_bounded_) {
                if (it->second[j] < bounds[j].min_position_ ||
                    it->second[j] > bounds[j].max_position_)
                {
                    ROS_DEBUG("Joint %s var %d [%.3f] outside limits [%.3f to %.3f]", joint_name.c_str(), j, it->second[j], bounds[j].min_position_, bounds[j].max_position_);
                    return false;
                }
            }
        }
    }
    return true;
}

bool URDFCollisionModel::getModelCollisionSpheres(
    const URDFModelCoords &coords,
    std::vector<Sphere> &spheres) const
{
    if (!updateFK(coords)) {
        return false;
    }
    if (coords.collision_links.empty()) {
        for (std::string link_name : links_with_collision_spheres_) {
            if (!getLinkCollisionSpheres_CurrentState(link_name, spheres)) {
                return false;
            }
        }
    }
    else {
        for (std::string link_name : coords.collision_links) {
            if (!getLinkCollisionSpheres_CurrentState(link_name, spheres)) {
                return false;
            }
        }
    }
    return true;
}

bool URDFCollisionModel::updateFK(const URDFModelCoords &coords) const
{
    return updateFK(*robot_state_, coords);
}

bool URDFCollisionModel::updateFK(
    moveit::core::RobotState &state,
    const URDFModelCoords &coords) const
{
    // set the root joint first
    const robot_model::JointModel* root = state.getRobotModel()->getRootJoint();

    state.setJointPositions(root, coords.root);

    // then go through the other joints and set them
    for (const auto &entry : coords.coordmap) {
        const std::string &joint_name = entry.first;
        const std::vector<double> &vals = entry.second;
        const moveit::core::JointModel *j = state.getRobotModel()->getJointModel(joint_name);
        if (!j) {
            ROS_ERROR("Joint '%s' not found in the Robot Model", joint_name.c_str());
            return false;
        }

        if (vals.size() != j->getVariableCount()) {
            ROS_ERROR("URDFCollisionModel::getModelCollisionSpheres -- coords.size() (%zu) != joint #vars (%zu) for joint %s!", vals.size(), j->getVariableCount(), joint_name.c_str());
            throw SBPL_Exception();
        }

        state.setJointPositions(j, vals);
    }

    state.update();
    state.updateLinkTransforms();
    state.updateCollisionBodyTransforms();

    return true;
}

/// \brief Get the contact spheres for a state
///
/// If the input state specifies a set of contact links, this method returns
/// the contact spheres for that set of links; otherwise, this method returns
/// returns the contact spheres for all links in the model.
bool URDFCollisionModel::getModelContactSpheres(
    const URDFModelCoords &coords,
    std::vector<Sphere> &spheres) const
{
    if (!updateFK(coords)) {
        return false;
    }
    if (coords.contact_links.empty()) {
        for (const std::string &link_name : links_with_contact_spheres_) {
            if (!getLinkContactSpheres_CurrentState(link_name, spheres)) {
                return false;
            }
        }
    }
    else {
        for (const std::string &link_name : coords.contact_links) {
            if (!getLinkContactSpheres_CurrentState(link_name, spheres)) {
                return false;
            }
        }
    }
    return true;
}

bool URDFCollisionModel::getInterpolatedPath(
    const URDFModelCoords &coords0,
    const URDFModelCoords &coords1,
    double resolution,
    std::vector<URDFModelCoords> &path) const
{
    return getInterpolatedPath(coords0, coords1, resolution, path, 0, -1);
}

bool URDFCollisionModel::getInterpolatedPath(
    const URDFModelCoords &coords0,
    const URDFModelCoords &coords1,
    double resolution,
    std::vector<URDFModelCoords> &path,
    int max_depth) const
{
    return getInterpolatedPath(coords0, coords1, resolution, path, 0, max_depth);
}

bool URDFCollisionModel::getInterpolatedCoordinates(
    const URDFModelCoords &coords0,
    const URDFModelCoords &coords1,
    double t,
    URDFModelCoords &interp) const
{
    const robot_model::JointModel* root = robot_model_->getRootJoint();
    if (!root) {
        ROS_ERROR("Could not find root joint!");
        return false;
    }
    int root_n_vars = root->getVariableCount();

    robot_state_->setJointPositions(root, coords0.root);
    const double* pos0 = robot_state_->getJointPositions(root);
    std::vector<double> root_vars0(pos0, pos0 + root->getVariableCount());

    robot_state_->setJointPositions(root, coords1.root);
    const double* pos1 = robot_state_->getJointPositions(root);
    std::vector<double> root_vars1(pos1, pos1 + root->getVariableCount());

    std::vector<double> root_varsT(root_n_vars);
    root->interpolate(&root_vars0[0], &root_vars1[0], t, &root_varsT[0]);
    robot_state_->setJointPositions(root, root_varsT);
    interp.root = robot_state_->getJointTransform(root);

    for (auto it0 = coords0.coordmap.begin(); it0 != coords0.coordmap.end(); it0++) {
        std::string joint_name = it0->first;
        std::vector<double> j0_pos = it0->second;
        std::vector<double> j1_pos;
        std::vector<double> jt_pos(j0_pos.size());

        if (!coords1.getCoords(joint_name, j1_pos)) {
            ROS_ERROR("Could not find joint %s in coords1", joint_name.c_str());
            return false;
        }

        const moveit::core::JointModel* jm = robot_model_->getJointModel(
                joint_name);
        if (!jm) {
            ROS_ERROR("URDFCollisionModel::getInterpolatedPath -- Joint %s not found in model!", joint_name.c_str());
            throw SBPL_Exception();
            return false;
        }
        robot_model::JointModel::Bounds bounds = jm->getVariableBounds();
        if (jm->getVariableCount() != j0_pos.size()) {
            ROS_ERROR("URDFCollisionModel::getInterpolatedPath -- %s : coords0.size() [%d] != jointVarCount [%d]", joint_name.c_str(), (int )j0_pos.size(), (int )jm->getVariableCount());
            throw SBPL_Exception();
            return false;
        }
        if (jm->getVariableCount() != j1_pos.size()) {
            ROS_ERROR("URDFCollisionModel::getInterpolatedPath -- %s : coords1.size() [%d] != jointVarCount [%d]", joint_name.c_str(), (int )j1_pos.size(), (int )jm->getVariableCount());
            throw SBPL_Exception();
            return false;
        }
        jm->interpolate(&j0_pos[0], &j1_pos[0], t, &jt_pos[0]);
        for (int j = 0; j < jt_pos.size(); j++) {
            if (bounds[j].position_bounded_) {
                if (bounds[j].min_position_ > jt_pos[j] ||
                    jt_pos[j] > bounds[j].max_position_)
                {
                    ROS_ERROR("Interpolation out of bounds! %.3f [%.3f, %.3f]", jt_pos[j], bounds[j].min_position_, bounds[j].max_position_);
                    jt_pos[j] = ALERP(j0_pos[j], j1_pos[j], t);
                    if (bounds[j].min_position_ > jt_pos[j] ||
                        jt_pos[j] > bounds[j].max_position_)
                    {
                        ROS_ERROR("Interpolation still out of bounds! %.3f [%.3f, %.3f]", jt_pos[j], bounds[j].min_position_, bounds[j].max_position_);
                        throw SBPL_Exception();
                    }
                }
            }
        }

        interp.set(joint_name, jt_pos);
    }
    return true;
}

bool URDFCollisionModel::getInterpolatedPath(
    const URDFModelCoords &coords0,
    const URDFModelCoords &coords1,
    int steps,
    std::vector<URDFModelCoords> &path) const
{
    path.clear();
    path.resize(steps);

    for (int t = 0; t < steps; t++) {
        double tt = t / (steps - 1);

        for (auto it0 = coords0.coordmap.begin(); it0 != coords0.coordmap.end(); it0++)
        {
            std::string joint_name = it0->first;
            std::vector<double> j0_pos = it0->second;
            std::vector<double> j1_pos;
            std::vector<double> jt_pos(j0_pos.size());
            if (!coords1.getCoords(joint_name, j1_pos)) {
                ROS_ERROR("Could not find joint %s in coords1", joint_name.c_str());
                return false;
            }

            const robot_model::JointModel* jm = robot_model_->getJointModel(joint_name);
            if (!jm) {
                ROS_ERROR("URDFCollisionModel::getInterpolatedPath -- Joint %s not found in model!", joint_name.c_str());
                throw SBPL_Exception();
                return false;
            }
            if (jm->getVariableCount() != j0_pos.size()) {
                ROS_ERROR("URDFCollisionModel::getInterpolatedPath -- %s : coords0.size() [%d] != jointVarCount [%d]", joint_name.c_str(), (int )j0_pos.size(), (int )jm->getVariableCount());
                throw SBPL_Exception();
                return false;
            }
            if (jm->getVariableCount() != j1_pos.size()) {
                ROS_ERROR("URDFCollisionModel::getInterpolatedPath -- %s : coords1.size() [%d] != jointVarCount [%d]", joint_name.c_str(), (int )j1_pos.size(), (int )jm->getVariableCount());
                throw SBPL_Exception();
                return false;
            }
            jm->interpolate(&j0_pos[0], &j1_pos[0], tt, &jt_pos[0]);
            path[t].set(joint_name, jt_pos);
        }
    }
    return true;
}

bool URDFCollisionModel::getModelPathCollisionSpheres(
    const URDFModelCoords &coords0,
    const URDFModelCoords &coords1,
    int steps,
    std::vector<Sphere> &spheres) const
{
    std::vector<URDFModelCoords> path;
    if (!getInterpolatedPath(coords0, coords1, steps, path)) {
        return false;
    }
    for (int i = 0; i < path.size(); i++) {
        if (!getModelCollisionSpheres(path[i], spheres)) {
            return false;
        }
    }
    return true;
}

bool URDFCollisionModel::getModelPathContactSpheres(
    const URDFModelCoords &coords0,
    const URDFModelCoords &coords1,
    int steps,
    std::vector<Sphere> &spheres) const
{
    std::vector<URDFModelCoords> path;
    if (!getInterpolatedPath(coords0, coords1, steps, path)) {
        return false;
    }
    for (int i = 0; i < path.size(); i++) {
        if (!getModelContactSpheres(path[i], spheres)) {
            return false;
        }
    }
    return true;
}

visualization_msgs::MarkerArray URDFCollisionModel::getAttachedObjectsVisualization(
    const URDFModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    const std_msgs::ColorRGBA &col,
    int &idx) const
{
    visualization_msgs::MarkerArray markers;

    for (auto it = attached_objects_.begin(); it != attached_objects_.end(); it++)
    {
        std::string link_name = it->first;
        Eigen::Affine3d tfm;
        if (!getLinkGlobalTransform(coords, link_name, tfm)) {
            continue;
        }
        for (const AttachedObject &o : it->second) {
            for (const Sphere s : o.spheres) {
                Sphere s_;
                s_.v = tfm * s.v;
                s_.radius = s.radius;
                visualization_msgs::Marker m =
                        getSphereMarker(s_, ns + "_" + o.name, frame_id, col, idx);
                markers.markers.push_back(m);
            }
        }
    }

    return markers;
}

//get more advanced mesh visualization when available
visualization_msgs::MarkerArray URDFCollisionModel::getModelVisualization(
    const URDFModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    const std_msgs::ColorRGBA &col,
    int &id) const
{
    visualization_msgs::MarkerArray markers;
    if (!updateFK(coords)) {
        return markers;
    }
    robot_state_->getRobotMarkers(markers, robot_model_->getLinkModelNames(), col, ns, ros::Duration(0), true);
    for (visualization_msgs::Marker &marker : markers.markers) {
        marker.id = id;
        id++;
    }
    return markers;
}

visualization_msgs::MarkerArray URDFCollisionModel::getModelBasicVisualizationByLink(
    const URDFModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    int &idx) const
{
    visualization_msgs::MarkerArray markers;
    std_msgs::ColorRGBA col;
    int i = 0;
    for (const std::string &link : links_with_collision_spheres_) {
        std::vector<Sphere> spheres;
        leatherman::msgHSVToRGB(240.0 * i / (double)links_with_collision_spheres_.size(), 1, 1, col);
        if (getLinkCollisionSpheres(coords, link, spheres)) {
            for (const Sphere &s : spheres) {
                visualization_msgs::Marker marker =
                        getSphereMarker(s, ns + "_" + link + "_collision_spheres", frame_id, col, idx);
                markers.markers.push_back(marker);
            }
        }
        i++;
    }
    col.a = 0.5 * col.a;
    i = 0;
    for (const std::string &link : links_with_contact_spheres_) {
        std::vector<Sphere> contact_spheres;
        leatherman::msgHSVToRGB(240 * i / (double)links_with_contact_spheres_.size(), 1, 1, col);
        col.a = 0.5;
        if (getLinkContactSpheres(coords, link, contact_spheres)) {
            int id = 0;
            for (const Sphere &s : contact_spheres) {
                visualization_msgs::Marker marker =
                        getSphereMarker(s, ns + "_" + link + "_contact_spheres", frame_id, col, id);
            }
        }
        i++;
    }
    return markers;
}

visualization_msgs::MarkerArray
URDFCollisionModel::getModelSelfCollisionVisualization(
    const URDFModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    const std_msgs::ColorRGBA &col,
    int &idx) const
{
    visualization_msgs::MarkerArray ma;
    if (!updateFK(coords)) {
        return ma;
    }

    for (int i = 0; i < links_with_collision_spheres_.size(); ++i) {
        for (int j = i + 1; j < links_with_collision_spheres_.size(); ++j) {
            const std::string &link1 = links_with_collision_spheres_[i];
            const std::string &link2 = links_with_collision_spheres_[j];

            if (hasIgnoreSelfPair(link1, link2)) {
                continue;
            }

            std::vector<Sphere> l1;
            std::vector<Sphere> l2;

            if (!getLinkCollisionSpheres_CurrentState(link1, l1)) {
                continue;
            }
            if (!getLinkCollisionSpheres_CurrentState(link2, l2)) {
                continue;
            }
            for (Sphere s1 : l1) {
                for (Sphere s2 : l2) {
                    Eigen::Vector3d d = s1.v - s2.v; //distance between sphere centers
                    if (d.norm() < s1.radius + s2.radius) { //if less than the sum of the radii, then self collision
                        visualization_msgs::Marker m;
                        m = getSphereMarker(s1, ns + "self_collisions", frame_id, col, idx);
                        ma.markers.push_back(m);
                        m = getSphereMarker(s2, ns + "self_collisions", frame_id, col, idx);
                        ma.markers.push_back(m);
                    }
                }
            }
        }
    }
    return ma;
}

visualization_msgs::MarkerArray URDFCollisionModel::getModelBasicVisualization(
    const URDFModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    std_msgs::ColorRGBA col,
    int &idx) const
{
    visualization_msgs::MarkerArray markers;
    std::vector<Sphere> spheres;
    if (getModelCollisionSpheres(coords, spheres)) {
        for (Sphere s : spheres) {
            visualization_msgs::Marker marker =
                    getSphereMarker(s, ns + "_collision_spheres", frame_id, col, idx);
            markers.markers.push_back(marker);
        }
    }
    col.a = 0.5 * col.a;
    std::vector<Sphere> contact_spheres;
    if (getModelContactSpheres(coords, contact_spheres)) {
        for (Sphere s : contact_spheres) {
            visualization_msgs::Marker marker =
                    getSphereMarker(s, ns + "_contact_spheres", frame_id, col, idx);
            markers.markers.push_back(marker);
        }
    }
    //ROS_INFO("Got %d spheres in basic visualization!", (int)markers.markers.size());
    return markers;
}

bool URDFCollisionModel::computeCOMRecursURDF(
    const boost::shared_ptr<const urdf::Link> &link,
    const URDFModelCoords &coords,
    const Eigen::Affine3d &tf,
    double &m,
    Eigen::Vector3d &com) const
{
    // compute transform from parent link to this link
    Eigen::Affine3d joint_trans;
    if (!link->parent_joint) {
        joint_trans = Eigen::Affine3d::Identity();
    }
    else {
        switch (link->parent_joint->type) {
        case urdf::Joint::UNKNOWN:
            return false;
        case urdf::Joint::REVOLUTE:
        case urdf::Joint::CONTINUOUS: {
            std::vector<double> vals;
            if (!coords.getCoords(link->parent_joint->name, vals)) {
                ROS_ERROR("failed to find joint value '%s'", link->parent_joint->name.c_str());
                return false;
            }
            if (vals.size() != 1) {
                return false;
            }
            Eigen::Vector3d axis(
                    link->parent_joint->axis.x,
                    link->parent_joint->axis.y,
                    link->parent_joint->axis.z);
            joint_trans = Eigen::AngleAxisd(vals[0], axis);
        }   break;
        case urdf::Joint::PRISMATIC: {
            std::vector<double> vals;
            if (!coords.getCoords(link->parent_joint->name, vals)) {
                ROS_ERROR("failed to find joint value '%s'", link->parent_joint->name.c_str());
                return false;
            }
            if (vals.size() != 1) {
                return false;
            }
            Eigen::Vector3d axis(
                    link->parent_joint->axis.x,
                    link->parent_joint->axis.y,
                    link->parent_joint->axis.z);
            joint_trans = Eigen::Translation3d(0.0, 0.0, vals[0]);
        }   break;
        case urdf::Joint::FLOATING:
        case urdf::Joint::PLANAR:
            ROS_ERROR("learn to compute center of mass for multi-dof joints");
            return false;
        case urdf::Joint::FIXED:
            joint_trans = Eigen::Affine3d::Identity();
            break;
        }
    }

    // transform this link into robot frame
    Eigen::Affine3d current_frame = tf * joint_trans;

    if (link->inertial) {
        // compute center-of-mass contributions
        const Eigen::Vector3d current_cog(
                link->inertial->origin.position.x,
                link->inertial->origin.position.y,
                link->inertial->origin.position.z);
        double mass = link->inertial->mass;
        com = com + mass * (current_frame * current_cog);
        m += mass;
    }

    // recursively compute center of mass for children
    for (auto child : link->child_links) {
        if (!computeCOMRecursURDF(child, coords, current_frame, m, com)) {
            return false;
        }
    }

    return true;
}

bool URDFCollisionModel::computeCOM(
    const URDFModelCoords &joint_positions,
    KDL::Vector& CoM,
    double& mass) const
{
    ROS_DEBUG("compute center of mass: urdf @ %p, root link @ %p", urdf_.get(), urdf_ ? urdf_->getRoot().get() : nullptr);

    mass = 0.0;
    Eigen::Vector3d com(0.0, 0.0, 0.0);
    Eigen::Affine3d trans(Eigen::Affine3d::Identity());
    if (!computeCOMRecursURDF(urdf_->getRoot(), joint_positions, trans, mass, com)) {
        return false;
    }

    if (mass <= 0.0) {
        return false;
    }

    com = (1.0 / mass) * com;
    CoM.x(com.x());
    CoM.y(com.y());
    CoM.z(com.z());
    return true;
}

bool URDFCollisionModel::computeGroupIK(
    const std::string &group_name,
    const Eigen::Affine3d &ee_pose_map,
    const URDFModelCoords &seed,
    URDFModelCoords &sol,
    bool bAllowApproxSolutions,
    int n_attempts,
    double time_s)
{
    const robot_model::JointModelGroup* jmg =
            robot_model_->getJointModelGroup(group_name);

    if (!jmg) {
        ROS_ERROR("Could not get joint model group: %s", group_name.c_str());
        return false;
    }

    if (!updateFK(seed)) {
        ROS_ERROR("Failed to update kinematics");
        return false;
    }

    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = bAllowApproxSolutions;

    if (!robot_state_->setFromIK(
            jmg,
            ee_pose_map,
            n_attempts,
            time_s,
            moveit::core::GroupStateValidityCallbackFn(),
            options))
    {
        std::vector<double> gpos;
        robot_state_->copyJointGroupPositions(jmg, gpos);
        ROS_DEBUG("Failed to compute IK for group '%s' to pose { %s } using seed %s", group_name.c_str(), to_str(ee_pose_map).c_str(), to_string(gpos).c_str());
        return false;
    }

    for (const moveit::core::JointModel *jm : jmg->getJointModels()) {
        if (jm->getVariableCount() > 0) {
            const double *v = robot_state_->getJointPositions(jm);
            std::vector<double> vals(v, v + jm->getVariableCount());
            sol.set(jm->getName(), vals);
        }
    }

    return true;
}

void URDFCollisionModel::PrintModelInfo(std::ostream &o) const
{
    o << "Root Joint: " << robot_model_->getRootJointName() << '\n';
    o << "Root Link: " <<  robot_model_->getRootLinkName() << '\n';

    o << "Active Joints:\n";
    for (robot_model::JointModel* j : robot_model_->getJointModels()) {
        if (j->isPassive()) {
            continue;
        }
        if (j->getType() == robot_model::JointModel::FIXED) {
            continue;
        }
        o << "  " << j->getName() << '\n';
        if (j->getVariableCount() > 0) {
            o << "    Variables:\n";
            std::vector<std::string> varnames = j->getVariableNames();
            robot_model::JointModel::Bounds bounds = j->getVariableBounds();
            for (int i = 0; i < varnames.size(); i++) {
                std::string vn = varnames[i];
                robot_model::VariableBounds bound = bounds[i];
                o << "      " << vn << " [" << bound.min_position_ << " : " << bound.max_position_ << "]\n";
            }
        }
    }
    o << "Fixed Joints:\n";
    for (robot_model::JointModel* j : robot_model_->getJointModels()) {
        if (j->isPassive()) {
            continue;
        }
        if (j->getType() != robot_model::JointModel::FIXED) {
            continue;
        }
        o << "  " << j->getName() << '\n';
        if (j->getVariableCount() > 0) {
            o << "    Variables:\n";
            std::vector<std::string> varnames = j->getVariableNames();
            robot_model::JointModel::Bounds bounds = j->getVariableBounds();
            for (int i = 0; i < varnames.size(); i++) {
                std::string vn = varnames[i];
                robot_model::VariableBounds bound = bounds[i];
                o << "      " << vn << " [" << bound.min_position_ << " : " << bound.max_position_ << "]\n";
            }
        }
    }

    o << "Links:\n";
    for (robot_model::LinkModel* l : robot_model_->getLinkModels()) {
        o << "  " << l->getName();
    }

    o << "Groups:\n";
    for (const robot_model::JointModelGroup *group : robot_model_->getJointModelGroups()) {
        o << "  " << group->getName() << '\n';
        o << "  Root Joints:\n";
        for (const robot_model::JointModel* r : group->getJointRoots()) {
            o << "    " << r->getName() << '\n';
        }
        o << "  End Effector: " << group->getEndEffectorName() << '\n';
    }
}

bool URDFCollisionModel::attachObjectToLink(
    const std::string &link_name,
    const Eigen::Affine3d &pose,
    const shapes::Shape &object,
    const std::string &object_name,
    double res)
{
    if (hasAttachedObject(link_name, object_name)) {
        ROS_ERROR("Object with name %s already attached to joint %s", object_name.c_str(), link_name.c_str());
        return false;
    }

    const robot_model::LinkModel* link = robot_model_->getLinkModel(link_name);
    if (!link) {
        ROS_ERROR("Could not find link %s", link_name.c_str());
        return false;
    }

    AttachedObject obj;

    if (!computeShapeBoundingSpheres(object, res, obj.spheres)) {
        return false;
    }

    // transform spheres
    for (auto& sphere : obj.spheres) {
        sphere.v = pose * sphere.v;
    }

    ROS_INFO("Got %zu spheres for attached object!", obj.spheres.size());

    // process obj.spheres
    obj.name = object_name;

    for (size_t i = 0; i < obj.spheres.size(); ++i) {
        Sphere& s = obj.spheres[i];
        s.name_ = obj.name + "_" + std::to_string(i);
        s.link_name_ = link_name;
    }

    return attachObject(link_name, obj);
}

bool URDFCollisionModel::computeShapeBoundingSpheres(
    const shapes::Shape& shape,
    double res,
    std::vector<Sphere>& spheres)
{
    switch (shape.type) {
    case shapes::BOX: {
        const shapes::Box& obj = dynamic_cast<const shapes::Box&>(shape);
        std::vector<Eigen::Vector3d> centers;
        sbpl::ComputeBoxBoundingSpheres(obj.size[0], obj.size[1], obj.size[2], res, centers);
        spheres.reserve(spheres.size() + centers.size());
        for (const auto& center : centers) {
            spheres.push_back(Sphere());
            spheres.back().v = center;
            spheres.back().radius = res;
        }
    }   break;
    case shapes::CYLINDER: {
        const shapes::Cylinder& obj = dynamic_cast<const shapes::Cylinder&>(shape);
        std::vector<Eigen::Vector3d> centers;
        sbpl::ComputeCylinderBoundingSpheres(obj.radius, obj.length, res, centers);
        spheres.reserve(spheres.size() + centers.size());
        for (const auto& center : centers) {
            spheres.push_back(Sphere());
            spheres.back().v = center;
            spheres.back().radius = res;
        }
    }   break;
    case shapes::SPHERE: {
        const shapes::Sphere& obj = dynamic_cast<const shapes::Sphere&>(shape);
        spheres.push_back(Sphere());
        spheres.back().v = Eigen::Vector3d::Zero();
        spheres.back().radius = obj.radius;
    }   break;
    case shapes::MESH: {
        const shapes::Mesh& obj = dynamic_cast<const shapes::Mesh&>(shape);

        std::vector<Eigen::Vector3d> vert;
        std::vector<int> tri;

        for (int v = 0; v < obj.vertex_count; v++) {
            double x = obj.vertices[3 * v];
            double y = obj.vertices[3 * v + 1];
            double z = obj.vertices[3 * v + 2];
            vert.push_back(Eigen::Vector3d(x, y, z));
        }

        for (int t = 0; t < obj.triangle_count; t++) {
            tri.push_back(obj.triangles[3 * t]);
            tri.push_back(obj.triangles[3 * t + 1]);
            tri.push_back(obj.triangles[3 * t + 2]);
        }

        /// bounding sphere strategy copy-pasted from old
        /// SphereEncloser.h since it's being removed from
        /// sbpl_geometry_utils
        double radius = sqrt(2.0) * res;
        std::vector<Eigen::Vector3d> centers;
        sbpl::VoxelizeMesh(vert, tri, radius, centers, false);
        spheres.reserve(spheres.size() + centers.size());
        for (const auto& center : centers) {
            spheres.push_back(Sphere());
            spheres.back().v = center;
            spheres.back().radius = res;
        }
    }   break;
    default:
        ROS_ERROR("Shape type %d not supported", shape.type);
        return false;
    }

    return true;
}

bool URDFCollisionModel::hasAttachedObjects(const std::string &link_name) const
{
    auto it = attached_objects_.find(link_name);
    return (it != attached_objects_.end());
}

const std::vector<AttachedObject> URDFCollisionModel::getAttachedObjects(
    const std::string &link_name) const
{
    auto it = attached_objects_.find(link_name);
    if (it == attached_objects_.end()) {
        return {};
    }
    return it->second;
}

bool URDFCollisionModel::getLinkAttachedObjectsSpheres(
    const std::string &link_name,
    const Eigen::Affine3d link_tfm,
    std::vector<Sphere> &spheres) const
{
    if (!hasAttachedObjects(link_name)) {
        return true;
    }
    const std::vector<AttachedObject> objs = getAttachedObjects(link_name);

    ROS_INFO("Got %d attached objects for link %s", (int )objs.size(), link_name.c_str());

    for (const AttachedObject &o : objs) {
        ROS_INFO("Object %s has %zu spheres!", o.name.c_str(), o.spheres.size());
        for (const Sphere &s : o.spheres) {
            Sphere s_;
            s_.name_ = s.name_;
            s_.v = link_tfm * s.v;
            s_.radius = s.radius;
            spheres.push_back(s_);
        }
    }
    return true;
}

bool URDFCollisionModel::getInterpolatedPath(
    const URDFModelCoords &coords0,
    const URDFModelCoords &coords1,
    double resolution,
    std::vector<URDFModelCoords> &path,
    int depth,
    int max_depth) const
{
    if (depth == max_depth) {
        ROS_WARN("Maximum interpolation recursion depth of %d reached", depth);
        return false;
    }

    //compute max_dist that collision spheres travel between c0 and c1
    double max_dist = 0.0;
    path.clear();
    for (std::string link : links_with_collision_spheres_) {
        std::vector<Sphere> s0;
        std::vector<Sphere> s1;
        if (!getLinkCollisionSpheres(coords0, link, s0)) {
            ROS_ERROR("URDFCollisionModel::getInterpolatedPath - Failed to get link %s collision spheres", link.c_str());
            return false;
        }
        if (!getLinkCollisionSpheres(coords1, link, s1)) {
            ROS_ERROR("URDFCollisionModel::getInterpolatedPath - Failed to get link %s collision spheres", link.c_str());
            return false;
        }
        if (s0.size() != s1.size()) {
            ROS_ERROR("URDFCollisionModel::getInterpolatedPath - Different number of spheres found between coords0 and coords1!");
            return false;
        }
        for (int s = 0; s < s0.size(); s++) {
            double dist = (s0[s].v - s1[s].v).norm();
            if (dist > max_dist) {
                max_dist = dist;
                if (max_dist > resolution) {
                    break;
                }
            }
        }
        if (max_dist > resolution) {
            break;
        }
    }
    // if max_dist is more than resolution, we need to subdivide path between c0 and c1
    if (max_dist > resolution) {
        //ROS_WARN("Sphere MaxDist = %.3f", max_dist);
        URDFModelCoords coords_half;
        if (!getInterpolatedCoordinates(coords0, coords1, 0.5, coords_half)) {
            ROS_ERROR("URDFCollisionModel::getInterpolatedPath - failed to get interpolation to midpoint!");
            return false;
        }

        std::vector<URDFModelCoords> sub_path0;
        std::vector<URDFModelCoords> sub_path1;
        if (!getInterpolatedPath(coords0, coords_half, resolution, sub_path0, depth + 1, max_depth)) {
            ROS_ERROR("URDFCollisionModel::getInterpolatedPath - failed to get interpolated sub-path 1");
            return false;
        }
        if (!getInterpolatedPath(coords_half, coords1, resolution, sub_path1, depth + 1, max_depth)) {
            ROS_ERROR("URDFCollisionModel::getInterpolatedPath - failed to get interpolated sub-path 2");
            return false;
        }

        path.insert(path.end(), sub_path0.begin(), sub_path0.end());
        path.insert(path.end(), sub_path1.begin() + 1, sub_path1.end()); //don't add duplicate of the midpoint
        return true;
    }
    else {
        //no need to subdivide further - interpolated path only c0 to c1
        path.push_back(coords0);
        path.push_back(coords1);
        return true;
    }
}

bool URDFCollisionModel::hasAttachedObject(
    const std::string &link_name,
    const std::string &object_name) const
{
    auto it = attached_objects_.find(link_name);
    if (it == attached_objects_.end()) {
        return false;
    }
    for (const AttachedObject &obj : it->second) {
        if (obj.name == object_name) {
            return true;
        }
    }
    return false;
}

bool URDFCollisionModel::attachObject(
    const std::string &link_name,
    const AttachedObject &obj)
{
    auto it = attached_objects_.find(link_name);
    if (it == attached_objects_.end()) {
        //no attached objects on this joint
        attached_objects_[link_name] = {obj};
        return true;
    }
    it->second.push_back(obj);
    return true;
}

} // namespace adim
