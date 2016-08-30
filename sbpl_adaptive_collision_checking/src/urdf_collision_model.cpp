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

#include <sbpl_geometry_utils/voxelize.h>

namespace sbpl_adaptive_collision_checking {

URDFCollisionModel::URDFCollisionModel() :
#ifndef __ROS_DISTRO_groovy__
    rm_loader_(nullptr),
    robot_model_(nullptr),
    robot_state_(nullptr),
    urdf_(nullptr),
#endif
    bFixedRoot(true)
{
    num_coords_ = 0;
    num_active_joints_ = 0;
    spinner = new ros::AsyncSpinner(1);
    spinner->start();
}

URDFCollisionModel::~URDFCollisionModel()
{

}

URDFModelCoords_t URDFCollisionModel::getDefaultCoordinates() const
{
    URDFModelCoords_t c;

    c.root = Eigen::Affine3d::Identity();
    c.collision_links = links_with_collision_spheres_;
    c.contact_links = links_with_contact_spheres_;

    const std::vector<robot_model::JointModel*> joints = robot_model_->getJointModels();

    for (robot_model::JointModel* j : joints) {
        std::vector<double> vals;
#ifdef __ROS_DISTRO_groovy__
        j->getVariableDefaultValues(vals);
#else
        vals.resize(j->getVariableCount());
        j->getVariableDefaultPositions(&vals[0]);
#endif
        c.coordmap[j->getName()] = vals;
    }

    return c;
}

URDFModelCoords_t URDFCollisionModel::getRandomCoordinates() const
{
    URDFModelCoords_t c;

    c.root = Eigen::Affine3d::Identity();
    c.collision_links = links_with_collision_spheres_;
    c.contact_links = links_with_contact_spheres_;

    const std::vector<robot_model::JointModel*> joints =
            robot_model_->getJointModels();

    for (robot_model::JointModel* j : joints) {
        std::vector<double> vals;
#ifdef __ROS_DISTRO_groovy__
        //j->getVariableDefaultValues(vals);
        j->getVariableRandomPositions(robot_state_->getRandomNumberGenerator(), vals);
#else
        vals.resize(j->getVariableCount());
        do {
            j->getVariableRandomPositions(
                    robot_state_->getRandomNumberGenerator(), &vals[0]);
        }
        while (!j->enforcePositionBounds(&vals[0]));
#endif
        c.coordmap[j->getName()] = vals;
    }

    return c;
}

bool URDFCollisionModel::initFromURDF(
    const std::string& urdf_string,
    const std::string& srdf_string)
{
    urdf_ = boost::shared_ptr<urdf::Model>(new urdf::Model());
    if (!urdf_->initString(urdf_string)) {
        ROS_WARN("Failed to parse the URDF");
        return false;
    }
    if (!initRobotModelFromURDF(urdf_string, srdf_string)) {
        ROS_WARN("Failed to load robot model from URDF/SRDF");
        return false;
    }
    if (!loadKDLModel()) {
        ROS_WARN("Could not load KDL model!");
        return false;
    }

    robot_state_->setToDefaultValues();

    return true;
}

bool URDFCollisionModel::loadKDLModel()
{
    if (!kdl_parser::treeFromUrdfModel(*robot_model_->getURDF().get(), kdl_tree_)) {
        ROS_ERROR("Could not initialize tree object");
        return false;
    }

    // walk the tree and add segments to segments_
    addChildren(kdl_tree_.getRootSegment());

    return true;
}

bool URDFCollisionModel::InitializeChains(
    const std::vector<std::string> &chain_tip_link_names)
{
    std::string root_link_name = robot_model_->getRootLink()->getName();
    for (std::string tip_link : chain_tip_link_names) {
        KDL::Chain chain;
        if (!kdl_tree_.getChain(root_link_name, tip_link, chain)) {
            ROS_ERROR("Failed to init chain for tip link: %s",
                    tip_link.c_str());
            return false;
        }
        kdl_chains_[tip_link] = chain;
        kdl_fk_solvers_[tip_link] = std::unique_ptr<
                KDL::ChainFkSolverPos_recursive>(
                new KDL::ChainFkSolverPos_recursive(chain));
        kdl_ik_solvers_[tip_link] = std::unique_ptr<KDL::ChainIkSolverPos_LMA>(
                new KDL::ChainIkSolverPos_LMA(chain));
        ROS_INFO("Initialized KDL chain for tip %s", tip_link.c_str());
    }
    return true;
}

void URDFCollisionModel::addChildren(
    const KDL::SegmentMap::const_iterator segment)
{
    const std::string& root = segment->second.segment.getName();

    const std::vector<KDL::SegmentMap::const_iterator>& children =
            segment->second.children;
    for (unsigned int i = 0; i < children.size(); i++) {
        const KDL::Segment& child = children[i]->second.segment;
        SegmentPair s(children[i]->second.segment, root, child.getName());
        if (child.getJoint().getType() == KDL::Joint::None) {
            // skip over fixed:
            //      segments_fixed_.insert(make_pair(child.getJoint().getName(), s));
            ROS_DEBUG(
                    "Tree initialization: Skipping fixed segment from %s to %s",
                    root.c_str(), child.getName().c_str());
        }
        else {
            segments_.insert(make_pair(child.getJoint().getName(), s));
            ROS_DEBUG(
                    "Tree initialization: Adding moving segment from %s to %s",
                    root.c_str(), child.getName().c_str());
        }
        addChildren(children[i]);
    }
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

#ifdef __ROS_DISTRO_groovy__
        // one shape during and before groovy
        shapes::ShapeConstPtr object = link->getShape();
#else
        // multiple shapes after than groovy
        std::vector<shapes::ShapeConstPtr> objects = link->getShapes();
        for (shapes::ShapeConstPtr object : objects)
#endif
        {
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

    robot_state_.reset(new robot_state::RobotState(robot_model_));
    if (!robot_state_) {
        ROS_ERROR("Failed to instantiate Robot State");
        return false;
    }

    const robot_model::JointModel* root = robot_model_->getJointModel(robot_model_->getRootJointName());
    bFixedRoot = (root->getType() == robot_model::JointModel::FIXED);

    if (!loadKDLModel()) {
        ROS_WARN("Could not load KDL model!");
        return false;
    }

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

    robot_state_.reset(new robot_state::RobotState(robot_model_));
    if (!robot_state_) {
        ROS_ERROR("Failed to instantiate Robot State");
        return false;
    }

    const robot_model::JointModel* root = robot_model_->getJointModel(
            robot_model_->getRootJointName());
    bFixedRoot = (root->getType() == robot_model::JointModel::FIXED);

    return true;
}

bool URDFCollisionModel::getLinkCollisionSpheres_CurrentState(
    std::string link_name,
    std::vector<Sphere> &spheres) const
{

#ifdef __ROS_DISTRO_groovy__
    robot_state::LinkState* link_state = robot_state_->getLinkState(link_name);
    Eigen::Affine3d tfm = link_state->getGlobalLinkTransform();
#else
    Eigen::Affine3d tfm = robot_state_->getGlobalLinkTransform(link_name);
#endif

    auto it = collision_spheres_.find(link_name);
    if (it == collision_spheres_.end()) {
        //no spheres on this link
        return true;
    }
    for (Sphere s : it->second) {

        Sphere s_;
        s_.name_ = s.name_;
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
    const URDFModelCoords_t &coords,
    std::string link_name,
    std::vector<Sphere> &spheres) const
{
    if (!updateFK(coords)) {
        ROS_ERROR("URDFCollisionModel::getLinkCollisionSpheres - failed to update FK!");
        return false;
    }
    return getLinkCollisionSpheres_CurrentState(link_name, spheres);
}

bool URDFCollisionModel::getLinkContactSpheres(
    const URDFModelCoords_t &coords,
    std::string link_name,
    std::vector<Sphere> &spheres) const
{
    if (!updateFK(coords)) {
        ROS_ERROR(
                "URDFCollisionModel::getLinkContactSpheres - failed to update FK!");
        return false;
    }
    return getLinkContactSpheres_CurrentState(link_name, spheres);
}

/// \brief Return the contact spheres for a link in the current state
bool URDFCollisionModel::getLinkContactSpheres_CurrentState(
    std::string link_name,
    std::vector<Sphere> &spheres) const
{
#ifdef __ROS_DISTRO_groovy__
    robot_state::LinkState* link_state = robot_state_->getLinkState(link_name);
    Eigen::Affine3d tfm = link_state->getGlobalLinkTransform();
#else
    Eigen::Affine3d tfm = robot_state_->getGlobalLinkTransform(link_name);
#endif

    std::map<std::string, std::vector<Sphere>>::const_iterator it =
            contact_spheres_.find(link_name);
    if (it == contact_spheres_.end()) {
        //no spheres on this link
        return true;
    }

    for (Sphere s : it->second) {
        Sphere s_;
        s_.v = tfm * s.v;
        s_.radius = s.radius;
        spheres.push_back(s_);
    }
    return true;
}

void URDFCollisionModel::autoIgnoreSelfCollisions()
{
    URDFModelCoords_t defaultCoords = getDefaultCoordinates();
    autoIgnoreSelfCollisions(defaultCoords);
}

void URDFCollisionModel::autoIgnoreSelfCollisions(
    const URDFModelCoords_t &coords)
{
    std::vector<std::pair<std::string, std::string>> colliding_links;
    do {
        colliding_links = getSelfCollisions(coords);
        for (auto colliding_pair : colliding_links) {
            if (!hasIgnoreSelfPair(colliding_pair.first, colliding_pair.second)) {
                addIgnoreSelfCollisionLinkPair(colliding_pair);
            }
        }
    }
    while (colliding_links.size() > 0);
}

std::vector<std::pair<std::string, std::string>> URDFCollisionModel::getSelfCollisions(
    const URDFModelCoords_t &coords) const
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
                ROS_ERROR("URDFCollisionModel::getSelfCollisions could not get link %s collision spheres!",
                        link1.c_str());
                throw SBPL_Exception();
            }
            if (!getLinkCollisionSpheres_CurrentState(link2, l2)) {
                ROS_ERROR("URDFCollisionModel::getSelfCollisions could not get link %s collision spheres!",
                        link2.c_str());
                throw SBPL_Exception();
            }
            for (Sphere s1 : l1) {
                for (Sphere s2 : l2) {
                    Eigen::Vector3d d = s1.v - s2.v; //distance between sphere centers
                    if (d.norm() < s1.radius + s2.radius) { //if less than the sum of the radii, then self collision
                        ROS_WARN("Sphere %s [link: %s] in collision with sphere %s [link: %s]",
                                s1.name_.c_str(), link1.c_str(), s2.name_.c_str(), link2.c_str());
                        colliding_links.push_back(std::make_pair(link1, link2));
                    }
                }
            }
        }
    }
    return colliding_links;
}

bool URDFCollisionModel::checkSelfCollisions(
    const URDFModelCoords_t &coords) const
{
    if (!updateFK(coords))
        return false;

    for (int i = 0; i < links_with_collision_spheres_.size(); i++) {
        for (int j = i + 1; j < links_with_collision_spheres_.size(); j++) {
            std::string link1 = links_with_collision_spheres_[i];
            std::string link2 = links_with_collision_spheres_[j];

            if (hasIgnoreSelfPair(link1, link2))
                continue;

            std::vector<Sphere> l1;
            std::vector<Sphere> l2;

            if (!getLinkCollisionSpheres_CurrentState(link1, l1))
                return false;
            if (!getLinkCollisionSpheres_CurrentState(link2, l2))
                return false;
            for (Sphere s1 : l1) {
                for (Sphere s2 : l2) {
                    Eigen::Vector3d d = s1.v - s2.v; //distance between sphere centers
                    if (d.norm() < s1.radius + s2.radius) { //if less than the sum of the radii, then self collision
                    //ROS_WARN("Sphere %s [link: %s] in collision with sphere %s [link: %s]", s1.name_.c_str(), link1.c_str(), s2.name_.c_str(), link2.c_str());
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

bool URDFCollisionModel::getLinkGlobalTransform(
    const URDFModelCoords_t &coords,
    const std::string &link_name,
    Eigen::Affine3d &tfm) const
{
    if (!updateFK(coords))
        return false;
#ifdef __ROS_DISTRO_groovy__
    robot_state::LinkState* link_state = robot_state_->getLinkState(link_name);
    tfm = link_state->getGlobalLinkTransform();
#else
    tfm = robot_state_->getGlobalLinkTransform(link_name);
#endif
    return true;
}

bool URDFCollisionModel::checkLimits(const URDFModelCoords_t &coords) const
{
    if (!robot_model_) {
        ROS_ERROR("robot_model_ not initialized!");
        throw SBPL_Exception();
    }
    for (std::map<std::string, std::vector<double>>::const_iterator it =
            coords.coordmap.begin(); it != coords.coordmap.end(); it++) {
        std::string joint_name = it->first;
        const robot_model::JointModel* jm = robot_model_->getJointModel(
                joint_name);

        if (jm == NULL) {
            ROS_ERROR("Could not get joint model for joint %s",
                    joint_name.c_str());
            throw SBPL_Exception();
        }

        int var_count = jm->getVariableCount();
        robot_model::JointModel::Bounds bounds = jm->getVariableBounds();

        if (var_count != it->second.size()) {
            ROS_ERROR(
                    "URDFCollisionModel::getModelCollisionSpheres -- coords.size() (%d) != joint #vars (%d) for joint %s!",
                    (int )it->second.size(), var_count, joint_name.c_str());
            throw SBPL_Exception();
            return false;
        }
        for (int j = 0; j < var_count; j++) {
#ifdef __ROS_DISTRO_groovy__
            if(bounds[j].first < bounds[j].second) {
                if(it->second[j] < bounds[j].first || it->second[j] > bounds[j].second) {
                    ROS_WARN("Joint %s var %d [%.3f] outside limits [%.3f to %.3f]",
                            joint_name.c_str(), j, it->second[j], bounds[j].first, bounds[j].second);
                    return false;
                }
            }
#else
            if (bounds[j].position_bounded_) {
                if (it->second[j] < bounds[j].min_position_
                        || it->second[j] > bounds[j].max_position_) {
                    ROS_WARN(
                            "Joint %s var %d [%.3f] outside limits [%.3f to %.3f]",
                            joint_name.c_str(), j, it->second[j],
                            bounds[j].min_position_, bounds[j].max_position_);
                    return false;
                }
            }
#endif
        }
    }
    return true;
}

bool URDFCollisionModel::getModelCollisionSpheres(
    const URDFModelCoords_t &coords,
    std::vector<Sphere> &spheres) const
{
    if (!updateFK(coords))
        return false;
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

bool URDFCollisionModel::updateFK(const URDFModelCoords_t &coords) const
{
    // set the root joint first
    const robot_model::LinkModel* rootlink = robot_model_->getRootLink();
    const robot_model::JointModel* root = robot_model_->getRootJoint();

#ifdef __ROS_DISTRO_groovy__
    robot_state::JointState* root_state = robot_state_->getJointState(root);
    root_state->setVariableValues(coords.root);
#else
    robot_state_->setJointPositions(root, coords.root);
#endif

    // then go through the other joints and set them
    for (auto it = coords.coordmap.begin(); it != coords.coordmap.end(); it++) {
        std::string joint_name = it->first;
        const robot_model::JointModel* jm = robot_model_->getJointModel(joint_name);
        int var_count = jm->getVariableCount();

        if (var_count != it->second.size()) {
            ROS_ERROR("URDFCollisionModel::getModelCollisionSpheres -- coords.size() (%d) != joint #vars (%d) for joint %s!",
                    (int)it->second.size(), var_count, joint_name.c_str());
            throw SBPL_Exception();
            return false;
        }
#ifdef __ROS_DISTRO_groovy__
        robot_state::JointState* js = robot_state_->getJointState(jm);
        js->setVariableValues(it->second);
#else
        robot_state_->setJointPositions(jm, it->second);
#endif
    }
    robot_state_->update();
    robot_state_->updateLinkTransforms();
    robot_state_->updateCollisionBodyTransforms();
    ros::spinOnce();

    //Eigen::Affine3d rootlink2 = robot_state_->getGlobalLinkTransform(rootlink);
    //printf("RootLinkeAfter:\n");
    //URDFModelCoords_t::print(rootlink2);

    return true;
}

/// \brief Get the contact spheres for a state
///
/// If the input state specifies a set of contact links, this method returns
/// the contact spheres for that set of links; otherwise, this method returns
/// returns the contact spheres for all links in the model.
bool URDFCollisionModel::getModelContactSpheres(
    const URDFModelCoords_t &coords,
    std::vector<Sphere> &spheres) const
{
    if (!updateFK(coords)) {
        return false;
    }
    if (coords.contact_links.empty()) {
        for (std::string link_name : links_with_contact_spheres_) {
            if (!getLinkContactSpheres_CurrentState(link_name, spheres)) {
                return false;
            }
        }
    }
    else {
        for (std::string link_name : coords.contact_links) {
            if (!getLinkContactSpheres_CurrentState(link_name, spheres)) {
                return false;
            }
        }
    }
    return true;
}

bool URDFCollisionModel::getInterpolatedPath(
    const URDFModelCoords_t &coords0,
    const URDFModelCoords_t &coords1,
    double resolution,
    std::vector<URDFModelCoords_t> &path) const
{
    //compute max_dist that collision spheres travel between c0 and c1
    double max_dist = 0.0;
    path.clear();
    for (std::string link : links_with_collision_spheres_) {
        std::vector<Sphere> s0;
        std::vector<Sphere> s1;
        if (!getLinkCollisionSpheres(coords0, link, s0)) {
            ROS_ERROR(
                    "URDFCollisionModel::getInterpolatedPath - Failed to get link %s collision spheres",
                    link.c_str());
            return false;
        }
        if (!getLinkCollisionSpheres(coords1, link, s1)) {
            ROS_ERROR(
                    "URDFCollisionModel::getInterpolatedPath - Failed to get link %s collision spheres",
                    link.c_str());
            return false;
        }
        if (s0.size() != s1.size()) {
            ROS_ERROR(
                    "URDFCollisionModel::getInterpolatedPath - Different number of spheres found between coords0 and coords1!");
            return false;
        }
        for (int s = 0; s < s0.size(); s++) {
            double dist = (s0[s].v - s1[s].v).norm();
            if (dist > max_dist) {
                /*ROS_INFO("MaxDist for link %s = %.3f [%s, %s]", link.c_str(), dist, s0[s].name_.c_str(), s1[s].name_.c_str());
                 printf("Sphere at c0\n");
                 Sphere::print(s0[s]);
                 printf("Sphere at c1\n");
                 Sphere::print(s1[s]);*/
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
    //if max_dist is more than resolution, we need to subdivide path between c0 and c1
    if (max_dist > resolution) {
        //ROS_WARN("Sphere MaxDist = %.3f", max_dist);
        URDFModelCoords_t coords_half;
        if (!getInterpolatedCoordinates(coords0, coords1, 0.5, coords_half)) {
            ROS_ERROR(
                    "URDFCollisionModel::getInterpolatedPath - failed to get interpolation to midpoint!");
            return false;
        }

        /*ROS_INFO("Wp1:");
         coords0.print(coords0);
         ROS_INFO("Wp2:");
         coords1.print(coords1);
         ROS_INFO("Wp_mid:");
         coords_half.print(coords_half);*/

        std::vector<URDFModelCoords_t> sub_path0;
        std::vector<URDFModelCoords_t> sub_path1;
        if (!getInterpolatedPath(coords0, coords_half, resolution, sub_path0)) {
            ROS_ERROR(
                    "URDFCollisionModel::getInterpolatedPath - failed to get interpolated sub-path 1");
            return false;
        }
        if (!getInterpolatedPath(coords_half, coords1, resolution, sub_path1)) {
            ROS_ERROR(
                    "URDFCollisionModel::getInterpolatedPath - failed to get interpolated sub-path 2");
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

bool URDFCollisionModel::getInterpolatedCoordinates(
    const URDFModelCoords_t &coords0,
    const URDFModelCoords_t &coords1,
    double t,
    URDFModelCoords_t &interp) const
{

    const robot_model::JointModel* root = robot_model_->getRootJoint();
    if (root == NULL) {
        ROS_ERROR("Could not find root joint!");
        return false;
    }
    int root_n_vars = root->getVariableCount();

    robot_state_->setJointPositions(root, coords0.root);
    const double* pos0 = robot_state_->getJointPositions(root);
    std::vector<double> root_vars0(root_n_vars);
    memcpy(&root_vars0[0], pos0, root_n_vars * sizeof(double));

    robot_state_->setJointPositions(root, coords1.root);
    const double* pos1 = robot_state_->getJointPositions(root);
    std::vector<double> root_vars1(root_n_vars);
    memcpy(&root_vars1[0], pos1, root_n_vars * sizeof(double));

    std::vector<double> root_varsT(root_n_vars);
    root->interpolate(&root_vars0[0], &root_vars1[0], t, &root_varsT[0]);
    robot_state_->setJointPositions(root, root_varsT);
    interp.root = robot_state_->getJointTransform(root);

    for (std::map<std::string, std::vector<double>>::const_iterator it0 =
            coords0.coordmap.begin(); it0 != coords0.coordmap.end(); it0++) {
        std::string joint_name = it0->first;
        std::vector<double> j0_pos = it0->second;
        std::vector<double> j1_pos;
        std::vector<double> jt_pos(j0_pos.size());
        if (!coords1.getCoords(joint_name, j1_pos)) {
            ROS_ERROR("Could not find joint %s in coords1", joint_name.c_str());
            return false;
        }

        const robot_model::JointModel* jm = robot_model_->getJointModel(
                joint_name);
        if (jm == NULL) {
            ROS_ERROR(
                    "URDFCollisionModel::getInterpolatedPath -- Joint %s not found in model!",
                    joint_name.c_str());
            throw SBPL_Exception();
            return false;
        }
        robot_model::JointModel::Bounds bounds = jm->getVariableBounds();
        if (jm->getVariableCount() != j0_pos.size()) {
            ROS_ERROR(
                    "URDFCollisionModel::getInterpolatedPath -- %s : coords0.size() [%d] != jointVarCount [%d]",
                    joint_name.c_str(), (int )j0_pos.size(),
                    (int )jm->getVariableCount());
            throw SBPL_Exception();
            return false;
        }
        if (jm->getVariableCount() != j1_pos.size()) {
            ROS_ERROR(
                    "URDFCollisionModel::getInterpolatedPath -- %s : coords1.size() [%d] != jointVarCount [%d]",
                    joint_name.c_str(), (int )j1_pos.size(),
                    (int )jm->getVariableCount());
            throw SBPL_Exception();
            return false;
        }
#ifdef __ROS_DISTRO_groovy__
        jm->interpolate(j0_pos, j1_pos, tt, jt_pos);
#else
        jm->interpolate(&j0_pos[0], &j1_pos[0], t, &jt_pos[0]);
        for (int j = 0; j < jt_pos.size(); j++) {
            if (bounds[j].position_bounded_) {
                if (bounds[j].min_position_ > jt_pos[j]
                        || jt_pos[j] > bounds[j].max_position_) {
                    ROS_ERROR("Interpolation out of bounds! %.3f [%.3f, %.3f]",
                            jt_pos[j], bounds[j].min_position_,
                            bounds[j].max_position_);
                    jt_pos[j] = ALERP(j0_pos[j], j1_pos[j], t);
                    if (bounds[j].min_position_ > jt_pos[j]
                            || jt_pos[j] > bounds[j].max_position_) {
                        ROS_ERROR(
                                "Interpolation still out of bounds! %.3f [%.3f, %.3f]",
                                jt_pos[j], bounds[j].min_position_,
                                bounds[j].max_position_);
                        throw SBPL_Exception();
                    }
                }
            }
        }

#endif
        interp.set(joint_name, jt_pos);
    }
    return true;
}

bool URDFCollisionModel::getInterpolatedPath(
    const URDFModelCoords_t &coords0,
    const URDFModelCoords_t &coords1,
    int steps,
    std::vector<URDFModelCoords_t> &path) const
{

    path.clear();
    path.resize(steps);

    for (int t = 0; t < steps; t++) {
        double tt = t / (steps - 1);

        for (std::map<std::string, std::vector<double>>::const_iterator it0 =
                coords0.coordmap.begin(); it0 != coords0.coordmap.end();
                it0++) {
            std::string joint_name = it0->first;
            std::vector<double> j0_pos = it0->second;
            std::vector<double> j1_pos;
            std::vector<double> jt_pos(j0_pos.size());
            if (!coords1.getCoords(joint_name, j1_pos)) {
                ROS_ERROR("Could not find joint %s in coords1",
                        joint_name.c_str());
                return false;
            }

            const robot_model::JointModel* jm = robot_model_->getJointModel(
                    joint_name);
            if (jm == NULL) {
                ROS_ERROR(
                        "URDFCollisionModel::getInterpolatedPath -- Joint %s not found in model!",
                        joint_name.c_str());
                throw SBPL_Exception();
                return false;
            }
            if (jm->getVariableCount() != j0_pos.size()) {
                ROS_ERROR(
                        "URDFCollisionModel::getInterpolatedPath -- %s : coords0.size() [%d] != jointVarCount [%d]",
                        joint_name.c_str(), (int )j0_pos.size(),
                        (int )jm->getVariableCount());
                throw SBPL_Exception();
                return false;
            }
            if (jm->getVariableCount() != j1_pos.size()) {
                ROS_ERROR(
                        "URDFCollisionModel::getInterpolatedPath -- %s : coords1.size() [%d] != jointVarCount [%d]",
                        joint_name.c_str(), (int )j1_pos.size(),
                        (int )jm->getVariableCount());
                throw SBPL_Exception();
                return false;
            }
#ifdef __ROS_DISTRO_groovy__
            jm->interpolate(j0_pos, j1_pos, tt, jt_pos);
#else
            jm->interpolate(&j0_pos[0], &j1_pos[0], tt, &jt_pos[0]);
#endif
            path[t].set(joint_name, jt_pos);
        }
    }
    return true;
}

bool URDFCollisionModel::getModelPathCollisionSpheres(
    const URDFModelCoords_t &coords0,
    const URDFModelCoords_t &coords1,
    int steps,
    std::vector<Sphere> &spheres) const
{
    std::vector<URDFModelCoords_t> path;
    if (!getInterpolatedPath(coords0, coords1, steps, path))
        return false;
    for (int i = 0; i < path.size(); i++) {
        if (!getModelCollisionSpheres(path[i], spheres))
            return false;
    }
    return true;
}

bool URDFCollisionModel::getModelPathContactSpheres(
    const URDFModelCoords_t &coords0,
    const URDFModelCoords_t &coords1,
    int steps,
    std::vector<Sphere> &spheres) const
{
    std::vector<URDFModelCoords_t> path;
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
    const URDFModelCoords_t &coords,
    std::string frame_id,
    std::string ns,
    std_msgs::ColorRGBA col,
    int &idx) const
{
    visualization_msgs::MarkerArray markers;

    for (std::map<std::string, std::vector<AttachedObject_t>>::const_iterator it =
            attached_objects_.begin(); it != attached_objects_.end(); it++) {
        std::string link_name = it->first;
        Eigen::Affine3d tfm;
        if (!getLinkGlobalTransform(coords, link_name, tfm))
            continue;
        for (const AttachedObject_t &o : it->second) {
            for (const Sphere s : o.spheres) {
                Sphere s_;
                s_.v = tfm * s.v;
                s_.radius = s.radius;
                visualization_msgs::Marker m = getSphereMarker(s_,
                        ns + "_" + o.name, frame_id, col, idx);
                markers.markers.push_back(m);
            }
        }
    }

    return markers;
}

//get more advanced mesh visualization when available
visualization_msgs::MarkerArray URDFCollisionModel::getModelVisualization(
    const URDFModelCoords_t &coords,
    std::string frame_id,
    std::string ns,
    std_msgs::ColorRGBA col,
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
    const URDFModelCoords_t &coords,
    std::string frame_id,
    std::string ns,
    int &idx) const
{
    visualization_msgs::MarkerArray markers;
    std_msgs::ColorRGBA col;
    int i = 0;
    for (std::string link : links_with_collision_spheres_) {
        std::vector<Sphere> spheres;
        col = fromHSV(240 * i / (double)links_with_collision_spheres_.size(), 1,
                1);
        if (getLinkCollisionSpheres(coords, link, spheres)) {
            for (Sphere s : spheres) {
                visualization_msgs::Marker marker = getSphereMarker(s,
                        ns + "_" + link + "_collision_spheres", frame_id, col,
                        idx);
                markers.markers.push_back(marker);
            }
        }
        i++;
    }
    col.a = 0.5 * col.a;
    i = 0;
    for (std::string link : links_with_contact_spheres_) {
        std::vector<Sphere> contact_spheres;
        col = fromHSV(240 * i / (double)links_with_contact_spheres_.size(), 1,
                1);
        col.a = 0.5;
        if (getLinkContactSpheres(coords, link, contact_spheres)) {
            int id = 0;
            for (Sphere s : contact_spheres) {
                visualization_msgs::Marker marker = getSphereMarker(s,
                        ns + "_" + link + "_contact_spheres", frame_id, col,
                        id);
            }
        }
        i++;
    }
    return markers;
}

visualization_msgs::MarkerArray
URDFCollisionModel::getModelSelfCollisionVisualization(
    const URDFModelCoords_t &coords,
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
    const URDFModelCoords_t &coords,
    std::string frame_id,
    std::string ns,
    std_msgs::ColorRGBA col,
    int &idx) const
{
    visualization_msgs::MarkerArray markers;
    std::vector<Sphere> spheres;
    if (getModelCollisionSpheres(coords, spheres)) {
        for (Sphere s : spheres) {
            visualization_msgs::Marker marker = getSphereMarker(s,
                    ns + "_collision_spheres", frame_id, col, idx);
            markers.markers.push_back(marker);
        }
    }
    col.a = 0.5 * col.a;
    std::vector<Sphere> contact_spheres;
    if (getModelContactSpheres(coords, contact_spheres)) {
        for (Sphere s : contact_spheres) {
            visualization_msgs::Marker marker = getSphereMarker(s,
                    ns + "_contact_spheres", frame_id, col, idx);
            markers.markers.push_back(marker);
        }
    }
    //ROS_INFO("Got %d spheres in basic visualization!", (int)markers.markers.size());
    return markers;
}

bool URDFCollisionModel::computeCOMRecurs(
    const KDL::SegmentMap::const_iterator& current_seg,
    const URDFModelCoords_t &joint_positions,
    const KDL::Frame& tf,
    double& m,
    KDL::Vector& com) const
{

    std::vector<double> jnt_p;

    if (current_seg->second.segment.getJoint().getType() != KDL::Joint::None) {
        if (!joint_positions.getCoords(current_seg->second.segment.getJoint().getName(), jnt_p)) {
            ROS_DEBUG("Could not find joint %s of %s in joint positions. Aborting tree branch.",
                    current_seg->second.segment.getJoint().getName().c_str(),
                    current_seg->first.c_str());
            return true;
        }
        if (jnt_p.size() > 1) {
            ROS_WARN("Got %d values for joint %s of segment %s",
                    (int )jnt_p.size(),
                    current_seg->second.segment.getJoint().getName().c_str(),
                    current_seg->first.c_str());
        }
    }

    KDL::Frame current_frame = tf * current_seg->second.segment.pose(jnt_p[0]);

    KDL::Vector current_cog = current_seg->second.segment.getInertia().getCOG();
    //ROS_INFO("Got CoG: %.3f, %.3f, %.3f", current_cog.x(), current_cog.y(), current_cog.z());
    double current_m = current_seg->second.segment.getInertia().getMass();
    //ROS_INFO("Got mass: %.3f", current_m);

    com = com + current_m * (current_frame * current_cog);

    m += current_m;
    //ROS_INFO("At link %s. local: %f / [%f %f %f]; global: %f / [%f %f %f]",current_seg->first.c_str(), current_m, current_cog.x(), current_cog.y(), current_cog.z(),
    // m, com.x(), com.y(), com.z());

    std::vector<KDL::SegmentMap::const_iterator>::const_iterator child_it;
    for (child_it = current_seg->second.children.begin();
            child_it != current_seg->second.children.end(); ++child_it) {
        if (!computeCOMRecurs(*child_it, joint_positions, current_frame, m, com)) {
            return false;
        }
    }

    return true;
}

bool URDFCollisionModel::computeCOM(
    const URDFModelCoords_t &joint_positions,
    KDL::Vector& CoM,
    double& mass) const
{
    mass = 0.0;
    KDL::Vector com;
    KDL::Frame ident = KDL::Frame::Identity();
    KDL::Frame transform = ident;
    KDL::Frame right_foot_tf = ident;
    KDL::Frame left_foot_tf = ident;

    if (!computeCOMRecurs(kdl_tree_.getRootSegment(), joint_positions,
            transform, mass, com)) {
        ROS_WARN("Failed to compute CoM recursively!");
        return false;
    }

    if (mass <= 0.0) {
        ROS_WARN("Total mass is 0, no CoM possible.");
        return false;
    }

    com = 1.0 / mass * com;
    ROS_DEBUG("Total mass: %f CoG: (%f %f %f)", mass, com.x(), com.y(),
            com.z());

    CoM = com;
    return true;
}

bool URDFCollisionModel::computeChainTipPosesRecurs(
    const KDL::SegmentMap::const_iterator& current_seg,
    const URDFModelCoords_t &joint_positions,
    const KDL::Frame& tf,
    std::map<std::string, KDL::Frame> &tip_frames)
{
    std::vector<double> jnt_p;

    std::string joint_name = current_seg->second.segment.getJoint().getName();
    std::string link_name = current_seg->second.segment.getName();

    if (current_seg->second.segment.getJoint().getType() != KDL::Joint::None) {
        if (!joint_positions.getCoords(joint_name, jnt_p)) {
            ROS_WARN(
                    "Could not find joint %s of %s in joint positions. Aborting tree branch.",
                    joint_name.c_str(), current_seg->first.c_str());
            return false;
        }
        if (jnt_p.size() > 1) {
            ROS_WARN("Got %d values for joint %s of segment %s [%s]",
                    (int )jnt_p.size(), joint_name.c_str(),
                    current_seg->first.c_str(), link_name.c_str());
        }
    }

    KDL::Frame current_frame = tf * current_seg->second.segment.pose(jnt_p[0]);

    std::map<std::string, KDL::Chain>::iterator it = kdl_chains_.find(
            joint_name);
    if (it != kdl_chains_.end()) {
        //this joint is chain tip!
        tip_frames[joint_name] = current_frame;
    }

    it = kdl_chains_.find(link_name);
    if (it != kdl_chains_.end()) {
        //this joint is chain tip!
        tip_frames[link_name] = current_frame;
    }

    std::vector<KDL::SegmentMap::const_iterator>::const_iterator child_it;
    for (child_it = current_seg->second.children.begin();
            child_it != current_seg->second.children.end(); ++child_it) {
        if (!computeChainTipPosesRecurs(*child_it, joint_positions,
                current_frame, tip_frames)) {
            return false;
        }
    }

    return true;
}

bool URDFCollisionModel::computeChainTipPoses(
    const URDFModelCoords_t &coords,
    std::map<std::string, Eigen::Affine3d> &tip_poses)
{
    std::map<std::string, KDL::Frame> tip_frames;

    KDL::Frame transform = KDL::Frame::Identity();
    if (!computeChainTipPosesRecurs(kdl_tree_.getRootSegment(), coords,
            transform, tip_frames)) {
        return false;
    }
    for (std::map<std::string, KDL::Frame>::iterator it = tip_frames.begin();
            it != tip_frames.end(); it++) {
        Eigen::Affine3d pose;
        tf::transformKDLToEigen(it->second, pose);
        tip_poses[it->first] = pose;
    }
    return true;
}

bool URDFCollisionModel::computeGroupIK(
    const std::string &group_name,
    const Eigen::Affine3d &ee_pose_map,
    const URDFModelCoords_t &seed,
    URDFModelCoords_t &sol,
    bool bAllowApproxSolutions,
    int n_attempts,
    double time_s)
{

    const robot_model::JointModelGroup* joint_model_group =
            robot_model_->getJointModelGroup(group_name);
#ifdef __ROS_DISTRO_groovy__
    robot_state::JointStateGroup* joint_state_group = robot_state_->getJointStateGroup(group_name);
    if(joint_state_group == NULL) {
        ROS_ERROR("Could not get joint state group: %s", group_name.c_str());
        return false;
    }
#endif

    if (joint_model_group == NULL) {
        ROS_ERROR("Could not get joint model group: %s", group_name.c_str());
        return false;
    }

    if (!updateFK(seed))
        return false;

#ifdef __ROS_DISTRO_groovy__
    if(!joint_state_group->setFromIK(ee_pose_in_group_root, 5, 0.1)) {
        return false;
    }
#else

    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = bAllowApproxSolutions;

    if (!robot_state_->setFromIK(joint_model_group, ee_pose_map, n_attempts,
            time_s, moveit::core::GroupStateValidityCallbackFn(), options)) {
        return false;
    }
#endif
    const std::vector<const robot_model::JointModel*> joints =
            joint_model_group->getJointModels();
    for (int j = 0; j < joints.size(); j++) {
#ifdef __ROS_DISTRO_groovy__
        robot_state::JointState* js = robot_state_->getJointState(joints[j]);
        std::vector<double> vals = js->getVariableValues();
#else
        size_t v_n = joints[j]->getVariableCount();
        if (v_n == 0)
            continue;
        const double* v_ = robot_state_->getJointPositions(joints[j]);
        std::vector<double> vals(v_, v_ + v_n);
#endif
        if (vals.size() > 0) {
            sol.set(joints[j]->getName(), vals);
        }
    }

    return true;
}

void URDFCollisionModel::PrintModelInfo() const
{
    ROS_INFO("Root Joint: %s", robot_model_->getRootJointName().c_str());
    ROS_INFO("Root Link: %s", robot_model_->getRootLinkName().c_str());

    ROS_INFO("Active Joints:");
    const std::vector<robot_model::JointModel*> joints =
            robot_model_->getJointModels();
    for (robot_model::JointModel* j : joints) {
        if (j->isPassive())
            continue;
        if (j->getType() == robot_model::JointModel::FIXED)
            continue;
        ROS_INFO("  %s", j->getName().c_str());
        if (j->getVariableCount() > 0) {
            ROS_INFO("\tVariables:");
            std::vector<std::string> varnames = j->getVariableNames();
            robot_model::JointModel::Bounds bounds = j->getVariableBounds();
            for (int i = 0; i < varnames.size(); i++) {
                std::string vn = varnames[i];
#ifdef __ROS_DISTRO_groovy__
                std::pair<double, double> bound = bounds[i];
#else
                robot_model::VariableBounds bound = bounds[i];
#endif
                ROS_INFO("\t\t%s [%.3f : %.3f]", vn.c_str(),
#ifdef __ROS_DISTRO_groovy__
                        bound.first, bound.second
#else
                        bound.min_position_, bound.max_position_
#endif
                        );
            }
        }
    }
    ROS_INFO("Fixed Joints:");
    for (robot_model::JointModel* j : joints) {
        if (j->isPassive())
            continue;
        if (j->getType() != robot_model::JointModel::FIXED)
            continue;
        ROS_INFO("  %s", j->getName().c_str());
        if (j->getVariableCount() > 0) {
            ROS_INFO("\tVariables:");
            std::vector<std::string> varnames = j->getVariableNames();
            robot_model::JointModel::Bounds bounds = j->getVariableBounds();
            for (int i = 0; i < varnames.size(); i++) {
                std::string vn = varnames[i];
#ifdef __ROS_DISTRO_groovy__
                std::pair<double, double> bound = bounds[i];
#else
                robot_model::VariableBounds bound = bounds[i];
#endif
                ROS_INFO("\t\t%s [%.3f : %.3f]", vn.c_str(),
#ifdef __ROS_DISTRO_groovy__
                        bound.first, bound.second
#else
                        bound.min_position_, bound.max_position_
#endif
                        );
            }
        }
    }

    ROS_INFO("Links:");
    const std::vector<robot_model::LinkModel*> links =
            robot_model_->getLinkModels();
    for (robot_model::LinkModel* l : links) {
        ROS_INFO("\t%s", l->getName().c_str());
    }
    ROS_INFO("Groups:");
    std::vector<std::string> group_names =
            robot_model_->getJointModelGroupNames();
    for (std::string gn : group_names) {
        const robot_model::JointModelGroup* group =
                robot_model_->getJointModelGroup(gn);
        const std::vector<const robot_model::JointModel*> roots =
                group->getJointRoots();
        ROS_INFO("\t%s:", gn.c_str());
        ROS_INFO("\tRoot joints:");
        for (const robot_model::JointModel* r : roots) {
            ROS_INFO("\t\t%s", r->getName().c_str());
        }
        ROS_INFO("\tEnd effector: %s", group->getEndEffectorName().c_str());
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
        ROS_ERROR("Object with name %s already attached to joint %s",
                object_name.c_str(), link_name.c_str());
        return false;
    }

    const robot_model::LinkModel* link = robot_model_->getLinkModel(link_name);
    if (link == NULL) {
        ROS_ERROR("Could not find link %s", link_name.c_str());
        return false;
    }

    AttachedObject_t obj;

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
    std::map<std::string, std::vector<AttachedObject_t>>::const_iterator it =
            attached_objects_.find(link_name);
    return (it != attached_objects_.end());
}

const std::vector<AttachedObject_t> URDFCollisionModel::getAttachedObjects(
    const std::string &link_name) const
{
    std::map<std::string, std::vector<AttachedObject_t>>::const_iterator it =
            attached_objects_.find(link_name);
    if (it == attached_objects_.end())
        return {};
    return it->second;
}

bool URDFCollisionModel::getLinkAttachedObjectsSpheres(
    const std::string &link_name,
    const Eigen::Affine3d link_tfm,
    std::vector<Sphere> &spheres) const
{
    if (!hasAttachedObjects(link_name))
        return true;
    const std::vector<AttachedObject_t> objs = getAttachedObjects(link_name);

    ROS_INFO("Got %d attached objects for link %s", (int )objs.size(),
            link_name.c_str());

    for (const AttachedObject_t &o : objs) {
        ROS_INFO("Object %s has %d spheres!", o.name.c_str(),
                (int )o.spheres.size());
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

bool URDFCollisionModel::hasAttachedObject(
    const std::string &link_name,
    const std::string &object_name) const
{
    std::map<std::string, std::vector<AttachedObject_t>>::const_iterator it =
            attached_objects_.find(link_name);
    if (it == attached_objects_.end()) {
        return false;
    }
    for (const AttachedObject_t &obj : it->second) {
        if (obj.name == object_name) {
            return true;
        }
    }
    return false;
}

bool URDFCollisionModel::attachObject(
    const std::string &link_name,
    const AttachedObject_t &obj)
{
    std::map<std::string, std::vector<AttachedObject_t>>::iterator it =
            attached_objects_.find(link_name);
    if (it == attached_objects_.end()) {
        //no attached objects on this joint
        attached_objects_[link_name] = {obj};
        return true;
    }
    it->second.push_back(obj);
    return true;
}

} // namespace sbpl_adaptive_collision_checking
