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
#include <smpl/geometry/bounding_spheres.h>
#include <smpl/geometry/voxelize.h>

namespace adim {

std::ostream &operator<<(std::ostream &o, const URDFModelCoords &c)
{
    o << c.positions;
    return o;
}

URDFCollisionModel::URDFCollisionModel() :
    urdf_(),
    srdf_(),
    links_with_collision_spheres_(),
    links_with_contact_spheres_(),
    self_collision_ignore_pairs_(),
    collision_spheres_(),
    contact_spheres_(),
    attached_objects_(),
    robot_model_(),
    robot_state_()
{
}

URDFCollisionModel::~URDFCollisionModel()
{
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

    robot_model_loader::RobotModelLoader loader(ops);

    return initFromModel(loader.getModel());
}

bool URDFCollisionModel::initFromURDF(
    const std::string &urdf_string,
    const std::string &srdf_string)
{
    auto urdf_model = boost::make_shared<urdf::Model>();
    if (!urdf_model->initString(urdf_string)) {
        ROS_WARN("Failed to parse the URDF");
        return false;
    }
    urdf_ = std::move(urdf_model);

    if (!initRobotModelFromURDF(urdf_string, srdf_string)) {
        ROS_WARN("Failed to load robot model from URDF/SRDF");
        return false;
    }

    robot_state_->setToDefaultValues();

    return true;
}

bool URDFCollisionModel::initFromModel(
    const moveit::core::RobotModelPtr &robot_model)
{
    if (!robot_model) {
        return false;
    }

    robot_model_ = robot_model;
    urdf_ = robot_model->getURDF();
    srdf_ = robot_model->getSRDF();

    robot_state_ = boost::make_shared<moveit::core::RobotState>(robot_model);
    robot_state_->setToDefaultValues();

    return true;
}

moveit::core::RobotStatePtr URDFCollisionModel::getStateAt(
    const URDFModelCoords &coords) const
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    auto state = boost::make_shared<moveit::core::RobotState>(robot_model_);
    updateFK(*state, coords);
    return state;
}

void URDFCollisionModel::PrintModelInfo(std::ostream &o) const
{
    o << "Root Joint: " << robot_model_->getRootJointName() << '\n';
    o << "Root Link: " <<  robot_model_->getRootLinkName() << '\n';

    o << "Active Joints:\n";
    for (auto* j : robot_model_->getJointModels()) {
        if (j->isPassive()) {
            continue;
        }
        if (j->getType() == robot_model::JointModel::FIXED) {
            continue;
        }
        o << "  " << j->getName() << '\n';
        if (j->getVariableCount() > 0) {
            o << "    Variables:\n";
            auto &varnames = j->getVariableNames();
            auto &bounds = j->getVariableBounds();
            for (int i = 0; i < varnames.size(); i++) {
                auto &vn = varnames[i];
                auto &bound = bounds[i];
                o << "      " << vn << " [" << bound.min_position_ << " : " << bound.max_position_ << "]\n";
            }
        }
    }
    o << "Fixed Joints:\n";
    for (auto* j : robot_model_->getJointModels()) {
        if (j->isPassive()) {
            continue;
        }
        if (j->getType() != robot_model::JointModel::FIXED) {
            continue;
        }
        o << "  " << j->getName() << '\n';
        if (j->getVariableCount() > 0) {
            o << "    Variables:\n";
            auto &varnames = j->getVariableNames();
            auto &bounds = j->getVariableBounds();
            for (int i = 0; i < varnames.size(); i++) {
                auto &vn = varnames[i];
                auto &bound = bounds[i];
                o << "      " << vn << " [" << bound.min_position_ << " : " << bound.max_position_ << "]\n";
            }
        }
    }

    o << "Links:\n";
    for (auto* l : robot_model_->getLinkModels()) {
        o << "  " << l->getName();
    }

    o << "Groups:\n";
    for (auto *group : robot_model_->getJointModelGroups()) {
        o << "  " << group->getName() << '\n';
        o << "  Root Joints:\n";
        for (auto* r : group->getJointRoots()) {
            o << "    " << r->getName() << '\n';
        }
        o << "  End Effector: " << group->getEndEffectorName() << '\n';
    }
}

auto URDFCollisionModel::getDefaultCoordinates() const -> URDFModelCoords
{
    robot_state_->setToDefaultValues();

    std::vector<double> vars(
            robot_state_->getVariablePositions(),
            robot_state_->getVariablePositions() + robot_model_->getVariableCount());

    return URDFModelCoords(std::move(vars));
}

bool URDFCollisionModel::checkLimits(const URDFModelCoords &coords) const
{
    robot_state_->setVariablePositions(coords.positions);
    return robot_state_->satisfiesBounds();
}

auto URDFCollisionModel::getLinkGlobalTransform(
    const URDFModelCoords &coords,
    const std::string &link_name) const
    -> const Eigen::Affine3d &
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    updateFK(coords);
    return robot_state_->getGlobalLinkTransform(link_name);
}

auto URDFCollisionModel::getLinkGlobalTransform(
    const URDFModelCoords &coords,
    const moveit::core::LinkModel *link) const
    -> const Eigen::Affine3d &
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    updateFK(coords);
    return robot_state_->getGlobalLinkTransform(link);
}

/// Compute an IK solution for the default tip of a joint group, using the
/// underlying kinematics solver attached to the RobotModel. The seed state is
/// adjusted to fit within joint limits. If the underlying solver returns
/// solutions invalid with respect to joint limits, the solution will be
/// adjusted only if approximate solutions are allowed, otherwise, this function
/// will return false.
///
/// \param group_name The group to compute IK for
/// \param pose The pose of the tip link in the model frame
/// \param seed The seed state
/// \param sol The solution state; this will contain the adjusted seed state
///     values and the solution values from the joints variables from the joint
///     group
/// \param bAllowApproxSolutions Whether to allow the underlying solver to
///     return approximate solutions
/// \param n_attempts The number of attempts to be used by the underlying solver
/// \param time_s The allowed time given to the underlying solver
///
/// \return true if a valid solution was computed
bool URDFCollisionModel::computeGroupIK(
    const std::string &group_name,
    const Eigen::Affine3d &pose,
    const URDFModelCoords &seed,
    URDFModelCoords &sol,
    bool allow_approx_solutions,
    int num_attempts,
    double timeout)
{
    auto *jmg = robot_model_->getJointModelGroup(group_name);
    if (!jmg) {
        ROS_ERROR("Could not get joint model group: %s", group_name.c_str());
        return false;
    }

    return computeGroupIK(
            jmg,
            pose,
            seed,
            sol,
            allow_approx_solutions, num_attempts, timeout);
}

bool URDFCollisionModel::computeGroupIK(
    const moveit::core::JointModelGroup *group,
    const Eigen::Affine3d &pose,
    const URDFModelCoords &seed,
    URDFModelCoords &sol,
    bool allow_approx_solutions,
    int num_attempts,
    double timeout)
{
    assert(seed.positions.size() == robot_model_->getVariableCount());

    updateFK(seed);

    // keep the solvers happy
    robot_state_->enforceBounds(group);

    // call ik
    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = allow_approx_solutions;
    if (!robot_state_->setFromIK(
            group,
            pose,
            num_attempts,
            timeout,
            moveit::core::GroupStateValidityCallbackFn(),
            options))
    {
        std::vector<double> gpos;
        robot_state_->copyJointGroupPositions(group, gpos);
        ROS_DEBUG("Failed to compute IK for group '%s' to pose { %s } using seed %s", group->getName().c_str(), to_str(pose).c_str(), to_string(gpos).c_str());
        return false;
    }

    if (allow_approx_solutions) {
        robot_state_->enforceBounds(group);
    }
    else {
        // foreach active joint in the joint group
        for (auto *jm : group->getActiveJointModels()) {
            // for revolute joints
            if (jm->getType() == moveit::core::JointModel::REVOLUTE) {
                if (!jm->getVariableBounds()[0].position_bounded_) {
                    // just normalize these...to keep RobotState from being upset
                    robot_state_->enforcePositionBounds(jm);
                }
            }
        }

        if (!robot_state_->satisfiesBounds(group)) {
            ROS_DEBUG("IK Solution angles are out of bounds");
            return false;
        }
    }

    sol.positions.assign(
            robot_state_->getVariablePositions(),
            robot_state_->getVariablePositions() + robot_state_->getVariableCount());

    return true;
}

bool URDFCollisionModel::computeCOM(
    const URDFModelCoords &coords,
    Eigen::Vector3d& com,
    double& mass) const
{
    ROS_DEBUG("compute center of mass: urdf @ %p, root link @ %p", urdf_.get(), urdf_ ? urdf_->getRoot().get() : nullptr);

    com = Eigen::Vector3d::Zero();
    mass = 0.0;

    updateFK(coords);

    // weighted sum of the positions of the centers of gravity for each link by
    // the link's mass
    for (auto* link : robot_model_->getLinkModels()) {
        auto ulink = urdf_->getLink(link->getName());
        if (!ulink) {
            ROS_DEBUG("Link %s not found in the URDF", link->getName().c_str());
            continue;
        }

        auto inertial = ulink->inertial;
        if (inertial) {
            mass += inertial->mass;
            const Eigen::Vector3d cog(
                    inertial->origin.position.x,
                    inertial->origin.position.y,
                    inertial->origin.position.z);
            Eigen::Vector3d p = robot_state_->getGlobalLinkTransform(link) * cog;
            com += inertial->mass * p;
        }
    }

    if (mass <= 0.0) {
        return false;
    }

    com = (1.0 / mass) * com;

    com = robot_state_->getGlobalLinkTransform(robot_model_->getRootLink()).inverse() * com;
    return true;
}

/// Compute a set of collision and contact spheres that bound the surface of the
/// robot's geometry. Overwrites the existing set of collision and contact
/// spheres. Bounding spheres are computed by 3d rasterization of the surface
/// geometry into a voxel grid with cells of size \p res and placing spheres at
/// the center of each occupied cell with radius \p res.
///
/// \param res The radius of computed collision/contact spheres.
/// \param A set of links to be ignored. Their existing collision spheres will
///     be retained and no additional collision spheres will be generated.
/// \param The set of contact links. Their existing contact spheres will be
///     overwritten.
/// \return true
bool URDFCollisionModel::computeSpheresFromURDFModel(
    double res,
    const std::vector<std::string> &ignore_collision_links,
    const std::vector<std::string> &contact_links)
{
    const auto &links = robot_model_->getLinkModels();

    for (const robot_model::LinkModel* link : links) {
        const std::string &link_name = link->getName();
        bool bIgnoreCollision = std::find(
                begin(ignore_collision_links),
                end(ignore_collision_links),
                link_name) != ignore_collision_links.end();
        bool bIsContact = std::find(
                begin(contact_links),
                end(contact_links),
                link_name) != contact_links.end();

        if (bIgnoreCollision && !bIsContact) {
            continue;
        }

        std::vector<Sphere> link_spheres;

        for (const shapes::ShapeConstPtr &object : link->getShapes()) {
            if (!object) {
                continue;
            }

            Eigen::Affine3d pose = Eigen::Affine3d::Identity(); //link->getJointOriginTransform();

            const size_t prev_size = link_spheres.size();

            // append spheres
            computeShapeBoundingSpheres(*object, res, link_spheres);

            // transform the spheres by the pose
            for (size_t i = prev_size; i < link_spheres.size(); ++i) {
                link_spheres[i].v = pose * link_spheres[i].v;
            }

            // give unique names to the spheres and reference the attached link
            for (size_t i = prev_size; i < link_spheres.size(); ++i) {
                Sphere& s = link_spheres[i];
                s.name_ = link->getName() + "_" + std::to_string(i);
                s.link_name_ = link->getName();
            }
        }

        if (!bIgnoreCollision) {
            collision_spheres_[link->getName()] = link_spheres;

            if (std::find(
                    begin(links_with_collision_spheres_),
                    end(links_with_collision_spheres_),
                    link->getName()) == links_with_collision_spheres_.end())
            {
                links_with_collision_spheres_.push_back(link->getName());
            }

            ROS_INFO("Adding %zu collision spheres for link %s", link_spheres.size(), link->getName().c_str());
        }

        if (bIsContact) {
            contact_spheres_[link->getName()] = link_spheres;

            if (std::find(
                    begin(links_with_contact_spheres_),
                    end(links_with_contact_spheres_),
                    link->getName()) == links_with_contact_spheres_.end())
            {
                links_with_contact_spheres_.push_back(link->getName());
            }

            ROS_INFO("Adding %zu contact spheres for link %s", link_spheres.size(), link->getName().c_str());
        }
    }
    return true;
}

/// \brief Add a set of contact spheres to a link
void URDFCollisionModel::addContactSpheres(
    const std::string &link_name,
    const std::vector<Sphere> &s)
{
    for (auto &sp : s) {
        addContactSphere(link_name, sp);
    }
}

/// \brief Add a set of collision spheres to a link
void URDFCollisionModel::addCollisionSpheres(
    const std::string &link_name,
    const std::vector<Sphere> &s)
{
    for (auto &sp : s) {
        addCollisionSphere(link_name, sp);
    }
}

/// \brief Add a contact sphere to a link
///
/// There sphere position relative to the link is specified in the link frame.
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

    auto* link = robot_model_->getLinkModel(link_name);
    if (!link) {
        ROS_ERROR("Could not find link %s", link_name.c_str());
        return false;
    }

    AttachedObject obj;

    if (!computeShapeBoundingSpheres(object, res, obj.spheres)) {
        return false;
    }

    // transform spheres
    for (auto &sphere : obj.spheres) {
        sphere.v = pose * sphere.v;
    }

    ROS_INFO("Got %zu spheres for attached object!", obj.spheres.size());

    // process obj.spheres
    obj.name = object_name;

    for (size_t i = 0; i < obj.spheres.size(); ++i) {
        auto &s = obj.spheres[i];
        s.name_ = obj.name + "_" + std::to_string(i);
        s.link_name_ = link_name;
    }

    return attachObject(link_name, obj);
}

/// Add all pairs of colliding links in the default state to the allowed
/// collision matrix.
void URDFCollisionModel::autoIgnoreSelfCollisions()
{
    URDFModelCoords defaultCoords = getDefaultCoordinates();
    autoIgnoreSelfCollisions(defaultCoords);
}

/// Add all pairs of colliding links in the given state to the allowed
/// collision matrix.
void URDFCollisionModel::autoIgnoreSelfCollisions(
    const URDFModelCoords &coords)
{
    std::vector<std::pair<std::string, std::string>> colliding_links;
    do {
        colliding_links = getSelfCollisions(coords);
        for (auto &colliding_pair : colliding_links) {
            if (!hasIgnoreSelfPair(colliding_pair.first, colliding_pair.second)) {
                addIgnoreSelfCollisionLinkPair(colliding_pair);
                ROS_WARN("Adding ignored self collision pair (%s, %s)", colliding_pair.first.c_str(), colliding_pair.second.c_str());
            }
        }
    }
    while (colliding_links.size() > 0);
}

/// Print the set of ignored self-collision pairs to an output stream.
void URDFCollisionModel::printIgnoreSelfCollisionLinkPairs(std::ostream& o)
{
    o << "{ ";
    for (size_t i = 0; i < self_collision_ignore_pairs_.size(); ++i) {
        const auto &pair = self_collision_ignore_pairs_[i];
        o << "(" << pair.first << ", " << pair.second << ")";
        if (i != self_collision_ignore_pairs_.size() - 1) {
            o << ',';
        }
        o << ' ';
    }
    o << '}';
}

void URDFCollisionModel::addIgnoreSelfCollisionLinkPair(
    const std::pair<std::string, std::string> &pair)
{
    if (!hasIgnoreSelfPair(pair.first, pair.second)) {
        self_collision_ignore_pairs_.push_back(pair);
    }
}

void URDFCollisionModel::addIgnoreSelfCollisionLinkPairs(
    const std::vector<std::pair<std::string, std::string>> &pairs)
{
    for (auto &pair : pairs) {
        addIgnoreSelfCollisionLinkPair(pair);
    }
}

bool URDFCollisionModel::checkSelfCollisions(
    const URDFModelCoords &coords) const
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    updateFK(coords);

    for (int i = 0; i < links_with_collision_spheres_.size(); i++) {
        for (int j = i + 1; j < links_with_collision_spheres_.size(); j++) {
            auto &link1_name = links_with_collision_spheres_[i];
            auto &link2_name = links_with_collision_spheres_[j];

            if (hasIgnoreSelfPair(link1_name, link2_name)) {
                continue;
            }

            std::vector<Sphere> l1;
            std::vector<Sphere> l2;

            if (!getLinkCollisionSpheres_CurrentState(link1_name, l1)) {
                return false;
            }
            if (!getLinkCollisionSpheres_CurrentState(link2_name, l2)) {
                return false;
            }
            for (const auto &s1 : l1) {
                for (const auto &s2 : l2) {
                    auto d = s1.v - s2.v;
                    if (d.norm() < s1.radius + s2.radius) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

std::vector<std::pair<std::string, std::string>>
URDFCollisionModel::getSelfCollisions(const URDFModelCoords &coords) const
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    updateFK(coords);

    std::vector<std::pair<std::string, std::string>> colliding_links;

    for (int i = 0; i < links_with_collision_spheres_.size(); i++) {
        for (int j = i + 1; j < links_with_collision_spheres_.size(); j++) {
            auto &link1 = links_with_collision_spheres_[i];
            auto &link2 = links_with_collision_spheres_[j];

            if (hasIgnoreSelfPair(link1, link2)) {
                continue;
            }

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
            for (auto &s1 : l1) {
                for (auto &s2 : l2) {
                    auto d = s1.v - s2.v; //distance between sphere centers
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

bool URDFCollisionModel::getModelCollisionSpheres(
    const URDFModelCoords &coords,
    std::vector<Sphere> &spheres) const
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    updateFK(coords);

    for (auto &link_name : links_with_collision_spheres_) {
        if (!getLinkCollisionSpheres_CurrentState(link_name, spheres)) {
            return false;
        }
    }
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
    assert(coords.positions.size() == robot_model_->getVariableCount());
    updateFK(coords);

    for (auto &link_name : links_with_contact_spheres_) {
        if (!getLinkContactSpheres_CurrentState(link_name, spheres)) {
            return false;
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
    for (auto &waypoint : path) {
        if (!getModelCollisionSpheres(waypoint, spheres)) {
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
    for (auto &waypoint : path) {
        if (!getModelContactSpheres(waypoint, spheres)) {
            return false;
        }
    }
    return true;
}

bool URDFCollisionModel::getInterpolatedCoordinates(
    const URDFModelCoords &coords0,
    const URDFModelCoords &coords1,
    double t,
    URDFModelCoords &interp) const
{
    moveit::core::RobotState src(robot_model_);
    src.setVariablePositions(coords0.positions.data());

    moveit::core::RobotState dst(robot_model_);
    dst.setVariablePositions(coords1.positions.data());

    moveit::core::RobotState mid(robot_model_);
    src.interpolate(dst, t, mid);

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

bool URDFCollisionModel::getInterpolatedPath(
    const URDFModelCoords &coords0,
    const URDFModelCoords &coords1,
    int steps,
    std::vector<URDFModelCoords> &path) const
{
    path.clear();
    path.reserve(steps);

    moveit::core::RobotState src(robot_model_);
    src.setVariablePositions(coords0.positions.data());

    moveit::core::RobotState dst(robot_model_);
    dst.setVariablePositions(coords1.positions.data());

    moveit::core::RobotState mid(robot_model_);

    for (auto t = 0; t < steps; ++t) {
        auto tt = double(t) / double(steps - 1);

        src.interpolate(dst, tt, mid);

        std::vector<double> vars(
                mid.getVariablePositions(),
                mid.getVariablePositions() + robot_model_->getVariableCount());

        path.push_back(URDFModelCoords(std::move(vars)));
    }
    return true;
}

auto URDFCollisionModel::getModelSelfCollisionVisualization(
    const URDFModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    const std_msgs::ColorRGBA &col,
    int &idx) const
    -> visualization_msgs::MarkerArray
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    updateFK(coords);

    visualization_msgs::MarkerArray ma;

    for (size_t i = 0; i < links_with_collision_spheres_.size(); ++i) {
        for (size_t j = i + 1; j < links_with_collision_spheres_.size(); ++j) {
            const auto &link1_name = links_with_collision_spheres_[i];
            const auto &link2_name = links_with_collision_spheres_[j];

            if (hasIgnoreSelfPair(link1_name, link2_name)) {
                continue;
            }

            std::vector<Sphere> l1;
            std::vector<Sphere> l2;

            if (!getLinkCollisionSpheres_CurrentState(link1_name, l1)) {
                continue;
            }
            if (!getLinkCollisionSpheres_CurrentState(link2_name, l2)) {
                continue;
            }
            for (const auto &s1 : l1) {
                for (const auto &s2 : l2) {
                    auto d = s1.v - s2.v; // distance between sphere centers
                    if (d.norm() < s1.radius + s2.radius) { // if less than the sum of the radii, then self collision
                        auto m = getSphereMarker(s1, ns + "self_collisions", frame_id, col, idx);
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

auto URDFCollisionModel::getModelBasicVisualizationByLink(
    const URDFModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    int &idx) const
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::MarkerArray markers;
    std_msgs::ColorRGBA col;
    auto i = 0;

    for (const auto &link_name : links_with_collision_spheres_) {
        std::vector<Sphere> spheres;
        leatherman::msgHSVToRGB(240.0 * i / (double)links_with_collision_spheres_.size(), 1, 1, col);
        if (getLinkCollisionSpheres(coords, link_name, spheres)) {
            for (const auto &s : spheres) {
                auto marker = getSphereMarker(s, ns + "_" + link_name + "_collision_spheres", frame_id, col, idx);
                markers.markers.push_back(marker);
            }
        }
        ++i;
    }

    i = 0;
    for (const auto &link_name : links_with_contact_spheres_) {
        std::vector<Sphere> contact_spheres;
        leatherman::msgHSVToRGB(240 * i / (double)links_with_contact_spheres_.size(), 1, 1, col);
        col.a = 0.5;
        if (getLinkContactSpheres(coords, link_name, contact_spheres)) {
            int id = 0;
            for (const auto &s : contact_spheres) {
                auto marker = getSphereMarker(s, ns + "_" + link_name + "_contact_spheres", frame_id, col, id);
            }
        }
        ++i;
    }

    return markers;
}

auto URDFCollisionModel::getModelBasicVisualization(
    const URDFModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    std_msgs::ColorRGBA col,
    int &idx) const
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::MarkerArray markers;
    std::vector<Sphere> spheres;

    if (getModelCollisionSpheres(coords, spheres)) {
        for (const auto &s : spheres) {
            auto marker = getSphereMarker(s, ns + "_collision_spheres", frame_id, col, idx);
            markers.markers.push_back(marker);
        }
    }

    col.a = 0.5 * col.a;
    std::vector<Sphere> contact_spheres;
    if (getModelContactSpheres(coords, contact_spheres)) {
        for (const auto &s : contact_spheres) {
            auto marker = getSphereMarker(s, ns + "_contact_spheres", frame_id, col, idx);
            markers.markers.push_back(marker);
        }
    }

    return markers;
}

auto URDFCollisionModel::getModelVisualization(
    const URDFModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    const std_msgs::ColorRGBA &col,
    int &id) const
    -> visualization_msgs::MarkerArray
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    updateFK(coords);

    visualization_msgs::MarkerArray markers;

    const auto include_attached = true;
    robot_state_->getRobotMarkers(
            markers,
            robot_model_->getLinkModelNames(),
            col, ns, ros::Duration(0), include_attached);

    for (auto &marker : markers.markers) {
        marker.id = id++;
    }

    return markers;
}

auto URDFCollisionModel::getAttachedObjectsVisualization(
    const URDFModelCoords &coords,
    const std::string &frame_id,
    const std::string &ns,
    const std_msgs::ColorRGBA &col,
    int &idx) const
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::MarkerArray markers;

    moveit::core::RobotState state(robot_model_);
    state.setVariablePositions(coords.positions.data());

    for (const auto &link_objects : attached_objects_) {
        const auto &link_name = link_objects.first;
        const auto &attached_objects = link_objects.second;

        auto &tfm = state.getGlobalLinkTransform(link_name);

        for (const auto &ao : attached_objects) {
            for (const auto &sphere : ao.spheres) {
                Sphere s;
                s.v = tfm * sphere.v;
                s.radius = sphere.radius;
                auto m = getSphereMarker(s, ns + "_" + ao.name, frame_id, col, idx);
                markers.markers.push_back(m);
            }
        }
    }

    return markers;
}

void URDFCollisionModel::updateFK(const URDFModelCoords &coords) const
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    return updateFK(*robot_state_, coords);
}

void URDFCollisionModel::updateFK(
    moveit::core::RobotState &state,
    const URDFModelCoords &coords) const
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    state.setVariablePositions(coords.positions);
    state.updateLinkTransforms();
}

bool URDFCollisionModel::hasIgnoreSelfPair(
    const std::string &link1,
    const std::string &link2) const
{
    for (int k = 0; k < self_collision_ignore_pairs_.size(); k++) {
        auto &p = self_collision_ignore_pairs_[k];
        if (link1 == p.first && link2 == p.second) {
            return true;
        }
        else if (link1 == p.second && link2 == p.first) {
            return true;
        }
    }
    return false;
}

bool URDFCollisionModel::getLinkCollisionSpheres(
    const URDFModelCoords &coords,
    const std::string &link_name,
    std::vector<Sphere> &spheres) const
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    updateFK(coords);
    return getLinkCollisionSpheres_CurrentState(link_name, spheres);
}

bool URDFCollisionModel::getLinkCollisionSpheres_CurrentState(
    const std::string &link_name,
    std::vector<Sphere> &spheres) const
{
    const auto &tfm = robot_state_->getGlobalLinkTransform(link_name);

    auto it = collision_spheres_.find(link_name);
    if (it != collision_spheres_.end()) {
        for (const auto &sphere : it->second) {
            auto s = Sphere{};
            s.name_ = sphere.name_;
            s.link_name_ = sphere.link_name_;
            s.v = tfm * sphere.v;
            s.radius = sphere.radius;
            spheres.push_back(s);
        }
    }

    if (hasAttachedObjects(link_name)) {
        ROS_INFO("Found attached objects for link %s", link_name.c_str());
        if (!getLinkAttachedObjectsSpheres(link_name, tfm, spheres)) {
            return false;
        }
    }

    return true;
}

bool URDFCollisionModel::getLinkContactSpheres(
    const URDFModelCoords &coords,
    const std::string &link_name,
    std::vector<Sphere> &spheres) const
{
    assert(coords.positions.size() == robot_model_->getVariableCount());
    updateFK(coords);
    return getLinkContactSpheres_CurrentState(link_name, spheres);
}

/// \brief Return the contact spheres for a link in the current state
bool URDFCollisionModel::getLinkContactSpheres_CurrentState(
    const std::string &link_name,
    std::vector<Sphere> &spheres) const
{
    auto &tfm = robot_state_->getGlobalLinkTransform(link_name);

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

bool URDFCollisionModel::getLinkAttachedObjectsSpheres(
    const std::string &link_name,
    const Eigen::Affine3d link_tfm,
    std::vector<Sphere> &spheres) const
{
    if (!hasAttachedObjects(link_name)) {
        return true;
    }
    auto objs = getAttachedObjects(link_name);

    ROS_INFO("Got %d attached objects for link %s", (int )objs.size(), link_name.c_str());

    for (auto &ao : objs) {
        ROS_INFO("Object %s has %zu spheres!", ao.name.c_str(), ao.spheres.size());
        for (auto &s : ao.spheres) {
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

    // compute max_dist that collision spheres travel between c0 and c1
    auto max_dist = 0.0;
    path.clear();
    for (const auto &link_name : links_with_collision_spheres_) {
        std::vector<Sphere> s0;
        std::vector<Sphere> s1;
        if (!getLinkCollisionSpheres(coords0, link_name, s0)) {
            ROS_ERROR("URDFCollisionModel::getInterpolatedPath - Failed to get link %s collision spheres", link_name.c_str());
            return false;
        }

        if (!getLinkCollisionSpheres(coords1, link_name, s1)) {
            ROS_ERROR("getInterpolatedPath - Failed to get link %s collision spheres", link_name.c_str());
            return false;
        }

        if (s0.size() != s1.size()) {
            ROS_ERROR("getInterpolatedPath - Different number of spheres found between coords0 and coords1!");
            return false;
        }

        for (size_t s = 0; s < s0.size(); s++) {
            auto dist = (s0[s].v - s1[s].v).norm();
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
            ROS_ERROR("getInterpolatedPath - failed to get interpolation to midpoint!");
            return false;
        }

        std::vector<URDFModelCoords> sub_path0;
        std::vector<URDFModelCoords> sub_path1;
        if (!getInterpolatedPath(coords0, coords_half, resolution, sub_path0, depth + 1, max_depth)) {
            ROS_ERROR("getInterpolatedPath - failed to get interpolated sub-path 1");
            return false;
        }
        if (!getInterpolatedPath(coords_half, coords1, resolution, sub_path1, depth + 1, max_depth)) {
            ROS_ERROR("getInterpolatedPath - failed to get interpolated sub-path 2");
            return false;
        }

        // don't add duplicate of the midpoint
        path.insert(path.end(), begin(sub_path0), end(sub_path0));
        path.insert(path.end(), begin(sub_path1) + 1, end(sub_path1));
        return true;
    }
    else {
        // no need to subdivide further - interpolated path only c0 to c1
        path.push_back(coords0);
        path.push_back(coords1);
        return true;
    }
}

bool URDFCollisionModel::initRobotModelFromURDF(
    const std::string &urdf_string,
    const std::string &srdf_string)
{
    robot_model_loader::RobotModelLoader::Options ops;
    ops.robot_description_ = "";
    ops.urdf_string_ = urdf_string;
    ops.srdf_string_ = srdf_string;
    ops.urdf_doc_ = nullptr;
    ops.srdf_doc_ = nullptr;
    ops.load_kinematics_solvers_ = true;

    robot_model_loader::RobotModelLoader loader(ops);

    return initFromModel(loader.getModel());
}

bool URDFCollisionModel::hasAttachedObject(
    const std::string &link_name,
    const std::string &object_name) const
{
    auto it = attached_objects_.find(link_name);
    if (it == end(attached_objects_)) {
        return false;
    }

    auto oit = std::find_if(begin(it->second), end(it->second),
            [&](const AttachedObject &ao) {
                return ao.name == object_name;
            });

    return oit != end(it->second);
}

bool URDFCollisionModel::hasAttachedObjects(const std::string &link_name) const
{
    auto it = attached_objects_.find(link_name);
    return (it != attached_objects_.end());
}

auto URDFCollisionModel::getAttachedObjects(const std::string &link_name) const
    -> const std::vector<AttachedObject>
{
    auto it = attached_objects_.find(link_name);
    if (it == attached_objects_.end()) {
        return {};
    }
    return it->second;
}

bool URDFCollisionModel::attachObject(
    const std::string &link_name,
    const AttachedObject &obj)
{
    attached_objects_[link_name].push_back(obj);
    return true;
}

bool URDFCollisionModel::computeShapeBoundingSpheres(
    const shapes::Shape& shape,
    double res,
    std::vector<Sphere>& spheres)
{
    switch (shape.type) {
    case shapes::BOX: {
        auto &obj = dynamic_cast<const shapes::Box&>(shape);
        std::vector<Eigen::Vector3d> centers;
        sbpl::geometry::ComputeBoxBoundingSpheres(obj.size[0], obj.size[1], obj.size[2], res, centers);
        spheres.reserve(spheres.size() + centers.size());
        for (const auto &center : centers) {
            spheres.push_back(Sphere());
            spheres.back().v = center;
            spheres.back().radius = res;
        }
    }   break;
    case shapes::CYLINDER: {
        auto &obj = dynamic_cast<const shapes::Cylinder&>(shape);
        auto centers = std::vector<Eigen::Vector3d>{};
        sbpl::geometry::ComputeCylinderBoundingSpheres(obj.radius, obj.length, res, centers);
        spheres.reserve(spheres.size() + centers.size());
        for (const auto &center : centers) {
            spheres.push_back(Sphere{});
            spheres.back().v = center;
            spheres.back().radius = res;
        }
    }   break;
    case shapes::SPHERE: {
        auto &obj = dynamic_cast<const shapes::Sphere&>(shape);
        spheres.push_back(Sphere());
        spheres.back().v = Eigen::Vector3d::Zero();
        spheres.back().radius = obj.radius;
    }   break;
    case shapes::MESH: {
        auto &obj = dynamic_cast<const shapes::Mesh&>(shape);

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
        auto radius = sqrt(2.0) * res;
        std::vector<Eigen::Vector3d> centers;
        sbpl::geometry::VoxelizeMesh(vert, tri, radius, centers, false);
        spheres.reserve(spheres.size() + centers.size());
        for (const auto &center : centers) {
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

} // namespace adim
