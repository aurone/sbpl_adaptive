#include <sbpl_adaptive_collision_checking/collision_model_impl.h>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>

namespace adim {

CollisionModelImpl::CollisionModelImpl() :
            nh_(),
            ph_("~"),
            rm_loader_(),
            robot_model_(),
            group_config_map_(),
            urdf_(),
            dgroup_(nullptr)
{
}

CollisionModelImpl::~CollisionModelImpl()
{
    for (auto iter = group_config_map_.begin(); iter != group_config_map_.end();
            iter++) {
        if (iter->second != NULL) {
            delete iter->second;
        }
    }
}

bool CollisionModelImpl::init(const std::string &urdf_string)
{
    return initURDF(urdf_string) && initRobotModel(urdf_string) && readGroups();
}

bool CollisionModelImpl::readGroups()
{
    XmlRpc::XmlRpcValue all_groups, all_spheres;

    // collision spheres
    std::string spheres_name = "collision_spheres";
    if (!ph_.hasParam(spheres_name)) {
        ROS_WARN_STREAM("No groups for planning specified in " << spheres_name);
        return false;
    }
    ph_.getParam(spheres_name, all_spheres);

    if (all_spheres.getType() != XmlRpc::XmlRpcValue::TypeArray)
        ROS_WARN_PRETTY("Spheres is not an array.");

    if (all_spheres.size() == 0) {
        ROS_WARN_PRETTY("No spheres in spheres");
        return false;
    }

    // collision groups
    std::string group_name = "collision_groups";
    if (!ph_.hasParam(group_name)) {
        ROS_WARN_STREAM("No groups for planning specified in " << group_name);
        return false;
    }
    ph_.getParam(group_name, all_groups);

    if (all_groups.getType() != XmlRpc::XmlRpcValue::TypeArray)
        ROS_WARN_PRETTY("Groups is not an array.");

    if (all_groups.size() == 0) {
        ROS_WARN_PRETTY("No groups in groups");
        return false;
    }

    for (int i = 0; i < all_groups.size(); i++) {
        if (!all_groups[i].hasMember("name")) {
            ROS_WARN_PRETTY("All groups must have a name.");
            return false;
        }
        std::string gname = all_groups[i]["name"];
        adim::Group* gc =
                new adim::Group(gname);
        std::map<std::string, adim::Group*>::iterator group_iterator =
                group_config_map_.find(gname);
        if (group_iterator != group_config_map_.end()) {
            ROS_WARN_STREAM("Already have group name " << gname);
            delete gc;
            continue;
        }
        group_config_map_[gname] = gc;
        if (!group_config_map_[gname]->getParams(all_groups[i], all_spheres)) {
            ROS_ERROR_PRETTY("Failed to get all params for %s", gname.c_str());
            return false;
        }
    }
    ROS_INFO_PRETTY("Successfully parsed collision model");
    return true;
}

void CollisionModelImpl::getGroupNames(std::vector<std::string> &names)
{
    for (std::map<std::string, adim::Group*>::const_iterator iter =
            group_config_map_.begin(); iter != group_config_map_.end();
            ++iter) {
        names.push_back(iter->first);
    }
}

bool CollisionModelImpl::setDefaultGroup(const std::string &group_name)
{
    if (group_config_map_.find(group_name) == group_config_map_.end()) {
        ROS_ERROR("Failed to find group '%s' in group_config_map_",
                group_name.c_str());
        ROS_ERROR("Expecting one of the following group names:");
        for (auto it = group_config_map_.cbegin();
                it != group_config_map_.cend(); ++it) {
            ROS_ERROR("%s", it->first.c_str());
        }
        return false;
    }

    dgroup_ = group_config_map_[group_name];
    return true;
}

void CollisionModelImpl::printGroups()
{
    if (group_config_map_.begin() == group_config_map_.end()) {
        ROS_ERROR_PRETTY("No groups found.");
        return;
    }

    for (auto iter = group_config_map_.begin(); iter != group_config_map_.end();
            ++iter) {
        if (!iter->second->init_) {
            ROS_ERROR_PRETTY(
                    "Failed to print %s group information because has not yet been initialized.",
                    iter->second->getName().c_str());
            continue;
        }
        iter->second->print();
        ROS_INFO_PRETTY("----------------------------------");
    }
}

bool CollisionModelImpl::getFrameInfo(
    const std::string &name,
    const std::string &group_name,
    int &chain,
    int &segment)
{
    return group_config_map_[group_name]->getFrameInfo(name, chain, segment);
}

bool CollisionModelImpl::initAllGroups()
{
    for (auto iter = group_config_map_.begin(); iter != group_config_map_.end();
            ++iter) {
        if (!iter->second->init(urdf_))
            return false;
    }
    return true;
}

bool CollisionModelImpl::computeDefaultGroupFK(
    const std::vector<double> &angles,
    std::vector<std::vector<KDL::Frame>> &frames)
{
    return computeGroupFK(angles, dgroup_, frames);
}

bool CollisionModelImpl::computeGroupFK(
    const std::vector<double> &angles,
    adim::Group *group,
    std::vector<std::vector<KDL::Frame>> &frames)
{
    return group->computeFK(angles, frames);
}

void CollisionModelImpl::setOrderOfJointPositions(
    const std::vector<std::string> &joint_names,
    const std::string &group_name)
{
    group_config_map_[group_name]->setOrderOfJointPositions(joint_names);
}

void CollisionModelImpl::setJointPosition(
    const std::string &name,
    double position)
{
    for (auto iter = group_config_map_.begin(); iter != group_config_map_.end();
            iter++)
        iter->second->setJointPosition(name, position);
}

void CollisionModelImpl::printDebugInfo(const std::string &group_name)
{
    adim::Group* group =
            group_config_map_[group_name];
    group->printDebugInfo();
}

void CollisionModelImpl::getDefaultGroupSpheres(
    std::vector<adim::Sphere*> &spheres)
{
    dgroup_->getSpheres(spheres);
}

bool CollisionModelImpl::getJointLimits(
    const std::string &group_name,
    const std::string &joint_name,
    double &min_limit,
    double &max_limit,
    bool &continuous)
{
    if (group_config_map_.find(group_name) == group_config_map_.end())
        return false;
    if (!group_config_map_[group_name]->init_)
        return false;

    return leatherman::getJointLimits(urdf_.get(),
            group_config_map_[group_name]->getReferenceFrame(),
            group_config_map_[group_name]->tip_name_, joint_name, min_limit,
            max_limit, continuous);
}

std::string CollisionModelImpl::getReferenceFrame(const std::string &group_name)
{
    if (group_config_map_.find(group_name) == group_config_map_.end())
        return "";
    if (!group_config_map_[group_name]->init_)
        return "";
    return group_config_map_[group_name]->getReferenceFrame();
}

adim::Group* CollisionModelImpl::getGroup(
    const std::string &name)
{
    adim::Group* r = NULL;
    if (group_config_map_.find(name) == group_config_map_.end())
        return r;
    return group_config_map_[name];
}

void CollisionModelImpl::getVoxelGroups(
    std::vector<adim::Group*> &vg)
{
    vg.clear();
    for (auto iter = group_config_map_.begin(); iter != group_config_map_.end();
            ++iter) {
        if (iter->second->type_
                == adim::Group::VOXELS)
            vg.push_back(iter->second);
    }
}

bool CollisionModelImpl::doesLinkExist(
    const std::string &name,
    const std::string &group_name)
{
    int chain, segment;
    return getFrameInfo(name, group_name, chain, segment);
}

bool CollisionModelImpl::setWorldToModelTransform(
    const moveit_msgs::RobotState &state,
    const std::string &world_frame)
{
    Eigen::Affine3d T_world_robot(Eigen::Affine3d::Identity());
    KDL::Frame f;

    // set all single-variable joints
    std::map<std::string, double> joint_value_map;
    for (size_t i = 0; i < state.joint_state.name.size(); ++i)
        joint_value_map[state.joint_state.name[i]] =
                state.joint_state.position[i];
    robot_state_->setVariablePositions(joint_value_map);
    robot_state_->updateLinkTransforms();

    // check for robot_pose joint variables
    const std::string& robot_pose_joint_name = "robot_pose";
    if (world_frame != robot_model_->getModelFrame()) {
        bool found_world_pose = false;
        for (size_t i = 0; i < state.multi_dof_joint_state.joint_names.size();
                ++i) {
            if (state.multi_dof_joint_state.joint_names[i]
                    == robot_pose_joint_name) {
                found_world_pose = true;
                tf::transformMsgToEigen(
                        state.multi_dof_joint_state.transforms[i],
                        T_world_robot);
            }
        }

        if (!found_world_pose) {
            ROS_ERROR_PRETTY(
                    "Failed to find 6-DoF joint state 'world_pose' from MultiDOFJointState");
            return false;
        }
    }

    // set the transform from the world frame to each group reference frame
    for (auto iter = group_config_map_.begin(); iter != group_config_map_.end();
            ++iter) {
        const std::string& group_frame = iter->second->getReferenceFrame();
        if (!robot_state_->knowsFrameTransform(
                iter->second->getReferenceFrame())) {
            ROS_ERROR_PRETTY(
                    "Robot Model does not contain transform from robot frame '%s' to group frame '%s'",
                    robot_model_->getModelFrame().c_str(), group_frame.c_str());
            return false;
        }
        else {
            Eigen::Affine3d T_world_group = T_world_robot
                    * robot_state_->getFrameTransform(group_frame);
            tf::transformEigenToKDL(T_world_group, f);
            iter->second->setGroupToWorldTransform(f);
            leatherman::printKDLFrame(f, "group-world");
        }
    }

    return true;
}

bool CollisionModelImpl::initURDF(const std::string &urdf_string)
{
    urdf_ = boost::shared_ptr < urdf::Model > (new urdf::Model());
    if (!urdf_->initString(urdf_string)) {
        ROS_WARN_PRETTY("Failed to parse the URDF");
        return false;
    }

    return true;
}

bool CollisionModelImpl::initRobotModel(const std::string &urdf_string)
{
    std::string srdf_string;
    if (!nh_.getParam("robot_description_semantic", srdf_string)) {
        ROS_ERROR_PRETTY(
                "Failed to retrieve 'robot_description_semantic' from the param server");
        return false;
    }

    robot_model_loader::RobotModelLoader::Options ops;
    ops.robot_description_ = "";
    ops.urdf_string_ = urdf_string;
    ops.srdf_string_ = srdf_string;
    ops.urdf_doc_ = nullptr;
    ops.srdf_doc_ = nullptr;
    ops.load_kinematics_solvers_ = false;

    rm_loader_.reset(new robot_model_loader::RobotModelLoader(ops));
    if (!rm_loader_) {
        ROS_ERROR_PRETTY("Failed to instantiate Robot Model Loader");
        return false;
    }

    robot_model_ = rm_loader_->getModel();
    if (!robot_model_) {
        ROS_ERROR_PRETTY("Failed to retrieve valid Robot Model");
        return false;
    }

    robot_state_.reset(new robot_state::RobotState(robot_model_));
    if (!robot_state_) {
        ROS_ERROR_PRETTY("Failed to instantiate Robot State");
        return false;
    }

    return true;
}

} // namespace manipulation

