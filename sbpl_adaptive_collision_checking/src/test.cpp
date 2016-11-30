/*
 * test.cpp
 *
 *  Created on: Mar 12, 2016
 *      Author: kalin
 */

#include <ros/ros.h>
#include <sbpl_adaptive_collision_checking/sbpl_collision_model.h>
#include <sbpl_adaptive_collision_checking/sbpl_collision_space.h>
#include <sbpl_adaptive_collision_checking/urdf_collision_model.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

using namespace adim;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sbpl_adaptive_collision_checking_test");

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ros::Publisher viz = nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_marker_array", 500);

    if (!nh.hasParam("robot_description")) {
        ROS_ERROR("robot_description not found on param server");
        exit(0);
    }

    std::string robot_desc;
    std::string robot_desc_sem;

    nh.param<std::string>("robot_description", robot_desc, "");
    nh.param<std::string>("robot_description_semantic", robot_desc_sem, "");

    std::shared_ptr<URDFCollisionModel> urdf_model;
    sbpl::OccupancyGridPtr grid;
    std::shared_ptr<SBPLCollisionSpace> cspace;

    grid.reset(new sbpl::OccupancyGrid(0, 0, 0, 0.05, 5.0, 5.0, 5.0, 0.3));
    if (!grid) {
        ROS_ERROR("Failed to init grid!");
        exit(0);
    }

    urdf_model.reset(new URDFCollisionModel());
    if (!urdf_model) {
        ROS_ERROR("Failed to create model!");
        exit(0);
    }

    if (robot_desc_sem.empty()) {
        if (!urdf_model->initFromParam("robot_description")) {
            ROS_ERROR("Could not initialize URDF model initRobotModelFromDescription!");
            exit(0);
        }
    }
    else {
        if (!urdf_model->initFromURDF(robot_desc, robot_desc_sem)) {
            ROS_ERROR("Could not initialize URDF model initFromURDF!");
            exit(0);
        }
    }

    ROS_INFO("Model loaded!");

    cspace.reset(new SBPLCollisionSpace(urdf_model, grid));

    if (!cspace) {
        ROS_ERROR("Failed to create collision space!");
    }

    std::ostringstream oss;
    urdf_model->PrintModelInfo(oss);
    ROS_INFO_STREAM(oss);

    if (urdf_model->computeSpheresFromURDFModel(0.05, { }, { }) == false) {
        ROS_WARN("Could not auto-compute model spheres!");
    }

    shapes::Cylinder obj;
    obj.radius = 0.05;
    obj.length = 0.80;

    tf::Transform p;
    p.setRotation(tf::Quaternion::getIdentity());
    p.setOrigin(tf::Vector3(0.2, 0.0, 0.0));

    Eigen::Affine3d objpose;
    tf::transformTFToEigen(p, objpose);

    if (!urdf_model->attachObjectToLink(
            "r_gripper_palm_link", objpose, obj, "attached_object", obj.radius))
    {
        ROS_WARN("Could not attach object!");
    }

    sleep(5);

    URDFModelCoords coords = urdf_model->getDefaultCoordinates();
    std_msgs::ColorRGBA col;
    col.r = 1;
    col.g = 1;
    col.b = 0;
    col.a = 1;
    int viz_id = 0;
    viz.publish(urdf_model->getModelBasicVisualization(
            coords, "/map", "model-spheres", col, viz_id));

    viz_id = 0;
    col.r = 0;
    col.g = 1;
    col.b = 0;
    col.a = 1;
    viz.publish(urdf_model->getModelVisualization(
                    coords, "/map", "model", col, viz_id));

    col.r = 1;
    col.g = 0;
    col.b = 0;
    col.a = 1;
    viz_id = 0;
    viz.publish(urdf_model->getAttachedObjectsVisualization(
            coords, "/map", "attached", col, viz_id));

    ros::shutdown();
}
