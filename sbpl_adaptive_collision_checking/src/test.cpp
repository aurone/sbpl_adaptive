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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sbpl_adaptive_collision_checking_test");

    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ros::Publisher viz = nh.advertise<visualization_msgs::MarkerArray>(
            "visualization_markers", 500);

    ///////////////////
    // test settings //
    ///////////////////

    const std::string model_frame("map");
    const std::string attach_link("r_gripper_palm_link");
    const std::string object_name("attached_object");

    shapes::Cylinder obj;
    obj.radius = 0.05;
    obj.length = 0.80;

    const Eigen::Affine3d objpose(Eigen::Translation3d(0.2, 0.0, 0.0));

    //////////////////////
    // Initialize model //
    //////////////////////

    std::string robot_desc;
    std::string robot_desc_sem;
    if (!nh.getParam("robot_description", robot_desc) ||
        !nh.getParam("robot_description_semantic", robot_desc_sem))
    {
        ROS_ERROR("Failed to retrieve 'robot_description' or 'robot_description_semantic' from the param server.");
        return 1;
    }

    auto urdf_model = std::make_shared<adim::URDFCollisionModel>();

    if (!urdf_model->initFromURDF(robot_desc, robot_desc_sem)) {
        ROS_ERROR("Failed to initialize URDF Collision Model from URDF/SRDF");
        return 1;
    }

    std::ostringstream oss;
    urdf_model->PrintModelInfo(oss);
    ROS_INFO_STREAM(oss);

    ///////////////////////////////////
    // Automatically compute spheres //
    ///////////////////////////////////

    const double res = 0.05;
    std::vector<std::string> ignore_collision_links = { };
    std::vector<std::string> contact_links = { };
    if (!urdf_model->computeSpheresFromURDFModel(
            res,
            ignore_collision_links,
            contact_links))
    {
        ROS_ERROR("Failed to automatically compute model spheres");
        return 1;
    }

    ////////////////////////////
    // Attach object to model //
    ////////////////////////////

    if (!urdf_model->attachObjectToLink(
            attach_link, objpose, obj, object_name, obj.radius))
    {
        ROS_ERROR("Failed to attach object!");
        return 1;
    }

    ros::Duration(1.0).sleep();

    ////////////////////
    // Visualizations //
    ////////////////////

    std_msgs::ColorRGBA color;
    int viz_id = 0;

    adim::URDFModelCoords coords = urdf_model->getDefaultCoordinates();
    color.r = 1.0f;
    color.g = 1.0f;
    color.b = 0.0f;
    color.a = 1.0f;
    viz.publish(urdf_model->getModelBasicVisualization(
            coords, model_frame, "model-spheres", color, viz_id));

    viz_id = 0;
    color.r = 0.0f;
    color.g = 1.0f;
    color.b = 0.0f;
    color.a = 1.0f;
    viz.publish(urdf_model->getModelVisualization(
            coords, model_frame, "model", color, viz_id));

    color.r = 1.0f;
    color.g = 0.0f;
    color.b = 0.0f;
    color.a = 1.0f;
    viz_id = 0;
    viz.publish(urdf_model->getAttachedObjectsVisualization(
            coords, model_frame, "attached", color, viz_id));

    ros::shutdown();
}
