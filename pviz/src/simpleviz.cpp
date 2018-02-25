/// \author Benjamin Cohen
/// \author Kalin Gochev

#include <pviz/simpleviz.h>

// standard includes
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>

// system includes
#include <boost/lexical_cast.hpp>
#include <leatherman/utils.h>
#include <smpl/debug/visualize.h>
#include <tf/tf.h>

static const char *LOG = "pviz";

namespace pviz {

auto MakeSphereVisualization(
    const std::vector<double> &pos3,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    double radius,
    int &id,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id; id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos3[0];
    marker.pose.position.y = pos3[1];
    marker.pose.position.z = pos3[2];
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = 2.0 * radius;
    marker.color = color;
    marker.lifetime = ros::Duration(0.0);

    return marker;
}

auto MakeSphereVisualization(
    const std::vector<double> &pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    double radius,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose[0];
    marker.pose.position.y = pose[1];
    marker.pose.position.z = pose[2];
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = 2.0 * radius;
    marker.color = color;
    marker.lifetime = ros::Duration(0.0);

    return marker;
}

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    double radius,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = "spheres-" + ns;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = 2.0 * radius;
    marker.color = color;
    marker.lifetime = ros::Duration(0.0);
    marker.id = 1;

    marker.points.resize(pose.size());
    for (size_t i = 0; i < pose.size(); i++) {
        marker.points[i].x = pose[i][0];
        marker.points[i].y = pose[i][1];
        marker.points[i].z = pose[i][2];
    }

    return marker;
}

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    const std::vector<double> &radius,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::MarkerArray ma;

    for (size_t i = 0; i < pose.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = frame_id;
        marker.ns = ns;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 2.0 * radius[i];
        marker.scale.y = 2.0 * radius[i];
        marker.scale.z = 2.0 * radius[i];
        marker.color = color;
        marker.lifetime = ros::Duration(0.0);
        marker.id = i;

        marker.pose.position.x = pose[i][0];
        marker.pose.position.y = pose[i][1];
        marker.pose.position.z = pose[i][2];

        ma.markers.push_back(std::move(marker));
    }

    return ma;
}

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;

    for (size_t i = 0; i < pose.size(); ++i) {
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = frame_id;
        marker.ns = ns;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 2.0 * pose[i][3];
        marker.scale.y = 2.0 * pose[i][3];
        marker.scale.z = 2.0 * pose[i][3];
        marker.color = color;
        marker.lifetime = ros::Duration(0.0);
        marker.id = i;

        marker.pose.position.x = pose[i][0];
        marker.pose.position.y = pose[i][1];
        marker.pose.position.z = pose[i][2];

        marker_array.markers.push_back(marker);
    }

    return marker_array;
}

auto MakeLineVisualization(
    const std::vector<geometry_msgs::Point> &points,
    const std::string &ns,
    int &id,
    const std_msgs::ColorRGBA &color,
    double thickness,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id; id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points = points;
    marker.scale.x = thickness;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    marker.color = color;
    marker.lifetime = ros::Duration(0.0);

    return marker;
}

auto MakeTextVisualization(
    const geometry_msgs::Pose &pose,
    const std::string &text,
    const std::string &ns,
    int id,
    const std_msgs::ColorRGBA &color,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.pose = pose;

    marker.color = color;
    marker.text = text;
    marker.lifetime = ros::Duration(0.0);

    return marker;
}

auto MakeCubeVisualization(
    const geometry_msgs::PoseStamped &pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    int id,
    const std::vector<double> &dim)
    -> visualization_msgs::Marker
{
    if (dim.empty()) {
        return visualization_msgs::Marker();
    }

    visualization_msgs::Marker marker;

    const std::vector<double>& dim_ = dim.size() < 3 ?
            std::vector<double>(3, dim[0]) : dim;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = pose.header.frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = dim_[0];
    marker.scale.y = dim_[1];
    marker.scale.z = dim_[2];
    marker.color = color;
    marker.lifetime = ros::Duration(0.0);

    return marker;
}

auto MakeMeshVisualization(
    const std::string& mesh_resource,
    const geometry_msgs::PoseStamped& pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    int id,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.pose.position.x;
    marker.pose.position.y = pose.pose.position.y;
    marker.pose.position.z = pose.pose.position.z;
    marker.pose.orientation.x = pose.pose.orientation.x;
    marker.pose.orientation.y = pose.pose.orientation.y;
    marker.pose.orientation.z = pose.pose.orientation.z;
    marker.pose.orientation.w = pose.pose.orientation.w;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color = color;
    marker.mesh_resource = mesh_resource;

    return marker;
}

auto MakeMeshTrianglesVisualization(
    const std::vector<geometry_msgs::Point>& vertices,
    const std::vector<int>& triangles,
    const geometry_msgs::PoseStamped& pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    int id,
    bool psychadelic,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose.pose.position.x;
    marker.pose.position.y = pose.pose.position.y;
    marker.pose.position.z = pose.pose.position.z;
    marker.pose.orientation.x = pose.pose.orientation.x;
    marker.pose.orientation.y = pose.pose.orientation.y;
    marker.pose.orientation.z = pose.pose.orientation.z;
    marker.pose.orientation.w = pose.pose.orientation.w;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.points = vertices;

    if (psychadelic) {
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        std_msgs::ColorRGBA red; red.a = 1.0f; red.r = 1.0f; red.g = 0.0f; red.b = 0.0f;
        std_msgs::ColorRGBA green; green.a = 1.0f; green.r = 0.0f; green.g = 1.0f; green.b = 0.0f;
        std_msgs::ColorRGBA blue; blue.a = 1.0f; blue.r = 0.0f; blue.g = 0.0f; blue.b = 1.0f;

        std::vector<std_msgs::ColorRGBA> colors;
        for (size_t i = 0; i < vertices.size(); i++) {
            if (i % 3 == 0) {
                colors.push_back(red);
            }
            if (i % 3 == 1) {
                colors.push_back(green);
            }
            if (i % 3 == 2) {
                colors.push_back(blue);
            }
        }

        marker.colors = colors;
    }
    else {
        marker.color = color;
    }

    return marker;
}

auto MakePoseVisualization(
    const std::vector<double> &pose,
    const std::string &text,
    const std::string &ns,
    int &id,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray
{
    tf::Quaternion pose_quaternion;
    geometry_msgs::Pose pose_msg;

    pose_msg.position.x = pose[0];
    pose_msg.position.y = pose[1];
    pose_msg.position.z = pose[2];

    pose_quaternion.setRPY(pose[3], pose[4], pose[5]);
    tf::quaternionTFToMsg(pose_quaternion, pose_msg.orientation);

    ROS_DEBUG_NAMED(LOG, "[%s] position: %0.3f %0.3f %0.3f quaternion: %0.3f %0.3f %0.3f %0.3f (frame: %s)", text.c_str(), pose[0], pose[1], pose[2], pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w, frame_id.c_str());

    return MakePoseVisualization(pose_msg, text, ns, id, frame_id);
}

auto MakePoseVisualization(
    const geometry_msgs::Pose &pose,
    const std::string &text,
    const std::string &ns,
    int &id,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::MarkerArray ma;

    int midx = -1;
    ma.markers.resize(3);
    ros::Time time = ros::Time::now();

    ROS_DEBUG_NAMED(LOG, "[%s] position: %0.3f %0.3f %0.3f quaternion: %0.3f %0.3f %0.3f %0.3f (frame: %s)", text.c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, frame_id.c_str());

    midx++;
    ma.markers[midx].header.stamp = time;
    ma.markers[midx].header.frame_id = frame_id;
    ma.markers[midx].ns = ns;
    ma.markers[midx].type = visualization_msgs::Marker::ARROW;
    ma.markers[midx].id = id;
    ma.markers[midx].action = visualization_msgs::Marker::ADD;
    ma.markers[midx].pose = pose;
    ma.markers[midx].scale.x = 0.125;
    ma.markers[midx].scale.y = 0.125;
    ma.markers[midx].scale.z = 0.125;
    ma.markers[midx].color.r = 0.0;
    ma.markers[midx].color.g = 0.7;
    ma.markers[midx].color.b = 0.6;
    ma.markers[midx].color.a = 0.7;
    ma.markers[midx].lifetime = ros::Duration(0.0);
    id++;
    midx++;
    ma.markers[midx].header.stamp = time;
    ma.markers[midx].header.frame_id = frame_id;
    ma.markers[midx].ns = ns;
    ma.markers[midx].id = id;
    ma.markers[midx].type = visualization_msgs::Marker::SPHERE;
    ma.markers[midx].action = visualization_msgs::Marker::ADD;
    ma.markers[midx].pose = pose;
    ma.markers[midx].scale.x = 0.10;
    ma.markers[midx].scale.y = 0.10;
    ma.markers[midx].scale.z = 0.10;
    ma.markers[midx].color.r = 1.0;
    ma.markers[midx].color.g = 0.0;
    ma.markers[midx].color.b = 0.6;
    ma.markers[midx].color.a = 0.6;
    ma.markers[midx].lifetime = ros::Duration(0.0);
    id++;
    midx++;
    ma.markers[midx].header.stamp = time;
    ma.markers[midx].header.frame_id = frame_id;
    ma.markers[midx].ns = ns;
    ma.markers[midx].id = id;
    ma.markers[midx].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    ma.markers[midx].action = visualization_msgs::Marker::ADD;
    ma.markers[midx].pose = pose;
    ma.markers[midx].pose.position.z += 0.05;
    ma.markers[midx].scale.x = 0.03;
    ma.markers[midx].scale.y = 0.03;
    ma.markers[midx].scale.z = 0.03;
    ma.markers[midx].color.r = 1.0;
    ma.markers[midx].color.g = 1.0;
    ma.markers[midx].color.b = 1.0;
    ma.markers[midx].color.a = 0.9;
    ma.markers[midx].text = text;
    ma.markers[midx].lifetime = ros::Duration(0.0);
    id++;

    return ma;
}

auto MakePosesVisualization(
    const std::vector<std::vector<double>> &poses,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::MarkerArray ma;
    ma.markers.resize(3 * poses.size());
    tf::Quaternion pose_quaternion;
    geometry_msgs::Quaternion quaternion_msg;

    int midx = -1;

    ros::Time time = ros::Time::now();

    for (size_t i = 0; i < poses.size(); ++i) {
        pose_quaternion.setRPY(poses[i][3], poses[i][4], poses[i][5]);
        tf::quaternionTFToMsg(pose_quaternion, quaternion_msg);

        midx++;
        ma.markers[midx].header.stamp = time;
        ma.markers[midx].header.frame_id = frame_id;
        ma.markers[midx].ns = "pose_arrows";
        ma.markers[midx].type = visualization_msgs::Marker::ARROW;
        ma.markers[midx].id = i;
        ma.markers[midx].action = visualization_msgs::Marker::ADD;
        ma.markers[midx].pose.position.x = poses[i][0];
        ma.markers[midx].pose.position.y = poses[i][1];
        ma.markers[midx].pose.position.z = poses[i][2];
        ma.markers[midx].pose.orientation = quaternion_msg;
        ma.markers[midx].scale.x = 0.1;
        ma.markers[midx].scale.y = 0.1;
        ma.markers[midx].scale.z = 0.1;
        ma.markers[midx].color.r = 0.0;
        ma.markers[midx].color.g = 0.7;
        ma.markers[midx].color.b = 0.6;
        ma.markers[midx].color.a = 0.7;
        ma.markers[midx].lifetime = ros::Duration(0.0);

        midx++;
        ma.markers[midx].header.stamp = time;
        ma.markers[midx].header.frame_id = frame_id;
        ma.markers[midx].ns = "pose_spheres";
        ma.markers[midx].id = i;
        ma.markers[midx].type = visualization_msgs::Marker::SPHERE;
        ma.markers[midx].action = visualization_msgs::Marker::ADD;
        ma.markers[midx].pose.position.x = poses[i][0];
        ma.markers[midx].pose.position.y = poses[i][1];
        ma.markers[midx].pose.position.z = poses[i][2];
        ma.markers[midx].pose.orientation = quaternion_msg;
        ma.markers[midx].scale.x = 0.07;
        ma.markers[midx].scale.y = 0.07;
        ma.markers[midx].scale.z = 0.07;
        ma.markers[midx].color.r = 1.0;
        ma.markers[midx].color.g = 0.0;
        ma.markers[midx].color.b = 0.6;
        ma.markers[midx].color.a = 0.6;
        ma.markers[midx].lifetime = ros::Duration(0.0);

        midx++;
        ma.markers[midx].header.stamp = time;
        ma.markers[midx].header.frame_id = frame_id;
        ma.markers[midx].ns = "pose_text_blocks";
        ma.markers[midx].id = i;
        ma.markers[midx].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        ma.markers[midx].action = visualization_msgs::Marker::ADD;
        ma.markers[midx].pose.position.x = poses[i][0];
        ma.markers[midx].pose.position.y = poses[i][1];
        ma.markers[midx].pose.position.z = poses[i][2];
        ma.markers[midx].scale.x = 0.03;
        ma.markers[midx].scale.y = 0.03;
        ma.markers[midx].scale.z = 0.03;
        ma.markers[midx].color.r = 1.0;
        ma.markers[midx].color.g = 1.0;
        ma.markers[midx].color.b = 1.0;
        ma.markers[midx].color.a = 0.9;
        ma.markers[midx].text = boost::lexical_cast<std::string>(i + 1);
        ma.markers[midx].lifetime = ros::Duration(0.0);
    }

    return ma;
}

auto MakeObstaclesVisualization(
    const std::vector<std::vector<double>> &obstacles,
    const std::string &frame_id,
    const std::string &_ns)
    -> visualization_msgs::MarkerArray
{
    visualization_msgs::MarkerArray ma;

    ma.markers.clear();
    ma.markers.resize(obstacles.size());

    ROS_DEBUG_NAMED(LOG, "Displaying %d obstacles in the %s frame", (int)obstacles.size(), frame_id.c_str());

    const std::string &ns = _ns.empty() ? "obstacles" : _ns;

    for (size_t i = 0; i < obstacles.size(); i++) {
        if (obstacles[i].size() < 6) {
            ROS_DEBUG_NAMED(LOG, "Obstacle description doesn't have length = 6");
            continue;
        }

        // TODO: Change this to use a CUBE_LIST
        ma.markers[i].header.stamp = ros::Time::now();
        ma.markers[i].header.frame_id = frame_id;
        ma.markers[i].ns = ns;
        ma.markers[i].id = i;
        ma.markers[i].type = visualization_msgs::Marker::CUBE;
        ma.markers[i].action = visualization_msgs::Marker::ADD;
        ma.markers[i].pose.position.x = obstacles[i][0];
        ma.markers[i].pose.position.y = obstacles[i][1];
        ma.markers[i].pose.position.z = obstacles[i][2];
        ma.markers[i].scale.x = obstacles[i][3];
        ma.markers[i].scale.y = obstacles[i][4];
        ma.markers[i].scale.z = obstacles[i][5];
        ma.markers[i].color.r = 0.0;
        ma.markers[i].color.g = 0.0;
        ma.markers[i].color.b = 0.5;
        ma.markers[i].color.a = 0.9;
        ma.markers[i].lifetime = ros::Duration(0.0);
    }

    return ma;
}

auto Make3DPathVisualization(
    const std::vector<std::vector<double>> &dpath,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    if (dpath.empty()) {
        ROS_DEBUG_NAMED(LOG, "The shortest path is empty.");
        return visualization_msgs::Marker();
    }
    else {
        ROS_DEBUG_NAMED(LOG, "There are %zu waypoints in the shortest path.", dpath.size());
    }

    visualization_msgs::Marker obs_marker;
    obs_marker.header.frame_id = frame_id;
    obs_marker.header.stamp = ros::Time();
    obs_marker.header.seq = 0;
    obs_marker.ns = "path";
    obs_marker.id = 0;
    obs_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    obs_marker.action = 0;
    obs_marker.scale.x = 3 * 0.02;
    obs_marker.scale.y = 3 * 0.02;
    obs_marker.scale.z = 3 * 0.02;
    obs_marker.color.r = 0.45;
    obs_marker.color.g = 0.3;
    obs_marker.color.b = 0.4;
    obs_marker.color.a = 0.8;
    obs_marker.lifetime = ros::Duration(0.0);

    obs_marker.points.resize(dpath.size());

    for (int k = 0; k < int(dpath.size()); k++) {
        if (dpath[k].size() < 3) {
            continue;
        }

        obs_marker.points[k].x = dpath[k][0];
        obs_marker.points[k].y = dpath[k][1];
        obs_marker.points[k].z = dpath[k][2];
    }

    return obs_marker;
}

auto MakeSphereVisualization(
    const std::vector<double> &pos3,
    int hue,
    const std::string &ns,
    double radius,
    int &id,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    return MakeSphereVisualization(pos3, color, ns, radius, id, frame_id);
}

auto MakeSphereVisualization(
    const std::vector<double> &pose,
    int hue,
    const std::string &ns,
    double radius,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    return MakeSphereVisualization(pose, color, ns, radius, frame_id);
}

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    int hue,
    const std::string &ns,
    double radius,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    return MakeSpheresVisualization(pose, color, ns, radius, frame_id);
}

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    int hue,
    const std::string &ns,
    const std::vector<double> &radius,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray
{
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    return MakeSpheresVisualization(pose, color, ns, radius, frame_id);
}

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    int hue,
    const std::string &ns,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray
{
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    return MakeSpheresVisualization(pose, color, ns, frame_id);
}

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    const std::vector<int> &hue,
    const std::string &ns,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray
{
    double r = 0.0, g = 0.0, b = 0.0;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;

    if (pose.size() != hue.size()) {
        ROS_WARN_NAMED(LOG, "Didn't receive as many colors as I did spheres. Not visualizing. (spheres: %zu, colors: %zu)", pose.size(), hue.size());
        return { };
    }

    for (std::size_t i = 0; i < pose.size(); ++i) {
        leatherman::HSVtoRGB(&r, &g, &b, hue[i], 1.0, 1.0);
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = frame_id;
        marker.ns = ns;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 2.0 * pose[i][3];
        marker.scale.y = 2.0 * pose[i][3];
        marker.scale.z = 2.0 * pose[i][3];
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.6;
        marker.lifetime = ros::Duration(0.0);
        marker.id = i;
        marker.pose.position.x = pose[i][0];
        marker.pose.position.y = pose[i][1];
        marker.pose.position.z = pose[i][2];
        marker_array.markers.push_back(marker);
    }

    return marker_array;
}

auto MakeLineVisualization(
    const std::vector<geometry_msgs::Point> &points,
    const std::string &ns,
    int &id,
    int hue,
    double thickness,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    return MakeLineVisualization(points, ns, id, color, thickness, frame_id);
}

auto MakeTextVisualization(
    const geometry_msgs::Pose &pose,
    const std::string &text,
    const std::string &ns,
    int id,
    int hue,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    return MakeTextVisualization(pose, text, ns, id, color, frame_id);
}

auto MakeTextVisualization(
    const geometry_msgs::Pose &pose,
    const std::string &text,
    const std::string &ns,
    int id,
    const std::vector<double> &color,
    double size,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    visualization_msgs::Marker marker;

    const std::vector<double> &color_ = color.size() < 4 ?
            std::vector<double>(4, 1.0) : color;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.pose = pose;

    marker.color.r = color_[0];
    marker.color.g = color_[1];
    marker.color.b = color_[2];
    marker.color.a = color_[3];
    marker.text = text;
    marker.lifetime = ros::Duration(0.0);

    return marker;
}

auto MakeCubeVisualization(
    const geometry_msgs::PoseStamped &pose,
    int hue,
    const std::string &ns,
    int id,
    const std::vector<double> &dim)
    -> visualization_msgs::Marker
{
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    return MakeCubeVisualization(pose, color, ns, id, dim);
}

auto MakeMeshVisualization(
    const std::string& mesh_resource,
    const geometry_msgs::PoseStamped& pose,
    int hue,
    const std::string &ns,
    int id,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    return MakeMeshVisualization(mesh_resource, pose, color, ns, id, frame_id);
}

auto MakeMeshTrianglesVisualization(
    const std::vector<geometry_msgs::Point>& vertices,
    const std::vector<int>& triangles,
    const geometry_msgs::PoseStamped& pose,
    int hue,
    const std::string &ns,
    int id,
    bool psychadelic,
    const std::string &frame_id)
    -> visualization_msgs::Marker
{
    std_msgs::ColorRGBA color;
    leatherman::msgHSVToRGB((double)hue, 1.0, 1.0, color);
    return MakeMeshTrianglesVisualization(
            vertices, triangles, pose, color, ns, id, psychadelic, frame_id);
}

} // namespace pviz

SimpleViz::SimpleViz() :
    marker_ns_id_map_()
{
}

SimpleViz::~SimpleViz()
{
}

int SimpleViz::getMaxMarkerID(const std::string &ns)
{
    auto it = marker_ns_id_map_.find(ns);
    if (it == marker_ns_id_map_.end()) {
        return 0;
    }
    return it->second;
}

bool SimpleViz::hasNamespace(const std::string &ns)
{
    auto it = marker_ns_id_map_.find(ns);
    if (it == marker_ns_id_map_.end()) {
        return false;
    }
    return true;
}

void SimpleViz::clearVisualization(const std::string &ns)
{
    visualization_msgs::MarkerArray markers;
    int maxID = getMaxMarkerID(ns);
    for (int i = 0; i <= maxID; i++) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";
        marker.ns = ns;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::DELETE;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0.0);
        markers.markers.push_back(marker);
    }
    update(ns, 0);
    SV_SHOW_INFO(markers);
}

void SimpleViz::clearAllVisualizations()
{
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "";
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = 3; // visualization_msgs::Marker::DELETEALL;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.0);
    marker_ns_id_map_.clear();

    showMarker(std::move(marker));
}

void SimpleViz::visualizePose(
    const std::vector<double> &pose,
    const std::string &text,
    const std::string &ns,
    int &id,
    const std::string &frame_id)
{
    auto ma = pviz::MakePoseVisualization(pose, text, ns, id, frame_id);
    publish(ma);
}

void SimpleViz::visualizePose(
    const geometry_msgs::Pose &pose,
    const std::string &text,
    const std::string &ns,
    int &id,
    const std::string &frame_id)
{
    auto ma = pviz::MakePoseVisualization(pose, text, ns, id, frame_id);
    publish(ma);
}

void SimpleViz::visualizePoses(
    const std::vector<std::vector<double>> &poses,
    const std::string &frame_id)
{
    auto ma = pviz::MakePosesVisualization(poses, frame_id);
    publish(ma);
}

void SimpleViz::visualizeObstacles(
    const std::vector<std::vector<double>> &obstacles,
    const std::string &frame_id,
    const std::string &ns)
{
    auto ma = pviz::MakeObstaclesVisualization(obstacles, frame_id, ns);
    publish(ma);
}

void SimpleViz::visualize3DPath(
    const std::vector<std::vector<double>> &dpath,
    const std::string &frame_id)
{
    auto m = pviz::Make3DPathVisualization(dpath, frame_id);
    publish(std::move(m));
}

void SimpleViz::visualizeSphere(
    const std::vector<double> &pos3,
    int hue,
    const std::string &ns,
    double radius,
    int &id,
    const std::string &frame_id)
{
    auto m = pviz::MakeSphereVisualization(pos3, hue, ns, radius, id, frame_id);
    publish(std::move(m));
}

void SimpleViz::visualizeSphere(
    const std::vector<double> &pose,
    int hue,
    const std::string &ns,
    double radius,
    const std::string &frame_id)
{
    auto m = pviz::MakeSphereVisualization(pose, hue, ns, radius, frame_id);
    publish(std::move(m));
}

void SimpleViz::visualizeSpheres(
    const std::vector<std::vector<double>> &pose,
    int hue,
    const std::string &ns,
    double radius,
    const std::string &frame_id)
{
    auto m = pviz::MakeSpheresVisualization(pose, hue, ns, radius, frame_id);
    publish(std::move(m));
}

void SimpleViz::visualizeSpheres(
    const std::vector<std::vector<double>> &pose,
    int hue,
    const std::string &ns,
    const std::vector<double> &radius,
    const std::string &frame_id)
{
    auto ma = pviz::MakeSpheresVisualization(pose, hue, ns, radius, frame_id);
    publish(ma);
}

void SimpleViz::visualizeSpheres(
    const std::vector<std::vector<double>> &pose,
    int hue,
    const std::string &ns,
    const std::string &frame_id)
{
    auto ma = pviz::MakeSpheresVisualization(pose, hue, ns, frame_id);
    publish(ma);
}

void SimpleViz::visualizeSpheres(
    const std::vector<std::vector<double>> &pose,
    const std::vector<int> &hue,
    const std::string &ns,
    const std::string &frame_id)
{
    auto ma = pviz::MakeSpheresVisualization(pose, hue, ns, frame_id);
    publish(ma);
}

void SimpleViz::visualizeLine(
    const std::vector<geometry_msgs::Point> &points,
    const std::string &ns,
    int &id,
    int hue,
    double thickness,
    const std::string &frame_id)
{
    auto m = pviz::MakeLineVisualization(points, ns, id, hue, thickness, frame_id);
    publish(std::move(m));
}

void SimpleViz::visualizeText(
    const geometry_msgs::Pose &pose,
    const std::string &text,
    const std::string &ns,
    int id,
    int hue,
    const std::string &frame_id)
{
    auto m = pviz::MakeTextVisualization(pose, text, ns, id, hue, frame_id);
    publish(std::move(m));
}

void SimpleViz::visualizeText(
    const geometry_msgs::Pose &pose,
    const std::string &text,
    const std::string &ns,
    int id,
    const std::vector<double> &color,
    double size,
    const std::string &frame_id)
{
    auto m = pviz::MakeTextVisualization(pose, text, ns, id, color, size, frame_id);
    publish(std::move(m));
}

void SimpleViz::visualizeCube(
    const geometry_msgs::PoseStamped &pose,
    int hue,
    const std::string &ns,
    int id,
    const std::vector<double> &dim)
{
    auto m = pviz::MakeCubeVisualization(pose, hue, ns, id, dim);
    publish(std::move(m));
}

void SimpleViz::visualizeMesh(
    const std::string &mesh_resource,
    const geometry_msgs::PoseStamped &pose,
    int hue,
    const std::string &ns,
    int id,
    const std::string &frame_id)
{
    auto m = pviz::MakeMeshVisualization(mesh_resource, pose, hue, ns, id, frame_id);
    publish(std::move(m));
}

void SimpleViz::visualizeMeshTriangles(
    const std::vector<geometry_msgs::Point>& vertices,
    const std::vector<int>& triangles,
    const geometry_msgs::PoseStamped& pose,
    int hue,
    const std::string &ns,
    int id,
    bool psychadelic,
    const std::string &frame_id)
{
    auto m = pviz::MakeMeshTrianglesVisualization(
            vertices, triangles, pose, hue, ns, id, psychadelic, frame_id);
    publish(std::move(m));
}

void SimpleViz::publish(const visualization_msgs::MarkerArray &markers)
{
    for (const visualization_msgs::Marker &m : markers.markers){
        update(m.ns, m.id);
    }
    SV_SHOW_INFO(markers);
}

void SimpleViz::publish(const visualization_msgs::Marker &marker)
{
    update(marker.ns, marker.id);
    showMarker(marker);
}

void SimpleViz::publish(visualization_msgs::Marker &&marker)
{
    update(marker.ns, marker.id);
    showMarker(std::move(marker));
}

void SimpleViz::showMarker(visualization_msgs::Marker&& m)
{
    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(std::move(m));
    SV_SHOW_INFO(ma);
}

void SimpleViz::showMarker(const visualization_msgs::Marker& m)
{
    visualization_msgs::MarkerArray ma;
    ma.markers.push_back(m);
    SV_SHOW_INFO(ma);
}

void SimpleViz::update(const std::string &ns, int id)
{
    if (hasNamespace(ns)) {
        marker_ns_id_map_[ns] = std::max(id, getMaxMarkerID(ns));
    }
    else {
        marker_ns_id_map_[ns] = id;
    }
}
