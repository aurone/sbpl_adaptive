/*
 * simpleviz.h
 *
 *  Created on: Feb 8, 2016
 *      Author: kalin
 */

#ifndef ADAPTIVE_PLANNING_PVIZ_INCLUDE_PVIZ_SIMPLEVIZ_H_
#define ADAPTIVE_PLANNING_PVIZ_INCLUDE_PVIZ_SIMPLEVIZ_H_


/* \author Benjamin Cohen */

// standard includes
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <string>

// system includes
#include <boost/lexical_cast.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <smpl/forward.h>
#include <smpl/debug/visualize.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

SBPL_CLASS_FORWARD(SimpleViz)
class SimpleViz
{
public:

    SimpleViz();

    ~SimpleViz();

    int getMaxMarkerID(std::string ns);
    bool hasNamespace(std::string ns);

    void waitForSubscribers() { }

    void clearVisualization(std::string ns);
    void clearAllVisualizations();

    /**************** Meshes, Shapes, Text, & Lines ****************/

    /* \brief visualize a pose (sphere, arrow, string of text) */
    void visualizePose(
        const std::vector<double> &pose,
        std::string text,
        std::string ns,
        int &id,
        std::string frame_id);

    void visualizePose(
        const geometry_msgs::Pose &pose,
        std::string text,
        std::string ns,
        int &id,
        std::string frame_id);

    /* \brief visualize a list of poses (sphere, arrow, pose index number) */
    void visualizePoses(
        const std::vector<std::vector<double>> &poses,
        std::string reference_frame_);

    /* \brief visualize cuboids */
    void visualizeObstacles(
        const std::vector<std::vector<double>> &obstacles,
        std::string reference_frame_);

    void visualize3DPath(
        std::vector<std::vector<double>> &dpath,
        std::string reference_frame_);

    /* \brief display a sphere */
    void visualizeSphere(
        std::vector<double> pos3,
        int color,
        std::string ns,
        double radius,
        int &id,
        std::string reference_frame_);

    void visualizeSphere(
        std::vector<double> pose,
        int color,
        std::string text,
        double radius,
        std::string reference_frame_);

    /* \brief display a list of spheres of the same radius and color */
    void visualizeSpheres(
        const std::vector<std::vector<double>> &pose,
        int color,
        std::string text,
        double radius,
        std::string reference_frame_);

    void visualizeSpheres(
        const std::vector<std::vector<double>> &pose,
        int color,
        std::string text,
        std::vector<double> &radius,
        std::string reference_frame_);

    void visualizeSpheres(
        const std::vector<std::vector<double>> &pose,
        int color,
        std::string text,
        std::string reference_frame_);

    void visualizeSpheres(
        const std::vector<std::vector<double>> &pose,
        const std::vector<int> &hue,
        std::string text,
        std::string reference_frame_);

    void visualizeLine(
        const std::vector<geometry_msgs::Point> points,
        std::string ns,
        int &id,
        int hue,
        double thickness,
        std::string reference_frame_);

    void visualizeText(
        geometry_msgs::Pose pose,
        std::string text,
        std::string ns,
        int id,
        int hue,
        std::string reference_frame_);

    void visualizeText(
        geometry_msgs::Pose pose,
        std::string text,
        std::string ns,
        int id,
        std::vector<double> color,
        double size,
        std::string reference_frame_);

    void visualizeCube(
        geometry_msgs::PoseStamped pose,
        int color,
        std::string ns,
        int id,
        std::vector<double> dim);

    // visualize a mesh where $mesh_resource is the path to the mesh using the
    // URI used by resource_retriever package
    void visualizeMesh(
        const std::string& mesh_resource,
        const geometry_msgs::PoseStamped& pose,
        int color,
        const std::string &ns,
        int id,
        const std::string &reference_frame_);

    // visualizes a triangle list; if psychadelic is true then each triangle has
    // one of each red, green, and blue vertices
    void visualizeMeshTriangles(
        const std::vector<geometry_msgs::Point>& vertices,
        const std::vector<int>& triangles,
        const geometry_msgs::PoseStamped& pose,
        int color,
        std::string ns,
        int id,
        bool psychadelic,
        std::string reference_frame_);

    void deleteVisualizations(
        std::string ns,
        int max_id,
        std::string reference_frame_);

    void publish(const visualization_msgs::MarkerArray &markers);
    void publish(const visualization_msgs::Marker &marker);

    void showMarker(visualization_msgs::Marker&& m);
    void showMarker(const visualization_msgs::Marker& m);

protected:

    void update(std::string ns, int id);

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;

    std::map<std::string, int> marker_ns_id_map_;

    visualization_msgs::MarkerArray marker_array_;
    visualization_msgs::Marker marker_;

    void toMarkers(const visualization_msgs::Marker& m) const;
};

#endif /* ADAPTIVE_PLANNING_PVIZ_INCLUDE_PVIZ_SIMPLEVIZ_H_ */
