#ifndef PVIZ_SIMPLE_VIZ_H
#define PVIZ_SIMPLE_VIZ_H

/// \author Benjamin Cohen
/// \author Kalin Gochev
/// \author Andrew Dornbush

// standard includes
#include <map>
#include <string>
#include <vector>

// system includes
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <smpl/forward.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace pviz {

auto MakeSphereVisualization(
    const std::vector<double> &pos3,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    double radius,
    int &id,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeSphereVisualization(
    const std::vector<double> &pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    double radius,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    double radius,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    const std::vector<double> &radius,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray;

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray;

auto MakeLineVisualization(
    const std::vector<geometry_msgs::Point> &points,
    const std::string &ns,
    int &id,
    const std_msgs::ColorRGBA &color,
    double thickness,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeTextVisualization(
    const geometry_msgs::Pose &pose,
    const std::string &text,
    const std::string &ns,
    int id,
    const std_msgs::ColorRGBA &color,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeCubeVisualization(
    const geometry_msgs::PoseStamped &pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    int id,
    const std::vector<double> &dim)
    -> visualization_msgs::Marker;

auto MakeMeshVisualization(
    const std::string& mesh_resource,
    const geometry_msgs::PoseStamped& pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    int id,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeMeshTrianglesVisualization(
    const std::vector<geometry_msgs::Point>& vertices,
    const std::vector<int>& triangles,
    const geometry_msgs::PoseStamped& pose,
    const std_msgs::ColorRGBA &color,
    const std::string &ns,
    int id,
    bool psychadelic,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

///////////////////
// Hue Overloads //
///////////////////

auto MakePoseVisualization(
    const std::vector<double> &pose,
    const std::string &text,
    const std::string &ns,
    int &id,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray;

auto MakePoseVisualization(
    const geometry_msgs::Pose &pose,
    const std::string &text,
    const std::string &ns,
    int &id,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray;

auto MakePosesVisualization(
    const std::vector<std::vector<double>> &poses,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray;

auto MakeObstaclesVisualization(
    const std::vector<std::vector<double>> &obstacles,
    const std::string &frame_id,
    const std::string &ns = "")
    -> visualization_msgs::MarkerArray;

auto Make3DPathVisualization(
    const std::vector<std::vector<double>> &dpath,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeSphereVisualization(
    const std::vector<double> &pos3,
    int hue,
    const std::string &ns,
    double radius,
    int &id,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeSphereVisualization(
    const std::vector<double> &pose,
    int hue,
    const std::string &ns,
    double radius,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    int hue,
    const std::string &ns,
    double radius,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    int hue,
    const std::string &ns,
    const std::vector<double> &radius,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray;

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    int hue,
    const std::string &ns,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray;

auto MakeSpheresVisualization(
    const std::vector<std::vector<double>> &pose,
    const std::vector<int> &hue,
    const std::string &ns,
    const std::string &frame_id)
    -> visualization_msgs::MarkerArray;

auto MakeLineVisualization(
    const std::vector<geometry_msgs::Point> &points,
    const std::string &ns,
    int &id,
    int hue,
    double thickness,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeTextVisualization(
    const geometry_msgs::Pose &pose,
    const std::string &text,
    const std::string &ns,
    int id,
    int hue,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeTextVisualization(
    const geometry_msgs::Pose &pose,
    const std::string &text,
    const std::string &ns,
    int id,
    const std::vector<double> &color,
    double size,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeCubeVisualization(
    const geometry_msgs::PoseStamped &pose,
    int hue,
    const std::string &ns,
    int id,
    const std::vector<double> &dim)
    -> visualization_msgs::Marker;

auto MakeMeshVisualization(
    const std::string& mesh_resource,
    const geometry_msgs::PoseStamped& pose,
    int hue,
    const std::string &ns,
    int id,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

auto MakeMeshTrianglesVisualization(
    const std::vector<geometry_msgs::Point>& vertices,
    const std::vector<int>& triangles,
    const geometry_msgs::PoseStamped& pose,
    int hue,
    const std::string &ns,
    int id,
    bool psychadelic,
    const std::string &frame_id)
    -> visualization_msgs::Marker;

} // namespace pviz

SBPL_CLASS_FORWARD(SimpleViz);
class SimpleViz
{
public:

    SimpleViz();

    ~SimpleViz();

    int getMaxMarkerID(const std::string &ns);
    bool hasNamespace(const std::string &ns);

    void clearVisualization(const std::string &ns);
    void clearAllVisualizations();

    /**************** Meshes, Shapes, Text, & Lines ****************/

    /// \brief visualize a pose (sphere, arrow, string of text)
    void visualizePose(
        const std::vector<double> &pose,
        const std::string &text,
        const std::string &ns,
        int &id,
        const std::string &frame_id);

    void visualizePose(
        const geometry_msgs::Pose &pose,
        const std::string &text,
        const std::string &ns,
        int &id,
        const std::string &frame_id);

    /// \brief visualize a list of poses (sphere, arrow, pose index number)
    void visualizePoses(
        const std::vector<std::vector<double>> &poses,
        const std::string &frame_id);

    /// \brief visualize cuboids
    void visualizeObstacles(
        const std::vector<std::vector<double>> &obstacles,
        const std::string &frame_id,
        const std::string &ns = "");

    void visualize3DPath(
        const std::vector<std::vector<double>> &dpath,
        const std::string &frame_id);

    /// \brief display a sphere
    void visualizeSphere(
        const std::vector<double> &pos3,
        int hue,
        const std::string &ns,
        double radius,
        int &id,
        const std::string &frame_id);

    void visualizeSphere(
        const std::vector<double> &pose,
        int hue,
        const std::string &text,
        double radius,
        const std::string &frame_id);

    /// \brief display a list of spheres of the same radius and color
    void visualizeSpheres(
        const std::vector<std::vector<double>> &pose,
        int hue,
        const std::string &text,
        double radius,
        const std::string &frame_id);

    void visualizeSpheres(
        const std::vector<std::vector<double>> &pose,
        int hue,
        const std::string &text,
        const std::vector<double> &radius,
        const std::string &frame_id);

    void visualizeSpheres(
        const std::vector<std::vector<double>> &pose,
        int hue,
        const std::string &text,
        const std::string &frame_id);

    void visualizeSpheres(
        const std::vector<std::vector<double>> &pose,
        const std::vector<int> &hue,
        const std::string &text,
        const std::string &frame_id);

    void visualizeLine(
        const std::vector<geometry_msgs::Point> &points,
        const std::string &ns,
        int &id,
        int hue,
        double thickness,
        const std::string &frame_id);

    void visualizeText(
        const geometry_msgs::Pose &pose,
        const std::string &text,
        const std::string &ns,
        int id,
        int hue,
        const std::string &frame_id);

    void visualizeText(
        const geometry_msgs::Pose &pose,
        const std::string &text,
        const std::string &ns,
        int id,
        const std::vector<double> &color,
        double size,
        const std::string &frame_id);

    void visualizeCube(
        const geometry_msgs::PoseStamped &pose,
        int hue,
        const std::string &ns,
        int id,
        const std::vector<double> &dim);

    /// visualize a mesh where $mesh_resource is the path to the mesh using the
    /// URI used by resource_retriever package
    void visualizeMesh(
        const std::string& mesh_resource,
        const geometry_msgs::PoseStamped& pose,
        int hue,
        const std::string &ns,
        int id,
        const std::string &frame_id);

    /// visualizes a triangle list; if psychadelic is true then each triangle
    /// has one of each red, green, and blue vertices
    void visualizeMeshTriangles(
        const std::vector<geometry_msgs::Point>& vertices,
        const std::vector<int>& triangles,
        const geometry_msgs::PoseStamped& pose,
        int hue,
        const std::string &ns,
        int id,
        bool psychadelic,
        const std::string &frame_id);

    void publish(const visualization_msgs::MarkerArray &markers);
    void publish(const visualization_msgs::Marker &marker);
    void publish(visualization_msgs::Marker &&marker);

    void showMarker(visualization_msgs::Marker&& m);
    void showMarker(const visualization_msgs::Marker& m);

protected:

    void update(const std::string &ns, int id);

    std::map<std::string, int> marker_ns_id_map_;
};

#endif
