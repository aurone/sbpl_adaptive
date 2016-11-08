#include <sbpl_adaptive_collision_checking/sbpl_collision_model.h>

namespace adim {

visualization_msgs::Marker SBPLCollisionModel::getSphereMarker(
    const Sphere &s,
    std::string ns,
    std::string frame_id,
    std_msgs::ColorRGBA col,
    int &id) const
{
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color = col;
    marker.header.frame_id = frame_id;
    marker.id = id;
    id++;
    marker.lifetime = ros::Duration(0);
    marker.ns = ns;
    marker.pose.position.x = s.v.x();
    marker.pose.position.y = s.v.y();
    marker.pose.position.z = s.v.z();
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.scale.x = 2 * s.radius;
    marker.scale.y = 2 * s.radius;
    marker.scale.z = 2 * s.radius;
    marker.type = visualization_msgs::Marker::SPHERE;
    return marker;
}

visualization_msgs::MarkerArray SBPLCollisionModel::getModelBasicVisualization(
    const ModelCoords &coords,
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
            visualization_msgs::Marker marker = getSphereMarker(
                    s, ns + "_contact_spheres", frame_id, col, idx);
        }
    }
    return markers;
}

} // namespace adim
