#include <sbpl_adaptive_collision_checking/collision_checker.h>

#include <ros/console.h>

namespace adim {

CollisionChecker::CollisionChecker()
{
}

CollisionChecker::~CollisionChecker()
{
}

visualization_msgs::MarkerArray CollisionChecker::getCollisionModelVisualization(
    const std::vector<double> &angles)
{
    ROS_ERROR("Function is not filled in.");
    return visualization_msgs::MarkerArray();
}

visualization_msgs::MarkerArray CollisionChecker::getVisualization(
    std::string type)
{
    ROS_ERROR("Function is not filled in.");
    return visualization_msgs::MarkerArray();
}

} // namespace adim
