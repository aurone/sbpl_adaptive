#ifndef _COLLISION_CHECKER_
#define _COLLISION_CHECKER_

#include <string>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

namespace adim {

class CollisionChecker
{
public:

    CollisionChecker();
    virtual ~CollisionChecker();

    /// \brief Return whether a state is valid.
    /// \param[in] angles[in] The joint angles of the default joint group
    /// \param[in] verbose[in] Whether to produce verbose output
    /// \param[in] visualize[in] Whether to store collision details for the next call to getVisualization
    /// \param[out] dist The distance to the nearest obstacle
    /// \return Whether the state is valid
    virtual bool isStateValid(
        const std::vector<double> &angles,
        bool verbose,
        bool visualize,
        double &dist) = 0;

    /// \brief Return whether the interpolated path between two points is valid.
    /// \param[in] angles0 The start configuration of the default joint group
    /// \param[in] angles1 The start configuration of the default joint group
    /// \param[out] path_length The number of waypoints in the path between angles0 and angles1
    /// \param[out] num_checks The number of collision checks to perform
    /// \param[out] dist The distance to the nearest obstacle
    /// \return Whether the interpolated path is valid
    virtual bool isStateToStateValid(
        const std::vector<double> &angles0,
        const std::vector<double> &angles1,
        int &path_length,
        int &num_checks,
        double &dist) = 0;

    /// \brief Return a linearly interpolated path between two joint states.
    ///        Oddly pure virtual because of its usage in post_processing.cpp.
    /// \param[in] start The start configuration of the default joint group
    /// \param[in] end The end configuration of the default joint group
    /// \param[in] inc The maximum joint angle increment to be applied between each set of waypoints
    /// \param[out] path The output path
    /// \return Whether a valid linearly interpolated path could be constructed
    virtual bool interpolatePath(
        const std::vector<double> &start,
        const std::vector<double> &end,
        const std::vector<double> &inc,
        std::vector<std::vector<double>> &path) = 0;

    /* Visualizations */
    virtual visualization_msgs::MarkerArray getCollisionModelVisualization(
        const std::vector<double> &angles);

    virtual visualization_msgs::MarkerArray getVisualization(std::string type);
};

} // namespace adim

#endif
