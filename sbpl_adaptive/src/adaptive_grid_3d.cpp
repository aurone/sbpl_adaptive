/*
 * sbpl_adaptive_grid3D.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Kalin Gochev
 */

#include <sbpl_adaptive/adaptive_grid_3d.h>

// system includes
#include <leatherman/utils.h>
#include <ros/console.h>
#include <ros/time.h>

namespace adim {

static double getDist2(
    int x1, int y1, int z1,
    int x2, int y2, int z2)
{
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2);
}

static double getDist(int x1, int y1, int z1, int x2, int y2, int z2)
{
    return sqrt(getDist2(x1, y1, z1, x2, y2, z2));
}

AdaptiveGrid3D::AdaptiveGrid3D(const sbpl::OccupancyGridPtr& grid) :
    trackMode_(false)
{
    oc_grid_ = grid;
    grid_sizes_.resize(3);
    oc_grid_->getGridSize(grid_sizes_[0], grid_sizes_[1], grid_sizes_[2]);

    AdaptiveGridCell default_cell;
    default_cell.costToGoal = INFINITECOST;
    default_cell.pDimID = InvalidDim;
    default_cell.pDefaultDimID = InvalidDim;
    default_cell.tDimID = InvalidDim;
    grid_.assign(grid_sizes_[0], grid_sizes_[1], grid_sizes_[2], default_cell);

    invalid_cell_ = default_cell;

    max_dimID_ = -1;
    max_costToGoal_ = 0;
}

void AdaptiveGrid3D::reset()
{
    clearAllSpheres();
    setPlanningMode();
}

void AdaptiveGrid3D::world2grid(
    double wx, double wy, double wz,
    size_t& gx, size_t& gy, size_t& gz) const
{
    int cx, cy, cz;
    oc_grid_->worldToGrid(wx, wy, wz, cx, cy, cz);
    gx = (size_t)cx;
    gy = (size_t)cy;
    gz = (size_t)cz;
}

void AdaptiveGrid3D::grid2world(
    size_t gx, size_t gy, size_t gz,
    double& wx, double& wy, double& wz) const
{
    double wx_, wy_, wz_;
    oc_grid_->gridToWorld(gx, gy, gz, wx_, wy_, wz_);
    wx = wx_;
    wy = wy_;
    wz = wz_;
}

void AdaptiveGrid3D::getBoundaryVisualizationPoints(
    std::vector<geometry_msgs::Point> &points,
    std::vector<std_msgs::ColorRGBA> &colors) const
{
    int x, y, z;
    geometry_msgs::Point p;

    size_t old_size = points.size();

    for (x = 0; x < grid_sizes_[0]; ++x) {
        y = 0, z = 0;
        geometry_msgs::Point p;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);

        y = grid_sizes_[1] - 1, z = 0;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);

        y = 0, z = grid_sizes_[2] - 1;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);

        y = grid_sizes_[1] - 1, z = grid_sizes_[2] - 1;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);
    }

    for (y = 1; y < grid_sizes_[1] - 1; ++y) {
        x = 0, z = 0;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);

        x = grid_sizes_[0] - 1, z = 0;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);

        x = 0, z = grid_sizes_[2] - 1;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);

        x = grid_sizes_[0] - 1, z = grid_sizes_[2] - 1;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);
    }

    for (z = 1; z < grid_sizes_[2] - 1; ++z) {
        y = 0, x = 0;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);

        y = grid_sizes_[1] - 1, x = 0;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);

        y = 0, x = grid_sizes_[0] - 1;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);

        y = grid_sizes_[1] - 1, x = grid_sizes_[0] - 1;
        grid2world(x, y, z, p.x, p.y, p.z);
        points.push_back(p);
    }

    colors.reserve(colors.size() + points.size() - old_size);
    for (size_t i = old_size; i < points.size(); ++i) {
        std_msgs::ColorRGBA color;
        color.a = 1.0f;
        color.r = color.g = color.b = 0.5f;
        colors.push_back(color);
    }
}

void AdaptiveGrid3D::resetTrackingGrid()
{
    // reset the tracking grid
    for (int i = 0; i < grid_sizes_[0]; i++) {
    for (int j = 0; j < grid_sizes_[1]; j++) {
    for (int k = 0; k < grid_sizes_[2]; k++) {
        // tracking grid becomes identical to planning grid
//        grid_(i, j, k).tDimID = grid_(i, j, k).pDimID;

        grid_(i, j, k).tDimID = InvalidDim;

        grid_(i, j, k).costToGoal = INFINITECOST;
    }
    }
    }
    max_costToGoal_ = 0;
}

void AdaptiveGrid3D::setCellCostToGoal(
    int gx, int gy, int gz,
    unsigned int costToGoal)
{
    if (trackMode_) {
        if (!isInBounds(gx, gy, gz)) {
            return;
        }
        max_costToGoal_ = std::max(max_costToGoal_, costToGoal);
        grid_(gx, gy, gz).costToGoal = costToGoal;
    }
}

bool AdaptiveGrid3D::setCellNearDim(
    bool tracking,
    size_t x,
    size_t y,
    size_t z,
    int dimID)
{
    max_dimID_ = std::max(max_dimID_, dimID);
    bool changed;
    if (tracking) {
        changed = (grid_(x, y, z).tNearDimID != dimID);
        grid_(x, y, z).tNearDimID = dimID;
    }
    else {
        changed = ((grid_(x, y, z).tNearDimID != dimID) ||
                (grid_(x, y, z).pNearDimID != dimID));
        grid_(x, y, z).pNearDimID = dimID;
        grid_(x, y, z).tNearDimID = dimID;
    }
    return changed;
}

void AdaptiveGrid3D::clearAllSpheres()
{
    //clears all HD regions from the grid
    for (size_t i = 0; i < (size_t)grid_sizes_[0]; ++i) {
    for (size_t j = 0; j < (size_t)grid_sizes_[1]; ++j) {
    for (size_t k = 0; k < (size_t)grid_sizes_[2]; ++k) {
        grid_(i, j, k).pDimID = grid_(i, j, k).pDefaultDimID;
        grid_(i, j, k).tDimID = grid_(i, j, k).pDefaultDimID;
        grid_(i, j, k).costToGoal = INFINITECOST;
    }
    }
    }
    max_costToGoal_ = 0;
}

void AdaptiveGrid3D::setPlanningMode()
{
    trackMode_ = false;
}

void AdaptiveGrid3D::setTrackingMode(
    const std::vector<AdaptiveSphere3D> &tunnel,
    std::vector<Position3D> &modCells)
{
    trackMode_ = true;
    resetTrackingGrid();
    for (const AdaptiveSphere3D &sphere : tunnel) {
        addTrackingSphere(sphere, modCells);
    }
}

unsigned int AdaptiveGrid3D::getCellCostToGoal(int gx, int gy, int gz) const
{
    if (!trackMode_) {
        return 0;
    }

    // returns invalid cell cost to goal for out of bounds cells
    return getCell(gx, gy, gz).costToGoal;
}

void AdaptiveGrid3D::getOverlappingSpheres(
    size_t x,
    size_t y,
    size_t z,
    int dimID,
    std::vector<std::vector<int>> &spheres)
{
    for (std::vector<int> s : spheres_) {
        if (s[4] != dimID) {
            continue;
        }
        if (getDist2(s[0], s[1], s[2], x, y, z) <= s[3] * s[3]) {
            //xyz is inside sphere
            spheres.push_back(s);
        }
    }
}

void AdaptiveGrid3D::addSphere(
    bool tracking,
    size_t x,
    size_t y,
    size_t z,
    int rad,
    int near_rad,
    int dimID,
    unsigned int costToGoal,
    std::vector<Position3D> &modCells)
{
    int min_x = 0; int max_x = grid_sizes_[0] - 1;
    int min_y = 0; int max_y = grid_sizes_[1] - 1;
    int min_z = 0; int max_z = grid_sizes_[2] - 1;

    if (isInPlanningMode() && dimEnabled(x, y, z, dimID, tracking)) {
        // HD at this location already
        std::vector<std::vector<int>> covering_spheres_;
        getOverlappingSpheres(x, y, z, dimID, covering_spheres_);
        printf("[adgrid] Location is of same dimension already! -- Growing %lu spheres!\n", covering_spheres_.size());
        //find the max radius and near radius of overlapping spheres
        for (size_t i = 0; i < covering_spheres_.size(); i++) {
            rad = std::max(rad, covering_spheres_[i][3]);
            near_rad = std::max(near_rad, covering_spheres_[i][4]);
        }
        //grow it by 50%
        rad *= 1.5;
    }

    min_x = std::max((int)x - rad - near_rad, min_x);
    max_x = std::min((int)x + rad + near_rad, max_x);

    min_y = std::max((int)y - rad - near_rad, min_y);
    max_y = std::min((int)y + rad + near_rad, max_y);

    min_z = std::max((int)z - rad - near_rad, min_z);
    max_z = std::min((int)z + rad + near_rad, max_z);

    for (int i = min_x; i <= max_x; i++) {
    for (int j = min_y; j <= max_y; j++) {
    for (int k = min_z; k <= max_z; k++) {
        double dist2 = getDist2(x, y, z, i, j, k);

        if (dist2 <= rad * rad) {
            // in sphere
            if (enableDim(i, j, k, dimID, tracking)) {
                adim::Position3D modp;
                grid2world(i, j, k, modp.x, modp.y, modp.z);
                modCells.push_back(modp);
            }
            setCellCostToGoal(i, j, k, costToGoal);
        }
        else if (dist2 <= (rad + near_rad) * (rad + near_rad)) {
            //near sphere
            if (enableNearDim(i, j, k, dimID, tracking)) {
                adim::Position3D modp;
                grid2world(i, j, k, modp.x, modp.y, modp.z);
                modCells.push_back(modp);
            }
        }
    }
    }
    }

    if (!trackMode_) {
        std::vector<int> sphere(5, 0);
        sphere[0] = x;
        sphere[1] = y;
        sphere[2] = z;
        sphere[3] = rad;
        sphere[4] = near_rad;
        spheres_.push_back(sphere);
    }
}

visualization_msgs::MarkerArray AdaptiveGrid3D::getVisualizations(
    std::string ns_prefix,
    int throttle,
    double scale)
{
    visualization_msgs::MarkerArray marker;
    marker.markers.resize(2);
    marker.markers[0] = getAdaptiveGridVisualization(ns_prefix, throttle, scale);
    marker.markers[1] = getCostToGoalGridVisualization(ns_prefix, throttle, scale);
    return marker;
}

visualization_msgs::Marker AdaptiveGrid3D::getAdaptiveGridVisualization(
    std::string ns_prefix,
    int throttle,
    double scale)
{
    if (max_dimID_ == -1) {
        // no dimensions assigned anywhere --> no visualizations
        return visualization_msgs::Marker();
    }

    visualization_msgs::Marker marker;
    double m_scale = scale * oc_grid_->resolution();
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = oc_grid_->getReferenceFrame();
    marker.ns = ns_prefix + "_AdaptiveGrid3D";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0;
    marker.scale.x = marker.scale.y = marker.scale.z = m_scale;
    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0f;
    marker.lifetime = ros::Duration(0.0);
    marker.frame_locked = false;
    for (int x = 0; x < grid_sizes_[0]; x += throttle) {
    for (int y = 0; y < grid_sizes_[1]; y += throttle) {
    for (int z = 0; z < grid_sizes_[2]; z += throttle) {
        std_msgs::ColorRGBA col;
        for (int i = 0; i <= max_dimID_; ++i) {
            if (dimEnabled(x, y, z, i, trackMode_)) {
                double hue = 360.0 * i / (double)(max_dimID_ + 1);
                std_msgs::ColorRGBA c;
                leatherman::msgHSVToRGB(hue, 1.0, 1.0, c);
                // additive color per dimension
                col.r += c.r;
                col.g += c.g;
                col.b += c.b;

            }
        }

        // normalize color
        float m = col.r;
        m = std::max(m, col.g);
        m = std::max(m, col.b);
        if (m == 0.0f) {
            continue;
        }
        float minv = 1.0 / m;
        col.r *= minv;
        col.g *= minv;
        col.b *= minv;
        col.a = 1.0f;

        double wx, wy, wz;
        grid2world(x, y, z, wx, wy, wz);
        geometry_msgs::Point p;
        p.x = wx;
        p.y = wy;
        p.z = wz;
        marker.points.push_back(p);
        marker.colors.push_back(col);
    }
    }
    }
    return marker;
}

visualization_msgs::Marker AdaptiveGrid3D::getCostToGoalGridVisualization(
    std::string ns_prefix,
    int throttle,
    double scale)
{
    visualization_msgs::Marker marker;
    double m_scale = scale * oc_grid_->resolution();
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = oc_grid_->getReferenceFrame();
    marker.ns = ns_prefix + "_AdaptiveGrid3D_indeces";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = m_scale;
    marker.scale.y = m_scale;
    marker.scale.z = m_scale;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;
    marker.color.a = 1;
    if (max_costToGoal_ > 0) {
        for (size_t x = 0; x < (size_t)grid_sizes_[0]; x += throttle) {
        for (size_t y = 0; y < (size_t)grid_sizes_[1]; y += throttle) {
        for (size_t z = 0; z < (size_t)grid_sizes_[2]; z += throttle) {
            unsigned int idx = getCellCostToGoal((int)x, (int)y, (int)z);

            if (idx > max_costToGoal_) {
                continue;
            }
            double hue = 360.0 * idx / (double)(max_costToGoal_);
            double wx, wy, wz;
            grid2world(x, y, z, wx, wy, wz);
            geometry_msgs::Point p;
            p.x = wx;
            p.y = wy;
            p.z = wz;
            std_msgs::ColorRGBA col;
            double r, g, b;
            leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);
            col.r = r;
            col.g = g;
            col.b = b;
            col.a = 1.0;
            marker.points.push_back(p);
            marker.colors.push_back(col);
        }
        }
        }
    }
    marker.lifetime = ros::Duration(0.0);
    return marker;
}

} // namespace adim
