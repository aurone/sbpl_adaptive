#include <sbpl_adaptive/sparse_adaptive_grid_3d.h>

// system includes
#include <map>

// system includes
#include <Eigen/Dense>
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

SparseAdaptiveGrid3D::SparseAdaptiveGrid3D(const sbpl::OccupancyGridPtr& grid) :
    trackMode_(false)
{
    oc_grid_ = grid;

    grid_sizes_[0] = oc_grid_->numCellsX();
    grid_sizes_[1] = oc_grid_->numCellsY();
    grid_sizes_[2] = oc_grid_->numCellsZ();
    grid_.resize(grid_sizes_[0], grid_sizes_[1], grid_sizes_[2]);

    AdaptiveGridCell default_cell;
    default_cell.costToGoal = INFINITECOST;
    default_cell.pDimID = InvalidDim;
    default_cell.pDefaultDimID = InvalidDim;
    default_cell.tDimID = InvalidDim;
    grid_.reset(default_cell);

    invalid_cell_ = default_cell;

    max_dimID_ = -1;
    max_costToGoal_ = 0;
}

void SparseAdaptiveGrid3D::reset()
{
    clearAllSpheres();
    setPlanningMode();
}

void SparseAdaptiveGrid3D::world2grid(
    double wx, double wy, double wz,
    int& gx, int& gy, int& gz) const
{
    int cx, cy, cz;
    oc_grid_->worldToGrid(wx, wy, wz, cx, cy, cz);
    gx = cx;
    gy = cy;
    gz = cz;
}

void SparseAdaptiveGrid3D::grid2world(
    int gx, int gy, int gz,
    double& wx, double& wy, double& wz) const
{
    double wx_, wy_, wz_;
    oc_grid_->gridToWorld(gx, gy, gz, wx_, wy_, wz_);
    wx = wx_;
    wy = wy_;
    wz = wz_;
}

void SparseAdaptiveGrid3D::getBoundaryVisualizationPoints(
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

void SparseAdaptiveGrid3D::resetTrackingGrid()
{
    auto invalidate_tracking = [](AdaptiveGridCell &c) {
        // tracking grid becomes identical to planning grid
//        grid_(i, j, k).tDimID = grid_(i, j, k).pDimID;
        c.tDimID = InvalidDim;
        c.costToGoal = INFINITECOST;
    };
    grid_.accept(invalidate_tracking);
}

void SparseAdaptiveGrid3D::setCellCostToGoal(
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

bool SparseAdaptiveGrid3D::setCellNearDim(
    bool tracking,
    int x,
    int y,
    int z,
    int dimID)
{
    max_dimID_ = std::max(max_dimID_, dimID);
    bool changed;
    if (tracking) {
        changed = (grid_.get(x, y, z).tNearDimID != dimID);
        if (changed) {
            grid_(x, y, z).tNearDimID = dimID;
        }
    }
    else {
        const AdaptiveGridCell &ccell = grid_.get(x, y, z);
        changed = (ccell.tNearDimID != dimID) || (ccell.pNearDimID != dimID);
        if (changed) {
            AdaptiveGridCell &cell = grid_(x, y, z);
            cell.pNearDimID = dimID;
            cell.tNearDimID = dimID;
        }
    }
    return changed;
}

void SparseAdaptiveGrid3D::clearAllSpheres()
{
    // clears all HD regions from the grid
    auto clear_all = [](AdaptiveGridCell &cell) {
        cell.pDimID = cell.pDefaultDimID;
        cell.tDimID = cell.pDefaultDimID;
        cell.costToGoal = INFINITECOST;
    };
    grid_.accept(clear_all);

    max_costToGoal_ = 0;
}

void SparseAdaptiveGrid3D::setPlanningMode()
{
    trackMode_ = false;
    grid_.prune();
}

void SparseAdaptiveGrid3D::setTrackingMode(
    const std::vector<AdaptiveSphere3D> &tunnel,
    std::vector<Position3D> &modCells)
{
    trackMode_ = true;
    resetTrackingGrid();
    for (const AdaptiveSphere3D &sphere : tunnel) {
        addTrackingSphere(sphere, modCells);
    }
    grid_.prune();
}

unsigned int SparseAdaptiveGrid3D::getCellCostToGoal(int gx, int gy, int gz) const
{
    if (!trackMode_) {
        return 0;
    }

    // returns invalid cell cost to goal for out of bounds cells
    return getCell(gx, gy, gz).costToGoal;
}

void SparseAdaptiveGrid3D::getOverlappingSpheres(
    int x,
    int y,
    int z,
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

void SparseAdaptiveGrid3D::addSphere(
    bool tracking,
    int x,
    int y,
    int z,
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

visualization_msgs::MarkerArray SparseAdaptiveGrid3D::getVisualizations(
    std::string ns_prefix,
    int throttle,
    double scale)
{
    visualization_msgs::MarkerArray marker;
    auto ma = getAdaptiveGridVisualization(ns_prefix, throttle, scale);
    marker.markers.insert(marker.markers.end(), ma.markers.begin(), ma.markers.end());
//    marker.markers.push_back(getCostToGoalGridVisualization(ns_prefix, throttle, scale));
    int id = 0;
    for (auto &m : marker.markers) {
        m.id = id++;
    }
    return marker;
}

visualization_msgs::MarkerArray SparseAdaptiveGrid3D::getAdaptiveGridVisualization(
    std::string ns_prefix,
    int throttle,
    double scale)
{
    if (max_dimID_ == -1) {
        // no dimensions assigned anywhere --> no visualizations
        return visualization_msgs::MarkerArray();
    }

    grid_.prune();

    int id = 0;
    std::map<std::tuple<int, int, int>, visualization_msgs::Marker> ma_map;
    auto gather = [&](
        const AdaptiveGridCell &cell,
        size_t xfirst, size_t yfirst, size_t zfirst,
        size_t xlast, size_t ylast, size_t zlast)
    {
        // get world coordinates of extents
        double wx_from, wy_from, wz_from;
        double wx_to, wy_to, wz_to;

        oc_grid_->gridToWorld(xfirst, yfirst, zfirst, wx_from, wy_from, wz_from);
        oc_grid_->gridToWorld(xlast, ylast, zlast, wx_to, wy_to, wz_to);
        if (!oc_grid_->isInBounds((int)xfirst, (int)yfirst, (int)zfirst) &&
            !oc_grid_->isInBounds((int)xlast, (int)ylast, (int)zlast))
        {
            return;
        }

        std::tuple<int, int, int> s(xlast - xfirst, ylast - yfirst, zlast - zfirst);

        // create the marker for leaves of this size
        auto it = ma_map.find(s);
        if (it == ma_map.end()) {
            bool inserted;
            std::tie(it, inserted) = ma_map.insert(
                    std::make_pair(s, visualization_msgs::Marker()));
            it->second.header.stamp = ros::Time::now();
            it->second.header.frame_id = oc_grid_->getReferenceFrame();
            it->second.ns = ns_prefix + "_AdaptiveGrid3D";
            it->second.id = id++;
            it->second.type = visualization_msgs::Marker::CUBE_LIST;
            it->second.action = visualization_msgs::Marker::ADD;
            it->second.scale.x = wx_to - wx_from;
            it->second.scale.y = wy_to - wy_from;
            it->second.scale.z = wz_to - wz_from;
            it->second.lifetime = ros::Duration(0.0);
            it->second.frame_locked = false;
            it->second.pose.orientation.w = 1.0;
        }

        std_msgs::ColorRGBA col;
        for (int i = 0; i <= max_dimID_; ++i) {
            if (dimEnabled(xfirst, yfirst, zfirst, i, trackMode_)) {
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
        float mc = col.r;
        mc = std::max(mc, col.g);
        mc = std::max(mc, col.b);
        if (mc == 0.0f) { // skip black (no dim) cells
            col.r = 0.4;
            col.g = 0.4;
            col.b = 0.4;
            col.a = 0.2;
            return; // uncomment me if you'd like some freespace prints
        } else {
            float minv = 1.0 / mc;
            col.r *= minv;
            col.g *= minv;
            col.b *= minv;
            col.a = 1.0f;
        }

        geometry_msgs::Point p;
        p.x = 0.5 * (wx_from + wx_to) - 0.5 * oc_grid_->resolution();
        p.y = 0.5 * (wy_from + wy_to) - 0.5 * oc_grid_->resolution();
        p.z = 0.5 * (wz_from + wz_to) - 0.5 * oc_grid_->resolution();
        it->second.points.push_back(p);
        it->second.colors.push_back(col);
    };

    grid_.accept_coords(gather);

    // gather all markers into MarkerArray
    visualization_msgs::MarkerArray ma;
    for (auto &entry : ma_map) {
        if (!entry.second.points.empty()) {
            ma.markers.push_back(std::move(entry.second));
        }
    }
    id = 0;
    for (auto &m : ma.markers) {
        m.id = id++;
    }

    ROS_INFO_STREAM("Created Visualization from " << grid_.num_nodes() << " nodes");
    return ma;

//    visualization_msgs::Marker marker;
//    double m_scale = scale * oc_grid_->resolution();
//    marker.header.stamp = ros::Time::now();
//    marker.header.frame_id = oc_grid_->getReferenceFrame();
//    marker.ns = ns_prefix + "_AdaptiveGrid3D";
//    marker.id = 0;
//    marker.type = visualization_msgs::Marker::CUBE_LIST;
//    marker.action = visualization_msgs::Marker::ADD;
//    marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.0;
//    marker.pose.orientation.w = 1.0;
//    marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z = 0.0;
//    marker.scale.x = marker.scale.y = marker.scale.z = m_scale;
//    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0f;
//    marker.lifetime = ros::Duration(0.0);
//    marker.frame_locked = false;
//    for (int x = 0; x < grid_sizes_[0]; x += throttle) {
//    for (int y = 0; y < grid_sizes_[1]; y += throttle) {
//    for (int z = 0; z < grid_sizes_[2]; z += throttle) {
//        std_msgs::ColorRGBA col;
//        for (int i = 0; i <= max_dimID_; ++i) {
//            if (dimEnabled(x, y, z, i, trackMode_)) {
//                double hue = 360.0 * i / (double)(max_dimID_ + 1);
//                std_msgs::ColorRGBA c;
//                leatherman::msgHSVToRGB(hue, 1.0, 1.0, c);
//                // additive color per dimension
//                col.r += c.r;
//                col.g += c.g;
//                col.b += c.b;
//
//            }
//        }
//
//        // normalize color
//        float m = col.r;
//        m = std::max(m, col.g);
//        m = std::max(m, col.b);
//        if (m == 0.0f) {
//            continue;
//        }
//        float minv = 1.0 / m;
//        col.r *= minv;
//        col.g *= minv;
//        col.b *= minv;
//        col.a = 1.0f;
//
//        double wx, wy, wz;
//        grid2world(x, y, z, wx, wy, wz);
//        geometry_msgs::Point p;
//        p.x = wx;
//        p.y = wy;
//        p.z = wz;
//        marker.points.push_back(p);
//        marker.colors.push_back(col);
//    }
//    }
//    }
//    return marker;
}

visualization_msgs::Marker SparseAdaptiveGrid3D::getCostToGoalGridVisualization(
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
        for (int x = 0; x < grid_sizes_[0]; x += throttle) {
        for (int y = 0; y < grid_sizes_[1]; y += throttle) {
        for (int z = 0; z < grid_sizes_[2]; z += throttle) {
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
