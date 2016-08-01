/*
 * sbpl_adaptive_grid3D.cpp
 *
 *  Created on: Nov 4, 2014
 *      Author: Kalin Gochev
 */
#include <sbpl_adaptive_components/sbpl_adaptive_grid3D.h>

namespace sbpl_adaptive_components {

void AdaptiveGrid3D_t::reset(){
	clearAllSpheres();
	setPlanningMode();
}

void AdaptiveGrid3D_t::world2grid(double wx, double wy, double wz, size_t& gx, size_t& gy, size_t& gz) const {
	int cx, cy, cz;
	oc_grid_->worldToGrid(wx, wy, wz, cx, cy, cz);
	if(!isInBounds({cx, cy, cz})){
	    ROS_WARN("World position %.3f %.3f %.3f [%d %d %d] out of bounds!", wx,wy,wz,cx,cy,cz);
        throw SBPL_Exception();
	}
	gx = (size_t)cx;
	gy = (size_t)cy;
	gz = (size_t)cz;
}

void AdaptiveGrid3D_t::grid2world(size_t gx, size_t gy, size_t gz, double& wx, double& wy, double& wz) const {
	double wx_, wy_, wz_;
	oc_grid_->gridToWorld(gx, gy, gz, wx_, wy_, wz_);
	wx = wx_;
	wy = wy_;
	wz = wz_;
}

double AdaptiveGrid3D_t::getDist2(int x1, int y1, int z1, int x2, int y2, int z2) {
	return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2);
}

double AdaptiveGrid3D_t::getDist(int x1, int y1, int z1, int x2, int y2, int z2) {
	return sqrt(getDist2(x1,y1,z1,x2,y2,z2));
}

void AdaptiveGrid3D_t::resetTrackingGrid(){
	//reset the tracking grid
	for(int i = 0; i < grid_sizes_[0]; i++){
		for(int j = 0; j < grid_sizes_[1]; j++){
			for(int k = 0; k < grid_sizes_[2]; k++){
			    grid_[i][j][k].tDimID = -1;//grid_[i][j][k].pDimID; //tracking grid becomes identical to planning grid
			    grid_[i][j][k].costToGoal = INFINITECOST;
			}
		}
	}
	max_costToGoal_ = 0;
}

void AdaptiveGrid3D_t::setCellCostToGoal(const std::vector<int> &coord, unsigned int costToGoal){
	if(trackMode_){
	    if(!isInBounds({coord[0],coord[1],coord[2]})) return;
	    max_costToGoal_ = std::max(max_costToGoal_, costToGoal);
	    grid_[coord[0]][coord[1]][coord[2]].costToGoal = costToGoal;
	}
}

bool AdaptiveGrid3D_t::setCellDim(bool bTrackMode, size_t x, size_t y, size_t z, int dimID){
    max_dimID_ = std::max(max_dimID_, dimID);
    bool changed;
    if(bTrackMode){
        changed = (grid_[x][y][z].tDimID != dimID);
        grid_[x][y][z].tDimID = dimID;
    } else {
        changed = ((grid_[x][y][z].tDimID != dimID) || (grid_[x][y][z].pDimID != dimID));
        grid_[x][y][z].pDimID = dimID;
        grid_[x][y][z].tDimID = dimID;
    }
    return changed;
}

bool AdaptiveGrid3D_t::setCellNearDim(bool bTrackMode, size_t x, size_t y, size_t z, int dimID){
    max_dimID_ = std::max(max_dimID_, dimID);
    bool changed;
    if(bTrackMode){
        changed = (grid_[x][y][z].tNearDimID != dimID);
        grid_[x][y][z].tNearDimID = dimID;
    } else {
        changed = ((grid_[x][y][z].tNearDimID != dimID) || (grid_[x][y][z].pNearDimID != dimID));
        grid_[x][y][z].pNearDimID = dimID;
        grid_[x][y][z].tNearDimID = dimID;
    }
    return changed;
}

void AdaptiveGrid3D_t::init(){
    max_costToGoal_ = 0;
    for(size_t i = 0; i < (size_t)grid_sizes_[0]; i++){
        grid_[i].resize(grid_sizes_[1]);
        for(size_t j = 0; j < (size_t)grid_sizes_[1]; j++){
            grid_[i][j].resize(grid_sizes_[2]);
            for(size_t k = 0; k < (size_t)grid_sizes_[2]; k++){
                grid_[i][j][k].costToGoal = INFINITECOST;
                grid_[i][j][k].pDimID = ldID_;
                grid_[i][j][k].tDimID = grid_[i][j][k].pDimID;
            }
        }
    }
}

AdaptiveGrid3D_t::AdaptiveGrid3D_t(std::shared_ptr<sbpl_adaptive_collision_checking::OccupancyGrid> grid, int ldID) : ph_("~"), AdaptiveGrid_t(ldID) {
	oc_grid_ = grid;
	grid_sizes_.resize(3);
	oc_grid_->getGridSize(grid_sizes_[0], grid_sizes_[1], grid_sizes_[2]);
	grid_.resize(grid_sizes_[0]);
	for(size_t i = 0; i < (size_t)grid_sizes_[0]; i++){
		grid_[i].resize(grid_sizes_[1]);
		for(size_t j = 0; j < (size_t)grid_sizes_[1]; j++){
			grid_[i][j].resize(grid_sizes_[2]);
			for(size_t k = 0; k < (size_t)grid_sizes_[2]; k++){
			    grid_[i][j][k].costToGoal = INFINITECOST;
			    grid_[i][j][k].pDimID = ldID_;
			    grid_[i][j][k].pDefaultDimID = ldID_;
			    grid_[i][j][k].tDimID = grid_[i][j][k].pDimID;
			}
		}
	}
	frame_ = grid->getReferenceFrame();
	SBPL_INFO("[ad_grid] Allocated grid of size %d x %d x %d in frame %s", grid_sizes_[0], grid_sizes_[1], grid_sizes_[2], frame_.c_str());
	trackMode_ = false;
	marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500, true);
	marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 500, true);
	max_dimID_ = 0;
	max_costToGoal_ = 0;
}

AdaptiveGrid3D_t::~AdaptiveGrid3D_t(){
	grid_.clear();
	spheres_.clear();
}

void AdaptiveGrid3D_t::clearAllSpheres(){
	//clears all HD regions from the grid
	for(size_t i = 0; i < (size_t)grid_sizes_[0]; i++){
		for(size_t j = 0; j < (size_t)grid_sizes_[1]; j++){
			for(size_t k = 0; k < (size_t)grid_sizes_[2]; k++){
				grid_[i][j][k].pDimID = grid_[i][j][k].pDefaultDimID;
				grid_[i][j][k].tDimID = grid_[i][j][k].pDefaultDimID;
				grid_[i][j][k].costToGoal = INFINITECOST;
			}
		}
	}
	max_costToGoal_ = 0;
}

void AdaptiveGrid3D_t::setPlanningMode(){
	trackMode_ = false;
	resetTrackingGrid();
}

void AdaptiveGrid3D_t::setTrackingMode(const std::vector<sbpl_adaptive::AdaptiveSphere3D_t> &tunnel, std::vector<sbpl_adaptive::Position3D_t> &modCells){
    trackMode_ = true;
    for(size_t i = 0; i < tunnel.size(); i++){
        addTrackingSphere(tunnel[i], modCells);
    }
}

void AdaptiveGrid3D_t::setTrackingMode(const std::vector<std::vector<int>> &tunnel, const std::vector<int> &costsToGoal, std::vector<sbpl_adaptive::Position3D_t> &modCells){
	trackMode_ = true;
	int min_val = -1;
	for(size_t i = 0; i < tunnel.size(); i++){
		if(tunnel[i].size() < 5) continue;
		//tunnel[i][0] = x, tunnel[i][1] = y, tunnel[i][2] = z, tunnel[i][3] = rad, tunnel[i][4] = dimID
		addSphere(true, tunnel[i][0], tunnel[i][1], tunnel[i][2], tunnel[i][3], 0, tunnel[i][4], costsToGoal[i], modCells);
	}
}

unsigned int AdaptiveGrid3D_t::getCellCostToGoal(const std::vector<int> &coord) const{
	if(!trackMode_) return 0;
	if(!isInBounds({coord[0],coord[1],coord[2]})) return INFINITECOST;
	return grid_[coord[0]][coord[1]][coord[2]].costToGoal;
}

AdaptiveGridCell_t AdaptiveGrid3D_t::getCell(const std::vector<int> &coord) const {
    if(!isInBounds({coord[0],coord[1],coord[2]})) {
        SBPL_ERROR("Coordinates out of bounds %d, %d, %d", coord[0], coord[1], coord[2]);
        throw SBPL_Exception();
    }
    return grid_[coord[0]][coord[1]][coord[2]];
}

int AdaptiveGrid3D_t::getCellPlanningDim(const std::vector<int> &coord) const {
    return grid_[coord[0]][coord[1]][coord[2]].pDimID;
}

void AdaptiveGrid3D_t::setCellPlanningDim(const std::vector<int> &coord, int dimID) {
    if(!isInBounds({coord[0],coord[1],coord[2]})) return;
    grid_[coord[0]][coord[1]][coord[2]].pDimID = dimID;
    grid_[coord[0]][coord[1]][coord[2]].tDimID = dimID;
}

int AdaptiveGrid3D_t::getCellTrackingDim(const std::vector<int> &coord) const{
    if(!isInBounds({coord[0],coord[1],coord[2]})) {
        SBPL_ERROR("Coordinates out of bounds %d, %d, %d", coord[0], coord[1], coord[2]);
        throw SBPL_Exception();
    }
    return grid_[coord[0]][coord[1]][coord[2]].tDimID;
}

void AdaptiveGrid3D_t::setCellTrackingDim(const std::vector<int> &coord, int dimID) {
    if(!isInBounds({coord[0],coord[1],coord[2]})) return;
    grid_[coord[0]][coord[1]][coord[2]].tDimID = dimID;
}

int AdaptiveGrid3D_t::getCellDim(bool bTrackMode, size_t x, size_t y, size_t z) const{
    return bTrackMode?grid_[x][y][z].tDimID:grid_[x][y][z].pDimID;
}

void AdaptiveGrid3D_t::setVisualizationReferenceFrame(std::string frm){
	frame_ = frm;
}

void AdaptiveGrid3D_t::getOverlappingSpheres(size_t x, size_t y, size_t z, int dimID, std::vector<std::vector<int>> &spheres){
    for(std::vector<int> s : spheres_){
        if(s[4] != dimID) continue;
        if(getDist2(s[0],s[1],s[2], x, y, z) <= s[3]*s[3]){
            //xyz is inside sphere
            spheres.push_back(s);
        }
    }
}

void AdaptiveGrid3D_t::addSphere(bool bTrackMode, size_t x, size_t y, size_t z, int rad, int near_rad, int dimID, unsigned int costToGoal, std::vector<sbpl_adaptive::Position3D_t> &modCells){
	int min_x = 0;
	int max_x = grid_sizes_[0] - 1;
	int min_y = 0;
	int max_y = grid_sizes_[1] - 1;
	int min_z = 0;
	int max_z = grid_sizes_[2] - 1;

	if(isInPlanningMode() && getCellDim(bTrackMode, x, y, z) == dimID){
		//HD at this location already
		std::vector<std::vector<int>> covering_spheres_;
		getOverlappingSpheres(x, y, z, dimID, covering_spheres_);
		printf("[adgrid] Location is of same dimension already! -- Growing %lu spheres!\n", covering_spheres_.size());
		//find the max radius and near radius of overlapping spheres
		for(size_t i = 0; i < covering_spheres_.size(); i++){
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
    /*printf("xyz(%d, %d, %d)\n",(int)x,(int)y,(int)z);
    printf("x: %d to %d\n", (int)min_x, (int)max_x);
    printf("y: %d to %d\n", (int)min_y, (int)max_y);
    printf("z: %d to %d\n", (int)min_z, (int)max_z);
    printf("r: %d / nr: %d / dID: %d", rad, near_rad, dimID);*/

	for(int i = min_x; i <= max_x; i++){
		for(int j = min_y; j <= max_y; j++){
			for(int k = min_z; k <= max_z; k++){
				double dist2 = getDist2(x, y, z, i, j, k);

				if(dist2 <= rad * rad){
					//in sphere
					if(setCellDim(bTrackMode, i, j, k, dimID)){
					    sbpl_adaptive::Position3D_t modp;
					    grid2world(i,j,k,modp.x,modp.y,modp.z);
					    modCells.push_back(modp);
					}
					setCellCostToGoal({(int)i,(int)j,(int)k},costToGoal);
				} else if (dist2 <= (rad + near_rad) * (rad + near_rad)){
					//near sphere
					if(setCellNearDim(bTrackMode, i, j, k, dimID)){
					    sbpl_adaptive::Position3D_t modp;
                        grid2world(i,j,k,modp.x,modp.y,modp.z);
                        modCells.push_back(modp);
					}
				}
			}
		}
	}
	if(!trackMode_){
		std::vector<int> sphere(5,0);
		sphere[0] = x;
		sphere[1] = y;
		sphere[2] = z;
		sphere[3] = rad;
		sphere[4] = near_rad;
		spheres_.push_back(sphere);
	}
}

visualization_msgs::MarkerArray AdaptiveGrid3D_t::getVisualizations(std::string ns_prefix,int throttle/*=1*/, double scale/*=-1*/){
	visualization_msgs::MarkerArray marker;
	marker.markers.resize(2);
	marker.markers[0] = this->getAdaptiveGridVisualization(ns_prefix, throttle, scale);
	marker.markers[1] = this->getCostToGoalGridVisualization(ns_prefix, throttle, scale);
	return marker;
}

visualization_msgs::Marker AdaptiveGrid3D_t::getAdaptiveGridVisualization(std::string ns_prefix,int throttle/*=1*/, double scale/*=-1*/){
	visualization_msgs::Marker marker;
	double m_scale = (scale<=0)?oc_grid_->getResolution():scale;
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = frame_;
	marker.ns = ns_prefix + "_AdaptiveGrid3D";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	tf::Quaternion q = tf::Quaternion(tf::Vector3(1, 0, 0), 0);
	q.normalize();
	tf::quaternionTFToMsg(q, marker.pose.orientation);
	marker.scale.x = m_scale;
	marker.scale.y = m_scale;
	marker.scale.z = m_scale;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 1;
	marker.color.a = 1;
	for(int x = 0; x < grid_sizes_[0]; x+=throttle){
		int boundary_x = 0;
		if(x == 0 || x == grid_sizes_[0] - 1) { boundary_x = 1; } else { boundary_x = 0; }
		for(int y = 0; y < grid_sizes_[1]; y+=throttle){
			int boundary_y = 0;
			if(y == 0 || y == grid_sizes_[1] - 1) { boundary_y = 1; } else { boundary_y = 0; }
			for(int z = 0; z < grid_sizes_[2]; z+=throttle){
				int boundary_z = 0;
				if(z == 0 || z == grid_sizes_[2] - 1) { boundary_z = 1; } else { boundary_z = 0; }

				int dimID = getCellDim(trackMode_,x,y,z);
				double hue = 360.0 * dimID / (double)max_dimID_;
				double wx, wy, wz;
				grid2world(x,y,z,wx,wy,wz);
				geometry_msgs::Point p;
				p.x = wx;
				p.y = wy;
				p.z = wz;
				std_msgs::ColorRGBA col = fromHSV(hue, 1.0, 1.0);
				if(dimID != ldID_){
				    marker.points.push_back(p);
				    marker.colors.push_back(col);
				}
				int boundary = boundary_x + boundary_y + boundary_z;
				if(boundary >= 2){
					col.r = 0.5;
					col.g = 0.5;
					col.b = 0.5;
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

visualization_msgs::Marker AdaptiveGrid3D_t::getCostToGoalGridVisualization(std::string ns_prefix, int throttle /*=1*/, double scale/*=-1*/){
	visualization_msgs::Marker marker;
	double m_scale = (scale<=0)?oc_grid_->getResolution():scale;
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = frame_;
	marker.ns = ns_prefix + "_AdaptiveGrid3D_indeces";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	tf::quaternionTFToMsg(tf::Quaternion::getIdentity(), marker.pose.orientation);
	marker.scale.x = m_scale;
	marker.scale.y = m_scale;
	marker.scale.z = m_scale;
	marker.color.r = 1;
	marker.color.g = 1;
	marker.color.b = 1;
	marker.color.a = 1;
	if(max_costToGoal_ > 0){
        for(size_t x = 0; x < (size_t)grid_sizes_[0]; x+=throttle){
            for(size_t y = 0; y < (size_t)grid_sizes_[1]; y+=throttle){
                for(size_t z = 0; z < (size_t)grid_sizes_[2]; z+=throttle){
                    unsigned int idx = getCellCostToGoal({(int)x, (int)y, (int)z});

                    if(idx > max_costToGoal_) continue;
                    double hue = 360.0 * idx / (double)(max_costToGoal_);
                    double wx, wy, wz;
                    grid2world(x,y,z,wx,wy,wz);
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

} //namespace
