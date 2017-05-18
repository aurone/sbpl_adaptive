#include <sbpl_adaptive/expansion_grid_3d.h>

#include <Eigen/Dense>

namespace adim {

ExpansionGrid3D::ExpansionGrid3D(adim::SBPLCollisionSpace* cspace) {
    if(cspace == NULL){
        SBPL_ERROR("Can't initialize ExpansionGrid3D with NULL collision space!");
        throw SBPL_Exception();
    }

    cspace_.reset(cspace);
    cspace_->getSize(size_.x, size_.y, size_.z);

    expands_grid_.resize(size_.x);
    for(int i = 0; i < expands_grid_.size(); i++){
        expands_grid_[i].resize(size_.y);
        for(int j = 0; j < expands_grid_[i].size(); j++){
            expands_grid_[i][j].resize(size_.z, UINT_MAX);
        }
    }
}

ExpansionGrid3D::~ExpansionGrid3D() {
    expands_grid_.clear();
}

bool ExpansionGrid3D::inGrid(int x, int y, int z){
    if(x < 0 || x >= size_.x) return false;
    if(y < 0 || y >= size_.y) return false;
    if(z < 0 || z >= size_.z) return false;
    return true;
}

void ExpansionGrid3D::reset(){
    for(int i = 0; i < expands_grid_.size(); i++){
        for(int j = 0; j < expands_grid_[i].size(); j++){
            for(int k = 0; k < expands_grid_[i][j].size(); k++){
                expands_grid_[i][j][k] = UINT_MAX;
            }
        }
    }
};

void ExpansionGrid3D::setExpansionStep(const adim::ModelCoords &model_coords, unsigned int exp_step){
    std::vector<Eigen::Vector3i> voxels;
    if(!cspace_->getModelVoxelsInGrid(model_coords, voxels)) return;
    for(Eigen::Vector3i voxel : voxels){
        if(!inGrid(voxel.x(), voxel.y(), voxel.z())) continue;
        expands_grid_[voxel.x()][voxel.y()][voxel.z()] =
                std::min(expands_grid_[voxel.x()][voxel.y()][voxel.z()], exp_step); //store the earliest step
    }
}

unsigned int ExpansionGrid3D::getEarliestExpansionStep(
    const std::vector<Cell3D> &voxels)
{
    unsigned int min_ = UINT_MAX;
    for(Cell3D voxel : voxels){
        if(!inGrid(voxel.x, voxel.y, voxel.z)) continue;
        min_ = std::min(expands_grid_[voxel.x][voxel.y][voxel.z], min_);
    }
    return min_;
}

visualization_msgs::MarkerArray ExpansionGrid3D::getVoxelVisualization(const adim::ModelCoords &model_coords, std::string ns, Color color){
    visualization_msgs::MarkerArray markers;

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = cspace_->getReferenceFrame();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    tf::quaternionTFToMsg(tf::Quaternion::getIdentity(), marker.pose.orientation);
    marker.scale.x = cspace_->getResolution();
    marker.scale.y = cspace_->getResolution();
    marker.scale.z = cspace_->getResolution();
    marker.color = toColorMSG(color);

    std::vector<Eigen::Vector3i> voxels;

    cspace_->getModelVoxelsInGrid(model_coords, voxels);

    if(voxels.size() == 0) return markers;

    for(Eigen::Vector3i voxel : voxels){
        geometry_msgs::Point p;
        p.x = voxel.x() * cspace_->getResolution();
        p.y = voxel.y() * cspace_->getResolution();
        p.z = voxel.z() * cspace_->getResolution();
        marker.points.push_back(p);
    }

    markers.markers.push_back(marker);

    return markers;
}

} // namespace adim
