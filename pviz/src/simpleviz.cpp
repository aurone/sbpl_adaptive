/* \author Ben Cohen */

#include <pviz/simpleviz.h>

#include <leatherman/utils.h>

SimpleViz::SimpleViz() :
    nh_(),
    ph_("~"),
    marker_ns_id_map_(),
    marker_array_(),
    marker_()
{
    srand(time(NULL));
}

SimpleViz::~SimpleViz()
{
}

void SimpleViz::visualizeObstacles(
    const std::vector<std::vector<double>> &obstacles,
    std::string reference_frame_)
{
    marker_array_.markers.clear();
    marker_array_.markers.resize(obstacles.size());

    ROS_INFO("[SimpleViz] Displaying %d obstaclesin the %s frame", (int)obstacles.size(), reference_frame_.c_str());

    std::string ns = "obstacles" + boost::lexical_cast<std::string>(rand());

    for (size_t i = 0; i < obstacles.size(); i++) {
        if (obstacles[i].size() < 6) {
            ROS_INFO("[SimpleViz] Obstacle description doesn't have length = 6");
            continue;
        }

        // TODO: Change this to use a CUBE_LIST
        marker_array_.markers[i].header.stamp = ros::Time::now();
        marker_array_.markers[i].header.frame_id = reference_frame_;
        marker_array_.markers[i].ns = ns;
        marker_array_.markers[i].id = i;
        marker_array_.markers[i].type = visualization_msgs::Marker::CUBE;
        marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
        marker_array_.markers[i].pose.position.x = obstacles[i][0];
        marker_array_.markers[i].pose.position.y = obstacles[i][1];
        marker_array_.markers[i].pose.position.z = obstacles[i][2];
        marker_array_.markers[i].scale.x = obstacles[i][3];
        marker_array_.markers[i].scale.y = obstacles[i][4];
        marker_array_.markers[i].scale.z = obstacles[i][5];
        marker_array_.markers[i].color.r = 0.0;
        marker_array_.markers[i].color.g = 0.0;
        marker_array_.markers[i].color.b = 0.5;
        marker_array_.markers[i].color.a = 0.9;
        marker_array_.markers[i].lifetime = ros::Duration(180.0);
    }
    update(ns, (int)obstacles.size());
    SV_SHOW_INFO(marker_array_);
}

void SimpleViz::visualizePoses(const std::vector<std::vector<double> > &poses, std::string reference_frame_)
{
    marker_array_.markers.clear();
    marker_array_.markers.resize(poses.size()*3);
    tf::Quaternion pose_quaternion;
    geometry_msgs::Quaternion quaternion_msg;

    int mind = -1;

    ros::Time time = ros::Time::now();

    for (size_t i = 0; i < poses.size(); ++i) {
        pose_quaternion.setRPY(poses[i][3],poses[i][4],poses[i][5]);
        tf::quaternionTFToMsg(pose_quaternion, quaternion_msg);

        mind++;
        marker_array_.markers[mind].header.stamp = time;
        marker_array_.markers[mind].header.frame_id = reference_frame_;
        marker_array_.markers[mind].ns = "pose_arrows";
        marker_array_.markers[mind].type = visualization_msgs::Marker::ARROW;
        marker_array_.markers[mind].id = i;
        marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
        marker_array_.markers[mind].pose.position.x = poses[i][0];
        marker_array_.markers[mind].pose.position.y = poses[i][1];
        marker_array_.markers[mind].pose.position.z = poses[i][2];
        marker_array_.markers[mind].pose.orientation = quaternion_msg;
        marker_array_.markers[mind].scale.x = 0.1;
        marker_array_.markers[mind].scale.y = 0.1;
        marker_array_.markers[mind].scale.z = 0.1;
        marker_array_.markers[mind].color.r = 0.0;
        marker_array_.markers[mind].color.g = 0.7;
        marker_array_.markers[mind].color.b = 0.6;
        marker_array_.markers[mind].color.a = 0.7;
        marker_array_.markers[mind].lifetime = ros::Duration(600.0);

        mind++;
        marker_array_.markers[mind].header.stamp = time;
        marker_array_.markers[mind].header.frame_id = reference_frame_;
        marker_array_.markers[mind].ns = "pose_spheres";
        marker_array_.markers[mind].id = i;
        marker_array_.markers[mind].type = visualization_msgs::Marker::SPHERE;
        marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
        marker_array_.markers[mind].pose.position.x = poses[i][0];
        marker_array_.markers[mind].pose.position.y = poses[i][1];
        marker_array_.markers[mind].pose.position.z = poses[i][2];
        marker_array_.markers[mind].pose.orientation = quaternion_msg;
        marker_array_.markers[mind].scale.x = 0.07;
        marker_array_.markers[mind].scale.y = 0.07;
        marker_array_.markers[mind].scale.z = 0.07;
        marker_array_.markers[mind].color.r = 1.0;
        marker_array_.markers[mind].color.g = 0.0;
        marker_array_.markers[mind].color.b = 0.6;
        marker_array_.markers[mind].color.a = 0.6;
        marker_array_.markers[mind].lifetime = ros::Duration(600.0);

        mind++;
        marker_array_.markers[mind].header.stamp = time;
        marker_array_.markers[mind].header.frame_id = reference_frame_;
        marker_array_.markers[mind].ns = "pose_text_blocks";
        marker_array_.markers[mind].id = i;
        marker_array_.markers[mind].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
        marker_array_.markers[mind].pose.position.x = poses[i][0];
        marker_array_.markers[mind].pose.position.y = poses[i][1];
        marker_array_.markers[mind].pose.position.z = poses[i][2];
        marker_array_.markers[mind].scale.x = 0.03;
        marker_array_.markers[mind].scale.y = 0.03;
        marker_array_.markers[mind].scale.z = 0.03;
        marker_array_.markers[mind].color.r = 1.0;
        marker_array_.markers[mind].color.g = 1.0;
        marker_array_.markers[mind].color.b = 1.0;
        marker_array_.markers[mind].color.a = 0.9;
        marker_array_.markers[mind].text = boost::lexical_cast<std::string>(i+1);
        marker_array_.markers[mind].lifetime = ros::Duration(600.0);
    }
    update("pose_arrows", (int)poses.size());
    update("pose_text_blocks", (int)poses.size());
    update("pose_spheres", (int)poses.size());
    ROS_DEBUG("[SimpleViz] %d markers in the array",(int)marker_array_.markers.size());
    SV_SHOW_INFO(marker_array_);
}

void SimpleViz::clearAllVisualizations(){
    visualization_msgs::Marker marker;

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "";
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = 3;//visualization_msgs::Marker::DELETEALL;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(500.0);
    marker_ns_id_map_.clear();

    showMarker(std::move(marker));
}

void SimpleViz::update(std::string ns, int id){
    if(hasNamespace(ns)){
        marker_ns_id_map_[ns] = std::max(id, getMaxMarkerID(ns));
    } else {
        marker_ns_id_map_[ns] = id;
    }
}

bool SimpleViz::hasNamespace(std::string ns){
    std::map<std::string,int>::const_iterator it = marker_ns_id_map_.find(ns);
    if(it == marker_ns_id_map_.end()) return false;
    return true;
}

int SimpleViz::getMaxMarkerID(std::string ns){
    std::map<std::string,int>::const_iterator it = marker_ns_id_map_.find(ns);
    if(it == marker_ns_id_map_.end()) return 0;
    return it->second;
}

void SimpleViz::clearVisualization(std::string ns){
    visualization_msgs::MarkerArray markers;
    int maxID = getMaxMarkerID(ns);
    //ROS_INFO("SimpleViz::Clearing namespace %s to index %d", ns.c_str(), maxID);
    for(int i = 0; i <= maxID; i++){
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "/map";
        marker.ns = ns;
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::DELETE;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(500.0);
        markers.markers.push_back(marker);
    }
    update(ns, 0);
    SV_SHOW_INFO(markers);
}

void SimpleViz::visualizePose(const std::vector<double> &pose, std::string text, std::string ns, int &id, std::string frame_id)
{
    tf::Quaternion pose_quaternion;
    geometry_msgs::Pose pose_msg;

    pose_msg.position.x = pose[0];
    pose_msg.position.y = pose[1];
    pose_msg.position.z = pose[2];

    pose_quaternion.setRPY(pose[3],pose[4],pose[5]);
    tf::quaternionTFToMsg(pose_quaternion, pose_msg.orientation);

    ROS_DEBUG("[SimpleViz] [%s] position: %0.3f %0.3f %0.3f quaternion: %0.3f %0.3f %0.3f %0.3f (frame: %s)", text.c_str(), pose[0], pose[1], pose[2], pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w, frame_id.c_str());

    visualizePose(pose_msg, text, ns, id, frame_id);
}

void SimpleViz::visualizePose(
    const geometry_msgs::Pose &pose,
    std::string text,
    std::string ns,
    int &id,
    std::string frame_id)
{
    int mind = -1;
    marker_array_.markers.clear();
    marker_array_.markers.resize(3);
    ros::Time time = ros::Time::now();

    ROS_DEBUG("[SimpleViz] [%s] position: %0.3f %0.3f %0.3f quaternion: %0.3f %0.3f %0.3f %0.3f (frame: %s)", text.c_str(), pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, frame_id.c_str());

    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = frame_id;
    marker_array_.markers[mind].ns = ns;
    marker_array_.markers[mind].type = visualization_msgs::Marker::ARROW;
    marker_array_.markers[mind].id = id;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose = pose;
    marker_array_.markers[mind].scale.x = 0.125;
    marker_array_.markers[mind].scale.y = 0.125;
    marker_array_.markers[mind].scale.z = 0.125;
    marker_array_.markers[mind].color.r = 0.0;
    marker_array_.markers[mind].color.g = 0.7;
    marker_array_.markers[mind].color.b = 0.6;
    marker_array_.markers[mind].color.a = 0.7;
    marker_array_.markers[mind].lifetime = ros::Duration(0.0);
    id++;
    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = frame_id;
    marker_array_.markers[mind].ns = ns;
    marker_array_.markers[mind].id = id;
    marker_array_.markers[mind].type = visualization_msgs::Marker::SPHERE;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose = pose;
    marker_array_.markers[mind].scale.x = 0.10;
    marker_array_.markers[mind].scale.y = 0.10;
    marker_array_.markers[mind].scale.z = 0.10;
    marker_array_.markers[mind].color.r = 1.0;
    marker_array_.markers[mind].color.g = 0.0;
    marker_array_.markers[mind].color.b = 0.6;
    marker_array_.markers[mind].color.a = 0.6;
    marker_array_.markers[mind].lifetime = ros::Duration(0.0);
    id++;
    mind++;
    marker_array_.markers[mind].header.stamp = time;
    marker_array_.markers[mind].header.frame_id = frame_id;
    marker_array_.markers[mind].ns = ns;
    marker_array_.markers[mind].id = id;
    marker_array_.markers[mind].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_array_.markers[mind].action = visualization_msgs::Marker::ADD;
    marker_array_.markers[mind].pose = pose;
    marker_array_.markers[mind].pose.position.z += 0.05;
    marker_array_.markers[mind].scale.x = 0.03;
    marker_array_.markers[mind].scale.y = 0.03;
    marker_array_.markers[mind].scale.z = 0.03;
    marker_array_.markers[mind].color.r = 1.0;
    marker_array_.markers[mind].color.g = 1.0;
    marker_array_.markers[mind].color.b = 1.0;
    marker_array_.markers[mind].color.a = 0.9;
    marker_array_.markers[mind].text = text;
    marker_array_.markers[mind].lifetime = ros::Duration(0.0);
    id++;
    update(ns, id);
    SV_SHOW_INFO(marker_array_);
}

void SimpleViz::visualizeSphere(std::vector<double> pos3, int color, std::string ns, double radius, int &id, std::string reference_frame_)
{
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;

    leatherman::HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = ns;
    marker.id = id; id++;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos3[0];
    marker.pose.position.y = pos3[1];
    marker.pose.position.z = pos3[2];
    marker.scale.x = radius*2;
    marker.scale.y = radius*2;
    marker.scale.z = radius*2;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(500.0);

    update(ns, id);
    showMarker(std::move(marker));
}

void SimpleViz::visualizeSphere(std::vector<double> pose, int color, std::string text, double radius, std::string reference_frame_)
{
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;

    leatherman::HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = text + "-sphere";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose[0];
    marker.pose.position.y = pose[1];
    marker.pose.position.z = pose[2];
    marker.scale.x = radius*2;
    marker.scale.y = radius*2;
    marker.scale.z = radius*2;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(500.0);

    update(text + "-sphere", 1);
    showMarker(std::move(marker));
}

void SimpleViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, double radius, std::string reference_frame_)
{
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;

    leatherman::HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = "spheres-" + text;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = radius*2.0;
    marker.scale.y = radius*2.0;
    marker.scale.z = radius*2.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.6;
    marker.lifetime = ros::Duration(500.0);
    marker.id = 1;

    marker.points.resize(pose.size());
    for(size_t i = 0; i < pose.size(); i++)
    {
        marker.points[i].x = pose[i][0];
        marker.points[i].y = pose[i][1];
        marker.points[i].z = pose[i][2];
    }
    update("spheres-" + text, 1);
    showMarker(std::move(marker));
}

void SimpleViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, std::vector<double> &radius, std::string reference_frame_)
{
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;

    leatherman::HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

    for(size_t i = 0; i < pose.size(); ++i)
    {
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = reference_frame_;
        marker.ns = text;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = radius[i]*2.0;
        marker.scale.y = radius[i]*2.0;
        marker.scale.z = radius[i]*2.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.6;
        marker.lifetime = ros::Duration(500.0);
        marker.id = i;

        marker.pose.position.x = pose[i][0];
        marker.pose.position.y = pose[i][1];
        marker.pose.position.z = pose[i][2];

        visualization_msgs::MarkerArray ma;
        showMarker(marker);
    }
    update(text, (int)pose.size());
}

void SimpleViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, std::string reference_frame_)
{
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;
    leatherman::HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

    for(size_t i = 0; i < pose.size(); ++i)
    {
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = reference_frame_;
        marker.ns = text;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = pose[i][3]*2.0;
        marker.scale.y = pose[i][3]*2.0;
        marker.scale.z = pose[i][3]*2.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.6;
        marker.lifetime = ros::Duration(500.0);
        marker.id = i;

        marker.pose.position.x = pose[i][0];
        marker.pose.position.y = pose[i][1];
        marker.pose.position.z = pose[i][2];

        marker_array.markers.push_back(marker);
    }
    update(text, (int)pose.size());
    SV_SHOW_INFO(marker_array);
}

void SimpleViz::visualizeSpheres(const std::vector<std::vector<double> > &pose, const std::vector<int> &hue, std::string text, std::string reference_frame_)
{
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray marker_array;

    if(pose.size() != hue.size())
    {
        ROS_WARN("[SimpleViz] Didn't receive as many colors as I did spheres. Not visualizing. (spheres: %d, colors: %d)", int(pose.size()), int(hue.size()));
        return;
    }

    for(std::size_t i = 0; i < pose.size(); ++i)
    {
        leatherman::HSVtoRGB(&r, &g, &b, hue[i], 1.0, 1.0);
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = reference_frame_;
        marker.ns = text;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = pose[i][3]*2.0;
        marker.scale.y = pose[i][3]*2.0;
        marker.scale.z = pose[i][3]*2.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.6;
        marker.lifetime = ros::Duration(500.0);
        marker.id = i;
        marker.pose.position.x = pose[i][0];
        marker.pose.position.y = pose[i][1];
        marker.pose.position.z = pose[i][2];
        marker_array.markers.push_back(marker);
    }
    update(text, (int)pose.size());
    SV_SHOW_INFO(marker_array);
}

void SimpleViz::visualize3DPath(std::vector<std::vector<double> > &dpath, std::string reference_frame_)
{
    if(dpath.empty())
    {
        ROS_INFO("[visualizeShortestPath] The shortest path is empty.");
        return;
    }
    else
        ROS_INFO("[visualizeShortestPath] There are %i waypoints in the shortest path.",int(dpath.size()));

    visualization_msgs::Marker obs_marker;
    obs_marker.header.frame_id = reference_frame_;
    obs_marker.header.stamp = ros::Time();
    obs_marker.header.seq = 0;
    obs_marker.ns = "path";
    obs_marker.id = 0;
    obs_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    obs_marker.action = 0;
    obs_marker.scale.x = 3*0.02;
    obs_marker.scale.y = 3*0.02;
    obs_marker.scale.z = 3*0.02;
    obs_marker.color.r = 0.45;
    obs_marker.color.g = 0.3;
    obs_marker.color.b = 0.4;
    obs_marker.color.a = 0.8;
    obs_marker.lifetime = ros::Duration(500.0);

    obs_marker.points.resize(dpath.size());

    for (int k = 0; k < int(dpath.size()); k++)
    {
        if(int(dpath[k].size()) < 3)
            continue;

        obs_marker.points[k].x = dpath[k][0];
        obs_marker.points[k].y = dpath[k][1];
        obs_marker.points[k].z = dpath[k][2];
    }
    update("path", 0);
    showMarker(std::move(obs_marker));
}

void SimpleViz::visualizeLine(const std::vector<geometry_msgs::Point> points, std::string ns, int &id, int hue, double thickness, std::string reference_frame_)
{
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;

    leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = ns;
    marker.id = id; id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.points = points;
    marker.scale.x = thickness;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.5;
    marker.lifetime = ros::Duration(500.0);

    //ROS_INFO("Visualizing a line with %d points", int(points.size()));
    update(ns, id);
    showMarker(marker);
}

void SimpleViz::visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, int hue, std::string reference_frame_)
{
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;

    leatherman::HSVtoRGB(&r, &g, &b, hue, 1.0, 1.0);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.pose = pose;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.text = text;
    marker.lifetime = ros::Duration(10.0);
    update(ns, id);
    showMarker(marker);
}

void SimpleViz::visualizeText(geometry_msgs::Pose pose, std::string text, std::string ns, int id, std::vector<double> color, double size, std::string reference_frame_)
{
    visualization_msgs::Marker marker;

    if(color.size() < 4)
        color.resize(4,1);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = reference_frame_;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;
    marker.pose = pose;

    marker.color.r = color[0];
    marker.color.g = color[1];
    marker.color.b = color[2];
    marker.color.a = color[3];
    marker.text = text;
    marker.lifetime = ros::Duration(360.0);
    update(ns, id);
    showMarker(marker);
}

void SimpleViz::visualizeCube(geometry_msgs::PoseStamped pose, int color, std::string ns, int id, std::vector<double> dim)
{
    double r=0,g=0,b=0;
    visualization_msgs::Marker marker;

    if(dim.size() < 3)
    {
        ROS_INFO("[aviz] Three dimensions are needed to visualize a cube.");
        if(dim.size() >= 1)
            dim.resize(3,dim[0]);
        else
            return;
    }

    leatherman::HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = pose.header.frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose.pose;
    marker.scale.x = dim[0];
    marker.scale.y = dim[1];
    marker.scale.z = dim[2];
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0.0);
    update(ns, id);
    showMarker(marker);
}

void SimpleViz::visualizeMesh(
    const std::string &mesh_resource,
    const geometry_msgs::PoseStamped &pose,
    int color,
    const std::string &ns,
    int id,
    const std::string &reference_frame_)
{
    double r = 0.0, g = 0.0, b = 0.0;
    leatherman::HSVtoRGB(&r, &g, &b, color, 1.0, 0.6);

    visualization_msgs::Marker marker;
    marker.header.frame_id = reference_frame_;
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
    marker.color.a = 0.8;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.mesh_resource = mesh_resource;
    update(ns, id);
    showMarker(marker);
}

void SimpleViz::visualizeMeshTriangles(const std::vector<geometry_msgs::Point>& vertices, const std::vector<int>& triangles,
    const geometry_msgs::PoseStamped& pose, int color, std::string ns, int id, bool psychadelic, std::string reference_frame_)
{
    double r = 0.0, g = 0.0, b = 0.0;
    leatherman::HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

    std_msgs::ColorRGBA red; red.a = 1.0f; red.r = 1.0f; red.g = 0.0f; red.b = 0.0f;
    std_msgs::ColorRGBA green; green.a = 1.0f; green.r = 0.0f; green.g = 1.0f; green.b = 0.0f;
    std_msgs::ColorRGBA blue; blue.a = 1.0f; blue.r = 0.0f; blue.g = 0.0f; blue.b = 1.0f;

    std::vector<std_msgs::ColorRGBA> colors;
    for (int i = 0; i < (int)vertices.size(); i++) {
        if (i % 3 == 0) colors.push_back(red);
        if (i % 3 == 1) colors.push_back(green);
        if (i % 3 == 2) colors.push_back(blue);
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = reference_frame_;
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
        marker.colors = colors;
    }
    else {
        marker.color.a = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
    }
    update(ns, id);
    showMarker(marker);
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

void SimpleViz::showMarker(visualization_msgs::Marker&& m)
{
    marker_array_.markers.resize(1);
    marker_array_.markers[0] = m;
    SV_SHOW_INFO(marker_array_);
}

void SimpleViz::showMarker(const visualization_msgs::Marker& m)
{
    marker_array_.markers.resize(1);
    marker_array_.markers[0] = m;
    SV_SHOW_INFO(marker_array_);
}
