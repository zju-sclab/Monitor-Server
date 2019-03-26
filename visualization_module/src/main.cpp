#include <dirent.h>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sstream>

#include <path_msgs/Cross.h>
#include <path_msgs/Lane.h>
#include <path_msgs/choose.h>

// #include <Eigen/Core>
// #include <Eigen/Eigen>
#include <algorithm>
#include <climits>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// #include "utils.hpp"
#include <dynamic_reconfigure/server.h>
#include <map>
#include <monitor_msgs/PoseState.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <smartcar_msgs/Lane.h>
#include <smartcar_msgs/Waypoint.h>
#include <tf/transform_broadcaster.h>

namespace VISUALIZATION_V1 {
class monitor {
private:
    ros::NodeHandle nh;

    ros::Subscriber sub_carx_state;
    std::map<std::string, ros::Publisher> pub_current_pose_car;
    std::map<std::string, ros::Publisher> pub_current_text;

    ros::Publisher pub_global_map;
    tf::TransformBroadcaster tf_broadcaster_;

    void load_map(std::string file);

    void car_pose_cb(const monitor_msgs::PoseStateConstPtr msg);

    void pub_car_state_message(const monitor_msgs::PoseStateConstPtr msg);

    visualization_msgs::Marker marker_initial();

    visualization_msgs::Marker make_car_message();

public:
    monitor()
    {
        sub_carx_state = nh.subscribe("/carx/current_state", 1, &monitor::car_pose_cb, this);
        pub_global_map = nh.advertise<sensor_msgs::PointCloud2>("/map/global_map", 100);
    };
    ~monitor(){};
    void run();
};
}

namespace VISUALIZATION_V1 {

void monitor::run()
{
    ros::NodeHandle pnh("~");
    std::string file;
    pnh.param<std::string>("map_file", file, "");
    load_map(file);

    ros::spin();
}

void monitor::load_map(std::string file)
{
    if (file == "") {
        ROS_WARN_STREAM("No map file found!");
        return;
    }
    sensor_msgs::PointCloud2::Ptr msg_globalmap(new sensor_msgs::PointCloud2());
    if (pcl::io::loadPCDFile(file, *msg_globalmap) == -1) {
        ROS_WARN_STREAM("Cannot load map");
        std::cout << "Please check file: " << file << std::endl;
        return;
    }
    msg_globalmap->header.frame_id = "/map";
    pub_global_map.publish(*msg_globalmap);
    std::cout << "Success load map: " << file << std::endl;
}

void monitor::car_pose_cb(const monitor_msgs::PoseStateConstPtr msg)
{
    visualization_msgs::Marker car_model = marker_initial();
    std::string car_vin = msg->vin;
    if (int(car_vin[12]) == 1) { // 虚拟车辆,定义为蓝色
        car_model.color.r = 0;
        car_model.color.b = 1;
    }
    std::string car_id = car_vin.substr(13, 4);
    if (pub_current_pose_car.find(car_id) == pub_current_pose_car.end()) {
        std::string topic = "/car" + car_id + "/model";
        ros::Publisher pub;
        pub = nh.advertise<visualization_msgs::Marker>(topic, 10);
        pub_current_pose_car.insert(std::pair<std::string, ros::Publisher>(car_id, pub));
    }
    // car_model.header.stamp = msg->pose.header.stamp;
    car_model.header.stamp = ros::Time::now();
    car_model.pose.position = msg->pose.pose.position;
    double current_roll, current_yaw, current_pitch;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(current_roll, current_pitch, current_yaw);
    car_model.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(90 * (M_PI / 180.0), 0 * (M_PI / 180.0), current_yaw + M_PI / 2.0);
    std::map<std::string, ros::Publisher>::iterator it = pub_current_pose_car.find(car_id);
    it->second.publish(car_model);
    pub_car_state_message(msg);

    geometry_msgs::Point p = msg->pose.pose.position;
    tf::Transform transform2(quat, tf::Vector3(p.x, p.y, p.z));
    // tf_broadcaster_.sendTransform(tf::StampedTransform(transform2, msg->pose.header.stamp, "/map", "/car"+car_id+"_link"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "/map", "/car" + car_id + "_link"));
}

void monitor::pub_car_state_message(const monitor_msgs::PoseStateConstPtr msg)
{
    std::string car_id = msg->vin.substr(13, 4);
    if (pub_current_text.find(car_id) == pub_current_text.end()) {
        std::string topic = "/car" + car_id + "/state";
        ros::Publisher pub;
        pub = nh.advertise<visualization_msgs::Marker>(topic, 10);
        pub_current_text.insert(std::pair<std::string, ros::Publisher>(car_id, pub));
    }
    visualization_msgs::Marker msg_text = make_car_message();
    // msg_text.header.stamp = msg->pose.header.stamp;
    msg_text.header.stamp = ros::Time::now();

    geometry_msgs::Point message_pose;
    message_pose.x = msg->pose.pose.position.x + 5.0;
    message_pose.y = msg->pose.pose.position.y - 5.0;
    message_pose.z = msg->pose.pose.position.z + 2.0;
    msg_text.pose.position = message_pose;

    std::stringstream text;
    text << "Vin: " << msg->vin << "\n";
    if (int(msg->vin[12]) == 0) {
        text << "Type: "
             << "Realcar"
             << "\n";
    } else {
        text << "Type: "
             << "Simulation"
             << "\n";
    }
    text << "Shift: "
         << "Undefined"
         << "\n";
    text << "Speed: "
         << "0.0"
         << "\n";
    text << "Steer: "
         << "0.0"
         << "\n";
    msg_text.text = text.str();
    std::map<std::string, ros::Publisher>::iterator it = pub_current_text.find(car_id);
    it->second.publish(msg_text);
}

visualization_msgs::Marker monitor::marker_initial()
{
    visualization_msgs::Marker marker_car;
    marker_car.header.frame_id = "map";
    marker_car.header.stamp = ros::Time::now();
    marker_car.ns = "/car/model";
    marker_car.id = 0;
    marker_car.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_car.action = visualization_msgs::Marker::ADD;
    // set marker_car.pose.position marker.pose.orientation
    marker_car.color.r = 1;
    // marker_car.color.g = 1;
    // marker_car.color.b = 1;
    marker_car.color.a = 1;
    marker_car.scale.x = 1.0;
    marker_car.scale.y = 1.0;
    marker_car.scale.z = 1.0;
    marker_car.mesh_use_embedded_materials = true;
    marker_car.mesh_resource = "package://car_model/ferrari/dae.DAE";
    return marker_car;
}

visualization_msgs::Marker monitor::make_car_message()
{
    visualization_msgs::Marker marker_message;
    marker_message.header.frame_id = "map";
    marker_message.ns = "/car/state";
    marker_message.header.stamp = ros::Time::now();
    marker_message.action = visualization_msgs::Marker::ADD;
    marker_message.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_message.id = 0;
    marker_message.scale.z = 1.0;
    marker_message.color.b = 1;
    marker_message.color.a = 1;
    return marker_message;
}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualization_v1");
    VISUALIZATION_V1::monitor app;
    app.run();
    return 0;
}