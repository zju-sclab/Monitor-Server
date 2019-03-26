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

#include <dynamic_reconfigure/server.h>
#include <visualization_module/VisualizationConfig.h>

// namespace VISUALIZATION_DYNAMIC {

ros::Subscriber sub_carx_state;
std::map<std::string, ros::Publisher> pub_current_state_array;
ros::Publisher pub_global_map;
// tf::TransformBroadcaster tf_broadcaster_;  // 这玩意居然"也"是个 nodehandle ，如果在全局定义的话会报错：You must call ros::init() before creating the first NodeHandle
tf::StampedTransform tf_base2laser_;

double offset_x, offset_y, offset_z;

void load_map(ros::NodeHandle nh, std::string file);

void car_pose_cb(const monitor_msgs::PoseStateConstPtr msg);

void pub_car_state_message(const monitor_msgs::PoseStateConstPtr msg);

visualization_msgs::Marker make_car_model(const monitor_msgs::PoseStateConstPtr msg);

visualization_msgs::Marker make_car_message(const monitor_msgs::PoseStateConstPtr msg, double offset_x, double offset_y, double offset_z, std::string car_id);

visualization_msgs::Marker make_line(geometry_msgs::Point start_point, double offset_x, double offset_y, double offset_z, std::string car_id);

visualization_msgs::Marker car_model_initial();

visualization_msgs::Marker message_initial();

void load_map(ros::NodeHandle nh, std::string file)
{
    if (file == "") {
        ROS_WARN_STREAM("No map file found!");
        return;
    }
    pub_global_map = nh.advertise<sensor_msgs::PointCloud2>("/map/global_map", 100);
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

void car_pose_cb(const monitor_msgs::PoseStateConstPtr msg)
{
    visualization_msgs::MarkerArray marker_array;
    std::string car_vin = msg->vin;
    std::string car_id = car_vin.substr(13, 4);
    if (pub_current_state_array.find(car_id) == pub_current_state_array.end()) {
        ros::NodeHandle nh;
        std::string topic = "/car" + car_id;
        ros::Publisher pub;
        pub = nh.advertise<visualization_msgs::MarkerArray>(topic, 10);
        pub_current_state_array.insert(std::pair<std::string, ros::Publisher>(car_id, pub));
    }
    // car_model.header.stamp = msg->pose.header.stamp;
    visualization_msgs::Marker car_model = make_car_model(msg);
    visualization_msgs::Marker car_message = make_car_message(msg, offset_x, offset_y, offset_z, car_id);
    visualization_msgs::Marker car_infoline = make_line(msg->pose.pose.position, offset_x, offset_y, offset_z, car_id);

    marker_array.markers.push_back(car_model);
    marker_array.markers.push_back(car_message);
    marker_array.markers.push_back(car_infoline);

    std::map<std::string, ros::Publisher>::iterator it = pub_current_state_array.find(car_id);
    it->second.publish(marker_array);
}

visualization_msgs::Marker make_car_model(const monitor_msgs::PoseStateConstPtr msg)
{
    visualization_msgs::Marker car_model = car_model_initial();
    car_model.header.stamp = ros::Time::now();
    car_model.pose.position = msg->pose.pose.position;
    std::string car_id = msg->vin.substr(13, 4);
    car_model.ns = "/car" + car_id + "/model";
    // TODO: 区分虚拟车和实际车 --或使用不同的颜色,或使用不同的车型

    double current_roll, current_yaw, current_pitch;
    tf::TransformBroadcaster tf_broadcaster_;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(current_roll, current_pitch, current_yaw);
    car_model.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(90 * (M_PI / 180.0), 0 * (M_PI / 180.0), current_yaw + M_PI / 2.0);
    geometry_msgs::Point p = msg->pose.pose.position;
    tf::Transform transform2(quat, tf::Vector3(p.x, p.y, p.z));
    // tf_broadcaster_.sendTransform(tf::StampedTransform(transform2, msg->pose.header.stamp, "/map", "/car"+car_id+"_link"));
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "/map", "/car" + car_id + "_link"));

    return car_model;
}

visualization_msgs::Marker make_car_message(const monitor_msgs::PoseStateConstPtr msg, double offset_x, double offset_y, double offset_z, std::string car_id)
{
    visualization_msgs::Marker car_message = message_initial();
    if (int(msg->vin[12]) == 1) {
        car_message.color.a = 1;
        car_message.color.b = 0;
        car_message.color.g = 1;
    }
    car_message.header.stamp = ros::Time::now();
    car_message.ns = "/car" + car_id + "/state";

    geometry_msgs::Point message_pose;
    message_pose.x = msg->pose.pose.position.x + offset_x;
    message_pose.y = msg->pose.pose.position.y + offset_y;
    message_pose.z = msg->pose.pose.position.z + offset_z;
    car_message.pose.position = message_pose;

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
    car_message.text = text.str();
    return car_message;
}

visualization_msgs::Marker car_model_initial()
{
    visualization_msgs::Marker marker_car;
    marker_car.header.frame_id = "map";
    marker_car.header.stamp = ros::Time::now();
    // marker_car.ns = "/car/model";
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

visualization_msgs::Marker message_initial()
{
    visualization_msgs::Marker marker_message;
    marker_message.header.frame_id = "map";
    // marker_message.ns = "/car/state";
    marker_message.header.stamp = ros::Time::now();
    marker_message.action = visualization_msgs::Marker::ADD;
    marker_message.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_message.id = 0;
    marker_message.scale.z = 1.0;
    marker_message.color.r = 0.91;
    marker_message.color.g = 0.585;
    marker_message.color.b = 0.476;
    marker_message.color.a = 1;
    return marker_message;
}

visualization_msgs::Marker make_line(geometry_msgs::Point start_point, double offset_x, double offset_y, double offset_z, std::string car_id)
{
    visualization_msgs::Marker line;
    line.header.frame_id = "map";
    line.ns = "/car" + car_id + "/infoline";
    line.header.stamp = ros::Time::now();
    line.action = visualization_msgs::Marker::ADD;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.id = 0;
    line.scale.x = 0.05;
    line.color.r = 1;
    line.color.g = 1;
    line.color.a = 1;
    line.points.push_back(start_point);
    geometry_msgs::Point end_point;
    end_point.x = start_point.x + offset_x;
    end_point.y = start_point.y + offset_y;
    end_point.z = start_point.z + offset_z;
    line.points.push_back(end_point);
    return line;
}

void config_callback(visualization_module::VisualizationConfig& config, uint32_t level)
{
    offset_x = config.offset_x;
    offset_y = config.offset_y;
    offset_z = config.offset_z;
}

// void run()
// {
//     param_initial();

//     sub_carx_state = nh.subscribe("/carx/current_state", 1, &car_pose_cb);
//     pub_global_map = nh.advertise<sensor_msgs::PointCloud2>("/map/global_map", 100);

//     dynamic_reconfigure::Server<visualization_module::VisualizationConfig> server;
//     dynamic_reconfigure::Server<visualization_module::VisualizationConfig>::CallbackType cf;
//     cf = boost::bind(&config_callback, _1, _2);
//     server.setCallback(cf);

//     ros::spin();
// }
// }

// using namespace VISUALIZATION_DYNAMIC;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "visualization_module");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<double>("offset_x", offset_x, 5.0);
    pnh.param<double>("offset_y", offset_y, -5.0);
    pnh.param<double>("offset_z", offset_z, 5.0);

    std::string file;
    pnh.param<std::string>("map_file", file, "");
    load_map(nh, file);

    sub_carx_state = nh.subscribe("/carx/current_state", 1, car_pose_cb);

    dynamic_reconfigure::Server<visualization_module::VisualizationConfig> server;
    dynamic_reconfigure::Server<visualization_module::VisualizationConfig>::CallbackType cf;
    cf = boost::bind(&config_callback, _1, _2);
    server.setCallback(cf);

    ros::spin();
    return 0;
}