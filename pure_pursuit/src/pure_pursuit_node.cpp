#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

private:
    float m_L;
    std::vector<double> m_waypoints;


    // Publishers and Subscribers
    //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_sub_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_pose;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_pub_drive;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_pub_marker;

    // Strings for the topics of interest
    std::string drive_topic = "/drive";
    std::string odom_topic  = "/ego_racecar/odom";
    std::string scan_topic  = "/scan";
    std::string marker_topic = "/visualization_marker";

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // Create ROS subscribers and publishers

        // For geometry message pose
        // m_sub_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     odom_topic, 10, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1)
        // );

        // For nav message pose
        m_sub_pose = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, std::bind(&PurePursuit::pose_callback, this, std::placeholders::_1)
        );

        m_pub_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic, 10);

        m_pub_marker = this->create_publisher<visualization_msgs::msg::Marker>(
        marker_topic, 10);

        // Get the waypoints
        this->declare_parameter<std::vector<double>>("waypoints", {});
        rclcpp::Parameter waypoints_param("waypoints", std::vector<double>({}));
        this->get_parameter("waypoints", waypoints_param);
        RCLCPP_INFO(get_logger(), "double[]: %s", waypoints_param.value_to_string().c_str());
        m_waypoints = waypoints_param.as_double_array();

        // Other parameters
        m_L = 5.0;
    }

    void find_waypoint_to_track(const geometry_msgs::msg::Pose & pose)
    {
        // Get the coordinate transform from base link to map
        tf2_ros::Buffer tf_buffer(get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);

        // Wait for the transformation from base_link to map
        try {
            //tf_buffer.canTransform("map", "ego_racecar/base_link", tf2::TimePointZero);
            tf_buffer.canTransform("ego_racecar/base_link", "map", tf2::TimePointZero);
        } catch (tf2::ExtrapolationException &e) {
            RCLCPP_ERROR(get_logger(), "Transform from base_link to map is not available: %s", e.what());
            return;
        }

        // Waypoints is a 1D array of [x1,y1, x2, y2, ...]
        auto rob_pos = pose.position;
        RCLCPP_INFO(get_logger(), "%s, %s", std::to_string(rob_pos.x).c_str(), std::to_string(rob_pos.y).c_str());
        std::vector<double> dist_from_tgts;


        std::cout << "oh yeah";
        for (int i = 0; i < m_waypoints.size(); i += 2)
        {
            geometry_msgs::msg::PointStamped src_pt;
            src_pt.point.x = m_waypoints.at(2*i);
            src_pt.point.y = m_waypoints.at(2*i+1);
            geometry_msgs::msg::PointStamped tgt_pt;
            //tf2_geometry_msgs::do_transform_pose(src_pt, tgt_pt, transform);
            tf_buffer.transform(src_pt, tgt_pt, "ego_racecar/base_link");

            RCLCPP_INFO(get_logger(), "%s, %s", std::to_string(tgt_pt.point.x).c_str(), std::to_string(tgt_pt.point.y).c_str());
            

            // Find vehicle distance from waypoints
            double dist_x = rob_pos.x - tgt_pt.point.x;
            double dist_y = rob_pos.y - tgt_pt.point.y;

            double euclid_dist = pow(pow(dist_x, 2) + pow(dist_y, 2), 0.5);

            // How far away from our target distance we are for each waypoint
            double dist_from_tgt = abs(euclid_dist - m_L);
            dist_from_tgts.push_back(dist_from_tgt);
        }

        // Find the closest to the target distance
        auto min_result = std::min_element(dist_from_tgts.begin(), dist_from_tgts.end());
        auto min_idx    = std::distance(dist_from_tgts.begin(), min_result);

        // Create the marker visualization message
        // visualization_msgs::msg::Marker marker;
        // marker.type = visualization_msgs::msg::Marker::SPHERE;
        // marker.header.frame_id = "map";
        // marker.pose.position.x = m_waypoints[2 * min_idx];
        // marker.pose.position.y = m_waypoints[2 * min_idx + 1];
        // marker.scale.x = 1;
        // marker.scale.y = 0.1;
        // marker.scale.z = 0.1;
        // marker.color.a = 1.0; // Don't forget to set the alpha!
        // marker.color.r = 0.0;
        // marker.color.g = 1.0;
        // marker.color.b = 0.0;
        // //RCLCPP_INFO(get_logger(), "%d, %d", dist_from_tgts[2 * min_idx], dist_from_tgts[2 * min_idx+1]);
        // m_pub_marker->publish(marker);

    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        // DEBUG:
        RCLCPP_INFO(get_logger(), "Receiving Pose Callback");

        // Find the current waypoint to track using methods mentioned in lecture
        find_waypoint_to_track(pose_msg->pose.pose);

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle.
        // ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        // drive_msg.drive.speed = 1;
        // drive_msg.drive.steering_angle = 0.5; 
        // m_pub_drive->publish(drive_msg);

    }

    // void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg)
    // {
    //     RCLCPP_INFO(get_logger(), "Receiving Pose Callback");
    //     // Find the current waypoint to track using methods mentioned in lecture
    //     std::vector<double> waypoints = m_waypoint_param.as_double_array();
    //     find_waypoint_to_track(waypoints, pose_msg->pose);

    //     // TODO: transform goal point to vehicle frame of reference

    //     // TODO: calculate curvature/steering angle

    //     // TODO: publish drive message, don't forget to limit the steering angle.
    //     ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    //     drive_msg.drive.speed = 1;
    //     drive_msg.drive.steering_angle = 0.5; 
    //     m_pub_drive->publish(drive_msg);

    // }

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}