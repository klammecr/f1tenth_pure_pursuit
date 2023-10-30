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
    double m_max_speed;
    std::vector<double> m_waypoints;
    int m_follow_idx;
    int m_marker_idx;

    // Publishers and Subscribers
    //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_sub_pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_pose;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_pub_drive;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_pub_marker;

    // Strings for the topics of interest
    std::string drive_topic = "/drive";
    std::string odom_topic  = "/ego_racecar/odom";
    std::string scan_topic  = "/scan";
    std::string marker_topic = "visualization_marker";

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
        m_max_speed = 5.0;
        m_L = m_max_speed * 0.50;
        m_follow_idx = -1;
        m_marker_idx = 0;
    }

    std::vector<double> transform_waypoints_to_egoframe()
    {
        // Initialize the transform
        geometry_msgs::msg::PointStamped src_pt;
        geometry_msgs::msg::PointStamped tgt_pt;

        // Get the coordinate transform from ego car base link to map
        tf2_ros::Buffer tf_buffer(get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);

        // Transform
        std::vector<double> out;

        // Wait for the transformation from base_link to map
        geometry_msgs::msg::TransformStamped T;
        try
        {
            if (tf_buffer.canTransform("ego_racecar/base_link", "map", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)))
            {
                std::cout << "GOT IT !";
                T = tf_buffer.lookupTransform("ego_racecar/base_link", "map", tf2::TimePointZero);
            }
            else
            {
                return out;
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could NOT transform");
            return out;
        }

        // Transform the points
        for (int i = 0; i < m_waypoints.size(); i+=2)
        {
            src_pt.point.x = m_waypoints.at(i)   * 0.05;
            src_pt.point.y = m_waypoints.at(i+1) * 0.05;
            tf2::doTransform(src_pt, tgt_pt, T);
            out.push_back(tgt_pt.point.x);
            out.push_back(tgt_pt.point.y);
        }
        return out;
    }

    geometry_msgs::msg::PointStamped transform_pt_to_egoframe()
    {
        // Initialize the transform
        geometry_msgs::msg::PointStamped src_pt;
        geometry_msgs::msg::PointStamped tgt_pt;

        // Get the coordinate transform from ego car base link to map
        tf2_ros::Buffer tf_buffer(get_clock());
        tf2_ros::TransformListener tf_listener(tf_buffer);

        // Wait for the transformation from base_link to map
        geometry_msgs::msg::TransformStamped T;
        try
        {
            if (tf_buffer.canTransform("ego_racecar/base_link", "map", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0)))
            {
                T = tf_buffer.lookupTransform("ego_racecar/base_link", "map", tf2::TimePointZero);
                RCLCPP_INFO(get_logger(), "Got transform");

                // Transform to car frame
                src_pt.point.x = m_waypoints.at(m_follow_idx) * 0.05;
                src_pt.point.y = m_waypoints.at(m_follow_idx+1) * 0.05;
                tf2::doTransform(src_pt, tgt_pt, T);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Could NOT transform");
                src_pt.point.x = std::numeric_limits<double>::infinity();
                return src_pt;
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(get_logger(), "Could NOT transform");
            src_pt.point.x = std::numeric_limits<double>::infinity();
            return src_pt;
        }

        return tgt_pt;
    }

    void find_waypoint_to_track_ego(const std::vector<double>& waypts)
    {
        std::vector<double> dist_from_tgts;
        for (int i = 0; i <= m_waypoints.size()-2; i += 2)
        {
            // Find vehicle distance from waypoints
            double dist_x = waypts.at(i);
            double dist_y = waypts.at(i+1);

            double euclid_dist = pow(pow(dist_x, 2) + pow(dist_y, 2), 0.5);

            // How far away from our target distance we are for each waypoint
            double dist_from_tgt = abs(euclid_dist - m_L);

            if (dist_x < 0)
            {
                dist_from_tgt = 100000.0;
            }

            dist_from_tgts.push_back(dist_from_tgt);
        }

        // Find the closest to the target distance
        auto min_result = std::min_element(dist_from_tgts.begin(), dist_from_tgts.end());
        auto min_idx    = std::distance(dist_from_tgts.begin(), min_result);

        // Set which waypoint to follow
        m_follow_idx = 2*min_idx;

        std::cout << "Following waypoint: " << min_idx << std::endl;

        //Create the marker visualization message
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.id = m_marker_idx;
        m_marker_idx += 1;
        auto clock = rclcpp::Clock();
        marker.header.stamp = clock.now();
        marker.header.frame_id = "ego_racecar/base_link";
        marker.pose.position.x = waypts[m_follow_idx];
        marker.pose.position.y = waypts[m_follow_idx + 1];
        marker.pose.position.z = 0.25;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        //RCLCPP_INFO(get_logger(), "%d, %d", dist_from_tgts[2 * min_idx], dist_from_tgts[2 * min_idx+1]);
        m_pub_marker->publish(marker);

    }


    double calc_steering_angle(const geometry_msgs::msg::PointStamped& waypt)
    {
        double gamma = 2 * waypt.point.y / std::pow(m_L, 2);
        return gamma;
    }

    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        // DEBUG:
        RCLCPP_INFO(get_logger(), "Receiving Pose Callback");

        // Transform waypoint to ego frame
        std::vector<double> waypts_ego =  transform_waypoints_to_egoframe();

        // If we have valid points then do the processing
        if (waypts_ego.size() > 0)
        {
            // Find the current waypoint to track using methods mentioned in lecture
            find_waypoint_to_track_ego(waypts_ego);

            geometry_msgs::msg::PointStamped vehicle_pt;
            vehicle_pt.point.x = waypts_ego[m_follow_idx];
            vehicle_pt.point.y = waypts_ego[m_follow_idx+1];

            if (vehicle_pt.point.x != std::numeric_limits<double>::infinity())
            {
                // Calculate curvature/steering angle
                double steering_angle = calc_steering_angle(vehicle_pt);

                // Publish drive message, don't forget to limit the steering angle.
                ackermann_msgs::msg::AckermannDriveStamped drive_msg;

                // Set speed
                double speed = 0.0;
                double steer_angle_deg = std::abs(steering_angle) * 180/M_PI;
                std::cout << steer_angle_deg << std::endl;
                if (m_follow_idx+1 < m_waypoints.size()-1)
                {
                    if (steer_angle_deg >= 0 && steer_angle_deg < 10)
                    {
                        speed = m_max_speed;
                        m_L = m_max_speed * 0.50;
                    }
                    else if (steer_angle_deg >= 10 && steer_angle_deg < 20)
                    {
                        speed = m_max_speed/2;
                        m_L = m_max_speed * 0.25;
                    }
                    else
                    {
                        speed = m_max_speed/4;
                        m_L = m_max_speed * 0.25;
                    }
                }

                // Set steering angle and speed
                drive_msg.drive.steering_angle = steering_angle;
                drive_msg.drive.speed = speed;

                // Publish drive message
                m_pub_drive->publish(drive_msg);
            }
        }
    }

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}