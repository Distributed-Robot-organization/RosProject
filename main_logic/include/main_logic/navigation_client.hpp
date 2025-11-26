#ifndef NAVIGATION_CLIENT_HPP
#define NAVIGATION_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "navigation_system/srv/navigate_to_goal.hpp"
#include "navigation_system/srv/navigate_arc.hpp"
#include "navigation_system/srv/center_point.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace main_logic
{

class NavigationClient
{
public:
    NavigationClient(rclcpp::Node *node, const std::string& robot_namespace = "shelfino1");

    // service
    void generate_random_path(double timeout_sec = 10.0);
    void pause_navigation(double timeout_sec = 10.0);
    void stop_navigation(double timeout_sec = 10.0);
    void generate_specific_path(const geometry_msgs::msg::Pose& pose, double timeout_sec = 10.0);

    void generate_arc(
        float radius,
        const geometry_msgs::msg::Point& center,
        const geometry_msgs::msg::Point& goal,
        double timeout_sec = 10.0);

    void rotate_to_center(
        const geometry_msgs::msg::Point& center,
        double timeout_sec = 10.0);

    // Getters for last response messages
    std::string get_last_random_path_message() const { return last_random_path_message_; }
    std::string get_last_specific_path_message() const { return last_specific_path_message_; }
    std::string get_last_arc_message() const { return last_arc_message_; }
    std::string get_last_rotate_message() const { return last_rotate_message_; }
    std::string get_last_pause_message() const { return last_pause_message_; }
    std::string get_last_stop_message() const { return last_stop_message_; }

private:
    // Robot namespace
    std::string robot_namespace_;

    // Service clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr generate_random_path_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pause_navigation_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_navigation_client_;
    rclcpp::Client<navigation_system::srv::NavigateToGoal>::SharedPtr generate_specific_path_client_;
    rclcpp::Client<navigation_system::srv::NavigateArc>::SharedPtr generate_arc_client_;
    rclcpp::Client<navigation_system::srv::CenterPoint>::SharedPtr rotate_to_center_client_;

    // Last response messages
    std::string last_random_path_message_;
    std::string last_specific_path_message_;
    std::string last_arc_message_;
    std::string last_rotate_message_;
    std::string last_pause_message_;
    std::string last_stop_message_;

    // Node reference
    rclcpp::Node *node_;
};

} // namespace main_logic

#endif // NAVIGATION_CLIENT_HPP
