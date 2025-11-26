#include "main_logic/navigation_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    float timer_wait = 40.0;
    bool okay= false;
    
    auto navigation_client = std::make_shared<main_logic::NavigationClient>("shelfino1");
    RCLCPP_INFO(navigation_client->get_logger(), "=== Navigation Client Pipeline ===");
    // ------------------------ ------------------------ ------------------------ ------------------------
    RCLCPP_INFO(navigation_client->get_logger(), "\n=== Step 1: Navigate to Specific Pose ===");
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 5.0;
    target_pose.position.y = 16.0;
    target_pose.position.z = 0.0;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;
    okay = navigation_client->generate_specific_path(target_pose, timer_wait, true);

    if (okay) {
        RCLCPP_INFO(navigation_client->get_logger(), 
                    "Specific path generated successfully: %s", 
                    navigation_client->get_last_specific_path_message().c_str());
    } else {
        RCLCPP_ERROR(navigation_client->get_logger(), 
                     "Specific path generation failed: %s",
                     navigation_client->get_last_specific_path_message().c_str());
    }
    rclcpp::sleep_for(std::chrono::seconds(2));


    // ------------------------ ------------------------ ------------------------ ------------------------
    RCLCPP_INFO(navigation_client->get_logger(), "\n=== Step 2: Generate Arc Path ===");
    geometry_msgs::msg::Point center, goal;
    center.x = 0.0;
    center.y = 16.0;
    center.z = 0.0;
    goal.x = 0.0;
    goal.y = 20.0;
    goal.z = 0.0;
    double radius = 5.0;
    okay = navigation_client->generate_arc(radius, center, goal, timer_wait, true);

    if (okay) {
        RCLCPP_INFO(navigation_client->get_logger(), 
                    "Arc path generated successfully: %s", 
                    navigation_client->get_last_arc_message().c_str());
    } else {
        RCLCPP_ERROR(navigation_client->get_logger(), 
                     "Arc path generation failed: %s",
                     navigation_client->get_last_arc_message().c_str());
    }
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // ------------------------ ------------------------ ------------------------ ------------------------
    RCLCPP_INFO(navigation_client->get_logger(), "\n=== Step 3: Generate Random Path ===");
    okay = navigation_client->generate_random_path(timer_wait, true);

    if (okay) {
        RCLCPP_INFO(navigation_client->get_logger(), 
                    "Random path generated successfully: %s", 
                    navigation_client->get_last_random_path_message().c_str());
    } else {
        RCLCPP_ERROR(navigation_client->get_logger(), 
                     "Random path generation failed: %s",
                     navigation_client->get_last_random_path_message().c_str());
    }
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    // ------------------------ ------------------------ ------------------------ ------------------------
    RCLCPP_INFO(navigation_client->get_logger(), "\n=== Step 4: Rotate to Center ===");
    geometry_msgs::msg::Point rotation_center;
    rotation_center.x = 0.0;
    rotation_center.y = 0.0;
    rotation_center.z = 0.0;
    okay = navigation_client->rotate_to_center(rotation_center, timer_wait, true);

    if (okay) {
        RCLCPP_INFO(navigation_client->get_logger(), 
                    "Rotation completed successfully: %s", 
                    navigation_client->get_last_rotate_message().c_str());
    } else {
        RCLCPP_ERROR(navigation_client->get_logger(), 
                     "Rotation failed: %s",
                     navigation_client->get_last_rotate_message().c_str());
    }

    RCLCPP_INFO(navigation_client->get_logger(), "\n===  All pipeline steps completed successfully! ===");
    rclcpp::shutdown();
    return 0;
}
