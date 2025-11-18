#ifndef VISION_CLIENT_HPP
#define VISION_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "vision_system/srv/name_object.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <atomic>
#include <condition_variable>
#include <mutex>

namespace main_logic
{

class VisionClient
{
public:
    VisionClient(rclcpp::Node *node, const std::string& robot_namespace = "shelfino1");
    

    void trigger_detection(const std::string& object_name, double timeout_sec = 5.0);
    void trigger_pcl(double timeout_sec = 5.0);
    void trigger_filter_pcl(double timeout_sec = 5.0);

    std::string get_last_detection_message() const { return last_detection_message_; }
    std::string get_last_pcl_message() const { return last_pcl_message_; }
    std::string get_last_filter_pcl_message() const { return last_filter_pcl_message_; }

private:
    // Service clients
    rclcpp::Client<vision_system::srv::NameObject>::SharedPtr detection_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pcl_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr filter_pcl_client_;

    // Robot namespace
    std::string robot_namespace_;

    // Last response messages
    std::string last_detection_message_;
    std::string last_pcl_message_;
    std::string last_filter_pcl_message_;

    // Node reference
    rclcpp::Node *node_;
};

} // namespace main_logic

#endif // VISION_CLIENT_HPP
