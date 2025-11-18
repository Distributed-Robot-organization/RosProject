#include "main_logic/vision_client.hpp"
#include <future>

namespace main_logic
{

VisionClient::VisionClient(rclcpp::Node *node, const std::string& robot_namespace) : node_(node), robot_namespace_(robot_namespace)
{
    // Initialize service clients
    std::string detection_service    = "/" + robot_namespace_ + "/trigger_detection";
    std::string pcl_service          = "/" + robot_namespace_ + "/trigger_pcl";
    std::string filter_pcl_service   = "/" + robot_namespace_ + "/trigger_filter_pcl";

    detection_client_  = node_->create_client<vision_system::srv::NameObject>(detection_service);
    pcl_client_        = node_->create_client<std_srvs::srv::Trigger>(pcl_service);
    filter_pcl_client_ = node_->create_client<std_srvs::srv::Trigger>(filter_pcl_service);

    RCLCPP_INFO(node_->get_logger(), "VisionClient initialized for robot: %s", robot_namespace_.c_str());
}


void VisionClient::trigger_detection(const std::string& object_name, double timeout_sec)
{
    RCLCPP_INFO(node_->get_logger(), "Calling trigger_detection service for object: %s", object_name.c_str());

    // Wait for service
    if (!detection_client_->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_sec)))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        last_detection_message_ = "Service not available";
        return;
    }

    // Create request
    auto request = std::make_shared<vision_system::srv::NameObject::Request>();
    request->name_object = object_name;

    // Call service
    auto result_future = detection_client_->async_send_request(request);

    // Wait for response
    auto status = result_future.wait_for(std::chrono::seconds(static_cast<int>(timeout_sec)));

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed or timed out");
        last_detection_message_ = "Service call timeout";
        return;
    }

    // Get response
    auto response = result_future.get();
    last_detection_message_ = response->message;

    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Service returned failure: %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Service call succeeded: %s", response->message.c_str());
}

void VisionClient::trigger_pcl(double timeout_sec)
{
    RCLCPP_INFO(node_->get_logger(), "Calling trigger_pcl service");

    // Wait for service
    if (!pcl_client_->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_sec)))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        last_pcl_message_ = "Service not available";
        return;
    }

    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call service
    auto result_future = pcl_client_->async_send_request(request);

    // Wait for response
    auto status = result_future.wait_for(std::chrono::seconds(static_cast<int>(timeout_sec)));

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed or timed out");
        last_pcl_message_ = "Service call timeout";
        return;
    }

    // Get response
    auto response = result_future.get();
    last_pcl_message_ = response->message;

    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Service returned failure: %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Service call succeeded: %s", response->message.c_str());
}

void VisionClient::trigger_filter_pcl(double timeout_sec)
{
    RCLCPP_INFO(node_->get_logger(), "Calling trigger_filter_pcl service");

    // Wait for service
    if (!filter_pcl_client_->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_sec)))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        last_filter_pcl_message_ = "Service not available";
        return;
    }

    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call service
    auto result_future = filter_pcl_client_->async_send_request(request);

    // Wait for response
    auto status = result_future.wait_for(std::chrono::seconds(static_cast<int>(timeout_sec)));
    
    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed or timed out");
        last_filter_pcl_message_ = "Service call timeout";
        return;
    }

    auto response = result_future.get();
    last_filter_pcl_message_ = response->message;

    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Service returned failure: %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Service call succeeded: %s", response->message.c_str());
}

} // namespace main_logic
