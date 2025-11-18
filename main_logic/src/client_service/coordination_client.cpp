#include "main_logic/coordination_client.hpp"
#include <future>

namespace main_logic
{

CoordinationClient::CoordinationClient(rclcpp::Node *node) : node_(node)
{
    // Initialize service clients
    std::string coordination_service = "/trigger_coordination_next_pose";
    coordination_next_pose_client_ = node_->create_client<std_srvs::srv::Trigger>(coordination_service);
    RCLCPP_INFO(node_->get_logger(), "CoordinationClient initialized");
}

void CoordinationClient::trigger_coordination_next_pose(double timeout_sec)
{
    RCLCPP_INFO(node_->get_logger(), "Calling trigger_coordination_next_pose service");
    // Wait for service to be available
    if (!coordination_next_pose_client_->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_sec)))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        last_coordination_message_ = "Service not available";
        return;
    }

    // Create and send request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result_future = coordination_next_pose_client_->async_send_request(request);

    // Wait for service response
    auto status = result_future.wait_for(std::chrono::seconds(static_cast<int>(timeout_sec)));

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed or timed out");
        last_coordination_message_ = "Service call timeout";
        return;
    }

    auto response = result_future.get();
    last_coordination_message_ = response->message;

    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Service returned failure: %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Service call succeeded: %s", response->message.c_str());

    return;
}

std::string CoordinationClient::get_last_coordination_message() const
{
    return last_coordination_message_;
}

} // namespace main_logic