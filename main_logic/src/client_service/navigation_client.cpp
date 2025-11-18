#include "main_logic/navigation_client.hpp"
#include <future>

namespace main_logic
{

NavigationClient::NavigationClient(rclcpp::Node *node, const std::string& robot_namespace) : node_(node), robot_namespace_(robot_namespace)
{
    // Initialize service clients
    std::string random_path_service = "/" + robot_namespace_ + "/generate_random_path";
    std::string specific_path_service = "/" + robot_namespace_ + "/generate_specific_path";
    std::string arc_service = "/" + robot_namespace_ + "/generate_arc";
    std::string rotate_service = "/" + robot_namespace_ + "/rotate_shelfino";
    std::string pause_service = "/" + robot_namespace_ + "/pause_navigation";
    std::string stop_service = "/" + robot_namespace_ + "/stop_navigation";

    generate_random_path_client_ = node_->create_client<std_srvs::srv::Trigger>(random_path_service);
    generate_specific_path_client_ = node_->create_client<navigation_system::srv::NavigateToGoal>(specific_path_service);
    generate_arc_client_ = node_->create_client<navigation_system::srv::NavigateArc>(arc_service);
    rotate_to_center_client_ = node_->create_client<navigation_system::srv::CenterPoint>(rotate_service);
    pause_navigation_client_ = node_->create_client<std_srvs::srv::Trigger>(pause_service);
    stop_navigation_client_ = node_->create_client<std_srvs::srv::Trigger>(stop_service);

    RCLCPP_INFO(node_->get_logger(), "NavigationClient initialized for robot: %s", robot_namespace_.c_str());
}

void NavigationClient::generate_random_path(double timeout_sec)
{
    RCLCPP_INFO(node_->get_logger(), "Calling generate_random_path service for [%s]", robot_namespace_.c_str());

    // Wait for service
    if (!generate_random_path_client_->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_sec)))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        last_random_path_message_ = "Service not available";
        return;
    }

    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call service
    auto result_future = generate_random_path_client_->async_send_request(request);

    // Wait for response
    auto status = result_future.wait_for(std::chrono::seconds(static_cast<int>(timeout_sec)));

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed or timed out");
        last_random_path_message_ = "Service call timeout";
        return;
    }

    // Get response
    auto response = result_future.get();
    last_random_path_message_ = response->message;

    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Service returned failure: %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Service call succeeded: %s", response->message.c_str());
}

void NavigationClient::pause_navigation(double timeout_sec)
{
    RCLCPP_INFO(node_->get_logger(), "Calling pause_navigation service for [%s]", robot_namespace_.c_str());

    // Wait for service
    if (!pause_navigation_client_->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_sec)))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        last_pause_message_ = "Service not available";
        return;
    }

    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call service
    auto result_future = pause_navigation_client_->async_send_request(request);

    // Wait for response
    auto status = result_future.wait_for(std::chrono::seconds(static_cast<int>(timeout_sec)));

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed or timed out");
        last_pause_message_ = "Service call timeout";
        return;
    }

    // Get response
    auto response = result_future.get();
    last_pause_message_ = response->message;

    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Service returned failure: %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Service call succeeded: %s", response->message.c_str());
}

void NavigationClient::stop_navigation(double timeout_sec)
{
    RCLCPP_INFO(node_->get_logger(), "Calling stop_navigation service for [%s]", robot_namespace_.c_str());

    // Wait for service
    if (!stop_navigation_client_->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_sec)))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        last_stop_message_ = "Service not available";
        return;
    }

    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call service
    auto result_future = stop_navigation_client_->async_send_request(request);

    // Wait for response
    auto status = result_future.wait_for(std::chrono::seconds(static_cast<int>(timeout_sec)));

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed or timed out");
        last_stop_message_ = "Service call timeout";
        return;
    }

    // Get response
    auto response = result_future.get();
    last_stop_message_ = response->message;

    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Service returned failure: %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Service call succeeded: %s", response->message.c_str());
}

void NavigationClient::generate_specific_path(const geometry_msgs::msg::Pose& pose, double timeout_sec)
{
    RCLCPP_INFO(node_->get_logger(), "Calling generate_specific_path service for [%s]", robot_namespace_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Target pose: [%.2f, %.2f, %.2f]", 
                pose.position.x, pose.position.y, pose.position.z);

    // Wait for service
    if (!generate_specific_path_client_->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_sec)))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        last_specific_path_message_ = "Service not available";
        return;
    }

    // Create request
    auto request = std::make_shared<navigation_system::srv::NavigateToGoal::Request>();
    request->pose = pose;

    // Call service
    auto result_future = generate_specific_path_client_->async_send_request(request);

    // Wait for response
    auto status = result_future.wait_for(std::chrono::seconds(static_cast<int>(timeout_sec)));

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed or timed out");
        last_specific_path_message_ = "Service call timeout";
        return;
    }

    // Get response
    auto response = result_future.get();
    last_specific_path_message_ = response->message;

    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Service returned failure: %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Service call succeeded: %s", response->message.c_str());
}

void NavigationClient::generate_arc(float radius, const geometry_msgs::msg::Point& center, const geometry_msgs::msg::Point& goal, double timeout_sec)
{
    RCLCPP_INFO(node_->get_logger(), "Calling generate_arc service for [%s]", robot_namespace_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Arc parameters: radius=%.2f, center=[%.2f, %.2f], goal=[%.2f, %.2f]",
                radius, center.x, center.y, goal.x, goal.y);

    // Wait for service
    if (!generate_arc_client_->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_sec)))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        last_arc_message_ = "Service not available";
        return;
    }

    // Create request
    auto request = std::make_shared<navigation_system::srv::NavigateArc::Request>();
    request->radius = radius;
    request->center = center;
    request->goal = goal;

    // Call service
    auto result_future = generate_arc_client_->async_send_request(request);

    // Wait for response
    auto status = result_future.wait_for(std::chrono::seconds(static_cast<int>(timeout_sec)));

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed or timed out");
        last_arc_message_ = "Service call timeout";
        return;
    }

    // Get response
    auto response = result_future.get();
    last_arc_message_ = response->message;

    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Service returned failure: %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Service call succeeded: %s", response->message.c_str());
}

void NavigationClient::rotate_to_center(const geometry_msgs::msg::Point& center, double timeout_sec)
{
    RCLCPP_INFO(node_->get_logger(), "Calling rotate_to_center service for [%s]", robot_namespace_.c_str());
    RCLCPP_INFO(node_->get_logger(), "Center point: [%.2f, %.2f, %.2f]", center.x, center.y, center.z);

    // Wait for service
    if (!rotate_to_center_client_->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_sec)))) {
        RCLCPP_ERROR(node_->get_logger(), "Service not available after waiting");
        last_rotate_message_ = "Service not available";
        return;
    }

    // Create request
    auto request = std::make_shared<navigation_system::srv::CenterPoint::Request>();
    request->center = center;

    // Call service
    auto result_future = rotate_to_center_client_->async_send_request(request);

    // Wait for response
    auto status = result_future.wait_for(std::chrono::seconds(static_cast<int>(timeout_sec)));

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Service call failed or timed out");
        last_rotate_message_ = "Service call timeout";
        return;
    }

    // Get response
    auto response = result_future.get();
    last_rotate_message_ = response->message;

    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "Service returned failure: %s", response->message.c_str());
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Service call succeeded: %s", response->message.c_str());
}


} // namespace main_logic
