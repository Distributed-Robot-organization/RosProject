#include "main_logic/navigation_client.hpp"
#include <thread>

namespace main_logic
{

NavigationClient::NavigationClient(const std::string& robot_namespace)
    : Node("navigation_client_node"), robot_namespace_(robot_namespace), tick_received_(false)
{
    // Initialize service clients
    std::string random_path_service = "/" + robot_namespace_ + "/generate_random_path";
    std::string specific_path_service = "/" + robot_namespace_ + "/generate_specific_path";
    std::string arc_service = "/" + robot_namespace_ + "/generate_arc";
    std::string rotate_service = "/" + robot_namespace_ + "/rotate_shelfino";
    std::string pause_service = "/" + robot_namespace_ + "/pause_navigation";
    std::string stop_service = "/" + robot_namespace_ + "/stop_navigation";

    generate_random_path_client_ = this->create_client<std_srvs::srv::Trigger>(random_path_service);
    generate_specific_path_client_ = this->create_client<navigation_system::srv::NavigateToGoal>(specific_path_service);
    generate_arc_client_ = this->create_client<navigation_system::srv::NavigateArc>(arc_service);
    rotate_to_center_client_ = this->create_client<navigation_system::srv::CenterPoint>(rotate_service);
    pause_navigation_client_ = this->create_client<std_srvs::srv::Trigger>(pause_service);
    stop_navigation_client_ = this->create_client<std_srvs::srv::Trigger>(stop_service);

    // Initialize tick subscriber
    std::string tick_topic = "/" + robot_namespace_ + "/navigation_system/tick_service_navigation";
    tick_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        tick_topic, 10,
        std::bind(&NavigationClient::tick_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "NavigationClient initialized for robot: %s", robot_namespace_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to tick topic: %s", tick_topic.c_str());
}

template<typename ServiceT>
bool NavigationClient::wait_for_service(typename rclcpp::Client<ServiceT>::SharedPtr client,const std::string& service_name,double timeout_sec)
{
    auto timeout = std::chrono::duration<double>(timeout_sec);
    if (!client->wait_for_service(timeout)) {
        RCLCPP_ERROR(this->get_logger(), "Service %s not available after waiting", service_name.c_str());
        return false;
    }
    return true;
}

void NavigationClient::tick_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Tick received from navigation system [%s]", robot_namespace_.c_str());
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = true;
        tick_cv_.notify_all();
    }
}

bool NavigationClient::wait_for_tick(double timeout_sec)
{
    RCLCPP_INFO(this->get_logger(), "Waiting for tick signal (timeout: %.1fs)...", timeout_sec);
    
    // Reset tick flag before waiting
    {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = timeout_sec > 0 ? std::chrono::duration<double>(timeout_sec) : std::chrono::hours(24);
    
    while (rclcpp::ok()) {
        // Check if tick received
        {
            std::lock_guard<std::mutex> lock(tick_mutex_);
            if (tick_received_) {
                return true;
            }
        }
        
        // Process callbacks
        rclcpp::spin_some(this->get_node_base_interface());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Check timeout
        if (timeout_sec > 0) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed >= timeout_duration) {
                RCLCPP_WARN(this->get_logger(), "Timeout waiting for tick");
                return false;
            }
        }
    }
    
    RCLCPP_WARN(this->get_logger(), "ROS shutdown while waiting for tick");
    return false;
}

template<typename ServiceT, typename RequestT>
bool NavigationClient::call_service_async(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    std::shared_ptr<RequestT> request,
    typename ServiceT::Response::SharedPtr& response,
    double timeout_sec)
{
    auto future = client->async_send_request(request);
    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::duration<double>(timeout_sec);
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(this->get_node_base_interface());
        
        if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            response = future.get();
            return true;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed >= timeout_duration) {
            RCLCPP_ERROR(this->get_logger(), "Service call timed out");
            return false;
        }
    }
    
    return false;
}

bool NavigationClient::generate_random_path(double timeout_sec, bool should_wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling generate_random_path service for [%s]", robot_namespace_.c_str());

    if (should_wait_for_tick) {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }

    if (!wait_for_service<std_srvs::srv::Trigger>(
            generate_random_path_client_, "generate_random_path", timeout_sec)) {
        last_random_path_message_ = "Service not available";
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    std_srvs::srv::Trigger::Response::SharedPtr response;

    if (!call_service_async<std_srvs::srv::Trigger>(
            generate_random_path_client_, request, response, timeout_sec)) {
        last_random_path_message_ = "Service call timed out";
        return false;
    }

    last_random_path_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Random path service successful: %s", response->message.c_str());
        if (should_wait_for_tick && !wait_for_tick(timeout_sec)) {
            RCLCPP_WARN(this->get_logger(), "Tick not received after random path generation");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Random path service failed: %s", response->message.c_str());
    }

    return response->success;
}

bool NavigationClient::pause_navigation(double timeout_sec, bool should_wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling pause_navigation service for [%s]", robot_namespace_.c_str());

    if (should_wait_for_tick) {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }

    if (!wait_for_service<std_srvs::srv::Trigger>(
            pause_navigation_client_, "pause_navigation", timeout_sec)) {
        last_pause_message_ = "Service not available";
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = pause_navigation_client_->async_send_request(request);
    
    auto timeout_duration = std::chrono::duration<double>(timeout_sec);
    if (future.wait_for(timeout_duration) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Pause navigation service call timed out");
        last_pause_message_ = "Service call timed out";
        return false;
    }

    auto response = future.get();
    last_pause_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Pause navigation successful: %s", response->message.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Pause navigation failed: %s", response->message.c_str());
    }

    return response->success;
}

bool NavigationClient::stop_navigation(double timeout_sec, bool should_wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling stop_navigation service for [%s]", robot_namespace_.c_str());

    if (should_wait_for_tick) {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }

    if (!wait_for_service<std_srvs::srv::Trigger>(
            stop_navigation_client_, "stop_navigation", timeout_sec)) {
        last_stop_message_ = "Service not available";
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = stop_navigation_client_->async_send_request(request);
    
    auto timeout_duration = std::chrono::duration<double>(timeout_sec);
    if (future.wait_for(timeout_duration) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "Stop navigation service call timed out");
        last_stop_message_ = "Service call timed out";
        return false;
    }

    auto response = future.get();
    last_stop_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Stop navigation successful: %s", response->message.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Stop navigation failed: %s", response->message.c_str());
    }

    return response->success;
}

bool NavigationClient::generate_specific_path(const geometry_msgs::msg::Pose& pose, double timeout_sec, bool should_wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling generate_specific_path service for [%s]", robot_namespace_.c_str());
    RCLCPP_INFO(this->get_logger(), "Target pose: [%.2f, %.2f, %.2f]", 
                pose.position.x, pose.position.y, pose.position.z);

    if (should_wait_for_tick) {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }

    if (!wait_for_service<navigation_system::srv::NavigateToGoal>(
            generate_specific_path_client_, "generate_specific_path", timeout_sec)) {
        last_specific_path_message_ = "Service not available";
        return false;
    }

    auto request = std::make_shared<navigation_system::srv::NavigateToGoal::Request>();
    request->pose = pose;

    navigation_system::srv::NavigateToGoal::Response::SharedPtr response;
    if (!call_service_async<navigation_system::srv::NavigateToGoal>(
            generate_specific_path_client_, request, response, timeout_sec)) {
        last_specific_path_message_ = "Service call timed out";
        return false;
    }

    last_specific_path_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Specific path service successful: %s", response->message.c_str());
        if (should_wait_for_tick && !wait_for_tick(timeout_sec)) {
            RCLCPP_WARN(this->get_logger(), "Tick not received after specific path generation");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Specific path service failed: %s", response->message.c_str());
    }

    return response->success;
}

bool NavigationClient::generate_arc(float radius, const geometry_msgs::msg::Point& center, const geometry_msgs::msg::Point& goal, double timeout_sec, bool should_wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling generate_arc service for [%s]", robot_namespace_.c_str());
    RCLCPP_INFO(this->get_logger(), "Arc parameters: radius=%.2f, center=[%.2f, %.2f], goal=[%.2f, %.2f]",
                radius, center.x, center.y, goal.x, goal.y);

    if (should_wait_for_tick) {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }

    if (!wait_for_service<navigation_system::srv::NavigateArc>(
            generate_arc_client_, "generate_arc", timeout_sec)) {
        last_arc_message_ = "Service not available";
        return false;
    }

    auto request = std::make_shared<navigation_system::srv::NavigateArc::Request>();
    request->radius = radius;
    request->center = center;
    request->goal = goal;

    navigation_system::srv::NavigateArc::Response::SharedPtr response;
    if (!call_service_async<navigation_system::srv::NavigateArc>(
            generate_arc_client_, request, response, timeout_sec)) {
        last_arc_message_ = "Service call timed out";
        return false;
    }

    last_arc_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Arc path service successful: %s", response->message.c_str());
        if (should_wait_for_tick && !wait_for_tick(timeout_sec)) {
            RCLCPP_WARN(this->get_logger(), "Tick not received after arc generation");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Arc path service failed: %s", response->message.c_str());
    }

    return response->success;
}

bool NavigationClient::rotate_to_center(const geometry_msgs::msg::Point& center, double timeout_sec, bool should_wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling rotate_to_center service for [%s]", robot_namespace_.c_str());
    RCLCPP_INFO(this->get_logger(), "Center point: [%.2f, %.2f, %.2f]", center.x, center.y, center.z);

    if (should_wait_for_tick) {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }

    if (!wait_for_service<navigation_system::srv::CenterPoint>(
            rotate_to_center_client_, "rotate_to_center", timeout_sec)) {
        last_rotate_message_ = "Service not available";
        return false;
    }

    auto request = std::make_shared<navigation_system::srv::CenterPoint::Request>();
    request->center = center;

    navigation_system::srv::CenterPoint::Response::SharedPtr response;
    if (!call_service_async<navigation_system::srv::CenterPoint>(
            rotate_to_center_client_, request, response, timeout_sec)) {
        last_rotate_message_ = "Service call timed out";
        return false;
    }

    last_rotate_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Rotate to center service successful: %s", response->message.c_str());
        if (should_wait_for_tick && !wait_for_tick(timeout_sec)) {
            RCLCPP_WARN(this->get_logger(), "Tick not received after rotation");
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Rotate to center service failed: %s", response->message.c_str());
    }

    return response->success;
}

// Template instantiations
template bool NavigationClient::wait_for_service<std_srvs::srv::Trigger>(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr,
    const std::string&,
    double);

template bool NavigationClient::wait_for_service<navigation_system::srv::NavigateToGoal>(
    rclcpp::Client<navigation_system::srv::NavigateToGoal>::SharedPtr,
    const std::string&,
    double);

template bool NavigationClient::wait_for_service<navigation_system::srv::NavigateArc>(
    rclcpp::Client<navigation_system::srv::NavigateArc>::SharedPtr,
    const std::string&,
    double);

template bool NavigationClient::wait_for_service<navigation_system::srv::CenterPoint>(
    rclcpp::Client<navigation_system::srv::CenterPoint>::SharedPtr,
    const std::string&,
    double);

template bool NavigationClient::call_service_async<std_srvs::srv::Trigger, std_srvs::srv::Trigger::Request>(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr,
    std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std_srvs::srv::Trigger::Response::SharedPtr&,
    double);

template bool NavigationClient::call_service_async<navigation_system::srv::NavigateToGoal, navigation_system::srv::NavigateToGoal::Request>(
    rclcpp::Client<navigation_system::srv::NavigateToGoal>::SharedPtr,
    std::shared_ptr<navigation_system::srv::NavigateToGoal::Request>,
    navigation_system::srv::NavigateToGoal::Response::SharedPtr&,
    double);

template bool NavigationClient::call_service_async<navigation_system::srv::NavigateArc, navigation_system::srv::NavigateArc::Request>(
    rclcpp::Client<navigation_system::srv::NavigateArc>::SharedPtr,
    std::shared_ptr<navigation_system::srv::NavigateArc::Request>,
    navigation_system::srv::NavigateArc::Response::SharedPtr&,
    double);

template bool NavigationClient::call_service_async<navigation_system::srv::CenterPoint, navigation_system::srv::CenterPoint::Request>(
    rclcpp::Client<navigation_system::srv::CenterPoint>::SharedPtr,
    std::shared_ptr<navigation_system::srv::CenterPoint::Request>,
    navigation_system::srv::CenterPoint::Response::SharedPtr&,
    double);

} // namespace main_logic
