#include "main_logic/vision_client.hpp"

namespace main_logic
{

VisionClient::VisionClient(const std::string& robot_namespace)
    : Node("vision_client_node"), robot_namespace_(robot_namespace), tick_received_(false)
{
    // Initialize service clients
    std::string detection_service    = "/" + robot_namespace_ + "/trigger_detection";
    std::string pcl_service          = "/" + robot_namespace_ + "/trigger_pcl";
    std::string filter_pcl_service   = "/" + robot_namespace_ + "/trigger_filter_pcl";

    detection_client_  = this->create_client<vision_system::srv::NameObject>(detection_service);
    pcl_client_        = this->create_client<std_srvs::srv::Trigger>(pcl_service);
    filter_pcl_client_ = this->create_client<std_srvs::srv::Trigger>(filter_pcl_service);

    // Initialize tick subscriber
    std::string tick_topic = "/" + robot_namespace_ + "/vision_system/tick_service_vision";
    tick_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        tick_topic, 10,
        std::bind(&VisionClient::tick_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "VisionClient initialized for robot: %s", robot_namespace_.c_str());
    RCLCPP_INFO(this->get_logger(), "Subscribing to tick topic: %s", tick_topic.c_str());
}

template<typename ServiceT>
bool VisionClient::wait_for_service(
    typename rclcpp::Client<ServiceT>::SharedPtr client,
    const std::string& service_name,
    double timeout_sec)
{
    auto timeout = std::chrono::duration<double>(timeout_sec);
    if (!client->wait_for_service(timeout)) {
        RCLCPP_ERROR(this->get_logger(), 
                     "Service %s not available after waiting for %.1f seconds", 
                     service_name.c_str(), timeout_sec);
        return false;
    }
    return true;
}

void VisionClient::tick_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Tick received from vision system");
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = true;
        tick_cv_.notify_all();
    }
}

bool VisionClient::wait_for_tick(double timeout_sec)
{
    RCLCPP_INFO(this->get_logger(), "Waiting for tick signal (timeout: %.1fs)...", timeout_sec);
    
    // Reset tick flag before waiting
    {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = std::chrono::duration<double>(timeout_sec);
    
    // Create executor for processing callbacks
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());
    
    while (rclcpp::ok()) {
        // Check if tick received
        {
            std::lock_guard<std::mutex> lock(tick_mutex_);
            if (tick_received_) {
                RCLCPP_INFO(this->get_logger(), " Tick received, continuing...");
                return true;
            }
        }
        
        // Spin once to process callbacks
        executor.spin_some(std::chrono::milliseconds(50));
        
        // Check timeout
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed >= timeout_duration) {
            RCLCPP_ERROR(this->get_logger(), "Timeout waiting for tick signal after %.1fs", timeout_sec);
            return false;
        }
    }
    
    return false;
}

bool VisionClient::trigger_detection(const std::string& object_name, double timeout_sec, bool wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling trigger_detection service for object: %s", object_name.c_str());

    // Wait for service
    if (!wait_for_service<vision_system::srv::NameObject>(
            detection_client_, "trigger_detection", timeout_sec)) {
        last_detection_message_ = "Service not available";
        return false;
    }

    // Create request
    auto request = std::make_shared<vision_system::srv::NameObject::Request>();
    request->name_object = object_name;

    // Call service
    auto future = detection_client_->async_send_request(request);

    // Wait for response
    auto timeout = std::chrono::duration<double>(timeout_sec);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) 
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call trigger_detection service");
        last_detection_message_ = "Service call timeout";
        return false;
    }

    // Get response
    auto response = future.get();
    last_detection_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Detection service successful: %s", response->message.c_str());
        
        // Wait for tick if requested
        if (wait_for_tick) {
            if (!this->wait_for_tick(timeout_sec * 2)) {
                RCLCPP_WARN(this->get_logger(), "Detection completed but tick not received");
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Detection service failed: %s", response->message.c_str());
    }

    return response->success;
}

bool VisionClient::trigger_pcl(double timeout_sec, bool wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling trigger_pcl service");

    // Wait for service
    if (!wait_for_service<std_srvs::srv::Trigger>(
            pcl_client_, "trigger_pcl", timeout_sec)) {
        last_pcl_message_ = "Service not available";
        return false;
    }

    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call service
    auto future = pcl_client_->async_send_request(request);

    // Wait for response
    auto timeout = std::chrono::duration<double>(timeout_sec);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) 
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call trigger_pcl service");
        last_pcl_message_ = "Service call timeout";
        return false;
    }

    // Get response
    auto response = future.get();
    last_pcl_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "PCL service successful: %s", response->message.c_str());
        
        // Wait for tick if requested
        if (wait_for_tick) {
            if (!this->wait_for_tick(timeout_sec * 2)) {
                RCLCPP_WARN(this->get_logger(), "PCL capture completed but tick not received");
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "PCL service failed: %s", response->message.c_str());
    }

    return response->success;
}

bool VisionClient::trigger_filter_pcl(double timeout_sec, bool wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling trigger_filter_pcl service");

    // Wait for service
    if (!wait_for_service<std_srvs::srv::Trigger>(
            filter_pcl_client_, "trigger_filter_pcl", timeout_sec)) {
        last_filter_pcl_message_ = "Service not available";
        return false;
    }

    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call service
    auto future = filter_pcl_client_->async_send_request(request);
    auto timeout = std::chrono::duration<double>(timeout_sec);
    
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) 
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call trigger_filter_pcl service");
        last_filter_pcl_message_ = "Service call timeout";
        return false;
    }

    auto response = future.get();
    last_filter_pcl_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Filter PCL service successful: %s", response->message.c_str());
        
        if (wait_for_tick) {
            if (!this->wait_for_tick(timeout_sec * 2)) {
                RCLCPP_WARN(this->get_logger(), "PCL filtering completed but tick not received");
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Filter PCL service failed: %s", response->message.c_str());
    }

    return response->success;
}

template bool VisionClient::wait_for_service<vision_system::srv::NameObject>(
    rclcpp::Client<vision_system::srv::NameObject>::SharedPtr,
    const std::string&,
    double);


    template bool VisionClient::wait_for_service<std_srvs::srv::Trigger>(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr,
    const std::string&,
    double);

} // namespace main_logic
