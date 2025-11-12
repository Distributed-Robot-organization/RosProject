#include "main_logic/coordination_client.hpp"

namespace main_logic
{

CoordinationClient::CoordinationClient()
    : Node("coordination_client_node"), tick_received_(false)
{
    // Initialize service clients
    std::string coordination_service = "/trigger_coordination_next_pose";
    std::string mesh_service = "/generate_mesh";
    std::string visualize_service = "/visualize_raw_ply_files";

    coordination_next_pose_client_ = this->create_client<std_srvs::srv::Trigger>(coordination_service);
    generate_mesh_client_ = this->create_client<std_srvs::srv::Trigger>(mesh_service);
    visualize_ply_client_ = this->create_client<std_srvs::srv::Trigger>(visualize_service);

    // Initialize tick subscriber
    std::string tick_topic = "/coordination/tick_service_coordination";
    tick_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        tick_topic, 10,
        std::bind(&CoordinationClient::tick_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "CoordinationClient initialized");
    RCLCPP_INFO(this->get_logger(), "Subscribing to tick topic: %s", tick_topic.c_str());
}

template<typename ServiceT>
bool CoordinationClient::wait_for_service(
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

void CoordinationClient::tick_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        RCLCPP_INFO(this->get_logger(), "Tick received from coordination system");
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = true;
        tick_cv_.notify_all();
    }
}

bool CoordinationClient::wait_for_tick(double timeout_sec)
{
    RCLCPP_INFO(this->get_logger(), "Waiting for tick signal...");
    
    // Reset tick flag before waiting
    {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }
    
    auto start_time = std::chrono::steady_clock::now();
    auto timeout_duration = timeout_sec > 0 ? std::chrono::duration<double>(timeout_sec) : std::chrono::hours(24);
    
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
        if (timeout_sec > 0) {
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (elapsed >= timeout_duration) {
                RCLCPP_ERROR(this->get_logger(), "Timeout waiting for tick signal after %.1fs", timeout_sec);
                return false;

                
            }
        }
    }
    
    if (!rclcpp::ok()) {
        RCLCPP_WARN(this->get_logger(), "ROS shutdown while waiting for tick");
    }
    
    return false;
}

bool CoordinationClient::trigger_coordination_next_pose(double timeout_sec, bool wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling trigger_coordination_next_pose service");

    // Reset tick flag before calling service
    if (wait_for_tick) {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }

    // Wait for service
    if (!wait_for_service<std_srvs::srv::Trigger>(
            coordination_next_pose_client_, "trigger_coordination_next_pose", timeout_sec)) {
        last_coordination_message_ = "Service not available";
        return false;
    }

    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call service
    auto future = coordination_next_pose_client_->async_send_request(request);
    
    // Wait for response while processing callbacks (to receive tick during service execution)
    auto timeout = std::chrono::duration<double>(timeout_sec);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());
    
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        executor.spin_some(std::chrono::milliseconds(50));
        
        if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            break;
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed >= timeout) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call trigger_coordination_next_pose service");
            last_coordination_message_ = "Service call timeout";
            return false;
        }
    }

    // Get response
    auto response = future.get();
    last_coordination_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Coordination service successful: %s", response->message.c_str());
        
        // Check if tick was already received during service execution
        if (wait_for_tick) {
            bool tick_already_received = false;
            {
                std::lock_guard<std::mutex> lock(tick_mutex_);
                tick_already_received = tick_received_;
            }
            
            if (tick_already_received) {
                RCLCPP_INFO(this->get_logger(), " Tick already received during service execution");
            } else {
                RCLCPP_INFO(this->get_logger(), "Waiting for tick after service completion...");
                if (!this->wait_for_tick(timeout_sec * 2)) {
                    RCLCPP_WARN(this->get_logger(), "Coordination completed but tick not received");
                }
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Coordination service failed: %s", response->message.c_str());
    }

    return response->success;
}

bool CoordinationClient::generate_mesh(double timeout_sec, bool wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling generate_mesh service");

    // Reset tick flag before calling service
    if (wait_for_tick) {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }

    // Wait for service
    if (!wait_for_service<std_srvs::srv::Trigger>(
            generate_mesh_client_, "generate_mesh", timeout_sec)) {
        last_mesh_message_ = "Service not available";
        return false;
    }

    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call service with executor to process callbacks
    auto future = generate_mesh_client_->async_send_request(request);

    auto timeout = std::chrono::duration<double>(timeout_sec);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());
    
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        executor.spin_some(std::chrono::milliseconds(50));
        
        if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            break;
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed >= timeout) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call generate_mesh service");
            last_mesh_message_ = "Service call timeout";
            return false;
        }
    }

    // Get response
    auto response = future.get();
    last_mesh_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Mesh generation service successful: %s", response->message.c_str());
        
        // Check if tick was already received
        if (wait_for_tick) {
            bool tick_already_received = false;
            {
                std::lock_guard<std::mutex> lock(tick_mutex_);
                tick_already_received = tick_received_;
            }
            
            if (tick_already_received) {
                RCLCPP_INFO(this->get_logger(), " Tick already received during service execution");
            } else {
                if (!this->wait_for_tick(timeout_sec * 2)) {
                    RCLCPP_WARN(this->get_logger(), "Mesh generation completed but tick not received");
                }
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Mesh generation service failed: %s", response->message.c_str());
    }

    return response->success;
}

bool CoordinationClient::visualize_raw_ply_files(double timeout_sec, bool wait_for_tick)
{
    RCLCPP_INFO(this->get_logger(), "Calling visualize_raw_ply_files service");

    // Reset tick flag before calling service
    if (wait_for_tick) {
        std::lock_guard<std::mutex> lock(tick_mutex_);
        tick_received_ = false;
    }

    // Wait for service
    if (!wait_for_service<std_srvs::srv::Trigger>(
            visualize_ply_client_, "visualize_raw_ply_files", timeout_sec)) {
        last_visualize_message_ = "Service not available";
        return false;
    }

    // Create request
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Call service with executor to process callbacks
    auto future = visualize_ply_client_->async_send_request(request);

    auto timeout = std::chrono::duration<double>(timeout_sec);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(this->get_node_base_interface());
    
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        executor.spin_some(std::chrono::milliseconds(50));
        
        if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            break;
        }
        
        auto elapsed = std::chrono::steady_clock::now() - start_time;
        if (elapsed >= timeout) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call visualize_raw_ply_files service");
            last_visualize_message_ = "Service call timeout";
            return false;
        }
    }

    // Get response
    auto response = future.get();
    last_visualize_message_ = response->message;

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Visualization service successful: %s", response->message.c_str());
        
        // Check if tick was already received
        if (wait_for_tick) {
            bool tick_already_received = false;
            {
                std::lock_guard<std::mutex> lock(tick_mutex_);
                tick_already_received = tick_received_;
            }
            
            if (tick_already_received) {
                RCLCPP_INFO(this->get_logger(), " Tick already received during service execution");
            } else {
                if (!this->wait_for_tick(timeout_sec * 2)) {
                    RCLCPP_WARN(this->get_logger(), "Visualization completed but tick not received");
                }
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "Visualization service failed: %s", response->message.c_str());
    }

    return response->success;
}

template bool CoordinationClient::wait_for_service<std_srvs::srv::Trigger>(
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr,
    const std::string&,
    double);

} // namespace main_logic
