#ifndef COORDINATION_CLIENT_HPP
#define COORDINATION_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
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

class CoordinationClient : public rclcpp::Node
{
public:
    explicit CoordinationClient();

    bool trigger_coordination_next_pose(double timeout_sec = 10.0, bool wait_for_tick = true);
    bool generate_mesh(double timeout_sec = 10.0, bool wait_for_tick = true);
    bool visualize_raw_ply_files(double timeout_sec = 10.0, bool wait_for_tick = true);

    // Wait for tick indefinitely (no timeout)
    bool wait_for_tick(double timeout_sec = 0.0);

    std::string get_last_coordination_message() const { return last_coordination_message_; }
    std::string get_last_mesh_message() const { return last_mesh_message_; }
    std::string get_last_visualize_message() const { return last_visualize_message_; }

private:
    // Service clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr coordination_next_pose_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr generate_mesh_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr visualize_ply_client_;

    // Tick subscriber
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr tick_subscriber_;

    // Last response messages
    std::string last_coordination_message_;
    std::string last_mesh_message_;
    std::string last_visualize_message_;

    // Tick synchronization
    std::atomic<bool> tick_received_;
    std::mutex tick_mutex_;
    std::condition_variable tick_cv_;

    /**
     * @brief Callback for tick messages
     */
    void tick_callback(const std_msgs::msg::Bool::SharedPtr msg);

    template<typename ServiceT>
    bool wait_for_service(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        const std::string& service_name,
        double timeout_sec);
};

} // namespace main_logic

#endif // COORDINATION_CLIENT_HPP
