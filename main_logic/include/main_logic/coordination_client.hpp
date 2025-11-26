#ifndef COORDINATION_CLIENT_HPP
#define COORDINATION_CLIENT_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace main_logic
{

class CoordinationClient
{
public:
    CoordinationClient(rclcpp::Node *node);
    void trigger_coordination_next_pose(double timeout_sec = 10.0);
    std::string get_last_coordination_message() const;

private:
    // Service clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr coordination_next_pose_client_;
    // Node reference
    rclcpp::Node *node_;
    // Last service response message
    std::string last_coordination_message_;
};
} // namespace main_logic
#endif // COORDINATION_CLIENT_HPP
