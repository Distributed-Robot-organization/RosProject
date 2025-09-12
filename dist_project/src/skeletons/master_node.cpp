#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using namespace std::chrono_literals;

class MasterNode : public rclcpp::Node
{
public:
  MasterNode() : Node("master_node")
  {
    client_change_state_ =
      this->create_client<lifecycle_msgs::srv::ChangeState>("/sensor_node/change_state");
    client_get_state_ =
      this->create_client<lifecycle_msgs::srv::GetState>("/sensor_node/get_state");

    // Example: Activate sensor after 2 seconds
    timer_ = this->create_wall_timer(2s, [this]() { this->activate_vision(); });
  }

private:
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
  rclcpp::TimerBase::SharedPtr timer_;

  void activate_vision()
  {
    if (!client_change_state_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "Service not available yet...");
      return;
    }

    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

    auto future = client_change_state_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "Requested sensor activation");
  }

  void deactivate_sensor()
  {
    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;

    auto future = client_change_state_->async_send_request(req);
    RCLCPP_INFO(this->get_logger(), "Requested sensor deactivation");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MasterNode>());
  rclcpp::shutdown();
  return 0;
}
