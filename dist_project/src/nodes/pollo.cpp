#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

class BasicNavigator : public rclcpp::Node
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    BasicNavigator()
        : Node("basic_navigator")
    {
        path_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
        follow_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
    }

    bool wait_for_servers()
    {
        return path_client_->wait_for_action_server(5s) && follow_client_->wait_for_action_server(5s);
    }

    void goToPose(double x, double y, double theta)
    {
        if (!wait_for_servers())
        {
            RCLCPP_ERROR(this->get_logger(), "One or more action servers not available!");
            return;
        }

        auto goal_msg = ComputePathToPose::Goal();
        goal_msg.goal.pose.position.x = x;
        goal_msg.goal.pose.position.y = y;
        goal_msg.goal.pose.orientation.z = sin(theta / 2.0);
        goal_msg.goal.pose.orientation.w = cos(theta / 2.0);
        goal_msg.goal.header.frame_id = "map";
        goal_msg.goal.header.stamp = this->now();

        auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleComputePath::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Path computed, sending to FollowPath...");
                this->followPath(result.result->path);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to compute path");
            }
        };

        path_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void followPath(nav_msgs::msg::Path path)
    {
        auto goal_msg = FollowPath::Goal();
        goal_msg.path = path;

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleFollowPath::WrappedResult &result)
        {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "FollowPath succeeded!");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "FollowPath failed or canceled");
            }
        };

        follow_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<ComputePathToPose>::SharedPtr path_client_;
    rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto navigator = std::make_shared<BasicNavigator>();

    if (!navigator->wait_for_servers())
    {
        std::cerr << "Action servers not available" << std::endl;
        return 1;
    }

    navigator->goToPose(1.0, 2.0, 0.0);

    rclcpp::spin(navigator);
    rclcpp::shutdown();
    return 0;
}
