#include <rclcpp/rclcpp.hpp>
#include <interfaces_pkg/msg/robot_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <chrono>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"
#include <string>
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ClusterFollower : public rclcpp::Node
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    ClusterFollower()
        : Node("pose_subscriber", rclcpp::NodeOptions())
    {
        robot_id_ = this->declare_parameter<std::string>("robot_id", "robot_id");
        std::string topic_f_cluster_out = this->declare_parameter<std::string>("cluster_pose_topic", "f_cluster_out");
        std::string nav2_topic_name = this->declare_parameter<std::string>("nav2_nav2pose_topic", "navigate_to_pose");

        other_shelfinos_ = this->declare_parameter<std::vector<std::string>>("other_shelfino_ids", {});
        path_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
        follow_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
        for (auto robot : other_shelfinos_)
        {
            std::string cluster_topic_name = "/" + robot +"/"+ topic_f_cluster_out;
            RCLCPP_INFO(this->get_logger(), "Setting up subscriber %s", cluster_topic_name.c_str());

            pcl_position_subscriber_.push_back(this->create_subscription<interfaces_pkg::msg::RobotPose>(cluster_topic_name, 10, std::bind(&ClusterFollower::subscribe_callback, this, std::placeholders::_1)));
        }
    }



private:
    std::string robot_id_;
    rclcpp::Publisher<interfaces_pkg::msg::RobotPose>::SharedPtr publisher_;
    std::vector<rclcpp::Subscription<interfaces_pkg::msg::RobotPose>::SharedPtr> pcl_position_subscriber_;
    std::vector<std::string> other_shelfinos_;
    bool going_to_position_ = false;

    rclcpp_action::Client<ComputePathToPose>::SharedPtr path_client_;
    rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;

    void subscribe_callback(const interfaces_pkg::msg::RobotPose requested_pose)
    {
        geometry_msgs::msg::Pose pose;

        pose = requested_pose.pose.pose;
        std::string log = "Pose received from " + requested_pose.robot_id;
        RCLCPP_INFO(this->get_logger(), log.c_str());

        if (!going_to_position_){
            going_to_position_ = true;
            this->goToPose(pose);
        }
    }

    bool wait_for_servers()
    {
        return path_client_->wait_for_action_server(5s) && follow_client_->wait_for_action_server(5s);
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

    void goToPose(geometry_msgs::msg::Pose goal_pose)
    {
        if (!wait_for_servers())
        {
            RCLCPP_ERROR(this->get_logger(), "One or more action servers not available!");
            return;
        }

        auto goal_msg = ComputePathToPose::Goal();
        goal_msg.goal.pose = goal_pose;

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
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ClusterFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}