#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <interfaces_pkg/msg/robot_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <memory>
#include <chrono>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"
#include <string>
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
using namespace std::chrono_literals;

class RobotManager : public rclcpp::Node
{
public:
    using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
    using FollowPath = nav2_msgs::action::FollowPath;
    using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

    RobotManager() : Node("robot_manager"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        // Parameter Definitions
        robot_id_ = this->declare_parameter<std::string>("robot_id", "pollo");
        std::string topic_f_cluster_out = this->declare_parameter<std::string>("cluster_pose_topic", "f_cluster_out");
        std::string topic_pcl_in = this->declare_parameter<std::string>("topic_pcl_in", "f_cluster_out");
        auto server_positions = this->declare_parameter<std::string>("server_positions", "positions_to_explore_vis");
        frame_id_ = this->declare_parameter<std::string>("world_frame", "map");
        initial_goals_ = this->declare_parameter<std::vector<double>>("initial_goals." + robot_id_, {.0, .0, .0});

        auto robot_ids_ = this->declare_parameter<std::vector<std::string>>("other_shelfino_ids", {});
        path_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
        follow_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");
        if (initial_goals_.size() % 3 != 0)
            throw std::runtime_error("The intial goals must be a multiple of 3");
        size_t i = 0;
        while (i < initial_goals_.size())
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = initial_goals_[i * 3];
            pose.position.y = initial_goals_[i * 3 + 1];

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, initial_goals_[i * 3 + 2]);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            initial_goals_poses_.push_back(pose);
            i++;
        }
        for (auto id : robot_ids_)
        {
            if (id != robot_id_)
                other_robot_ids_.push_back(id);
        }

        // Sensors managment
        client_change_state_ =
            this->create_client<lifecycle_msgs::srv::ChangeState>("vision_node/change_state");
        client_get_state_ =
            this->create_client<lifecycle_msgs::srv::GetState>("vision_node/get_state");
        pcl_subscriber_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_pcl_in, 1, std::bind(&pcl_callback, this, std::placeholders::_1));
        // Server managment
        server_position_subsciber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(server_positions, 1, std::bind(&server_callback, this, std::placeholders::_1));
    }

private:
    std::string robot_id_, frame_id_;
    std::vector<std::string> other_robot_ids_;
    std::vector<double> initial_goals_;
    std::vector<geometry_msgs::msg::Pose> initial_goals_poses_;

    // Decision handling
    bool moving_, going_around_, cmd_stop_robot_;
    int following_position_number_ = 0;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::mutex m_;

    // Vision Handling
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;

    // Movement handling
    rclcpp_action::Client<ComputePathToPose>::SharedPtr path_client_;
    rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
    GoalHandleFollowPath::SharedPtr follow_goal_handle_;

    // Server Communication
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr server_position_subsciber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2> pcl_publisher_;

    void plan_manager()
    {
        std::lock_guard<std::mutex> lock(m_);

        if (cmd_stop_robot_)
        {
            RCLCPP_INFO(this->get_logger(), "The %s was commanded to stop;", std::string(robot_id_));
            stopMovement();
            cmd_stop_robot_ = false;
        }
    }

    void server_callback(const visualization_msgs::msg::MarkerArray positions)
    {
        cmd_stop_robot_ = true;
        std::map<geometry_msgs::msg::Pose, double> pose_distance;
        auto robot_pose = get_robot_pose(tf_buffer_, frame_id_, robot_id_, this->get_clock()->now());
        for (auto marker : positions.markers)
        {
            auto p = marker.pose.position;
            auto distance = sqrt(pow(p.x - robot_pose.position.x, 2) + pow(p.y - robot_pose.position.y, 2));
            pose_distance.emplace((marker.pose,distance));
        }
        std::sort(pose_distance.begin(), pose_distance.end());
        pose_distance.
        plan_manager();
    }

    void pcl_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl)
    {
        cmd_stop_robot_ = true;

        pcl_publisher_.publish(*pcl);
        plan_manager();
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
            else if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                RCLCPP_INFO(this->get_logger(), "FollowPath canceled!");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "FollowPath failed");
            }
            moving_ = false; // allow new goals again
        };

        // Send the goal asynchronously
        auto future_goal_handle = follow_client_->async_send_goal(goal_msg, send_goal_options);

        // Store the goal handle once the future completes (non-blocking)
        std::thread([this, future_goal_handle]() mutable
                    {
        try {
            auto goal_handle = future_goal_handle.get();  // blocks until ready
            if (goal_handle) {
                follow_goal_handle_ = goal_handle;
                RCLCPP_INFO(this->get_logger(), "FollowPath goal accepted.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "FollowPath goal was rejected.");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception while getting goal handle: %s", e.what());
        } })
            .detach();
    }

    void
    stopMovement()
    {
        if (follow_goal_handle_)
        {
            RCLCPP_INFO(this->get_logger(), "Cancelling current goal...");
            follow_client_->async_cancel_goal(follow_goal_handle_);
            follow_goal_handle_.reset();
            moving_ = false;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
        }
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

        goal_msg.goal.header.frame_id = frame_id_;
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

    geometry_msgs::msg::Pose get_robot_pose(
        const tf2_ros::Buffer &buffer,
        const std::string &target_frame,
        const std::string &robot_name,
        rclcpp::Time now,
        double sleeptime,
        int attempts)
    {
        double waited = 0.0;
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(sleeptime);
        geometry_msgs::msg::TransformStamped tf;
        std::string robot_frame = robot_name + "/base_link";
        while (!buffer.canTransform(target_frame, robot_frame, now, timeout))
        {
            waited += sleeptime;

            std::cout << "Waiting " << waited << " seconds for " << robot_frame << std::endl;

            if (attempts-- <= 0)
            {
                throw std::runtime_error("TF lookup failed: Tried to wait for transform to no avail");
            }
        }
        try
        {
            tf = buffer.lookupTransform(target_frame, robot_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            throw std::runtime_error("TF lookup failed: " + std::string(ex.what()));
        }

        geometry_msgs::msg::Pose pose;
        pose.position.x = tf.transform.translation.x;
        pose.position.y = tf.transform.translation.y;
        pose.position.z = tf.transform.translation.z;
        pose.orientation = tf.transform.rotation;
        return pose;
    }

    void activate_vision()
    {
        if (!client_change_state_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Service not available yet...");
            return;
        }

        auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        req->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;

        auto future = client_change_state_->async_send_request(req);
        RCLCPP_INFO(this->get_logger(), "Requested sensor activation");
    }

    void deactivate_vision()
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
    rclcpp::spin(std::make_shared<RobotManager>());
    rclcpp::shutdown();
    return 0;
}
