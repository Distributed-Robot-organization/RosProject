#include "rclcpp/rclcpp.hpp"

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

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include "lifecycle_msgs/msg/transition.hpp"

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
        std::string topic_pcl_in = this->declare_parameter<std::string>("topics.cluster_pcl", "f_cluster_out");
        auto pcl_to_server = this->declare_parameter<std::string>("server.pcl_topic_in", "f_cluster_out");
        auto server_positions = "/mesh_server/" + this->declare_parameter<std::string>("topics.poses_to_be_in", "poses");
        sensor_node_ = this->declare_parameter<std::string>("robot_logic.vision_manager", "vision_node");
        hz_ = this->declare_parameter<float>("robot_logic.vision_activation_check_hz", 1.0);
        activation_distance_ = this->declare_parameter<float>("robot_logic.vision_activation_distance", 5.0);

        frame_id_ = this->declare_parameter<std::string>("world_frame", "map");
        initial_goals_ = this->declare_parameter<std::vector<double>>("checkpoints." + robot_id_, {.0, .0, .0});

        auto robot_ids = this->declare_parameter<std::vector<std::string>>("init_names", std::vector<std::string>{"shelfino1", "pollo"});

        path_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "compute_path_to_pose");
        follow_client_ = rclcpp_action::create_client<FollowPath>(this, "follow_path");

        // Set initial poses to follow to explore the enviroment [x_cord0,y_cord0,angle0, x_cord1,y_cord1,angle1,...]
        if (initial_goals_.size() % 3 != 0)
            throw std::runtime_error("The intial goals must be a multiple of 3");
        size_t i = 0;
        while (i * 3 < initial_goals_.size())
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
            explore_poses_.push_back(pose);
            i++;
        }
        for (auto id : robot_ids)
        {
            if (id != robot_id_)
                other_robot_ids_.push_back(id);
        }
        // Plan making
        // At start the exploring flag is true such that the robot will
        // follow the list of poses until the object is found
        std::string change_state_client = sensor_node_ + "/change_state";
        std::string get_state_client = sensor_node_ + "/get_state";
        RCLCPP_INFO(this->get_logger(), "%s", sensor_node_.c_str());
        // Sensors managment
        client_change_state_ =
            this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_client.c_str());
        client_get_state_ =
            this->create_client<lifecycle_msgs::srv::GetState>(get_state_client.c_str());
        pcl_subscriber_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_pcl_in, 1, std::bind(&RobotManager::pcl_callback, this, std::placeholders::_1));
        // Server managment
        pcl_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(pcl_to_server, 200);

        server_position_subsciber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(server_positions, 1, std::bind(&RobotManager::server_callback, this, std::placeholders::_1));
        distance_activation_period_ = std::chrono::milliseconds(static_cast<int>(std::floor((1 / hz_) * 1000)));
        rclcpp::on_shutdown([this]()
                            { this->onShutdown(); });

        RCLCPP_INFO(this->get_logger(), "Given %ld checkpoints to explore", explore_poses_.size());
        last_state_requested_ = lifecycle_msgs::msg::Transition::TRANSITION_DESTROY;
        change_state_sensor(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        change_state_sensor(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        plan_manager();
    }

private:
    std::string robot_id_, frame_id_, sensor_node_;
    std::vector<std::string> other_robot_ids_;
    std::vector<double> initial_goals_;

    // Vision Handling
    float hz_, activation_distance_;
    std::chrono::milliseconds distance_activation_period_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_change_state_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client_get_state_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber_;
    rclcpp::TimerBase::SharedPtr activation_timer_;

    uint8_t last_state_requested_, last_transition_success_ = false;

    // Movement handling
    rclcpp_action::Client<ComputePathToPose>::SharedPtr path_client_;
    rclcpp_action::Client<FollowPath>::SharedPtr follow_client_;
    GoalHandleFollowPath::SharedPtr follow_goal_handle_;
    std::vector<geometry_msgs::msg::Pose> explore_poses_;
    u_int exploring_position_number_ = 0;

    // Server Communication
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr server_position_subsciber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
    geometry_msgs::msg::Pose goal_pose_;

    // Decision handling
    bool moving_ = false, exploring_ = true, received_position_ = false, pcl_called_ = false, following_commands_ = false, arrived_to_position_ = false,
         allow_pcl_input_ = true, interrupted_movement_ = false, discard_movement_ = false, in_distance_for_vision_activation_ = false, consumed_goal_pose = false;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::mutex plan_, server_, pcl_;

    void plan_manager()
    {
        std::lock_guard<std::mutex> lock(plan_);
        RCLCPP_INFO(this->get_logger(), "received update, status :\n moving_ %d, exploring_ %d, received_position_%d, pcl_called_%d, following_commands_%d, arrived_to_position_%d,allow_pcl_input_%d, interrupted_movement_%d,discard_movement_%d, in_distance_for_vision_activation_%d, consumed_goal_pose%d",
                    moving_, exploring_, received_position_, pcl_called_, following_commands_, arrived_to_position_, allow_pcl_input_, interrupted_movement_, discard_movement_, in_distance_for_vision_activation_, consumed_goal_pose);

        stopMovement();
        // Used to iterrupt actions until at least a command from server is sent
        if (received_position_)
        {
            consumed_goal_pose = false;
            following_commands_ = true;
        }

        if (in_distance_for_vision_activation_)
        {
            RCLCPP_INFO(this->get_logger(), "In range for an early activation of the sensor");
            in_distance_for_vision_activation_ = false;
            activate_vision();
            activation_timer_->cancel();
            return;
        }
        if (exploring_ && (received_position_ || pcl_called_))
        {
            // should be the only instance of deactivating exloring since true is the default value and
            // should be deactivated only when the object is discovered
            deactivate_vision();
            if (received_position_)
            {
                activation_timer_ = create_wall_timer(
                    std::chrono::milliseconds(distance_activation_period_),
                    [this]()
                    { distance_based_vision_activation(); });
            }

            RCLCPP_INFO(this->get_logger(), "Object Found");
            exploring_ = false;
        }
        if (exploring_)
        {
            // cycle between the checkpoints to search the space
            if (exploring_position_number_ < (explore_poses_.size() - 1))
                exploring_position_number_++;
            else
                exploring_position_number_ = 0;
            auto pose_to_explore = explore_poses_[exploring_position_number_];
            RCLCPP_INFO(this->get_logger(), "Exploring checkpoint %d at %f %f %f", exploring_position_number_,
                        pose_to_explore.position.x,
                        pose_to_explore.position.y,
                        pose_to_explore.orientation.z);

            goToPose(pose_to_explore);
        }
        else if (!moving_ && following_commands_)
        {

            if (arrived_to_position_)
            {
                arrived_to_position_ = false;
                allow_pcl_input_ = true;
                change_state_sensor(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
            }
            else if (!consumed_goal_pose)
            {
                goToPose(goal_pose_);
                consumed_goal_pose = true;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "I DON'T KNOW");
            }
        }
        if (in_distance_for_vision_activation_)
        {
            RCLCPP_INFO(this->get_logger(), "I'm less than %f away from object. Activating vision...", activation_distance_);
        }
        if (received_position_)
            received_position_ = false;

        if (interrupted_movement_)
            interrupted_movement_ = false;

        if (pcl_called_)
        {
            pcl_called_ = false;
            // to allow only one pcl at a time
            deactivate_vision();
        }
        // discard_movement_ = false;
    }
    
    void distance_based_vision_activation()
    {
        auto robot_pose = get_robot_pose(tf_buffer_, frame_id_, robot_id_, this->get_clock()->now(), .5, 3);

        double dist = std::sqrt(
            std::pow(goal_pose_.position.x - robot_pose.position.x, 2) +
            std::pow(goal_pose_.position.y - robot_pose.position.y, 2));
        if (dist < activation_distance_)
        {
            in_distance_for_vision_activation_ = true;
            plan_manager();
        }
    }
    
    void deactivate_vision()
    {
        if (allow_pcl_input_)
        {
            allow_pcl_input_ = false;
            change_state_sensor(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
    }

    void activate_vision()
    {
        if (!allow_pcl_input_)
        {
            allow_pcl_input_ = true;
            change_state_sensor(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        }
    }

    void server_callback(const visualization_msgs::msg::MarkerArray positions)
    {
        std::lock_guard<std::mutex> lock(server_);

        RCLCPP_INFO(this->get_logger(), "Server sent message, going to position");

        auto robot_pose = get_robot_pose(tf_buffer_, frame_id_, robot_id_, this->get_clock()->now(), .5, 3);
        double min_distance = std::numeric_limits<double>::max();
        geometry_msgs::msg::Pose nearest_pose;

        auto robot_position = robot_pose.position;

        for (const auto &marker : positions.markers)
        {
            auto marker_position = marker.pose.position;
            double dist = std::sqrt(
                std::pow(marker_position.x - robot_position.x, 2) +
                std::pow(marker_position.y - robot_position.y, 2));
            if (dist < min_distance)
            {
                min_distance = dist;
                nearest_pose = marker.pose;
            }
        }
        goal_pose_ = nearest_pose;
        received_position_ = true;
        plan_manager();
    }

    void pcl_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl)
    {
        std::lock_guard<std::mutex> lock(pcl_);

        if (allow_pcl_input_)
        {
            RCLCPP_INFO(this->get_logger(), "Sensor detected pcl, sending to Server");
            pcl_publisher_->publish(*pcl);
            pcl_called_ = true;
            RCLCPP_INFO(this->get_logger(), "Calling plan_manager");
            plan_manager();
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
                arrived_to_position_ = true;
            }
            else if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                interrupted_movement_ = true;
                RCLCPP_INFO(this->get_logger(), "FollowPath canceled!");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "FollowPath failed");
            }
            moving_ = false; // allow new goals are
            // call again plan manager such that if we are exploring, it can trigger another follow goal
            // Or activate the vision module
            plan_manager();
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

    void stopMovement()
    {
        discard_movement_ = true;
        if (follow_goal_handle_)
        {
            RCLCPP_INFO(this->get_logger(), "Cancelling current goal...");
            try
            {
                follow_client_->async_cancel_goal(follow_goal_handle_);
            }
            catch (const rclcpp_action::exceptions::UnknownGoalHandleError &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Exception while getting goal handle: %s", e.what());
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
        }
    }

    void goToPose(geometry_msgs::msg::Pose goal_pose)
    {
        discard_movement_ = false;
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
                if (discard_movement_)
                {
                    // Can happen that the pcl is found in the interval between stopping and computing a new plan
                    RCLCPP_INFO(this->get_logger(), "Path computed, but disgarded");
                }
                RCLCPP_INFO(this->get_logger(), "Path computed, sending to FollowPath...");

                this->followPath(result.result->path);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to compute path");
            }
        };

        path_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Going to %f, %f, %f", goal_pose.position.x, goal_pose.position.y, goal_pose.orientation.z);
        moving_ = true;
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

    template <typename FutureT, typename WaitTimeT>
    std::future_status
    wait_for_result(
        FutureT &future,
        WaitTimeT time_to_wait)
    {
        auto end = std::chrono::steady_clock::now() + time_to_wait;
        std::chrono::milliseconds wait_period(100);
        std::future_status status = std::future_status::timeout;
        do
        {
            auto now = std::chrono::steady_clock::now();
            auto time_left = end - now;
            if (time_left <= std::chrono::seconds(0))
            {
                break;
            }
            status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
        } while (rclcpp::ok() && status != std::future_status::ready);
        return status;
    }

    bool
    change_state_sensor(std::uint8_t transition, std::chrono::seconds time_out = 3s)
    {
        // lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
        // lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        // lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
        if (last_state_requested_ != transition || !last_transition_success_)
        {
            last_state_requested_ = transition;
            auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            request->transition.id = transition;

            if (!client_change_state_->wait_for_service(time_out))
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Service %s is not available.",
                    client_change_state_->get_service_name());
                return false;
            }

            // We send the request with the transition we want to invoke.
            auto future_result = client_change_state_->async_send_request(request).future.share();

            // Let's wait until we have the answer from the node.
            // If the request times out, we return an unknown state.
            auto future_status = wait_for_result(future_result, time_out);

            if (future_status != std::future_status::ready)
            {
                RCLCPP_ERROR(
                    get_logger(), "Server time out while getting current state for node %s", sensor_node_.c_str());
                return false;
            }

            // We have an answer, let's print our success.
            if (future_result.get()->success)
            {
                RCLCPP_INFO(
                    get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
                last_transition_success_ = true;
                return true;
            }
            else
            {
                RCLCPP_WARN(
                    get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
                last_transition_success_ = false;
                return false;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Already requested %u transition, Ignoring Request", transition);
            return true;
        }
    }

    void onShutdown()
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node is shutting down! Cleaning up...");
        stopMovement();
        // Your custom cleanup logic here
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotManager>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
