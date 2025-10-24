#include <cmath>
#include <random>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "navigation_system/srv/navigate_to_goal.hpp"
#include "navigation_system/srv/navigate_arc.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rmw/qos_profiles.h"
#include "nav_msgs/msg/odometry.hpp"

using FollowPath = nav2_msgs::action::FollowPath;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

class PathGenerator : public rclcpp::Node
{
public:
  PathGenerator() : Node("random_path_generator"),
                          gen_(std::random_device{}())
  {
    // Parametrs
    this->declare_parameter("max_distance", 5.0);
    this->declare_parameter("step_size", 0.3);
    this->declare_parameter("frame_id", "map");
    
    max_distance_ = this->get_parameter("max_distance").as_double();
    step_size_ = this->get_parameter("step_size").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .reliability(rclcpp::ReliabilityPolicy::Reliable)
                   .durability(rclcpp::DurabilityPolicy::TransientLocal);

    // Service
    service_random_pose = this->create_service<std_srvs::srv::Trigger>(
        "/generate_random_path",
        std::bind(&PathGenerator::callback_random_trigger, this,
                  std::placeholders::_1, std::placeholders::_2));

    service_specific_pose = this->create_service<navigation_system::srv::NavigateToGoal>(
        "/generate_specific_path",
        std::bind(&PathGenerator::callback_specific_trigger, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Arc navigation service
    service_arc_ = this->create_service<navigation_system::srv::NavigateArc>(
        "/navigate_arc",
        std::bind(&PathGenerator::callback_arc_trigger, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Pause service
    service_pause_ = this->create_service<std_srvs::srv::Trigger>(
        "/pause_navigation",
        std::bind(&PathGenerator::callback_pause_trigger, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Stop service
    service_stop_ = this->create_service<std_srvs::srv::Trigger>(
        "/stop_navigation",
        std::bind(&PathGenerator::callback_stop_trigger, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Subscription 
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/shelfino1/amcl_pose", qos,
        std::bind(&PathGenerator::amcl_callback, this, std::placeholders::_1));
    
    // Publisher per visualizzare il path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/shelfino1/planned_path", 10);
    
    // Action clients
    path_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "/shelfino1/compute_path_to_pose");
    action_client_ = rclcpp_action::create_client<FollowPath>(this, "/shelfino1/follow_path");
    
    //respose of the action service
    if (!path_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server /shelfino1/compute_path_to_pose not available.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Action server /shelfino1/compute_path_to_pose OK.");
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server /shelfino1/follow_path not available.");
    } else {
      RCLCPP_INFO(this->get_logger(), "Action server /shelfino1/follow_path OK.");
    }

    RCLCPP_INFO(this->get_logger(), "pathgenerator ready. Call:  /generate_random_path or ... services.");
  }

private:
  void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    has_pose_ = true;
  }
  
  void callback_random_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (!has_pose_) {
      response->success = false;
      response->message = "Robot pose not yet received.";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    std::uniform_real_distribution<double> dist_radius(0.5, max_distance_);
    std::uniform_real_distribution<double> dist_angle(0.0, 2.0 * M_PI);

    double radius = dist_radius(gen_);
    double angle = dist_angle(gen_);

    double goal_x = current_pose_.position.x + radius * std::cos(angle);
    double goal_y = current_pose_.position.y + radius * std::sin(angle);

    RCLCPP_INFO(this->get_logger(), 
                "Generated random goal: (%.2f, %.2f) at distance %.2f from robot (%.2f, %.2f)",
                goal_x, goal_y, radius, current_pose_.position.x, current_pose_.position.y);

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = goal_x;
    goal_pose.position.y = goal_y;
    goal_pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle);
    goal_pose.orientation = tf2::toMsg(q);
    compute_path_to_goal(goal_pose);

    response->success = true;
    response->message = "Random path computation requested.";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void callback_specific_trigger(
      const std::shared_ptr<navigation_system::srv::NavigateToGoal::Request> request,
      std::shared_ptr<navigation_system::srv::NavigateToGoal::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), 
                "Received specific goal: (%.2f, %.2f)", 
                request->pose.position.x, request->pose.position.y);

    compute_path_to_goal(request->pose);
    
    response->success = true;
    response->message = "Specific path computation requested.";
    RCLCPP_INFO(this->get_logger(), "Specific path computation requested.");
  }

  void callback_arc_trigger(
      const std::shared_ptr<navigation_system::srv::NavigateArc::Request> request,
      std::shared_ptr<navigation_system::srv::NavigateArc::Response> response)
  {
    if (!has_pose_) {
      response->success = false;
      response->message = "Robot pose not yet received.";
      RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), 
                "Received arc navigation request: goal(%.2f, %.2f), center(%.2f, %.2f), radius=%.2f",
                request->goal_pose.position.x, request->goal_pose.position.y,
                request->center.position.x, request->center.position.y,
                request->radius);

    // Generate arc path
    nav_msgs::msg::Path arc_path = generate_arch_path(
        request->goal_pose, request->radius, request->center);

    if (arc_path.poses.empty()) {
      response->success = false;
      response->message = "Failed to generate arc path.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    // Publish and send path
    path_pub_->publish(arc_path);
    send_path_goal(arc_path);

    response->success = true;
    response->message = "Arc path generated and navigation started.";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }
nav_msgs::msg::Path generate_arch_path(
    const geometry_msgs::msg::Pose& goal_pose,
    double radius,
    const geometry_msgs::msg::Pose& center)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id_;
  path.header.stamp = this->now();

  // Current robot position
  double robot_x = current_pose_.position.x;
  double robot_y = current_pose_.position.y;
  double center_x = center.position.x;
  double center_y = center.position.y;

  // Project robot position onto circle
  double dx_robot = robot_x - center_x;
  double dy_robot = robot_y - center_y;
  double dist_robot = std::sqrt(dx_robot * dx_robot + dy_robot * dy_robot);
  
  double start_x, start_y;
  if (dist_robot > 1e-6) {
    start_x = center_x + (dx_robot / dist_robot) * radius;
    start_y = center_y + (dy_robot / dist_robot) * radius;
  } else {
    start_x = center_x + radius;
    start_y = center_y;
  }

  // Project goal position onto circle
  double dx_goal = goal_pose.position.x - center_x;
  double dy_goal = goal_pose.position.y - center_y;
  double dist_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
  
  double end_x, end_y;
  if (dist_goal > 1e-6) {
    end_x = center_x + (dx_goal / dist_goal) * radius;
    end_y = center_y + (dy_goal / dist_goal) * radius;
  } else {
    end_x = center_x + radius;
    end_y = center_y;
  }

  // Calculate angles
  double start_angle = std::atan2(start_y - center_y, start_x - center_x);
  double end_angle = std::atan2(end_y - center_y, end_x - center_x);

  // Choose shortest arc direction
  double angle_diff = end_angle - start_angle;
  if (angle_diff > M_PI) {
    angle_diff -= 2.0 * M_PI;
  } else if (angle_diff < -M_PI) {
    angle_diff += 2.0 * M_PI;
  }

  // Generate waypoints along arc
  double arc_length = std::abs(angle_diff) * radius;
  int num_points = std::max(10, static_cast<int>(arc_length / step_size_));
  
  for (int i = 0; i <= num_points; ++i) {
    double t = static_cast<double>(i) / num_points;
    double current_angle = start_angle + t * angle_diff;

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.header.stamp = this->now();
    
    pose.pose.position.x = center_x + radius * std::cos(current_angle);
    pose.pose.position.y = center_y + radius * std::sin(current_angle);
    pose.pose.position.z = 0.0;

    path.poses.push_back(pose);
  }

  // double linear_distance = radius / 3.0;
  // int num_linear_points = std::max(5, static_cast<int>(linear_distance / step_size_));
  // double arc_end_x = end_x;
  // double arc_end_y = end_y;
  
  // // Direzione verso il centro
  // double dx_to_center = center_x - arc_end_x;
  // double dy_to_center = center_y - arc_end_y;
  // double dist_to_center = std::sqrt(dx_to_center * dx_to_center + dy_to_center * dy_to_center);
  // // Normalizza la direzione
  // dx_to_center /= dist_to_center;
  // dy_to_center /= dist_to_center;
  
  // for (int i = 1; i <= num_linear_points; ++i) {
  //   double t = static_cast<double>(i) / num_linear_points;
    
  //   geometry_msgs::msg::PoseStamped pose;
  //   pose.header.frame_id = frame_id_;
  //   pose.header.stamp = this->now();
    
  //   pose.pose.position.x = arc_end_x + t * linear_distance * dx_to_center;
  //   pose.pose.position.y = arc_end_y + t * linear_distance * dy_to_center;
  //   pose.pose.position.z = 0.0;    
  //   path.poses.push_back(pose);
  // }

  // RCLCPP_INFO(this->get_logger(), 
  //             "Generated arc path with %zu waypoints (arc: %.2fm, linear: %.2fm)",
  //             path.poses.size(), arc_length, linear_distance);

  return path;
}
  void callback_pause_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (!is_paused_) {
      // Pause navigation
      if (current_goal_handle_) {
        auto cancel_future = action_client_->async_cancel_goal(current_goal_handle_);
        is_paused_ = true;
        response->success = true;
        response->message = "Navigation paused. Goal and path saved.";
        RCLCPP_INFO(this->get_logger(), "Navigation PAUSED.");
      } else {
        response->success = false;
        response->message = "No active navigation to pause.";
        RCLCPP_WARN(this->get_logger(), "No active navigation to pause.");
      }
    } else {
      // Resume navigation
      if (saved_path_.poses.empty()) {
        response->success = false;
        response->message = "No saved path to resume.";
        RCLCPP_WARN(this->get_logger(), "No saved path to resume.");
      } else {
        send_path_goal(saved_path_);
        is_paused_ = false;
        response->success = true;
        response->message = "Navigation resumed.";
        RCLCPP_INFO(this->get_logger(), "Navigation RESUMED.");
      }
    }
  }

  void callback_stop_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (current_goal_handle_) {
      auto cancel_future = action_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_.reset();
      saved_path_.poses.clear();
      is_paused_ = false;
      
      response->success = true;
      response->message = "Navigation stopped and path cleared.";
      RCLCPP_INFO(this->get_logger(), "Navigation STOPPED. Path cleared.");
    } else {
      response->success = false;
      response->message = "No active navigation to stop.";
      RCLCPP_WARN(this->get_logger(), "No active navigation to stop.");
    }
  }

  // == NAV2 FUNCTION
  void compute_path_to_goal(const geometry_msgs::msg::Pose& goal_pose)
  {
    if (!path_client_ || !path_client_->action_server_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "ComputePathToPose action server not ready.");
      return;
    }

    auto goal_msg = ComputePathToPose::Goal();
    goal_msg.goal.pose = goal_pose;
    goal_msg.goal.header.frame_id = frame_id_;
    goal_msg.goal.header.stamp = this->now();

    auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    send_goal_options.result_callback = 
        [this](const GoalHandleComputePath::WrappedResult &result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Path computed successfully!");
            
            path_pub_->publish(result.result->path);
            send_path_goal(result.result->path);
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute path");
          }
        };

    path_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Path computation requested to Nav2...");
  }

  void send_path_goal(const nav_msgs::msg::Path &path)
  {
    if (!action_client_ || !action_client_->action_server_is_ready()) {
      RCLCPP_ERROR(this->get_logger(), "FollowPath action server not ready.");
      return;
    }

    if (path.poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty path, not sending goal.");
      return;
    }

    // Save path for pause/resume functionality
    saved_path_ = path;

    FollowPath::Goal goal;
    goal.path = path;
    goal.controller_id = "";
    goal.goal_checker_id = "";

    rclcpp_action::Client<FollowPath>::SendGoalOptions opts;
    opts.goal_response_callback =
        [this](std::shared_ptr<GoalHandleFollowPath> handle) {
          if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal REJECTED.");
            current_goal_handle_.reset();
          } else {
            RCLCPP_INFO(this->get_logger(), "Goal ACCEPTED.");
            current_goal_handle_ = handle;
          }
        };
    opts.result_callback =
        [this](const GoalHandleFollowPath::WrappedResult &res) {
          switch (res.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              RCLCPP_INFO(this->get_logger(), "Goal COMPLETED successfully.");
              current_goal_handle_.reset();
              saved_path_.poses.clear();
              is_paused_ = false;
              break;
            case rclcpp_action::ResultCode::ABORTED:
              RCLCPP_ERROR(this->get_logger(), "Goal ABORTED.");
              if (!is_paused_) {
                current_goal_handle_.reset();
                saved_path_.poses.clear();
              }
              break;
            case rclcpp_action::ResultCode::CANCELED:
              RCLCPP_WARN(this->get_logger(), "Goal CANCELED.");
              if (!is_paused_) {
                current_goal_handle_.reset();
                saved_path_.poses.clear();
              }
              break;
            default:
              RCLCPP_ERROR(this->get_logger(), "Goal result UNKNOWN.");
              break;
          }
        };

    action_client_->async_send_goal(goal, opts);
    RCLCPP_INFO(this->get_logger(), "Path goal sent to shelfino1.");
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_random_pose;
  rclcpp::Service<navigation_system::srv::NavigateToGoal>::SharedPtr service_specific_pose;
  rclcpp::Service<navigation_system::srv::NavigateArc>::SharedPtr service_arc_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_pause_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_stop_;
  
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;

  rclcpp_action::Client<ComputePathToPose>::SharedPtr path_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  
  geometry_msgs::msg::Pose current_pose_;
  bool has_pose_ = false;

  double max_distance_;
  double step_size_;
  std::string frame_id_;

  std::mt19937 gen_;

  bool is_paused_ = false;
  nav_msgs::msg::Path saved_path_;
  std::shared_ptr<GoalHandleFollowPath> current_goal_handle_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// ros2 service call /stop_navigation std_srvs/srv/Trigger "{}"
// ros2 service call /pause_navigation std_srvs/srv/Trigger "{}"
// ros2 service call /generate_specific_path navigation_system/srv/NavigateToGoal '{pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
// ros2 service call /navigate_arc navigation_system/srv/NavigateArc "{goal_pose: {position: {x: 5.0, y: 0., z: 0.0}}, radius: 4, center: {position: {x: 0.0, y: 0.0, z: 0.0}}}"

