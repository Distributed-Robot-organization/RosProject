#include <cmath>
#include <random>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rmw/qos_profiles.h"
#include "nav_msgs/msg/odometry.hpp"

#include "navigation_system/srv/navigate_to_goal.hpp"
#include "navigation_system/srv/navigate_arc.hpp"
#include "navigation_system/srv/center_point.hpp"

using FollowPath = nav2_msgs::action::FollowPath;
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;
using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

class PathGenerator : public rclcpp::Node
{
public:
  PathGenerator() : Node("navigation_system_node"), gen_(std::random_device{}())
  {
    // Declare and get robot namespace parameter
    this->declare_parameter<std::string>("robot_namespace", "shelfino1");
    robot_namespace_ = this->get_parameter("robot_namespace").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Initializing navigation system for robot: %s", robot_namespace_.c_str());

    max_distance_ = 5.0;
    step_size_ = 0.3;
    frame_id_ = "map";

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                   .reliability(rclcpp::ReliabilityPolicy::Reliable)
                   .durability(rclcpp::DurabilityPolicy::TransientLocal);

    // Services - with namespace
    service_random_pose = this->create_service<std_srvs::srv::Trigger>(
        "/" + robot_namespace_ + "/generate_random_path", std::bind(&PathGenerator::callback_random_trigger, this,
                  std::placeholders::_1, std::placeholders::_2));

    service_specific_pose = this->create_service<navigation_system::srv::NavigateToGoal>(
        "/" + robot_namespace_ + "/generate_specific_path", std::bind(&PathGenerator::callback_specific_trigger, this,
                  std::placeholders::_1, std::placeholders::_2));

    service_arc_ = this->create_service<navigation_system::srv::NavigateArc>("/" + robot_namespace_ + "/generate_arc", 
        std::bind(&PathGenerator::callback_arch_trigger, this, std::placeholders::_1, std::placeholders::_2));

    service_rotate_to_center_ = this->create_service<navigation_system::srv::CenterPoint>("/" + robot_namespace_ + "/rotate_shelfino",std::bind(&PathGenerator::callback_rotate_trigger, this,
        std::placeholders::_1, std::placeholders::_2));

    service_pause_ = this->create_service<std_srvs::srv::Trigger>("/" + robot_namespace_ + "/pause_navigation",
        std::bind(&PathGenerator::callback_pause_trigger, this, std::placeholders::_1, std::placeholders::_2));

    service_stop_ = this->create_service<std_srvs::srv::Trigger>("/" + robot_namespace_ + "/stop_navigation",
        std::bind(&PathGenerator::callback_stop_trigger, this, std::placeholders::_1, std::placeholders::_2));

    // Subscriptions - with namespace
    amcl_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/" + robot_namespace_ + "/amcl_pose", qos, std::bind(&PathGenerator::amcl_callback, this, std::placeholders::_1));
    
    // Publishers - with namespace
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/" + robot_namespace_ + "/planned_path", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + robot_namespace_ + "/cmd_vel", 10);
    tick_service_navigation_pub = this->create_publisher<std_msgs::msg::Bool>("/" + robot_namespace_ + "/navigation_system/tick_service_navigation", 10);
    
    // Action clients - with namespace
    path_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "/" + robot_namespace_ + "/compute_path_to_pose");
    action_client_ = rclcpp_action::create_client<FollowPath>(this, "/" + robot_namespace_ + "/follow_path");
    
    // Wait for action servers
    if (!path_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server /%s/compute_path_to_pose not available.", robot_namespace_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Action server /%s/compute_path_to_pose OK.", robot_namespace_.c_str());
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server /%s/follow_path not available.", robot_namespace_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Action server /%s/follow_path OK.", robot_namespace_.c_str());
    }

    // Timers
    distance_check_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
        std::bind(&PathGenerator::check_distance_to_goal, this));

    rotation_timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
        std::bind(&PathGenerator::rotation_control_loop, this));

    RCLCPP_INFO(this->get_logger(), "PathGenerator ready for robot: %s", robot_namespace_.c_str());
  }

private:
  void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    has_pose_ = true;

    if (!pose_logged_) {
      RCLCPP_INFO(this->get_logger(), "AMCL pose received: (%.2f, %.2f)", 
                  current_pose_.position.x, current_pose_.position.y);
      pose_logged_ = true;
    }
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

    RCLCPP_INFO(this->get_logger(), "Generated random goal: (%.2f, %.2f) at distance %.2f",
                goal_x, goal_y, radius);

    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = goal_x;
    goal_pose.position.y = goal_y;
    goal_pose.position.z = 0.0;

    final_goal = goal_pose.position;
    
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle);
    goal_pose.orientation = tf2::toMsg(q);
    compute_path_to_goal(goal_pose);

    response->success = true;
    response->message = "Random path computation requested.";
  }

  void callback_specific_trigger(
      const std::shared_ptr<navigation_system::srv::NavigateToGoal::Request> request,
      std::shared_ptr<navigation_system::srv::NavigateToGoal::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received specific goal: (%.2f, %.2f)", 
                request->pose.position.x, request->pose.position.y);
    final_goal = request->pose.position;
    compute_path_to_goal(request->pose);
    
    response->success = true;
    response->message = "Specific path computation requested.";
  }

  void callback_arch_trigger(
      const std::shared_ptr<navigation_system::srv::NavigateArc::Request> request,
      std::shared_ptr<navigation_system::srv::NavigateArc::Response> response)
  {
    if (!has_pose_) {
      response->success = false;
      response->message = "Robot pose not yet received from AMCL.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    
    double radius = request->radius;
    geometry_msgs::msg::Point center = request->center;
    geometry_msgs::msg::Point goal = request->goal;

    RCLCPP_INFO(this->get_logger(), "Arc navigation: goal(%.2f, %.2f), center(%.2f, %.2f), radius=%.2f",
                goal.x, goal.y, center.x, center.y, radius);

    // Project goal onto circle
    double dx_goal = goal.x - center.x;
    double dy_goal = goal.y - center.y;
    double dist_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
    
    if (dist_goal > 1e-6) {
      final_goal.x = center.x + (dx_goal / dist_goal) * radius;
      final_goal.y = center.y + (dy_goal / dist_goal) * radius;
    } else {
      final_goal.x = center.x + radius;
      final_goal.y = center.y;
    }
    final_goal.z = 0.0;
    
    // Generate and publish arc path
    nav_msgs::msg::Path path = generate_arch_path(final_goal, radius, center);
    if (path.poses.empty()) {
      response->success = false;
      response->message = "Failed to generate arc path.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Generated arc path with %zu waypoints", path.poses.size());
    path_pub_->publish(path);
    send_path_goal(path);
    response->success = true;
    response->message = "Arc path generated and navigation started.";
  }

  void callback_rotate_trigger(
    
    const std::shared_ptr<navigation_system::srv::CenterPoint::Request> request,
    std::shared_ptr<navigation_system::srv::CenterPoint::Response> response)
{
    (void)request;
    
    if (!has_pose_) {
      response->success = false;
      response->message = "Robot pose not yet received from AMCL.";
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }

    // Centro del campo
    double center_x = request->center.x;
    double center_y = request->center.y;
    
    // Calcola l'angolo verso il centro
    double dx = center_x - current_pose_.position.x;
    double dy = center_y - current_pose_.position.y;
    target_yaw_ = std::atan2(dy, dx);
    
    double current_yaw = get_yaw_from_quaternion(current_pose_.orientation);
    double angle_error = normalize_angle(target_yaw_ - current_yaw);
    
    RCLCPP_INFO(this->get_logger(), "Starting rotation to center. Current: %.2f, Target: %.2f, Error: %.2f",
                current_yaw, target_yaw_, angle_error);
    
    is_rotating_ = true;
    response->success = true;
    response->message = "Rotation to center started.";
  }

  void rotation_control_loop()
  {
    if (!is_rotating_ || !has_pose_) return;
    
    double current_yaw = get_yaw_from_quaternion(current_pose_.orientation);
    double angle_error = normalize_angle(target_yaw_ - current_yaw);
    const double angle_threshold = 0.2;
    
    if (std::abs(angle_error) < angle_threshold) {
      RCLCPP_INFO(this->get_logger(), "Rotation completed. Final error: %.3f rad", angle_error);
      
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(cmd);
      
      is_rotating_ = false;

      // Publish goal reached status
      std_msgs::msg::Bool msg;
      msg.data = true;
      tick_service_navigation_pub->publish(msg);
      return;
    }
    
    const double angular_vel = 0.3;
    double cmd_angular_z = (angle_error > 0) ? angular_vel : -angular_vel;
    
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd.angular.z = cmd_angular_z;
    cmd_vel_pub_->publish(cmd);
  }

  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double normalize_angle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  void callback_pause_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;

    if (!is_paused_) {
      if (current_goal_handle_) {
        auto cancel_future = action_client_->async_cancel_goal(current_goal_handle_);
        is_paused_ = true;
        response->success = true;
        response->message = "Navigation paused.";
        RCLCPP_INFO(this->get_logger(), "Navigation PAUSED.");
      } else {
        response->success = false;
        response->message = "No active navigation to pause.";
      }
    } else {
      if (saved_path_.poses.empty()) {
        response->success = false;
        response->message = "No saved path to resume.";
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
      response->message = "Navigation stopped.";
      RCLCPP_INFO(this->get_logger(), "Navigation STOPPED.");
    } else {
      response->success = false;
      response->message = "No active navigation to stop.";
    }
  }

  nav_msgs::msg::Path generate_arch_path(const geometry_msgs::msg::Point& goal, double radius,
                                          const geometry_msgs::msg::Point& center)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = this->get_clock()->now();

    double robot_x = current_pose_.position.x;
    double robot_y = current_pose_.position.y;

    // Project robot onto circle
    double dx_robot = robot_x - center.x;
    double dy_robot = robot_y - center.y;
    double dist_robot = std::sqrt(dx_robot * dx_robot + dy_robot * dy_robot);
    
    double start_x = (dist_robot > 1e-6) ? center.x + (dx_robot / dist_robot) * radius : center.x + radius;
    double start_y = (dist_robot > 1e-6) ? center.y + (dy_robot / dist_robot) * radius : center.y;

    // Project goal onto circle
    double dx_goal = goal.x - center.x;
    double dy_goal = goal.y - center.y;
    double dist_goal = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);
    
    double end_x = (dist_goal > 1e-6) ? center.x + (dx_goal / dist_goal) * radius : center.x + radius;
    double end_y = (dist_goal > 1e-6) ? center.y + (dy_goal / dist_goal) * radius : center.y;

    // Calculate angles and arc direction
    double start_angle = std::atan2(start_y - center.y, start_x - center.x);
    double end_angle = std::atan2(end_y - center.y, end_x - center.x);

    double angle_diff = end_angle - start_angle;
    if (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    else if (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    // Generate waypoints
    double arc_length = std::abs(angle_diff) * radius;
    int num_points = std::max(10, static_cast<int>(arc_length / 0.1));
    
    RCLCPP_INFO(this->get_logger(), "Arc: start=%.2f, end=%.2f, length=%.2fm, points=%d",
                start_angle, end_angle, arc_length, num_points);
    
    for (int i = 0; i <= num_points; ++i) {
      double t = static_cast<double>(i) / num_points;
      double current_angle = start_angle + t * angle_diff;

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = this->get_clock()->now();
      
      pose.pose.position.x = center.x + radius * std::cos(current_angle);
      pose.pose.position.y = center.y + radius * std::sin(current_angle);
      pose.pose.position.z = 0.0;
      
      double tangent_angle = current_angle + (angle_diff > 0 ? M_PI/2 : -M_PI/2);
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = std::sin(tangent_angle / 2.0);
      pose.pose.orientation.w = std::cos(tangent_angle / 2.0);

      path.poses.push_back(pose);
    }

    return path;
  }

  void check_distance_to_goal()
  {
    if (!has_pose_ || !current_goal_handle_) return;

    double dx = final_goal.x - current_pose_.position.x;
    double dy = final_goal.y - current_pose_.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= 0.15) { 
      RCLCPP_INFO(this->get_logger(), "Robot reached goal! Distance: %.3f m", distance);
      
      
      if (current_goal_handle_) {
        auto cancel_future = action_client_->async_cancel_goal(current_goal_handle_);
        current_goal_handle_.reset();
      }

      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.linear.y = 0.0;
      stop_cmd.angular.z = 0.0;
      cmd_vel_pub_->publish(stop_cmd); // Pubblica lo stop immediato

      std_msgs::msg::Bool msg;
      msg.data = true;
      tick_service_navigation_pub->publish(msg);
      
      saved_path_.poses.clear();
    }
  }

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
        [this, goal_pose](const GoalHandleComputePath::WrappedResult &result) { // Nota: ho aggiunto goal_pose nel capture
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Path computed successfully!");
            path_pub_->publish(result.result->path);
            send_path_goal(result.result->path);
          } else {
            // DEBUG AVANZATO
            std::string error_msg;
            switch(result.code) {
                case rclcpp_action::ResultCode::ABORTED: error_msg = "ABORTED (Planner blocked)"; break;
                case rclcpp_action::ResultCode::CANCELED: error_msg = "CANCELED"; break;
                case rclcpp_action::ResultCode::UNKNOWN: error_msg = "UNKNOWN"; break;
                default: error_msg = "OTHER"; break;
            }
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to compute path. Result Code: %s. Target was: (%.2f, %.2f)", 
                error_msg.c_str(), goal_pose.position.x, goal_pose.position.y);
          }
        };

    path_client_->async_send_goal(goal_msg, send_goal_options);
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

    saved_path_ = path;

    FollowPath::Goal goal;
    goal.path = path;
    goal.controller_id = "";
    goal.goal_checker_id = "";

    rclcpp_action::Client<FollowPath>::SendGoalOptions opts;
    opts.goal_response_callback = [this](std::shared_ptr<GoalHandleFollowPath> handle) {
      if (!handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal REJECTED.");
        current_goal_handle_.reset();
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal ACCEPTED.");
        current_goal_handle_ = handle;
      }
    };
    
    opts.result_callback = [this](const GoalHandleFollowPath::WrappedResult &res) {
      switch (res.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
          RCLCPP_INFO(this->get_logger(), "Goal COMPLETED.");
          
          // Publish goal completed status
          std_msgs::msg::Bool msg;
          msg.data = true;
          tick_service_navigation_pub->publish(msg);
          
          RCLCPP_INFO(this->get_logger(), "Current robot pose: (%.2f, %.2f)", current_pose_.position.x, current_pose_.position.y);
          current_goal_handle_.reset();
          saved_path_.poses.clear();
          is_paused_ = false;
          break;
        }
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
    RCLCPP_INFO(this->get_logger(), "Path goal sent.");
  }

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_random_pose;
  rclcpp::Service<navigation_system::srv::NavigateToGoal>::SharedPtr service_specific_pose;
  rclcpp::Service<navigation_system::srv::NavigateArc>::SharedPtr service_arc_;
  rclcpp::Service<navigation_system::srv::CenterPoint>::SharedPtr service_rotate_to_center_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_pause_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_stop_;
  
  // Subscribers & Publishers
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tick_service_navigation_pub;

  // Action clients
  rclcpp_action::Client<ComputePathToPose>::SharedPtr path_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr action_client_;

  // State variables
  std::string robot_namespace_;
  geometry_msgs::msg::Pose current_pose_;
  nav_msgs::msg::Path saved_path_;
  std::shared_ptr<GoalHandleFollowPath> current_goal_handle_;
  geometry_msgs::msg::Point final_goal;

  rclcpp::TimerBase::SharedPtr distance_check_timer_;
  rclcpp::TimerBase::SharedPtr rotation_timer_;

  double max_distance_;
  double step_size_;
  std::string frame_id_;
  
  bool is_rotating_ = false;
  double target_yaw_ = 0.0;
  bool is_paused_ = false;
  bool pose_logged_ = false;  
  bool has_pose_ = false;

  std::mt19937 gen_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathGenerator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// old command
// ros2 service call /stop_navigation std_srvs/srv/Trigger "{}"
// ros2 service call /pause_navigation std_srvs/srv/Trigger "{}"
// ros2 service call /rotate_shelfino std_srvs/srv/Trigger "{}"
// ros2 service call /generate_random_path std_srvs/srv/Trigger "{}"
// ros2 service call /generate_specific_path navigation_system/srv/NavigateToGoal '{pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
// ros2 service call /generate_arc navigation_system/srv/NavigateArc "{goal: {x: 4.0, y: 0.0, z: 0.0}, center: {x: 0.0, y: 0.0, z: 0.0}, radius: 4.0}"

// new command
// ros2 service call /shelfino1/stop_navigation std_srvs/srv/Trigger "{}"
// ros2 service call /shelfino1/pause_navigation std_srvs/srv/Trigger "{}"
// ros2 service call /shelfino1/rotate_shelfino std_srvs/srv/Trigger "{}"
// ros2 service call /shelfino1/generate_random_path std_srvs/srv/Trigger "{}"
// ros2 service call /shelfino1/generate_specific_path navigation_system/srv/NavigateToGoal '{pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
// ros2 service call /shelfino1/generate_arc navigation_system/srv/NavigateArc "{goal: {x: 4.0, y: 0.0, z: 0.0}, center: {x: 0.0, y: 0.0, z: 0.0}, radius: 4.0}"
