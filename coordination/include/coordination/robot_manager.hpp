#include <cmath>
#include <iostream>
#include "coordination/server_types.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose.hpp"

#ifndef ROBOT_MANAGER_H
#define ROBOT_MANAGER_H
Polygon circle_polygon(double cx, double cy, double radius, int num_segments = 64);
geometry_msgs::msg::Pose pose_point_to_circle(
    const point_t &center,
    double radius,
    const point_t &point);

std::vector<geometry_msgs::msg::Pose> generate_circle_poses(
    const point_t &center,
    double radius,
    int num_poses,
    double offset_rad);

geometry_msgs::msg::Pose get_robot_pose(
    const tf2_ros::Buffer &buffer,
    const std::string &target_frame,
    const std::string &robot_name);

// -------------------- thread-safe queue --------------------
template <typename T>
class ConcurrentQueue // Buffer class to manage multiple subsciptions
{
public:
  void push(T item)
  {
    std::lock_guard<std::mutex> lock(m_);
    q_.push_back(std::move(item));
  }

  bool pop(T &out)
  {
    std::lock_guard<std::mutex> lock(m_);
    if (q_.empty())
      return false;
    out = std::move(q_.front());
    q_.pop_front();
    return true;
  }

private:
  std::mutex m_;
  std::deque<T> q_;
};
#endif