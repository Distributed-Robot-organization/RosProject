#include "coordination/robot_manager.hpp"

Polygon circle_polygon(double cx, double cy, double radius, int num_segments)
{
  Polygon poly;
  auto &outer = poly.outer();
  outer.reserve(num_segments + 1);

  for (int i = 0; i <= num_segments; i++)
  {
    double theta = 2.0 * M_PI * double(i) / double(num_segments);
    double x = cx + radius * cos(theta);
    double y = cy + radius * sin(theta);
    outer.emplace_back(x, y);
  }

  return poly;
}

geometry_msgs::msg::Pose pose_point_to_circle(
    const point_t &center,
    double radius,
    const point_t &point)
{
  const double px = point.x;
  const double py = point.y;

  const double dx = px - center.x;
  const double dy = py - center.y;
  const double dist = std::sqrt(dx * dx + dy * dy);

  if (dist == 0.0)
  {
    throw std::runtime_error("Pose coincides with circle center — direction undefined.");
  }

  // Unit vector from center to robot pose
  const double ux = dx / dist;
  const double uy = dy / dist;

  // Intersection on circle boundary
  geometry_msgs::msg::Pose result;
  result.position.x = center.x + radius * ux;
  result.position.y = center.y + radius * uy;
  result.position.z = 0.0;

  // Vector from intersection back to center
  const double vx = center.x - result.position.x;
  const double vy = center.y - result.position.y;

  // Angle of that vector
  const double angle = std::atan2(vy, vx);

  // Convert angle into quaternion (yaw only, 2D case)
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, angle);

  result.orientation.x = q.x();
  result.orientation.y = q.y();
  result.orientation.z = q.z();
  result.orientation.w = q.w();

  return result;
}

std::vector<geometry_msgs::msg::Pose> generate_circle_poses(
    const point_t &center,
    double radius,
    int num_poses,
    double offset_rad)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(num_poses);

  const double angle_step = 2.0 * M_PI / static_cast<double>(num_poses);

  for (int i = 0; i < num_poses; ++i)
  {
    double theta = offset_rad + i * angle_step;

    geometry_msgs::msg::Pose pose;
    pose.position.x = center.x + radius * std::cos(theta);
    pose.position.y = center.y + radius * std::sin(theta);
    pose.position.z = 0.0;

    // Vector pointing from pose to center
    double vx = center.x - pose.position.x;
    double vy = center.y - pose.position.y;
    double angle_to_center = std::atan2(vy, vx);

    // Orientation = pointing toward circle center
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle_to_center);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    poses.push_back(pose);
  }

  return poses;
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
  std::string robot_frame = robot_name+"/base_link";
  while (!buffer.canTransform(target_frame, robot_frame, now, timeout))
  {
    waited+=sleeptime;

    std::cout<<"Waiting "<<waited<<" seconds for "<<robot_frame<< std::endl;

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
