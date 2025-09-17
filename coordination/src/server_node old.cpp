#include "coordination/cloud_to_voxel.hpp"
#include "coordination/publishers.hpp"
#include "coordination/robot_manager.hpp"

#include <memory>
#include <string>
#include <thread>
#include <vector>

using std::placeholders::_1;

class MeshServerNode : public rclcpp::Node
{
public:
  std_msgs::msg::ColorRGBA green, red, blue, violet, orange, white;

  MeshServerNode() : Node("mesh_server_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {

    // Parameter definitions---------------
    std::string voxel_topic_out = this->declare_parameter<std::string>("voxel_topic_out", "voxel_estimate_out");
    std::string positions_to_explore_vis = this->declare_parameter<std::string>("positions_to_explore_topic_vis", "positions_to_explore_vis");
    std::string robot_pcl_topic_ = this->declare_parameter<std::string>("pcl_topic_in", "cluster_pcl");
    robot_ids_ = this->declare_parameter<std::vector<std::string>>("robot_ids", std::vector<std::string>{"shelfino1", "pollo"});
    voxel_leaf_size_ = this->declare_parameter<float>("voxel_size", 0.05);
    maximum_count_per_voxel_ = this->declare_parameter<int>("threshold_count_per_voxel", 30);
    minimum_percentage_ = this->declare_parameter<float>("minimum_percentage", 0.1);
    radius_multiplier_ = this->declare_parameter<float>("radius_multiplier", 2.);

    hz_ = this->declare_parameter<int>("hz", 3);
    if (robot_ids_.empty())
    {
      RCLCPP_FATAL(get_logger(), "No robot_ids given!");
      rclcpp::shutdown();
      return;
    }

    // Subscribe to robot topics-------------
    for (auto robot : robot_ids_)
    {
      std::string cluster_topic_name = "/" + robot + "/" + robot_pcl_topic_;
      RCLCPP_INFO(this->get_logger(), "Setting up subscriber %s", cluster_topic_name.c_str());
      pcl_sub_vect_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
          cluster_topic_name, 10,
          [this, robot](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
          {
            // The subscribers will push the messages to the node buffer
            pcl_msg_buffer_.push(ItemMsg{std::move(msg), robot});
          }));
      robots_pcl_counter_.emplace(robot, 0);
    }

    // --- main-loop timer -------------
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(std::floor((1 / hz_) * 1000))),
        [this]()
        { server_executor(); });

    // Publishers-----------------------
    voxel_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(voxel_topic_out, 200);
    under_explored_publisher_ = create_publisher<visualization_msgs::msg::Marker>(positions_to_explore_vis, 200);
    points_generic_ = create_publisher<visualization_msgs::msg::Marker>("Generic", 200);
    circle_pub_ = create_publisher<visualization_msgs::msg::Marker>("circle_pub", 200);
    pose_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("poses", 200);
    // Variables definition-------------
    pcl_manager_ = new CloudToVoxel(voxel_leaf_size_, maximum_count_per_voxel_, minimum_percentage_);

    violet.a = 1.0f;
    violet.r = .5f;
    violet.g = .0f;
    violet.b = 0.5f;

    green.a = 1.0f;
    green.r = .0f;
    green.g = 1.f;
    green.b = 0.0f;

    red.a = 1.0f;
    red.r = .1f;
    red.g = .0f;
    red.b = 0.f;

    blue.a = 1.0f;
    blue.r = .0f;
    blue.g = .0f;
    blue.b = 1.f;

    orange.a = 1.0f;
    orange.r = 1.f;
    orange.g = .5f;
    orange.b = .0f;

    white.a = 1.0f;
    white.r = 1.f;
    white.g = 1.f;
    white.b = 1.0f;
  }

private:
  // PCL managment
  float minimum_percentage_;
  float voxel_leaf_size_, hz_;
  int maximum_count_per_voxel_;
  CloudToVoxel *pcl_manager_;
  rclcpp::TimerBase::SharedPtr timer_;
  // Robot managment---------------------------
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pcl_sub_vect_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  struct ItemMsg // Buffer structure
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    std::string robot_id;
  };
  ConcurrentQueue<ItemMsg> pcl_msg_buffer_;

  std::vector<std::string> robot_ids_;
  std::map<std::string, int> robots_pcl_counter_;
  bool estimating_;
  Polygon search_perimeter_;
  point_t center_of_the_perimeter_;
  double radius_, radius_multiplier_;
  // Visualization Publishers------------------
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr under_explored_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_generic_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr circle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_pub_;

  // #############
  // # MAIN LOOP #
  // #############
  //     Timer
  //       ╎
  //       ╎
  //       ∨
  // Consume buffer of point_clouds to update the master point_cloud
  //       ╎
  //       ╎
  //       ∨
  // All robot have published at least once ? ╶╶╶NO-╶> Don't do anything yet
  //       ╎
  //      Yes
  //       ╎
  //       ∨
  // Start voxel estimation
  void server_executor()
  {
    auto time_stamp = this->get_clock()->now();
    ItemMsg item;
    // Check if there is the possibility of starting estimation
    // based on if all the robot sent at least one pointcloud
    bool perhaps_start_estimation = true;

    while (pcl_msg_buffer_.pop(item))
    {
      std::string robot_sender = item.robot_id;

      pcl::PointCloud<point_t> received_cloud;
      // 1. Convert ROS ->PCL
      pcl::fromROSMsg(*item.msg, received_cloud);

      robots_pcl_counter_[robot_sender] = robots_pcl_counter_[robot_sender] + 1;
      // don't start to estimate until all robot have published at least one pcl

      pcl_manager_->expandPCL(received_cloud);
    }
    for (auto keyval : robots_pcl_counter_)
    {

      if (keyval.second == 0)
      {
        RCLCPP_INFO(this->get_logger(), "robot %s didn't sent anything yet, didn't start voxel estimation", keyval.first.c_str());
        perhaps_start_estimation = false;
      }
    }
    if (!estimating_ && perhaps_start_estimation)
    {
      RCLCPP_INFO(this->get_logger(), "received at least one pcl from each robot, starting voxel estimation");
      estimating_ = true;
      pcl_manager_->startEstimating();

      std::ostringstream oss;
      for (auto keyval1 : pcl_manager_->voxel_parameters_)
      {
        oss << keyval1.first << ":\n";
        for (auto keyval2 : keyval1.second)
        {
          oss << keyval2.first << " : " << keyval2.second;
        }
      }
      RCLCPP_INFO(this->get_logger(), "voxel parameters \n%s", oss.str().c_str());
      RCLCPP_INFO(this->get_logger(), "voxel Count %lu", pcl_manager_->voxel_cloud_->size());
      // Compute search radius where the robots should position themselves
      center_of_the_perimeter_ = pcl_manager_->centroid_;
      point_t p_max = pcl_manager_->p_max_;
      point_t p_min = pcl_manager_->p_min_;
      radius_ = sqrt(pow(p_max.x - center_of_the_perimeter_.x, 2) + pow(p_max.y - center_of_the_perimeter_.y, 2)) * radius_multiplier_;
      search_perimeter_ = circle_polygon(center_of_the_perimeter_.x, center_of_the_perimeter_.y, radius_, 64);
      auto robot_pose = get_robot_pose(tf_buffer_, "map", "pollo");
      const double vx = center_of_the_perimeter_.x - robot_pose.position.x;
      const double vy = center_of_the_perimeter_.y - robot_pose.position.y;

      double angle_to_center = std::atan2(vy, vx);
      //auto poses = generate_circle_poses(center_of_the_perimeter_, radius_, robot_ids_.size(), 30.0 * M_PI / 180.0);
      auto poses = generate_circle_poses(center_of_the_perimeter_, radius_, robot_ids_.size(),angle_to_center);
      publishPoligon(circle_pub_, search_perimeter_, time_stamp, violet);
      publishPoseMarkers(pose_pub_, poses, time_stamp, green);
    }

    if (estimating_)
    {
      // point_2_norm_cloud_map_t *norm = new point_2_norm_cloud_map_t();
      // pcl_manager_->getNormalizedCountPerVoxel(norm);
      pcl_manager_->voxelDensityEstimate();
      auto probability_pcl = pcl_manager_->probability_pcl_;
      auto raw_cloud = pcl_manager_->raw_cloud_;
      RCLCPP_INFO(this->get_logger(), "raw Count %lu", raw_cloud->size());
      auto points_to_check = pcl_manager_->getUnderExploredVoxels();
      std::vector<geometry_msgs::msg::Pose> poses;
      for (auto point : points_to_check)
      {
        auto pose = pose_point_to_circle(center_of_the_perimeter_,radius_,point);

        poses.emplace_back(pose);
      }
      publishPoseMarkers(pose_pub_, poses, time_stamp, green);
      publishPointMarkers(under_explored_publisher_, points_to_check, time_stamp, blue);
      publishVoxelEstimate(voxel_publisher_, probability_pcl, time_stamp);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // Multi-threaded executor so every subscription runs in its own thread
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<MeshServerNode>();
  exec.add_node(node);
  exec.spin();
  // rclcpp::spin(std::make_shared<MeshServerNode>());
  rclcpp::shutdown();
  return 0;
}
