#include "coordination/cloud_to_voxel.hpp"
#include "coordination/publishers.hpp"
#include "coordination/robot_manager.hpp"


#include <memory>
#include <string>
#include <thread>
#include <vector>
using std::placeholders::_1;
using namespace std::chrono_literals;

class MeshServerNode : public rclcpp::Node
{
public:
  std_msgs::msg::ColorRGBA green, red, blue, violet, orange, white;

  MeshServerNode() : Node("mesh_server_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {

    // Parameter definitions---------------
    std::string voxel_topic_out = this->declare_parameter<std::string>("topics.voxel_topic_out", "voxel_estimate_out");
    std::string positions_to_explore_vis = this->declare_parameter<std::string>("topics.positions_to_explore_topic_vis", "positions_to_explore_vis");
    std::string robot_pcl_topic_ = this->declare_parameter<std::string>("server.pcl_topic_in", "cluster_pcl");
    robot_ids_ = this->declare_parameter<std::vector<std::string>>("init_names", std::vector<std::string>{"shelfino1", "pollo"});
    voxel_leaf_size_ = this->declare_parameter<float>("server.voxel_size", 0.05);
    threshold_count_per_voxel_ = this->declare_parameter<int>("server.threshold_count_per_voxel", 30);
    minimum_percentage_ = this->declare_parameter<float>("server.minimum_percentage", 0.1);
    radius_multiplier_ = this->declare_parameter<float>("server.radius_multiplier", 2.);
    world_frame_ = this->declare_parameter<std::string>("world_frame", "map");

    hz_ = this->declare_parameter<int>("server.hz", 3);
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
    pcl_manager_ = new CloudToVoxel(voxel_leaf_size_, threshold_count_per_voxel_, minimum_percentage_);

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
    // Starting check-----
    startupRosCheck();
  }

private:
  // PCL managment
  float minimum_percentage_;
  float voxel_leaf_size_, hz_;
  int threshold_count_per_voxel_;
  CloudToVoxel *pcl_manager_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time time_stamp_;
  // Robot managment---------------------------
  std::string first_discoverer_;
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

  Polygon search_perimeter_;
  point_t center_of_the_perimeter_;
  double radius_, radius_multiplier_;
  // Decision flags------------------------
  bool object_found_ = false,
       fleet_is_warned_ = false,
       sketch_scan_done_ = false,
       first_full_scan_completed_ = false,
       estimating_voxel_ = false,
       satisfied_ = false;
  // Visualization Publishers------------------
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr under_explored_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_generic_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr circle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pose_pub_;
  // ROS parameters-------------
  std::string world_frame_;

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
    time_stamp_ = this->get_clock()->now();
    ItemMsg item;
    bool pcl_was_updated_ = false;
    while (pcl_msg_buffer_.pop(item))
    {
      std::string robot_sender = item.robot_id;

      pcl::PointCloud<point_t> received_cloud;
      // 1. Convert ROS ->PCL
      pcl::fromROSMsg(*item.msg, received_cloud);

      robots_pcl_counter_[robot_sender] = robots_pcl_counter_[robot_sender] + 1;
      // don't start to estimate until all robot have published at least one pcl

      pcl_manager_->expandPCL(received_cloud);
      // if at least a PCL is sent, this means that the object was found
      if (!object_found_)
      {
        first_discoverer_ = robot_sender;
        object_found_ = true;
      }
      pcl_was_updated_ = true;
    }
    // Is useless to update the pcl if there are any updates
    if (object_found_ && pcl_was_updated_)
    {
      RCLCPP_INFO(this->get_logger(), "raw point Count %lu", pcl_manager_->raw_cloud_->size());

      if (!sketch_scan_done_)
      {
        // Without a sketch scan of the object a first point cloud received from all the robots,
        // such that we have different perspective of same object,
        // is very difficult to obtain a general bounding box

        if (!fleet_is_warned_)
        {
          // One robot has found the object the fleet must be warned
          warnFleet();
          fleet_is_warned_ = true;
        }
        else
        {
          // wait until all robots sent a PCL
          bool start_estimation = true;
          for (auto keyval : robots_pcl_counter_)
          {
            if (keyval.second == 0)
            {
              RCLCPP_INFO(this->get_logger(), "robot %s didn't sent anything yet, didn't start voxel estimation", keyval.first.c_str());
              start_estimation = false;
            }
          }
          if (start_estimation)
          {
            // all robots sent a PCL, we can start the voxel estimation and have a general BBox
            startEstimating();
            sketch_scan_done_ = true;
          }
        }
      }
      else
      {
        // if the sketch scan is completed we can start the procedure of full scan
        // The full scan is a scan all around the object to detect at least all the interesting
        // voxels.
        first_full_scan_completed_ = true; // TODO:REMOVE and define full scan procedure
        if (!first_full_scan_completed_)
        {
        }
        else
        {
          // Now we only need to scan the most uncertain parts of the object
          // TODO: add a waiting list to be sure of sending requests only when all robots completed their scan
          pcl_manager_->voxelDensityEstimate();
          if (!pcl_manager_->isEstimateSatified())
          {
            sendUnderExplored();
          }
        }
      }
    }
  };

  void sendUnderExplored()
  {
    // TODO: Refine the UnderExplored to set a k-NN and define as many clusters as robot_ids
    auto points_to_check = pcl_manager_->getUnderExploredVoxels();
    auto probability_pcl = pcl_manager_->probability_pcl_;

    std::vector<geometry_msgs::msg::Pose> poses;
    for (auto point : points_to_check)
    {
      auto pose = pose_point_to_circle(center_of_the_perimeter_, radius_, point);

      poses.emplace_back(pose);
    }
    publishPoseMarkers(pose_pub_, poses, time_stamp_, green);
    publishPointMarkers(under_explored_publisher_, points_to_check, time_stamp_, blue);
    publishVoxelEstimate(voxel_publisher_, probability_pcl, time_stamp_);
  }

  // Get the cluster main and first positions to send to all the robots in the fleet
  void warnFleet()
  {
    point_t p_max, p_min, centroid;
    pcl_manager_->getBBoxParameters(p_max, p_min, centroid);
    radius_ = sqrt(pow(p_max.x - centroid.x, 2) + pow(p_max.y - centroid.y, 2)) * radius_multiplier_;
    search_perimeter_ = circle_polygon(centroid.x, centroid.y, radius_, 64);
    auto robot_pose = get_robot_pose(tf_buffer_, world_frame_, first_discoverer_, this->get_clock()->now());
    const double vx = centroid.x - robot_pose.position.x;
    const double vy = centroid.y - robot_pose.position.y;
    double angle_to_center = std::atan2(vy, vx);
    // auto poses = generate_circle_poses(centroid, radius_, robot_ids_.size(), 30.0 * M_PI / 180.0);
    auto poses = generate_circle_poses(centroid, radius_, robot_ids_.size(), angle_to_center);
    publishPoligon(circle_pub_, search_perimeter_, time_stamp_, violet);
    publishPoseMarkers(pose_pub_, poses, time_stamp_, green);
    RCLCPP_INFO(this->get_logger(), "The fleet is warned to go towards the object advertised by %s", std::string(first_discoverer_));
  }
  void startEstimating()
  {
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
  }

  void startupRosCheck()
  {
    // Some times the node will not connect correctly so these are basic checks to the transform tree
    rclcpp::sleep_for(2s);
  
    for (auto id : robot_ids_)
    {
      std::string robot_frame = id + "/base_link";
      rclcpp::Duration timeout = rclcpp::Duration::from_seconds(.5);

      float waited = 0.0;
      int attempts = 3;
      while (!tf_buffer_.canTransform(world_frame_, robot_frame, this->get_clock()->now(), timeout))
      {
        waited += .5;

        std::cout << "Waiting " << waited << " seconds for " << robot_frame << std::endl;

        if (attempts-- <= 0)
        {
          throw std::runtime_error("TF lookup failed: Tried to wait for transform to no avail, try restarting the node");
        }
      }
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // Multi-threaded executor so every subscription runs in its first_discoverer_own thread
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<MeshServerNode>();
  exec.add_node(node);
  exec.spin();
  // rclcpp::spin(std::make_shared<MeshServerNode>());
  rclcpp::shutdown();
  return 0;
}
