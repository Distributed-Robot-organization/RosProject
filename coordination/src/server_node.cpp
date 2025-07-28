#include "coordination/cloud_to_voxel.hpp"
//#include "interfaces_pkg/publishers.hpp"
#include <memory>
#include <string>
#include <thread>
#include <vector>

using std::placeholders::_1;

//     Timer
//      ╎
//      ╎
//      ∨ 
//Consume buffer of point_clouds to update the master pointcloud
//     ╎
//     ╎
//     ∨
// All robot have published at least once ? ╶╶╶NO-╶> Don't do anything yet
//       ╎
//      Yes
//       ╎
//       ∨ 
//Start voxel estimation

void publishMarkers(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
                    const std::vector<point_t> points,
                    builtin_interfaces::msg::Time stamp,
                    std_msgs::msg::ColorRGBA color)
{
  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header.frame_id = "map"; // your fixed frame
  marker_msg.header.stamp = stamp;
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;

  // Define the scale of the points (size)
  marker_msg.scale.x = 0.1; // width of points
  marker_msg.scale.y = 0.1; // height of points

  // Color RGBA (red here)
  marker_msg.color = color; // alpha (opacity)
  geometry_msgs::msg::Point p_msg;
  for (const auto &p : points)
  {
    p_msg.x = p.x;
    p_msg.y = p.y;
    p_msg.z = p.z;
    marker_msg.points.push_back(p_msg);
  }
  pub->publish(marker_msg);
}

void publishVoxelEstimate(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<PointXYZProb>::Ptr normalized_pcl,
                          builtin_interfaces::msg::Time stamp)
{

  // normalized vector between 0 and 1
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr norm_vox_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>((*normalized_pcl).width, (*normalized_pcl).height));
  for (auto p : *normalized_pcl)
  {
    try
    {
      pcl::PointXYZRGBA p_out;
      p_out.x = p.x;
      p_out.y = p.y;
      p_out.z = p.z;
      p_out.g = uint8_t(255);
      p_out.b = uint8_t(255);
      p_out.r = uint8_t(255);
      p_out.a = static_cast<uint8_t>(std::floor(p.probability * 254));
      norm_vox_cloud->push_back(p_out);
      std::cout << "punto " << std::floor(p.probability * 254) << " prob" << p.probability << std::endl;
    }
    catch (std::out_of_range &ex)
    {
      std::ostringstream oss;
      oss << "std::out_of_range for reading point " << p.x << " " << p.y << " " << p.z << "Description: " << ex.what();
      throw std::runtime_error(oss.str());
    }
  }

  sensor_msgs::msg::PointCloud2::SharedPtr ros_msg(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*norm_vox_cloud, *ros_msg);
  ros_msg->header.frame_id = "map";
  ros_msg->header.stamp = stamp;
  pub->publish(*ros_msg);
}

void publishVoxelEstimate(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub,
                          const pcl::PointCloud<point_t>::Ptr cloud_voxel,
                          const point_2_norm_cloud_map_t normalized_count_per_voxel,
                          builtin_interfaces::msg::Time stamp)
{

  // normalized vector between 0 and 1
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr norm_vox_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>((*cloud_voxel).width, (*cloud_voxel).height));
  for (auto p : *cloud_voxel)
  {
    try
    {
      pcl::PointXYZRGBA p_out;
      p_out.x = p.x;
      p_out.y = p.y;
      p_out.z = p.z;
      p_out.g = uint8_t(255);
      p_out.b = uint8_t(255);
      p_out.r = uint8_t(255);
      p_out.a = static_cast<uint8_t>(std::floor((normalized_count_per_voxel.at(std::tuple<float, float, float>(p.x, p.y, p.z))) * 255));
      norm_vox_cloud->push_back(p_out);
    }
    catch (std::out_of_range &ex)
    {
      std::ostringstream oss;
      oss << "std::out_of_range for reading point " << p.x << " " << p.y << " " << p.z << "Description: " << ex.what();
      throw std::runtime_error(oss.str());
    }
  }

  sensor_msgs::msg::PointCloud2::SharedPtr ros_msg(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*norm_vox_cloud, *ros_msg);
  ros_msg->header.frame_id = "map";
  ros_msg->header.stamp = stamp;
  pub->publish(*ros_msg);
}



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

class MeshServerNode : public rclcpp::Node
{
public:
  MeshServerNode() : Node("mesh_server_node")
  {
    std::string voxel_topic_out = this->declare_parameter<std::string>("voxel_topic_out", "voxel_estimate_out");
    std::string positions_to_explore_vis = this->declare_parameter<std::string>("positions_to_explore_topic_vis", "positions_to_explore_vis");

    std::string robot_pcl_topic_ = this->declare_parameter<std::string>("pcl_topic_in", "cluster_pcl");
    robot_ids_ = this->declare_parameter<std::vector<std::string>>("robot_ids", std::vector<std::string>{"shelfino1", "pollo"});
    voxel_leaf_size_ = this->declare_parameter<float>("voxel_size", 0.05);
    threshold_count_per_voxel_ = this->declare_parameter<int>("threshold_count_per_voxel_", 30);
    minimum_count_per_voxel_ = this->declare_parameter<int>("minumum_count", 30);
    

        hz_ = this->declare_parameter<int>("hz", 3);
    if (robot_ids_.empty())
    {
      RCLCPP_FATAL(get_logger(), "No robot_ids given!");
      rclcpp::shutdown();
      return;
    }

    pcl_manager_ = new CloudToVoxel(voxel_leaf_size_, threshold_count_per_voxel_, minimum_count_per_voxel_);

    for (auto robot : robot_ids_)
    {
      std::string cluster_topic_name = "/" + robot + "/" + robot_pcl_topic_;
      RCLCPP_INFO(this->get_logger(), "Setting up subscriber %s", cluster_topic_name.c_str());
      pcl_sub_vect_.push_back(this->create_subscription<sensor_msgs::msg::PointCloud2>(
          cluster_topic_name, 10,
          [this, robot](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
          {
            // The subscribers will push the messages to the node buffer
            buffer_.push(ItemMsg{std::move(msg), robot});
          }));
      robots_pcl_it_.emplace(robot, 0);
    }

    // Optional: publish the voxelised cloud
    voxel_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(voxel_topic_out, 200);
    under_explored_publisher_ = create_publisher<visualization_msgs::msg::Marker>(positions_to_explore_vis,200);

    // --- main-loop timer (e.g. 20 Hz) ---
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(std::floor((1 / hz_) * 1000))),
        [this]()
        { server_executor(); });
  }

private:
  struct ItemMsg // Buffer structure
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    std::string robot_id;
  };
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pcl_sub_vect_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr under_explored_publisher_;

  std::vector<std::string> robot_ids_;
  std::map<std::string, int> robots_pcl_it_;
  bool estimating_;
  float voxel_leaf_size_, hz_;
  int threshold_count_per_voxel_, minimum_count_per_voxel_;
  CloudToVoxel *pcl_manager_;
  ConcurrentQueue<ItemMsg> buffer_;
  rclcpp::TimerBase::SharedPtr timer_;

  // #############
  // # MAIN LOOP #
  // #############  
  void server_executor()
  {
    ItemMsg item;
    bool perhaps_start_estimation = true;
    std_msgs::msg::Header header;

    while (buffer_.pop(item))
    {
      std::string robot_sender = item.robot_id;

      pcl::PointCloud<point_t> received_cloud;
      // 1. Convert ROS → PCL
      pcl::fromROSMsg(*item.msg, received_cloud);

      robots_pcl_it_[robot_sender] = robots_pcl_it_[robot_sender] + 1;
      header = item.msg->header; // TODO: da rimpiazzare con valori sensati;
      // don't start to estimate until all robot have published at least one pcl

      pcl_manager_->expandPCL(received_cloud);
    }
    for (auto keyval : robots_pcl_it_)
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
    }

    if (estimating_)
    {
      //point_2_norm_cloud_map_t *norm = new point_2_norm_cloud_map_t();
      //pcl_manager_->getNormalizedCountPerVoxel(norm);
      pcl::PointCloud<PointXYZProb>::Ptr probability_pcl = pcl_manager_->probability_pcl_;
      pcl::PointCloud<point_t>::Ptr raw_cloud = pcl_manager_->raw_cloud_;
      RCLCPP_INFO(this->get_logger(), "raw Count %lu", raw_cloud->size());
      publishVoxelEstimate(voxel_publisher_, probability_pcl, this->get_clock()->now());
      std::vector<point_t> points_to_check = pcl_manager_->getUnderExploredVoxels();
      std_msgs::msg::ColorRGBA centroid_color;
                    centroid_color.a = 1.0f;
                    centroid_color.r = .5f;
                    centroid_color.g = .5f;
                    centroid_color.b = 0.5f;

      publishMarkers(under_explored_publisher_, points_to_check,this->get_clock()->now(),centroid_color);
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