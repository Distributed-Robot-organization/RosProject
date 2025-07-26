#include "coordination/cloud_to_voxel.hpp"
#include <memory>
#include <string>
#include <thread>
#include <vector>

using std::placeholders::_1;

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
    std::string robot_pcl_topic_ = this->declare_parameter<std::string>("pcl_topic_in", "cluster_pcl");
    robot_ids_ = this->declare_parameter<std::vector<std::string>>("robot_ids", std::vector<std::string>{"shelfino1", "pollo"});
    voxel_leaf_size_ = this->declare_parameter<float>("voxel_size", 0.05);
    if (robot_ids_.empty())
    {
      RCLCPP_FATAL(get_logger(), "No robot_ids given!");
      rclcpp::shutdown();
      return;
    }

    pcl_manager_ = new CloudToVoxel(voxel_leaf_size_);

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
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(voxel_topic_out, 200);

    // --- main-loop timer (e.g. 20 Hz) ---
    timer_ = create_wall_timer(
        std::chrono::seconds(3),
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
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  std::vector<std::string> robot_ids_;
  std::map<std::string, int> robots_pcl_it_;
  bool estimating_;
  float voxel_leaf_size_;
  CloudToVoxel *pcl_manager_;
  ConcurrentQueue<ItemMsg> buffer_;
  rclcpp::TimerBase::SharedPtr timer_;

  void
  server_executor()
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
      for(auto keyval1 :pcl_manager_->voxel_parameters_){
        oss << keyval1.first << ":\n";
        for (auto keyval2 : keyval1.second)
        {
          oss << keyval2.first << " : "<<keyval2.second;
        }
      }
      RCLCPP_INFO(this->get_logger(), "voxel parameters \n%s", oss.str().c_str());
      RCLCPP_INFO(this->get_logger(), "voxel Count %lu", pcl_manager_->voxel_cloud_->size());
    }

    if (estimating_)
    {
      point_2_norm_cloud_map_t *norm = new point_2_norm_cloud_map_t();
      pcl_manager_->getNormalizedCountPerVoxel(norm);
      pcl::PointCloud<point_t>::Ptr raw_cloud = pcl_manager_->raw_cloud_;
      RCLCPP_INFO(this->get_logger(), "raw Count %lu", raw_cloud->size());
      publishVoxelEstimate(pub_, pcl_manager_->voxel_cloud_, *norm, this->get_clock()->now());
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
