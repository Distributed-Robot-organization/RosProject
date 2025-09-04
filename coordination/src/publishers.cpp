#include "coordination/publishers.hpp"

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
      //std::cout << "punto " << std::floor(p.probability * 254) << " prob" << p.probability << std::endl;
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

void publishPoligon(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub,
    Polygon poly,
    builtin_interfaces::msg::Time stamp,
    std_msgs::msg::ColorRGBA color){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";  // or "world", adjust as needed
        marker.header.stamp =stamp;
        marker.ns = "circle";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.05;  // line width
        marker.color= color;
        for (auto const& p : poly.outer()) {
            geometry_msgs::msg::Point pt;
            pt.x = p.x();
            pt.y = p.y();
            pt.z = 0.0;
            marker.points.push_back(pt);
        }

        pub->publish(marker);
    }

