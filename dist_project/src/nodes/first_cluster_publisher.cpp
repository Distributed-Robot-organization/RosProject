#include <rclcpp/rclcpp.hpp>
#include <interfaces_pkg/msg/robot_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

class ClusterConvergence : public rclcpp::Node
{
public:
    ClusterConvergence()
        : Node("pose_publisher", rclcpp::NodeOptions())
    {
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");
        robot_id_ = this->declare_parameter<std::string>("robot_id", "robot_id");
        std::string topic_f_cluster_out = this->declare_parameter<std::string>("cluster_pos_pub", "f_cluster_out");
        std::string topic_f_cluster_in = this->declare_parameter<std::string>("f_cluster_in", "cluster_med_point");
        cloud_point_subscriber_ = this->create_subscription<visualization_msgs::msg::Marker>(topic_f_cluster_in, 10, std::bind(&ClusterConvergence::subscribe_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<interfaces_pkg::msg::RobotPose>(topic_f_cluster_out, 1);
    }

private:
    std::string robot_id_;
    rclcpp::Publisher<interfaces_pkg::msg::RobotPose>::SharedPtr publisher_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr cloud_point_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    visualization_msgs::msg::Marker received_message_;
    bool sending_cloud_ = false;

    void subscribe_callback(const visualization_msgs::msg::Marker cluster_point)
    {
        if (!sending_cloud_){
            received_message_ = cluster_point;
            sending_cloud_ = true;
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&ClusterConvergence::timer_callback, this));
        }
    }

    void timer_callback()
    {
        auto message = interfaces_pkg::msg::RobotPose();
        message.robot_id = robot_id_;

        message.pose.header.stamp = this->now();
        message.pose.header.frame_id = "map";

        message.pose.pose.position.x = received_message_.points[0].x;
        message.pose.pose.position.y = received_message_.points[0].y;
        message.pose.pose.position.z = 0;

        double yaw = 1.57; // 90 deg in rad

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);

        message.pose.pose.orientation.x = q.x();
        message.pose.pose.orientation.y = q.y();
        message.pose.pose.orientation.z = q.z();
        message.pose.pose.orientation.w = q.w();

        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published pose by %s", robot_id_.c_str());
    }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ClusterConvergence>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}