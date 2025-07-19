/**
 * @file minimal_example_node.cpp
 * @author Adrian Sochaniwsky (sochania@mcmaster.ca)
 * @brief Minimal working example of PCL and ROS 2
 * @version 0.1
 * @date 2023-03-22
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <exception>
#include <stdexcept>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

typedef pcl::PointXYZRGB pcl_t;

/**
 * @brief Creates an object that will subscribe to a pointCloud2 topic and
 *        performs various basic tasks.
 *
 */

bool enforceNormalOrIntensitySimilarity(const pcl::PointXYZRGBNormal &point_a, const pcl::PointXYZRGBNormal &point_b, float /*squared_distance*/)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(), point_b_normal = point_b.getNormalVector3fMap();
    if (std::abs(point_a_normal.dot(point_b_normal)) > std::cos(30.0f / 180.0f * static_cast<float>(M_PI)))
        return (true);
    return (false);
}
class MinimalPointCloudProcessor : public rclcpp::Node
{
public:
    MinimalPointCloudProcessor()
        : Node("minimal_point_cloud_processor", rclcpp::NodeOptions()
                                                    .allow_undeclared_parameters(true)
                                                    .automatically_declare_parameters_from_overrides(true))
    {
        /*
         * SET UP PUBLISHERS
         */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");

        clustered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered", 1);
        segmented_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("segmented", 1);
        pre_filter_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pre_filter", 1);
        centroid_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("centroid", 1);
        median_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("median", 1);

        /*
         * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
         */
        rclcpp::Parameter cloud_topic_param,
            world_frame_param, camera_frame_param, voxel_leaf_size_param,
            cluster_tolerance_param, min_cluster_size_param, max_cluster_size_param, plane_max_tree_iterations_param, plane_distance_treshold_param,
            z_filter_max_param, max_camera_depth_param;

        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        this->get_parameter_or("cloud_topic", cloud_topic_param, rclcpp::Parameter("", "/points"));
        this->get_parameter_or("world_frame", world_frame_param, rclcpp::Parameter("", "laser_data_frame"));
        this->get_parameter_or("camera_frame", camera_frame_param, rclcpp::Parameter("", "laser_data_frame"));
        this->get_parameter_or("max_camera_depth", max_camera_depth_param, rclcpp::Parameter("", 8.0));

        this->get_parameter_or("cluster_tolerance", cluster_tolerance_param, rclcpp::Parameter("", 0.02));
        this->get_parameter_or("min_cluster_size", min_cluster_size_param, rclcpp::Parameter("", 100));
        this->get_parameter_or("max_cluster_size", max_cluster_size_param, rclcpp::Parameter("", 99000));
        this->get_parameter_or("plane_max_tree_iterations_max", plane_max_tree_iterations_param, rclcpp::Parameter("", 100));
        this->get_parameter_or("plane_distance_treshold", plane_distance_treshold_param, rclcpp::Parameter("", 0.02));

        this->get_parameter_or("z_filter_max", z_filter_max_param, rclcpp::Parameter("", 8.0));
        this->get_parameter_or("voxel_leaf_size", voxel_leaf_size_param, rclcpp::Parameter("", 0.25));

        cloud_topic = cloud_topic_param.as_string();
        world_frame = world_frame_param.as_string();
        camera_frame = camera_frame_param.as_string();
        voxel_leaf_size = float(voxel_leaf_size_param.as_double());
        cluster_tolerance = cluster_tolerance_param.as_double();
        min_cluster_size = min_cluster_size_param.as_int();
        max_cluster_size = max_cluster_size_param.as_int();
        plane_max_tree_iterations = plane_max_tree_iterations_param.as_int();
        plane_distance_treshold = plane_distance_treshold_param.as_double();
        z_filter_max = z_filter_max_param.as_double();
        max_camera_depth = max_camera_depth_param.as_double();
        /*
         * SET UP SUBSCRIBER
         */
        RCLCPP_INFO(this->get_logger(), "Setting up subscriber");

        cloud_subscriber_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                cloud_topic, 1, std::bind(&MinimalPointCloudProcessor::cloud_callback, this, std::placeholders::_1));

        /*
         * SET UP TF. Optional for transforming between coordinate frames
         *          You need to create a static tranform publisher to use this
         */
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    /*
     * Subscriber and Publisher declaration
     */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr segmented_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_filter_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr centroid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr median_pub_;

    /*
     * Parameters
     */
    std::string cloud_topic;
    std::string world_frame;
    std::string camera_frame;

    float voxel_leaf_size;
    float cluster_tolerance;

    int min_cluster_size;
    int max_cluster_size, plane_max_tree_iterations;
    float plane_distance_treshold, z_filter_max;
    float max_camera_depth;

    /*
     * TF
     */
    std::unique_ptr<tf2_ros::Buffer>
        tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    /**
     * @brief Callback when PointCloud2 message is received. Processes the data and
     *      publishes intermediate results. Make sure this callback
     *      executes faster than the rate the message arrive.
     *
     * @param recent_cloud
     *
     * @result Published intermediate PointCloud2 results
     */
    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
    {
        //-------------------------------Filtering far away points
        // Use for timing callback execution time
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<Eigen::Vector4f> centroid_vect;

        // Transform for pointcloud in world frame
        geometry_msgs::msg::TransformStamped stransform;

        // Convert to PCL cloud in camera frame
        pcl::PointCloud<pcl_t> non_tf_cloud;
        pcl::fromROSMsg(*recent_cloud, non_tf_cloud);

        // Filter points by distance in camera frame
        pcl::PointCloud<pcl_t>::Ptr filtered_cloud(new pcl::PointCloud<pcl_t>());
        float max_distance = max_camera_depth - 0.1;
        for (const auto &point : non_tf_cloud)
        {
            // float dist = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            if (point.z <= max_distance)
            {
                filtered_cloud->points.push_back(point);
            }
        }
        filtered_cloud->width = filtered_cloud->points.size();
        filtered_cloud->height = 1;
        filtered_cloud->is_dense = true;

        // Convert filtered cloud back to ROS msg for transform
        sensor_msgs::msg::PointCloud2 n_tf_filtered_msg;
        pcl::toROSMsg(*filtered_cloud, n_tf_filtered_msg);
        n_tf_filtered_msg.header = recent_cloud->header;

        //-------------------Transforming points in frame
        try
        {
            stransform = tf_buffer_->lookupTransform(world_frame, recent_cloud->header.frame_id,
                                                     tf2::TimePointZero, tf2::durationFromSec(3));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }
        // Transform filtered cloud to world frame
        sensor_msgs::msg::PointCloud2 tf_filtered_msg;
        pcl_ros::transformPointCloud(world_frame, stransform, n_tf_filtered_msg, tf_filtered_msg);

        // Convert ROS message to PCL type
        pcl::PointCloud<pcl_t> tf_filtered_pcl;
        pcl::fromROSMsg(tf_filtered_msg, tf_filtered_pcl);
        pcl::PointCloud<pcl_t>::Ptr tf_filtered_pcl_ptr(new pcl::PointCloud<pcl_t>(tf_filtered_pcl));

        /* ========================================
         * GAUSSIAN CLUSTERING
         * ========================================*/
        pcl::PointCloud<pcl_t>::Ptr cloud_f(new pcl::PointCloud<pcl_t>);

        // // Create the filtering object: downsample the dataset using a leaf size of 1cm
        // pcl::VoxelGrid<pcl_t> vg;
        // pcl::PointCloud<pcl_t>::Ptr cloud_filtered(new pcl::PointCloud<pcl_t>);
        // vg.setInputCloud(cloud);
        // vg.setLeafSize(0.01f, 0.01f, 0.01f);
        // vg.filter(*cloud_filtered);

        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl_t> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointCloud<pcl_t>::Ptr cloud_plane(new pcl::PointCloud<pcl_t>());

        int nr_points = (int)tf_filtered_pcl_ptr->size();

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(plane_max_tree_iterations);
        seg.setDistanceThreshold(plane_distance_treshold);
        while (tf_filtered_pcl_ptr->size() > 0.1 * nr_points)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud(tf_filtered_pcl_ptr);
            seg.segment(*inliers, *coefficients);
            if (inliers->indices.size() == 0)
            {
                // std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<pcl_t> extract;
            extract.setInputCloud(tf_filtered_pcl_ptr);
            extract.setIndices(inliers);
            extract.setNegative(false);
            // Get the points associated with the planar surface
            extract.filter(*cloud_plane);
            //std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;
            // Remove the planar inliers, extract the rest
            extract.setNegative(true);
            extract.filter(*cloud_f);
            *tf_filtered_pcl_ptr = *cloud_f;
        }

        if (tf_filtered_pcl_ptr->size() != 0)
        {
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl_t>::Ptr tree(new pcl::search::KdTree<pcl_t>);
            pcl::EuclideanClusterExtraction<pcl_t> ec;
            std::vector<pcl::PointIndices> cluster_indices;
            // There are no point remaining in the point cloud to clusterize
            tree->setInputCloud(tf_filtered_pcl_ptr);

            ec.setClusterTolerance(cluster_tolerance); // 2cm
            ec.setMinClusterSize(min_cluster_size);

            ec.setMaxClusterSize(max_cluster_size);
            ec.setSearchMethod(tree);
            ec.setInputCloud(tf_filtered_pcl_ptr);
            ec.extract(cluster_indices);

            pcl::PointIndices merged_indices;
            pcl::PointIndices cluster;

            int max_detected = 0;
            pcl::PointIndices biggest_cluster_indices;

            for (const auto &cluster : cluster_indices)
            {
                if (max_detected < cluster.indices.size())
                {
                    max_detected = cluster.indices.size();
                    biggest_cluster_indices = cluster;
                }
            }


            pcl::PointCloud<pcl_t>::Ptr clustered_pcl(new pcl::PointCloud<pcl_t>);

            for (const auto &idx : biggest_cluster_indices.indices)
            {
                clustered_pcl->push_back((*tf_filtered_pcl_ptr)[idx]);
            }
            clustered_pcl->width = clustered_pcl->size();
            clustered_pcl->height = 1;
            clustered_pcl->is_dense = true;
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*clustered_pcl, centroid);
            centroid_vect.push_back(centroid);
            std_msgs::msg::ColorRGBA centroid_color;
            centroid_color.a = 1.0f;
            centroid_color.r = .5f;
            centroid_color.g = .5f;
            centroid_color.b = 0.0f;

            Eigen::Vector4f median_point = computeMedianPoint(clustered_pcl);
            std::vector<Eigen::Vector4f> median_vect;
            median_vect.push_back(median_point);
            std_msgs::msg::ColorRGBA median_color;
            median_color.a = 1.0f;
            median_color.r = .5f;
            median_color.g = .5f;
            median_color.b = 0.5f;

            // /* ========================================
            //  * CONVERT PointCloud2 PCL->ROS, PUBLISH CLOUD
            //  * ========================================*/
            // this->publishPointCloud(segmented_pub_, *plane_seg_cloud);
            this->publishPointCloud(clustered_pub_, *clustered_pcl);
            this->publishMarker(centroid_pub_, centroid_vect, centroid_color);
            this->publishMarker(median_pub_, median_vect, median_color);
        }
        this->publishPointCloud(pre_filter_pub_, tf_filtered_pcl);

        // Get duration and log to console
        auto stop = std::chrono::high_resolution_clock::now();
        auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        //RCLCPP_INFO(get_logger(), "Time (msec): %ld", t_ms.count());
    } // cloud_callback

    /**
     * @brief Publishes a point cloud given a PCL cloud and a ROS publisher handle
     *
     * @param publisher
     * @param point_cloud
     */
    void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                           pcl::PointCloud<pcl_t> point_cloud)
    {
        sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);

        pcl::toROSMsg(point_cloud, *pc2_cloud);
        pc2_cloud->header.frame_id = world_frame;
        pc2_cloud->header.stamp = this->get_clock()->now();
        publisher->publish(*pc2_cloud);
    }
    void publishMarker(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
                       std::vector<Eigen::Vector4f> points, std_msgs::msg::ColorRGBA color)
    {
        visualization_msgs::msg::Marker marker_msg;
        marker_msg.header.frame_id = "map"; // your fixed frame
        marker_msg.header.stamp = this->get_clock()->now();
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
            p_msg.x = p[0];
            p_msg.y = p[1];
            p_msg.z = p[2];
            marker_msg.points.push_back(p_msg);
        }
        publisher->publish(marker_msg);
    }

    Eigen::Vector4f computeMedianPoint(const pcl::PointCloud<pcl_t>::Ptr &p_cloud)
    {
        std::vector<float> x_values, y_values, z_values;

        // Extract the points
        for (const auto &point : p_cloud->points)
        {
            x_values.push_back(point.x);
            y_values.push_back(point.y);
            z_values.push_back(point.z);
        }

        // Sort the coordinates
        std::sort(x_values.begin(), x_values.end());
        std::sort(y_values.begin(), y_values.end());
        std::sort(z_values.begin(), z_values.end());

        // Compute the median
        size_t n = p_cloud->points.size();
        Eigen::Vector4f median_point;

        if (n % 2 == 1)
        {
            // Odd number of points
            median_point[0] = x_values[n / 2];
            median_point[1] = y_values[n / 2];
            median_point[2] = z_values[n / 2];
        }
        else
        {
            // Even number of points
            median_point[0] = (x_values[n / 2 - 1] + x_values[n / 2]) / 2.0;
            median_point[1] = (y_values[n / 2 - 1] + y_values[n / 2]) / 2.0;
            median_point[2] = (z_values[n / 2 - 1] + z_values[n / 2]) / 2.0;
        }

        return median_point;
    }
}; // end MinimalPointCloudProcessor class

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalPointCloudProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}