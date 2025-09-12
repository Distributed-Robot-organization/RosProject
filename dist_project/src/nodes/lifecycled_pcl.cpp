
#include <exception>
#include <stdexcept>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
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

#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
typedef pcl::PointXYZRGB pcl_t;

using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using CallbackReturn = LifecycleNodeInterface::CallbackReturn;

class SensorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
      explicit SensorNode(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms).allow_undeclared_parameters(true)
                                                                      .automatically_declare_parameters_from_overrides(true)) {}

protected:
    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring sensor...");
        // Allocate resources, open sensor connections
        /*
         * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
         */
        rclcpp::Parameter cloud_topic_param,
            cluster_tolerance_param, min_cluster_size_param, max_cluster_size_param, plane_max_tree_iterations_param, plane_distance_treshold_param,
            z_filter_max_param, max_camera_depth_param, topic_pcl_filtered_param, world_frame_param, topic_cluster_pcl_param, noise_mean_param,noise_dev_param;

        RCLCPP_INFO(this->get_logger(), "Getting parameters");

        this->get_parameter_or("topics.raw_pcl", cloud_topic_param, rclcpp::Parameter("", "/points"));
        this->get_parameter_or("topics.filtered_pcl", topic_pcl_filtered_param, rclcpp::Parameter("", "laser_data_frame"));
        this->get_parameter_or("world_frame", world_frame_param, rclcpp::Parameter("", "map"));
        this->get_parameter_or("topics.cluster_pcl", topic_cluster_pcl_param, rclcpp::Parameter("", "topic_cluster_pcl"));

        this->get_parameter_or("shelfino_additions.max_camera_depth", max_camera_depth_param, rclcpp::Parameter("", 8.0));

        this->get_parameter_or("pcl_filter_params.cluster_tolerance", cluster_tolerance_param, rclcpp::Parameter("", 0.05));
        this->get_parameter_or("pcl_filter_params.min_cluster_size", min_cluster_size_param, rclcpp::Parameter("", 100));
        this->get_parameter_or("pcl_filter_params.max_cluster_size", max_cluster_size_param, rclcpp::Parameter("", 99000));
        this->get_parameter_or("pcl_filter_params.plane_max_tree_iterations_max", plane_max_tree_iterations_param, rclcpp::Parameter("", 100));
        this->get_parameter_or("pcl_filter_params.plane_distance_treshold", plane_distance_treshold_param, rclcpp::Parameter("", 0.02));

        this->get_parameter_or("pcl_filter_params.noise_mean", noise_mean_param, rclcpp::Parameter("",.0));
        this->get_parameter_or("pcl_filter_params.noise_dev", noise_dev_param, rclcpp::Parameter("", 0.02));


        this->get_parameter_or("pcl_filter_params.z_filter_max", z_filter_max_param, rclcpp::Parameter("", 8.0));

        topic_pcl_raw = cloud_topic_param.as_string();
        topic_pcl_filtered_ = topic_pcl_filtered_param.as_string();
        world_frame = world_frame_param.as_string();
        topic_cluster_pcl_ = topic_cluster_pcl_param.as_string();

        cluster_tolerance = cluster_tolerance_param.as_double();
        min_cluster_size = min_cluster_size_param.as_int();
        max_cluster_size = max_cluster_size_param.as_int();
        plane_max_tree_iterations = plane_max_tree_iterations_param.as_int();
        plane_distance_treshold = plane_distance_treshold_param.as_double();
        noise_dev= noise_dev_param.as_double();
        noise_mean= noise_mean_param.as_double();
        // z_filter_max = z_filter_max_param.as_double();
        max_camera_depth = max_camera_depth_param.as_double();
        /*
         * SET UP SUBSCRIBER
         */
        RCLCPP_INFO(this->get_logger(), "Setting up subscriber");
        cloud_subscriber_ =
            this->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic_pcl_raw, 1, std::bind(&SensorNode::cloud_callback, this, std::placeholders::_1));

        /*
         * SET UP PUBLISHERS
         */
        RCLCPP_INFO(this->get_logger(), "Setting up publishers");
        clustered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_cluster_pcl_, 1);
        pre_filter_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_pcl_filtered_, 1);
        centroid_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("centroid", 1);
        median_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("median", 1);

        /*
         * SET UP TF. Optional for transforming between coordinate frames
         *          You need to create a static tranform publisher to use this
         */
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating sensor...");
        // Start publishers/timers
        kill_callback = false;

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating sensor...");
        // Stop publishers/timers
        clustered_pub_->on_deactivate();
        kill_callback = true;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up sensor...");
        cloud_subscriber_.reset();
        pre_filter_pub_.reset();
        clustered_pub_.reset();
        centroid_pub_.reset();
        median_pub_.reset();
        tf_buffer_.reset();
        tf_listener_.reset();
        br.reset();
        // Free resources
        return CallbackReturn::SUCCESS;
    }

private:
    /*
     * Subscriber and Publisher declaration
     */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscriber_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pre_filter_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr centroid_pub_;
    rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr median_pub_;

    /*
     * Parameters
     */
    std::string topic_pcl_raw;
    std::string topic_pcl_filtered_;
    std::string topic_cluster_pcl_;

    std::string world_frame;
    std::string camera_frame;

    float voxel_leaf_size;
    float cluster_tolerance;

    int min_cluster_size;
    int max_cluster_size, plane_max_tree_iterations;
    float plane_distance_treshold, z_filter_max;
    float max_camera_depth, noise_mean, noise_dev;

    bool kill_callback = false;

    /*
     * TF
     */
    std::unique_ptr<tf2_ros::Buffer>
        tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr recent_cloud)
    {
        if(kill_callback)return;
        //-------------------------------Filtering far away points
        // Use for timing callback execution time
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<Eigen::Vector4f> centroid_vect;
        bool error = false;

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
        this->addGaussianNoiseToPointCloud(filtered_cloud, noise_mean, noise_dev);

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
        while (tf_filtered_pcl_ptr->size() > 0.3 * nr_points && !error)
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
            try
            {
                extract.setInputCloud(tf_filtered_pcl_ptr);
                extract.setIndices(inliers);
                extract.setNegative(false);
                // Get the points associated with the planar surface
                extract.filter(*cloud_plane);
                // std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;
                //  Remove the planar inliers, extract the rest
                extract.setNegative(true);
                extract.filter(*cloud_f);
                *tf_filtered_pcl_ptr = *cloud_f;
            }
            catch (const pcl::PCLException &ex)
            {
                // sporadically it gives the Assertion `point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!"' failed.

                RCLCPP_ERROR(this->get_logger(), "At line %d %s", ex.getLineNumber(), ex.what());
            }
        }

        if (tf_filtered_pcl_ptr->size() != 0)
        {
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<pcl_t>::Ptr tree(new pcl::search::KdTree<pcl_t>);
            pcl::EuclideanClusterExtraction<pcl_t> ec;
            std::vector<pcl::PointIndices> cluster_indices;
            // There are no point remaining in the point cloud to clusterize
            try
            {
                tree->setInputCloud(tf_filtered_pcl_ptr);

                ec.setClusterTolerance(cluster_tolerance); // maximum search distance
                ec.setMinClusterSize(min_cluster_size);
                ec.setMaxClusterSize(max_cluster_size);
                ec.setSearchMethod(tree);
                ec.setInputCloud(tf_filtered_pcl_ptr);
                ec.extract(cluster_indices);
            }
            catch (const pcl::PCLException &ex)
            {
                // sporadically it gives the Assertion `point_representation_->isValid (point) && "Invalid (NaN, Inf) point coordinates given to radiusSearch!"' failed.

                RCLCPP_ERROR(this->get_logger(), "At line %d %s", ex.getLineNumber(), ex.what());
            }
            if (!error)
            {

                pcl::PointIndices merged_indices;
                pcl::PointIndices cluster;

                int max_detected = 0;
                pcl::PointIndices biggest_cluster_indices;

                for (const auto &cluster : cluster_indices)
                {
                    if (max_detected < int(cluster.indices.size()))
                    {
                        max_detected = cluster.indices.size();
                        biggest_cluster_indices = cluster;
                    }
                }

                pcl::PointCloud<pcl_t>::Ptr clustered_pcl(new pcl::PointCloud<pcl_t>);
                if (biggest_cluster_indices.indices.size() > 0)
                {
                    for (const auto &idx : biggest_cluster_indices.indices)
                    {
                        clustered_pcl->push_back((*tf_filtered_pcl_ptr)[idx]);
                    }
                    this->publishPointCloud(clustered_pub_, *clustered_pcl);
                }
            }
        }
        this->publishPointCloud(pre_filter_pub_, tf_filtered_pcl);

        // Get duration and log to console
        auto stop = std::chrono::high_resolution_clock::now();
        auto t_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        // RCLCPP_INFO(get_logger(), "Time (msec): %ld", t_ms.count());
    } // cloud_callback
    void publishPointCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
                           pcl::PointCloud<pcl_t> point_cloud)
    {
        sensor_msgs::msg::PointCloud2::SharedPtr pc2_cloud(new sensor_msgs::msg::PointCloud2);

        pcl::toROSMsg(point_cloud, *pc2_cloud);
        pc2_cloud->header.frame_id = world_frame;
        pc2_cloud->header.stamp = this->get_clock()->now();
        publisher->publish(*pc2_cloud);
    }

    void addGaussianNoiseToPointCloud(pcl::PointCloud<pcl_t>::Ptr cloud, float mean, float stddev)
    {
        // Random number generator for Gaussian noise
        std::default_random_engine generator;
        std::normal_distribution<float> distribution(mean, stddev);

        for (auto &point : cloud->points)
        {
            // point.x += distribution(generator);
            // point.y += distribution(generator);
            point.z += distribution(generator);
        }
    }
};

int main(int argc, char **argv)
{
    //  force flush of the stdout buffer.
    // this ensures a correct sync of all prints
    // even when executed simultaneously within the launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<SensorNode> lc_node =
        std::make_shared<SensorNode>("lifecycled_pcl");

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}
