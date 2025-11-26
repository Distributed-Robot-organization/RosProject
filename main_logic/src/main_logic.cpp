#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "main_logic/coordination_client.hpp"
#include "main_logic/vision_client.hpp"
#include "main_logic/navigation_client.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <vector>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <mutex>

class MainLogicNode : public rclcpp::Node
{
public:
    MainLogicNode(): Node("main_logic_node")
    {
        // Declare and get parameter
        this->declare_parameter<std::string>("yaml_path", "");
        std::string yaml_path = this->get_parameter("yaml_path").as_string();
        
        this->declare_parameter<std::string>("shelfino_params_path", "");
        std::string shelfino_params_path = this->get_parameter("shelfino_params_path").as_string();

        if (yaml_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "yaml_path parameter not set!");
            return;
        }
        if (shelfino_params_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "shelfino_params_path parameter not set!");
            return;
        }

        // Load YAML file
        objects_yaml_ = YAML::LoadFile(yaml_path);
        shelfino_yaml_ = YAML::LoadFile(shelfino_params_path);
        
        // Extract NAME RADIUS from yaml
        if (shelfino_yaml_["/**"]["ros__parameters"]["init_names"]) {
            YAML::Node init_names_node = shelfino_yaml_["/**"]["ros__parameters"]["init_names"];
            for (size_t i = 0; i < init_names_node.size(); i++) {
                robot_names_.push_back(init_names_node[i].as<std::string>());
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu robot names:", robot_names_.size());
            for (const auto& name : robot_names_) {
                RCLCPP_INFO(this->get_logger(), "  - %s", name.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "init_names not found in shelfino_params.yaml");
        }
        for (YAML::const_iterator it = objects_yaml_.begin(); it != objects_yaml_.end(); ++it) {
            std::string obj_name = it->first.as<std::string>();
            float radius = it->second["radius"].as<float>();
            RCLCPP_INFO(this->get_logger(), "Object: %s, radius: %.2f", obj_name.c_str(), radius);
        }
        // client for each robot
        for (const auto& robot_name : robot_names_) { 
            RCLCPP_INFO(this->get_logger(), "Initializing clients for robot: %s", robot_name.c_str());
            navigation_clients_.push_back(std::make_shared<main_logic::NavigationClient>(this,robot_name));
            vision_clients_.push_back(std::make_shared<main_logic::VisionClient>(this, robot_name));

            std::string vision_tick_topic = "/" + robot_name + "/vision_system/tick_service_vision";
            std::string nav_tick_topic = "/" + robot_name + "/navigation_system/tick_service_navigation";

            auto vision_tick_sub = this->create_subscription<std_msgs::msg::Bool>(
                vision_tick_topic, 10,
                [this, robot_name = robot_name](const std_msgs::msg::Bool::SharedPtr msg) {
                    this->tick_vision_subscriber_callback(msg, robot_name);
                });
            tick_vision_subscribers_.push_back(vision_tick_sub);
            RCLCPP_INFO(this->get_logger(), "Subscribed to vision tick: %s", vision_tick_topic.c_str());
            
            auto nav_tick_sub = this->create_subscription<std_msgs::msg::Bool>(
                nav_tick_topic, 10,
                [this, robot_name = robot_name](const std_msgs::msg::Bool::SharedPtr msg) {
                    this->tick_navigation_subscriber_callback(msg, robot_name);
                });
            tick_navigation_subscribers_.push_back(nav_tick_sub);
            RCLCPP_INFO(this->get_logger(), "Subscribed to navigation tick: %s", nav_tick_topic.c_str());
        }
        
        coordination_client_ = std::make_shared<main_logic::CoordinationClient>(this);

        // subscribe and publisher        
        tick_coordination_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
            "/coordination/tick_service_coordination", 10,
            std::bind(&MainLogicNode::tick_coordination_subscriber_callback, this, std::placeholders::_1));
        
        next_array_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/coordination/next_array_pose", 10,
            std::bind(&MainLogicNode::next_array_pose_callback, this, std::placeholders::_1));
        
        mean_observation_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/coordination/mean_observation_object", 10,
            std::bind(&MainLogicNode::mean_observation_callback, this, std::placeholders::_1));
        
        obj_to_detect_sub_ = this->create_subscription<std_msgs::msg::String>("/main_logic/obj_to_detect", 10, 
            std::bind(&MainLogicNode::obj_to_detect_callback, this, std::placeholders::_1));

        status_publisher_ = this->create_publisher<std_msgs::msg::String>("/main_logic/status", 10);
        
        // parameters
        state_machine_ready_ = false;

        RCLCPP_INFO(this->get_logger(), "main logic is READY!");
    }

private:
    
    void obj_to_detect_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received object to detect: %s", msg->data.c_str());
        current_object_ = msg->data;
        // Check if object exists in YAML
        if (!objects_yaml_[current_object_]) {
            RCLCPP_ERROR(this->get_logger(), "Object '%s' not found in YAML!", current_object_.c_str());
            // save radius of the object
            return;
        }
        radius_obj_ = objects_yaml_[current_object_]["radius"].as<float>();
        RCLCPP_INFO(this->get_logger(), "Object '%s'  found, radius: %.2f meters", current_object_.c_str(), radius_obj_);
        
        state_machine_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Starting state machine logic in a NEW THREAD");
        std::thread(&MainLogicNode::state_machine, this).detach();
    }
    
    void tick_coordination_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Tick received from coordination system");
            std::lock_guard<std::mutex> lock(mtx_);
            coordination_tick_received_ = true;
            cv_.notify_one();
        }
    }

    void tick_vision_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg, const std::string& robot_name)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Vision tick received from robot: %s", robot_name.c_str());
            std::lock_guard<std::mutex> lock(mtx_);
            vision_ticks_received_[robot_name] = true;
            cv_.notify_one();
        }
    }

    void tick_navigation_subscriber_callback(const std_msgs::msg::Bool::SharedPtr msg, const std::string& robot_name)
    {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Navigation tick received from robot: %s", robot_name.c_str());
            std::lock_guard<std::mutex> lock(mtx_);
            navigation_ticks_received_[robot_name] = true;
            cv_.notify_one();
        }
    }

    void reset_navigation_ticks()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        navigation_ticks_received_.clear();
        for (const auto& robot_name : robot_names_) {
            navigation_ticks_received_[robot_name] = false;
        }
    }

    void reset_vision_ticks()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        vision_ticks_received_.clear();
        for (const auto& robot_name : robot_names_) {
            vision_ticks_received_[robot_name] = false;
        }
    }

    void reset_coordination_tick()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        coordination_tick_received_ = false;
        next_poses_received_ = false;
        mean_observation_received_ = false;
    }

    bool wait_or_fail(std::function<bool(float)> wait_fn, const std::string& step_name, float timeout)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for %s...", step_name.c_str());

        if (!wait_fn(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "%s step FAILED!", step_name.c_str());
            state_machine_ready_ = false;
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "%s completed successfully.", step_name.c_str());
        return true;
    }

    void publish_status(const std::string& text)
    {
        std_msgs::msg::String msg;
        msg.data = text;
        status_publisher_->publish(msg);
    }

    void next_array_pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        next_poses_ = *msg;
        next_poses_received_ = true;
        
        // Extract global centroid (first pose) 
        if (!msg->poses.empty()) {
            center_obj.x = msg->poses[0].position.x;
            center_obj.y = msg->poses[0].position.y;
            center_obj.z = 0.0;
            RCLCPP_INFO(this->get_logger(), 
                "Received next array pose with %zu poses. Global centroid: [%.3f, %.3f, %.3f]", 
                msg->poses.size(), center_obj.x, center_obj.y, center_obj.z);
        } else {
            RCLCPP_INFO(this->get_logger(), "Received next array pose with %zu poses", msg->poses.size());
        }
        
        cv_.notify_one();
    }

    void mean_observation_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        observation_mean = msg->data;
        mean_observation_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received mean observation: %.4f", observation_mean);
        cv_.notify_one();
    }

    bool wait_for_all_vision_ticks(float timeout_sec)
    {
        std::unique_lock<std::mutex> lock(mtx_);
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(static_cast<int>(timeout_sec));
        
        while (std::chrono::steady_clock::now() < deadline) {
            bool all_received = true;
            for (const auto& robot_name : robot_names_) {
                if (!vision_ticks_received_[robot_name]) {
                    all_received = false;
                    break;
                }
            }
            
            if (all_received) {
                RCLCPP_INFO(this->get_logger(), "All vision ticks received!");
                return true;
            }
            
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
            if (remaining.count() > 0) {
                cv_.wait_for(lock, remaining);
            }
        }
        
        // Timeout - log which robots didn't respond
        RCLCPP_ERROR(this->get_logger(), "TIMEOUT: Not all vision ticks received!");
        for (const auto& robot_name : robot_names_) {
            if (!vision_ticks_received_[robot_name]) {
                RCLCPP_ERROR(this->get_logger(), "  Missing tick from robot: %s", robot_name.c_str());
            }
        }
        return false;
    }

    bool wait_for_all_navigation_ticks(float timeout_sec)
    {
        std::unique_lock<std::mutex> lock(mtx_);
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(static_cast<int>(timeout_sec));
        
        while (std::chrono::steady_clock::now() < deadline) {
            bool all_received = true;
            for (const auto& robot_name : robot_names_) {
                if (!navigation_ticks_received_[robot_name]) {
                    all_received = false;
                    break;
                }
            }
            
            if (all_received) {
                RCLCPP_INFO(this->get_logger(), "All navigation ticks received!");
                return true;
            }
            
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
            if (remaining.count() > 0) {
                cv_.wait_for(lock, remaining);
            }
        }
        
        // Timeout - log which robots didn't respond
        RCLCPP_ERROR(this->get_logger(), "TIMEOUT: Not all navigation ticks received!");
        for (const auto& robot_name : robot_names_) {
            if (!navigation_ticks_received_[robot_name]) {
                RCLCPP_ERROR(this->get_logger(), "  Missing tick from robot: %s", robot_name.c_str());
            }
        }
        return false;
    }

    bool wait_for_coordination_results(float timeout_sec)
    {
        std::unique_lock<std::mutex> lock(mtx_);
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(static_cast<int>(timeout_sec));
        
        while (std::chrono::steady_clock::now() < deadline) {
            if (coordination_tick_received_ && next_poses_received_ && mean_observation_received_) {
                RCLCPP_INFO(this->get_logger(), "All coordination results received!");
                return true;
            }
            
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
            if (remaining.count() > 0) {
                cv_.wait_for(lock, remaining);
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "TIMEOUT: Not all coordination results received!");
        RCLCPP_ERROR(this->get_logger(), "  Tick received: %s", coordination_tick_received_ ? "yes" : "no");
        RCLCPP_ERROR(this->get_logger(), "  Poses received: %s", next_poses_received_ ? "yes" : "no");
        RCLCPP_ERROR(this->get_logger(), "  Mean observation received: %s", mean_observation_received_ ? "yes" : "no");
        return false;
    }


    geometry_msgs::msg::Pose load_checkpoint_pose(const std::string& robot_name)
    {
        geometry_msgs::msg::Pose pose;

        YAML::Node checkpoints = shelfino_yaml_["/**"]["ros__parameters"]["checkpoints"];

        if (!checkpoints[robot_name]) {
            throw std::runtime_error("Checkpoint not found for robot " + robot_name);
        }

        auto cp = checkpoints[robot_name];
        if (!cp.IsSequence() || cp.size() < 3) {
            throw std::runtime_error("Invalid checkpoint format for robot " + robot_name);
        }

        float x = cp[0].as<float>();
        float y = cp[1].as<float>();
        float yaw = cp[2].as<float>();

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = std::sin(yaw / 2.0);
        pose.orientation.w = std::cos(yaw / 2.0);

        return pose;
    }

    geometry_msgs::msg::Point load_center_point()
    {
        geometry_msgs::msg::Point center;

        YAML::Node center_node = shelfino_yaml_["/**"]["ros__parameters"]["checkpoints"]["center"];

        if (!center_node || !center_node.IsSequence() || center_node.size() < 3) {
            throw std::runtime_error("Center point not found or invalid format in shelfino_params.yaml");
        }

        center.x = center_node[0].as<float>();
        center.y = center_node[1].as<float>();
        center.z = center_node[2].as<float>();

        return center;
    }

    void send_all_specific_path(const std::vector<geometry_msgs::msg::Pose>& poses, float timeout)
    {
        for (size_t i = 0; i < navigation_clients_.size(); i++) {
            RCLCPP_INFO(this->get_logger(),
                "Robot %s navigating to [%.2f, %.2f]",
                robot_names_[i].c_str(),
                poses[i].position.x,
                poses[i].position.y
            );
            navigation_clients_[i]->generate_specific_path(poses[i], timeout);
        }
    }

    void send_all_rotate_to_center(const geometry_msgs::msg::Point& center, float timeout)
    {
        for (size_t i = 0; i < navigation_clients_.size(); i++) {
            RCLCPP_INFO(this->get_logger(),
                "Robot %s rotating to center [%.2f, %.2f, %.2f]",
                robot_names_[i].c_str(),
                center.x,
                center.y,
                center.z
            );
            navigation_clients_[i]->rotate_to_center(center, timeout);
        }
    }

    void send_all_arc_path(const geometry_msgs::msg::Point& center,
                             const std::vector<geometry_msgs::msg::Point>& goals,
                             float radius, float timeout)
    {
        for (size_t i = 0; i < navigation_clients_.size(); i++) {
            RCLCPP_INFO(this->get_logger(),
                "Robot %s generating arc path with radius %.2f to goal [%.2f, %.2f]",
                robot_names_[i].c_str(),
                radius,
                goals[i].x,
                goals[i].y
            );
            navigation_clients_[i]->generate_arc(radius, center, goals[i], timeout);
        }
    }
    
    void trigger_all_detections(const std::string& object_name, float timeout)
    {
        for (size_t i = 0; i < vision_clients_.size(); i++) {
            // RCLCPP_INFO(this->get_logger(), "Robot %s triggering detection for object: %s", robot_names_[i].c_str(), object_name.c_str());
            vision_clients_[i]->trigger_detection(object_name, timeout);
        }
    }

    void trigger_all_pcl(float timeout)
    {
        for (size_t i = 0; i < vision_clients_.size(); i++) {
            // RCLCPP_INFO(this->get_logger(), "Robot %s triggering PCL processing", robot_names_[i].c_str());
            vision_clients_[i]->trigger_pcl(timeout);
        }
    }
    
    void trigger_all_filter_pcl(float timeout)
    {
        for (size_t i = 0; i < vision_clients_.size(); i++) {
            // RCLCPP_INFO(this->get_logger(), "Robot %s triggering Filter PCL processing", robot_names_[i].c_str());
            vision_clients_[i]->trigger_filter_pcl(timeout);
        }
    }

    std::vector<geometry_msgs::msg::Pose> sort_goals_by_distance(const std::vector<geometry_msgs::msg::Pose>& current_poses, const std::vector<geometry_msgs::msg::Point>& goals)
    {
        size_t n = goals.size();
        std::vector<geometry_msgs::msg::Pose> sorted_goals(n);
        std::vector<bool> goal_assigned(n, false);
        std::vector<bool> robot_assigned(n, false);

        for (size_t i = 0; i < n; i++) {
            double min_distance = std::numeric_limits<double>::max();
            size_t best_robot_idx = 0;
            size_t best_goal_idx = 0;

            // Trova la coppia (robot, goal) non ancora assegnata con distanza minima
            for (size_t i = 0; i < n; i++) {
                if (robot_assigned[i]) continue;

                for (size_t j = 0; j < n; j++) {
                    if (goal_assigned[j]) continue;

                    double dx = current_poses[i].position.x - goals[j].x;
                    double dy = current_poses[i].position.y - goals[j].y;
                    double distance = std::sqrt(dx*dx + dy*dy);

                    if (distance < min_distance) {
                        min_distance = distance;
                        best_robot_idx = i;
                        best_goal_idx = j;
                    }
                }
            }

            // Assegna il goal migliore al robot (convert Point to Pose)
            sorted_goals[best_robot_idx].position = goals[best_goal_idx];
            sorted_goals[best_robot_idx].orientation.w = 1.0;
            goal_assigned[best_goal_idx] = true;
            robot_assigned[best_robot_idx] = true;

            RCLCPP_INFO(this->get_logger(), 
                "Assigning to robot %s: goal [%.2f, %.2f] at distance %.2f",
                robot_names_[best_robot_idx].c_str(),
                goals[best_goal_idx].x,
                goals[best_goal_idx].y,
                min_distance);
        }

        return sorted_goals;
    }

    void state_machine()
    {   
        int iteration = 0;
        float timer_action = 180.0; // tempo di attesa per ogni azione, questo è messo su tutti, molto lungo ma non dovrebbe mai agganciarsi di norma
        observation_mean = 0.0;
        // thresholds for state machine
        float observation_threshold = 5.0;
        int itereation_thresh = 10;
        
        if (!state_machine_ready_) {
            RCLCPP_WARN(this->get_logger(), "State machine not ready, wait state_machine_ready_ = true");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Starting state machine for object: %s", current_object_.c_str());
        
        // Load center point once into center_obj
        try {
            center_obj = load_center_point();
            RCLCPP_INFO(this->get_logger(), "Loaded center point: [%.2f, %.2f, %.2f]", center_obj.x, center_obj.y, center_obj.z);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load center point: %s", e.what());
            state_machine_ready_ = false;
            return;
        }

        if (iteration ==0) {
            robot_activated = robot_names_;
        }
        else if (current_poses.size() < robot_names_.size()) {
            // robot activate deve avere un numero di parametri pari alla sua quantita: robot_names_.size - current_poses.size()
            robot_activated = std::vector<std::string>(
                robot_names_.begin(), 
                robot_names_.begin() + (robot_names_.size() - current_poses.size())
            );
        }
        else {
            robot_activated = robot_names_;
        }
        
        while (observation_mean < observation_threshold && iteration < itereation_thresh) {

            // ================================
            // STEP 1: Navigation Phase
            RCLCPP_INFO(this->get_logger(), "=== Step 1: Navigation phase ===");
            //  SPECIFIC POINT
            if (iteration == 0){
                
                publish_status("Navigation to initial positions");
                reset_navigation_ticks();
                
                try {
                    for (const auto& robot_name : robot_activated) {
                        current_poses.push_back(load_checkpoint_pose(robot_name));
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to load checkpoints: %s", e.what());
                    state_machine_ready_ = false;
                    return;
                }

                send_all_specific_path(current_poses, timer_action);

                if (!wait_or_fail([this](float t) { return wait_for_all_navigation_ticks(t); }, 
                                "Navigation to initial positions", timer_action)) {
                    return;
                }
            }
            //  ARC PATH (fatto dalla seconda iterazione in poi)
            else {
                publish_status("Navigation in arc paths");
                publish_status("Arc navigation to new viewpoints");
                reset_navigation_ticks();
                
                geometry_msgs::msg::Point arc_center;

                arc_center.x = center_obj.x;
                arc_center.y = center_obj.y;
                arc_center.z = 0.0;
                
                std::vector<geometry_msgs::msg::Point> arc_goals;
                for (size_t i = 0; i < robot_activated.size(); i++) {
                    arc_goals.push_back(current_poses[i].position);
                    RCLCPP_INFO(this->get_logger(), "Robot %s arc goal: (%.2f, %.2f)", 
                               robot_activated[i].c_str(), current_poses[i].position.x, current_poses[i].position.y);
                }
                
                send_all_arc_path(arc_center, arc_goals, radius_obj_, timer_action);
                if (!wait_or_fail([this](float t) { return wait_for_all_navigation_ticks(t); }, 
                                "Navigation in arc paths", timer_action)) {
                    return;
                }
            }

            // 1b. Rotate all robots to face the center
            RCLCPP_INFO(this->get_logger(), "=== Step 1b: Rotate phase ===");
            publish_status("Rotating to center");
            reset_navigation_ticks();
            send_all_rotate_to_center(center_obj, timer_action);
            if (!wait_or_fail([this](float t) { return wait_for_all_navigation_ticks(t); }, 
                            "Rotation to center", timer_action)) {
                return;
            }

            // ================================
            // STEP 2: Vision Phase
            RCLCPP_INFO(this->get_logger(), "=== Step 2: Vision Detection phase ===");
            publish_status("Vision detection step");
            reset_vision_ticks();
            trigger_all_detections(current_object_, timer_action);
            if (!wait_or_fail([this](float t) { return wait_for_all_vision_ticks(t); }, 
                            "Object detection", timer_action)) {
                return;
            }
            // 2b. Trigger PCL for all robots --> not usefull now
            RCLCPP_INFO(this->get_logger(), "=== Step 2b: Point Cloud Processing ===");
            publish_status("Point cloud processing step");
            reset_vision_ticks();
            trigger_all_pcl(timer_action);

            if (!wait_or_fail([this](float t) { return wait_for_all_vision_ticks(t); }, 
                            "Point cloud processing", timer_action)) {
                return;
            }

            // 2c. Trigger filter PCL for all robots
            RCLCPP_INFO(this->get_logger(), "=== Step 2c: Point Cloud Filtering ===");
            publish_status("Point cloud filtering step");
            reset_vision_ticks();
            trigger_all_filter_pcl(timer_action);
            if (!wait_or_fail([this](float t) { return wait_for_all_vision_ticks(t); }, 
                            "Point cloud filtering", timer_action)) {
                return;
            }

            // // 3. Trigger coordination for next poses
            RCLCPP_INFO(this->get_logger(), "=== Step 3: Triggering Coordination ===");
            publish_status("Coordinating step");

            reset_coordination_tick();
            coordination_client_->trigger_coordination_next_pose(timer_action);
            if (!wait_for_coordination_results(timer_action)) {
                RCLCPP_ERROR(this->get_logger(), "TIMEOUT: Did not receive coordination results in %f seconds!", timer_action);
                state_machine_ready_ = false;
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Coordination completed successfully");
            
            // ==========================================================
            // SOLO PER TEST VALORI MANUALI,
            // observation_mean = 0.1;

            // // 2. Next poses manuale
            // next_poses_.poses.clear();

            // // Primo elemento = centro oggetto
            // geometry_msgs::msg::Pose center_pose;
            // center_pose.position.x = 0.0;  
            // center_pose.position.y = 20.0;
            // center_pose.position.z = 0.0;
            // next_poses_.poses.push_back(center_pose);
            // // punto primo robot 4.0  20.0
            // geometry_msgs::msg::Pose robot1_pose;
            // robot1_pose.position.x = 4.0; 
            // robot1_pose.position.y = 20.0;
            // robot1_pose.position.z = 0.0;
            // next_poses_.poses.push_back(robot1_pose);
            // // punto secondo robot -4.0  20.0
            // geometry_msgs::msg::Pose robot2_pose;
            // robot2_pose.position.x = -4.0;  
            // robot2_pose.position.y = 20.0;
            // robot2_pose.position.z = 0.0;
            // next_poses_.poses.push_back(robot2_pose);
            // ==========================================================

            if (next_poses_.poses.size() <= 1) {
                            RCLCPP_WARN(this->get_logger(), 
                                        "Coordination did not return any new viewpoints (found %zu poses). Exiting loop.", 
                                        next_poses_.poses.size());
                            state_machine_ready_ = false;
                            publish_status("No new viewpoints found. Exiting state machine. FINISH");
                            break; 
                        }

            RCLCPP_INFO(this->get_logger(), "=== Step 4: update target poses, center point, mean obs and iteration ===");
            RCLCPP_INFO(this->get_logger(), "  Mean observation: %.4f", observation_mean);
            RCLCPP_INFO(this->get_logger(), "  NEW Next poses FIND: %zu", next_poses_.poses.size());
            target_goals.clear();

            for (size_t i = 0; i < robot_activated.size(); i++) {
                target_goals.push_back(next_poses_.poses[i + 1].position);
            }
            RCLCPP_INFO(this->get_logger(), "Updated observation mean: %.4f", observation_mean);
            center_obj.x = next_poses_.poses[0].position.x;
            center_obj.y = next_poses_.poses[0].position.y;
            center_obj.z = 0.0;

            RCLCPP_INFO(this->get_logger(), "Updated center object to: [%.2f, %.2f, %.2f]",center_obj.x, center_obj.y, center_obj.z);
            current_poses = sort_goals_by_distance(current_poses, target_goals);
            
            iteration++;
            RCLCPP_INFO(this->get_logger(), "Updated iteration: %d", iteration);
            
        }
        
        state_machine_ready_ = false;
        publish_status("State machine completed");
        RCLCPP_INFO(this->get_logger(), "State machine finished. Final observation mean: %.4f", observation_mean);
    }

    // tick subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                tick_coordination_subscriber_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>   tick_vision_subscribers_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>   tick_navigation_subscribers_;
    
    // publishers and subscribers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;  
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obj_to_detect_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr next_array_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mean_observation_sub_;

    // clients
    std::shared_ptr<main_logic::CoordinationClient> coordination_client_;
    std::vector<std::shared_ptr<main_logic::NavigationClient>> navigation_clients_;
    std::vector<std::shared_ptr<main_logic::VisionClient>> vision_clients_;

    // control variables
    std::mutex mtx_;
    std::condition_variable cv_;
    bool next_poses_received_{false};
    bool mean_observation_received_{false};
    std::atomic<bool>           coordination_tick_received_{false};
    std::map<std::string, bool> vision_ticks_received_;
    std::map<std::string, bool> navigation_ticks_received_;

    std::string current_object_;
    bool state_machine_ready_;
    float radius_obj_;
    float observation_mean;
    geometry_msgs::msg::PoseArray next_poses_;
    YAML::Node objects_yaml_;
    YAML::Node shelfino_yaml_;
    std::vector<std::string> robot_names_;
    std::vector<std::string> robot_activated;
    std::vector<geometry_msgs::msg::Pose> current_poses;
    std::vector<geometry_msgs::msg::Point> target_goals;
    geometry_msgs::msg::Point center_obj;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MainLogicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
