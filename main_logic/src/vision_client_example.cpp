#include "main_logic/vision_client.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create the vision client with robot namespace
    auto vision_client = std::make_shared<main_logic::VisionClient>("shelfino1");

    RCLCPP_INFO(vision_client->get_logger(), "=== Vision Client Pipeline ===");
    
    RCLCPP_INFO(vision_client->get_logger(), "\n=== Step 1: Trigger Detection ===");
    RCLCPP_INFO(vision_client->get_logger(), "Starting detection for 'cone'...");
    
    if (vision_client->trigger_detection("small_cone", 5.0, true)) {
        RCLCPP_INFO(vision_client->get_logger(), 
                    "Detection completed and tick received: %s", 
                    vision_client->get_last_detection_message().c_str());
    } else {
        RCLCPP_ERROR(vision_client->get_logger(), "Detection failed");
        rclcpp::shutdown();
        return 1;
    }
    


    // --------------------------
    // DELAY
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(vision_client->get_logger(), "\n=== Step 2: Trigger PCL ===");
    RCLCPP_INFO(vision_client->get_logger(), "Starting PCL capture...");
    if (vision_client->trigger_pcl(5.0, true)) {
        RCLCPP_INFO(vision_client->get_logger(), 
                    " PCL capture completed and tick received: %s", 
                    vision_client->get_last_pcl_message().c_str());
    } else {
        RCLCPP_ERROR(vision_client->get_logger(), "PCL capture failed");
        rclcpp::shutdown();
        return 1;
    }

    // --------------------------
    // DELAY
    RCLCPP_INFO(vision_client->get_logger(), "\n=== Step 3: Trigger Filter PCL ===");
    RCLCPP_INFO(vision_client->get_logger(), "Starting PCL filtering...");
    if (vision_client->trigger_filter_pcl(5.0, true)) {
        RCLCPP_INFO(vision_client->get_logger(), 
                    " PCL filtering completed and tick received: %s", 
                    vision_client->get_last_filter_pcl_message().c_str());
    } else {
        RCLCPP_ERROR(vision_client->get_logger(), "PCL filtering failed");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(vision_client->get_logger(), "\n===  All pipeline steps completed successfully! ===");


    rclcpp::shutdown();
    return 0;
}
