#include "main_logic/vision_client.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    float timer_wait = 40.0;
    bool okay = false;
    
    auto vision_client = std::make_shared<main_logic::VisionClient>("shelfino1");
    
    RCLCPP_INFO(vision_client->get_logger(), "=== Vision Client Pipeline ===");
    
    // ------------------------ ------------------------ ------------------------ ------------------------
    RCLCPP_INFO(vision_client->get_logger(), "\n=== Step 1: Trigger Detection ===");
    okay = vision_client->trigger_detection("small_cone", timer_wait, false);
    
    if (okay) {
        RCLCPP_INFO(vision_client->get_logger(), 
                    "Detection completed: %s", 
                    vision_client->get_last_detection_message().c_str());
        // Wait for first tick after detection service completes
        if (!vision_client->wait_for_tick(timer_wait)) {
            RCLCPP_WARN(vision_client->get_logger(), "No first tick received after detection");
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(vision_client->get_logger(), "First tick received after detection");
        // Wait for second tick before proceeding to next step
        if (!vision_client->wait_for_tick(timer_wait)) {
            RCLCPP_WARN(vision_client->get_logger(), "No second tick received after detection");
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(vision_client->get_logger(), "Second tick received, ready for next step");
        
    } else {
        RCLCPP_ERROR(vision_client->get_logger(), "Detection failed");
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::sleep_for(std::chrono::seconds(2));

    // ------------------------ ------------------------ ------------------------ ------------------------
    RCLCPP_INFO(vision_client->get_logger(), "\n=== Step 2: Trigger PCL ===");
    okay = vision_client->trigger_pcl(timer_wait, false);
    
    if (okay) {
        RCLCPP_INFO(vision_client->get_logger(), 
                    "PCL capture completed: %s", 
                    vision_client->get_last_pcl_message().c_str());
        // Wait for tick after service completes
        if (!vision_client->wait_for_tick(timer_wait)) {
            RCLCPP_WARN(vision_client->get_logger(), "No tick received after PCL");
        }
    } else {
        RCLCPP_ERROR(vision_client->get_logger(), "PCL capture failed");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::sleep_for(std::chrono::seconds(2));

    // ------------------------ ------------------------ ------------------------ ------------------------
    RCLCPP_INFO(vision_client->get_logger(), "\n=== Step 3: Trigger Filter PCL ===");
    okay = vision_client->trigger_filter_pcl(timer_wait, false);
    
    if (okay) {
        RCLCPP_INFO(vision_client->get_logger(), 
                    "PCL filtering completed: %s", 
                    vision_client->get_last_filter_pcl_message().c_str());
        // Wait for tick after service completes
        if (!vision_client->wait_for_tick(timer_wait)) {
            RCLCPP_WARN(vision_client->get_logger(), "No tick received after filter PCL");
        }
    } else {
        RCLCPP_ERROR(vision_client->get_logger(), "PCL filtering failed");
        rclcpp::shutdown();
        return 1;
    }
    
    RCLCPP_INFO(vision_client->get_logger(), "\n===  All pipeline steps completed successfully! ===");
    rclcpp::shutdown();
    return 0;
}
