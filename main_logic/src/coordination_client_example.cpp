#include "main_logic/coordination_client.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create the coordination client with robot namespace
    auto coordination_client = std::make_shared<main_logic::CoordinationClient>();

    RCLCPP_INFO(coordination_client->get_logger(), "=== Coordination Client Pipeline ===");
    RCLCPP_INFO(coordination_client->get_logger(), "\n=== Step 1: Trigger Coordination for Next Poses ===");
    RCLCPP_INFO(coordination_client->get_logger(), "Starting coordination pipeline...");
    if (coordination_client->trigger_coordination_next_pose(20.0, true)) {
        RCLCPP_INFO(coordination_client->get_logger(), 
                    "Coordination completed successfully: %s", 
                    coordination_client->get_last_coordination_message().c_str());
    } else {
        RCLCPP_ERROR(coordination_client->get_logger(), "Coordination failed");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(coordination_client->get_logger(), "\n=== All coordination steps completed successfully! ===");
    RCLCPP_INFO(coordination_client->get_logger(), "Note: All tick waits were indefinite (no timeout)");

    rclcpp::shutdown();
    return 0;
}
