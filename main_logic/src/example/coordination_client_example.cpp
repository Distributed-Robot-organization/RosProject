#include "main_logic/coordination_client.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    float timer_wait = 40.0;
    bool okay= false;

    auto coordination_client = std::make_shared<main_logic::CoordinationClient>();

    RCLCPP_INFO(coordination_client->get_logger(), "=== Coordination Client Pipeline ===");
    RCLCPP_INFO(coordination_client->get_logger(), "\n=== Step 1: Trigger Coordination for Next Poses ===");
    // ------------------------ ------------------------ ------------------------ ------------------------
    okay = coordination_client->trigger_coordination_next_pose(timer_wait, true);
    
    if (okay) {
        RCLCPP_INFO(coordination_client->get_logger(), 
                    "Coordination completed successfully: %s", 
                    coordination_client->get_last_coordination_message().c_str());
    } else {
        RCLCPP_ERROR(coordination_client->get_logger(), "Coordination failed");
        rclcpp::shutdown();
        return 1;
    }


    RCLCPP_INFO(coordination_client->get_logger(), "\n=== All pipeline steps completed successfully! ===");
    rclcpp::shutdown();
    return 0;
}
