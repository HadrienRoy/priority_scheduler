#include "scheduler.hpp"

void SchedulerNode::stateUpdateClient(std::string id, std::string state, int num)
{

    // Create client
    auto client = this->create_client<docking_interfaces::srv::StateUpdate>(id+"/docking_controller/state_update_service"); // TODO: add id into 
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for the State Update Server to be up...");
    }

    // Create request
    auto request = std::make_shared<docking_interfaces::srv::StateUpdate::Request>();
    request->queue_state = state;
    request->queue_num = num;

    auto future = client->async_send_request(request);

    try
    {
        auto response = future.get();
        // RCLCPP_INFO(this->get_logger(), "State Update service request (state:=%s) successful.", state.c_str());
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(this->get_logger(), "State Update service request (state:=%s) failed.", state.c_str());
    }
}