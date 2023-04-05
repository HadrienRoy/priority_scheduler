#include "scheduler.hpp"

void SchedulerNode::queueUpdateServer(
    const std::shared_ptr<docking_interfaces::srv::QueueUpdate::Request> request,
    const std::shared_ptr<docking_interfaces::srv::QueueUpdate::Response> response)
{
    if (request->type == "add_new_robot")
    {
        RCLCPP_INFO(this->get_logger(), "Request: Add New Robot");

        if (addNewRobot(request->id, request->distance, request->battery))
        {
            response->success = true;
            response->state = "";
            response->command = 0;
        }
    }
    else if(request->type == "state_change")
    {
        RCLCPP_INFO(this->get_logger(), "Request: State Change");

        if (stateChange(request->id))
        {
            response->success = true;
            response->state = "";
            response->command = 0;
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Request: Invalid");
        response->success = false;
    }

    std::cout << "Charging Queue\n";
    print_queue(queue);

}