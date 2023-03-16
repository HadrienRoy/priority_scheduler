#include "scheduler.hpp"


// Main Function to handle scheduling
void SchedulerNode::priorityScheduler(std::string id, float distance, float percent)
{
    // Check request
        // dock request or state changer


    // DOCK REQUEST
    // get information from robot
    // get queue information
    // comapre data, rank, and give command
    // send command to robot

    // Add new robot
    if (add_new_robot)
    {
        addNewRobot(id, distance, percent);
        // give command/state
    }

    // STATE CHANGE
    // check change
    // if robot done charging -> shift up and change state
    else if (state_change)
    {
        // stateChange();
        // give commands/state
    }

    // Send states/commands to all robots
    sendStates();
}

// add new robot to queue
bool SchedulerNode::addNewRobot(std::string id, float distance, float percent)
{
    Robot temp_robot;
    std::string state;

    // if no robot in queue -> robot can dock
    if (queue.empty())
    {
        state = "Docking";
    }
    else
    {
        Robot last_robot = queue.back();

        // if queueing -> priority check
        if (last_robot.state == "Queuing")
        {
            // if priority -> queue 1
            priorityCheck(id, distance, percent);    // what is more than 1 robot queuing at this time?
        }
    
        state = "Queuing"; 
    }

    // The lower rank = higher priority
    int rank = floor((distance*distance_const) + (percent*percent_const));

    temp_robot.state = state;
    temp_robot.id = id;
    temp_robot.rank = rank;
    temp_robot.distance = distance;
    temp_robot.percent = percent;

    queue.push_back(temp_robot);

    return true;
}

// Check priority of queuing robots and adjust queue if needed
void SchedulerNode::priorityCheck (std::string id, float distance, float percent)
{
    // if already docking +  new robot

    // if already queuing +  new robo

}

// When a state of a robot needs to be changes
bool SchedulerNode::stateChange(std::string id)
{
    /*  Maybe this should search for robot id and just find single robot and change */

    bool charging_complete = false;

    std::list<Robot>::iterator it;

    for (it = queue.begin(); it != queue.end(); it++)
    {
        // 1. Robot has docked successfully
        if ((it->id == id) && (it->state == "Docking"))
        {
            it->state = "Charging";
        }
        // 2. Robot in queue area
        else if ((it->id == id) && (it->state == "Queuing"))
        {
            it->state = "Queued";
        }
        // 3. If robot charging is completed
        else if ((it->id == id) && (it->state == "Charging"))
        {
            // remove charged robot from queue
            charging_complete = true;     
        }
    }  

    // If robot is done charging
    if (charging_complete) 
    {
        queue.pop_front();  // pop charged robot

        // If more robots in queue, make next in queue docking
        if (!queue.empty())
        {
            it = queue.begin();
            it->state = "Docking";
        }
        charging_complete = false;
    }

    return true;
}

// Send states to robots 
void SchedulerNode::sendStates()
{}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SchedulerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
