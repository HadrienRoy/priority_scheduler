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

    // The lower rank = higher priority
    // int rank = floor((distance*distance_const) + (percent*percent_const));

    int rank = fuzzify_9(distance, percent);

    temp_robot.id = id;
    temp_robot.rank = rank;
    temp_robot.distance = distance;
    temp_robot.percent = percent;

    // if no robot in queue -> robot can dock
    if (queue.empty())
    {
        temp_robot.state = "Docking";
        queue.push_back(temp_robot);    // Add robot to queue
    }
    else
    {
        temp_robot.state = "Queuing";       // Add state to new robot

        Robot last_robot = queue.back();    // Get last robot

        queue.push_back(temp_robot);        // Add robot to queue

        // if docking/queueing -> priority check
        if (last_robot.state == "Queuing")
        {
            // if priority -> queue 1
            priorityCheck(last_robot.state); 
        }
    }
    return true;
}

// Check priority of queuing robots and adjust queue if needed
void SchedulerNode::priorityCheck(std::string state)
{
    // std::list<Robot> temp_list;
    Robot temp_queue[queue.size()];

    // iter, go through queue and add robots to list
    std::list<Robot>::iterator it;
    std::list<Robot>::iterator it_start;
    bool it_found = false;
    int temp_size =0;
    for (it = queue.begin(); it != queue.end(); it++)
    {
        // find all queuing or docking
        if (it->state == state)
        {
            // temp_list.push_back(*it);
            temp_queue[temp_size++] = *it;

            if(!it_found)
            {
                it_found = true;
                it_start = it;
            }   
        }
    }

    // Delete old queue part
    std::list<Robot>::iterator it_end = queue.end();
    queue.erase(it_start, it_end);

    // Sort (array) array of robots by rank
    for (int i = 0; i < temp_size - 1; i++)
    {
        for (int j = 0; j < temp_size-i-1; j++)
        {
            if (temp_queue[j].rank < temp_queue[j+1].rank)
            {
                std::swap(temp_queue[j], temp_queue[j+1]);
            }
        }
    }
    
    // Add sorted list to queue
    for (int i = 0; i < temp_size; i++)
    {
        queue.push_back(temp_queue[i]);
    }

    // Add temp queue section to main queue
    // for (const Robot & robot : temp_list)
    // {
    //     queue.push_back(robot);
    // }

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
{    int position = 0;
    // Go through queue
    for (const Robot & robot : queue)
    {
        // Send command to each robot via service
        threads.push_back(std::thread(std::bind(&SchedulerNode::stateUpdateClient, this, robot.id, robot.state, position)));   
        position++;
    }
}


int SchedulerNode::fuzzify(float distance, float percent)
{
    std::string d_m; // distance_member
    std::string p_m; // percent member

    int output;

    if (distance >= 50)
        d_m = "FAR";
    else    
        d_m = "CLOSE";

    if (percent >= 62.5)
        p_m = "HIGH";
    else    
        p_m = "LOW";


    if (d_m == "FAR" && p_m == "HIGH")
        output = 1;
    else if (d_m == "FAR" && p_m == "LOW")
        output = 2;
    else if (d_m == "CLOSE" && p_m == "HIGH")
        output = 2;
    else if (d_m == "CLOSE" && p_m == "LOW")
        output = 3;

    return output;   
}

int SchedulerNode::fuzzify_9(float distance, float percent)
{
    std::string d_m; // distance_member
    std::string p_m; // percent member

    int output;

    // Distance member function
    if (distance >= 62.5)
        d_m = "FAR";
    else if(distance < 62.5 && distance > 25)
        d_m = "MID";
    if (distance <= 25)
        d_m = "CLOSE";
    
    // Percent member function
    if (percent >= 75)
        p_m = "HIGH";
    else if(percent < 75 && percent > 50)
        p_m = "MED";
    if (percent <= 50)
        p_m = "LOW";

    // Distance = Far
    if (d_m == "FAR" && p_m == "HIGH")
        output = 1;
    else if (d_m == "FAR" && p_m == "MED")
        output = 2;
    else if (d_m == "FAR" && p_m == "LOW")
        output = 3;
    // Distance = Mid
    else if (d_m == "MID" && p_m == "HIGH")
        output = 2;
    else if (d_m == "MID" && p_m == "MED")
        output = 3;
    else if (d_m == "MID" && p_m == "LOW")
        output = 4;
    // Distance = Close
    else if (d_m == "CLOSE" && p_m == "HIGH")
        output = 3;
    else if (d_m == "CLOSE" && p_m == "MED")
        output = 4;
    else if (d_m == "CLOSE" && p_m == "LOW")
        output = 5;

    return output;   
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SchedulerNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
