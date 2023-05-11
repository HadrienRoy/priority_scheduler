#include "scheduler.hpp"


/** QUEUE UPDATE FUNCTIONS **/

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

bool SchedulerNode::addNewRobotV2(std::string id, float distance, float percent)
{
    Robot temp_robot;
    std::string state;

    int rank = fuzzyy_rank(distance, percent);

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
        // Add new robot
        temp_robot.state = "Queuing";       // Add state to new robot
        Robot last_robot = queue.back();    // Get last robot
        queue.push_back(temp_robot);        // Add robot to queue

        // priorityCheck("Queuing");

        // Update ranks of all robots
        std::list<Robot>::iterator it;
        for (it = queue.begin(); it != queue.end(); it++)
        {
            if ((it->state == "Queuing") )
            {
                threads.push_back(std::thread(std::bind(&SchedulerNode::rankUpdateClient, this, it->id)));   
            }
        } 
    }

    // For test
    num_robots += 1;    // add robot to total number this test
    if (use_timer && num_robots == 1)// if the first roboot, start timer to keep track of total time
    {
        timer_start = steady_clock::now();
    }
    
    return true;
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

            num_robots_done += 1;

            if (use_timer && num_robots_done == total_robots)
            {
                steady_clock::time_point time_passed = steady_clock::now();
                total_time_passed = duration_cast<duration<float>>(time_passed - timer_start).count();
                // RCLCPP_INFO_ONCE(get_logger(), "Total Time Passed: %f", (total_time_passed));
                std::cout << "Total Time Passed: " << total_time_passed << std::endl;
            }
            
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


/** HELPER FUNCTIONS **/

// Check priority of queuing robots and adjust queue if needed
void SchedulerNode::priorityCheck(std::string state)
{
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
            // +1 so it doesnt rank if one rank above 
            if (temp_queue[j].rank+1 < temp_queue[j+1].rank)
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
}

void SchedulerNode::rankUpdateClient(std::string id)
{
    // Create client
    auto client = this->create_client<docking_interfaces::srv::RankUpdate>(id+"/docking_controller/rank_update_service"); 
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for the State Update Server to be up...");
    }

    // Create request
    auto request = std::make_shared<docking_interfaces::srv::RankUpdate::Request>();

    auto future = client->async_send_request(request);

    try
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Rank Update service request (id:=%s) successful.", id.c_str());
        updateRank(response->id, response->distance, response->battery);
    }
    catch (const std::exception &e)
    {
        RCLCPP_INFO(this->get_logger(), "Rank Update service request failed.");
    }
}

void SchedulerNode::updateRank(std::string id, float distance, float percent)
{
    // get data from robot
    num_update += 1;

    // std::cout << "Start updateRank"<<std::endl;
    
    // calculate new rank
    int rank = fuzzy_rank(distance, percent);

    // update rank in queue
    std::list<Robot>::iterator it;
    for (it = queue.begin(); it != queue.end(); it++)
    {
        if (it->id == id)
        {
            it->rank = rank;
            it->distance = distance;
            it->percent = percent;
        }
    }  

    if (num_update == queue.size())
    {
        priorityCheck("Queuing"); 
        std::cout << "Priority Rank Update"<<std::endl;
        print_queue(queue);
        num_update = 0;
    }

}


/** Fuzzy Logic**/
int SchedulerNode::fuzzify_9(float distance, float percent)
{
    std::string d_m; // distance_member
    std::string p_m; // percent member

    int output;

    // Fuzzification

    // Inference Enginer
    // Distance member function
    if (distance > mid_distance_threshold)
        d_m = "FAR";
    else if(distance <= mid_distance_threshold && distance > close_distance_threshold)
        d_m = "MID";
    else if (distance <= close_distance_threshold)
        d_m = "CLOSE";

    // Percent member function
    if (percent > 75)
        p_m = "HIGH";
    else if(percent <= 75 && percent > 50)
        p_m = "MED";
    else if (percent <= 50)
        p_m = "LOW";


    // Defuzzify
    if (d_m == "FAR" && p_m == "HIGH")  // Distance = Far
        output = 1;
    else if (d_m == "FAR" && p_m == "MED")
        output = 2;
    else if (d_m == "FAR" && p_m == "LOW")
        output = 3;
    else if (d_m == "MID" && p_m == "HIGH") // Distance = Mid
        output = 2;
    else if (d_m == "MID" && p_m == "MED")
        output = 3;
    else if (d_m == "MID" && p_m == "LOW")
        output = 4;
    else if (d_m == "CLOSE" && p_m == "HIGH") // Distance = Close
        output = 3;
    else if (d_m == "CLOSE" && p_m == "MED")
        output = 4;
    else if (d_m == "CLOSE" && p_m == "LOW")
        output = 5;

    return output;   
}

int SchedulerNode::fuzzy_rank(float distance, float percent)
{
    std::string d_m; // distance_member
    std::string p_m; // percent member

    int output;

    
    // Inference Engine

    /** Distance MF **/
    std::map<std::string,float> distance_set;
    float max_dist = 100;

    // for close < 25, 25 < mid < 62.5, 62.5 < far 
    float close = 100*1/8; // 12.5
    float mid_1 = 100*3/8; // 37.5
    float mid_r = 100*4/8; // 50
    float far =   100*6/8; // 75

    // Low (trapezoidal R function)
    float a=0; float b=0; float c=12.5; float d=37.5;
    float close_r = (d-distance)/(d-c);
    float d_m_close = fmax(fmin(1,close_r),0);
    distance_set["CLOSE"] = d_m_close;

    // Medium (triangle function)
    a=12.5; b=37.5; c=50; d=75;
    float mid_l = (distance-a)/(b-a); // a <= percent <= b
    float mid_r = (d-distance)/(d-c); // b <= percent <= c
    float d_m_mid = fmax(fmin(fmin(mid_l,1),mid_r),0);
    distance_set["MID"] = d_m_mid;

    // High (trapezoidal L function)
    a=50; b=75; c=100; d=100;
    float far_l = (distance-a)/(b-a);
    float d_m_far = fmax(fmin(far_l,1),0);
    distance_set["FAR"] = d_m_far;

    // Highest value fires 
    float last=0;
    for (auto it = distance_set.begin(); it != distance_set.end(); ++it)
    {
        std::cout<<it->first<< " " << it->second << " ";
        if (it->second > last)
        {
            d_m = it->first;
            last = it->second;
        }
    }

    std::cout << "\n" << d_m << std::endl;

    /** Battery Percent MF **/
    std::map<std::string,float> percent_set;
    float low=37.5; float med=62.5; float high=87.5;

    // Low (trapezoidal R function)
    a=0; b=0; c=low; d=med;
    float low_r = (d-percent)/(d-c);
    float p_m_low = fmax(fmin(1,low_r),0);
    percent_set["LOW"] = p_m_low;

    // Medium (triangle function)
    a=low; b=med; c=high;
    float med_l = (percent-a)/(b-a); // a <= percent <= b
    float med_r = (c-percent)/(c-b); // b <= percent <= c
    float p_m_med = fmax(fmin(med_l,med_r),0);
    percent_set["MED"] = p_m_med;

    // High (trapezoidal L function)
    a=med; b=high; c=100; d=100;
    float high_l = (percent-a)/(b-a);
    float p_m_high = fmax(fmin(high_l,1),0);
    percent_set["HIGH"] = p_m_high;

    // Highest value fires 
    last=0;
    for (auto it = percent_set.begin(); it != percent_set.end(); it++)
    {
        std::cout<<it->first<< " " << it->second << " ";
        if (it->second > last)
        {
            p_m = it->first;
            last = it->second;
        }
    }

    std::cout << "\n" << p_m << std::endl;

    // Defuzzify to rank
    if (d_m == "FAR" && p_m == "HIGH")  // Distance = Far
        output = 1;
    else if (d_m == "FAR" && p_m == "MED")
        output = 2;
    else if (d_m == "FAR" && p_m == "LOW")
        output = 3;
    else if (d_m == "MID" && p_m == "HIGH") // Distance = Mid
        output = 2;
    else if (d_m == "MID" && p_m == "MED")
        output = 3;
    else if (d_m == "MID" && p_m == "LOW")
        output = 4;
    else if (d_m == "CLOSE" && p_m == "HIGH") // Distance = Close
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
