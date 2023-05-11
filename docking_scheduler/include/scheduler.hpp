#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <list>
#include <iostream>
#include <string>
#include <iomanip>
#include "chrono"
#include <map>

#include "docking_interfaces/msg/charging_queue.hpp"
#include "docking_interfaces/srv/queue_update.hpp"
#include "docking_interfaces/srv/state_update.hpp"
#include "docking_interfaces/srv/rank_update.hpp"





using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;
using std::placeholders::_2;

class SchedulerNode : public rclcpp::Node 
{
    public:
        SchedulerNode() : Node("scheduler")
        {
            /*** SUBSCRIPTION DEFINITIONS***/

            /** PUBLISHER DEFINITIONS **/
            queue_pub = this->create_publisher<docking_interfaces::msg::ChargingQueue>("charging_queue", 10);
            
            /** SERVICE DEFINITIONS **/
            queue_update_service = this->create_service<docking_interfaces::srv::QueueUpdate>(
                "scheduler/queue_update_service",
                std::bind(&SchedulerNode::queueUpdateServer, this, _1, _2));
            
            RCLCPP_INFO(this->get_logger(), "Scheduler Node has been started.");
        }

        struct Robot 
        {
            std::string state;
            std::string id;
            int rank;
            float distance;
            float percent;
        };

    private:
        

        /** VARIABLES **/
        std::list<Robot> queue;

        std::vector<std::thread> threads;

        const float distance_const = 1;
        const float percent_const = 1;

        bool add_new_robot = false;
        bool state_change = false;

        int num_update = 0;

        float close_distance_threshold = 25;
        float mid_distance_threshold = 62.5;


        // Testing variables
        int num_robots = 0;
        int num_robots_done = 0;
        int total_robots = 4;
        steady_clock::time_point timer_start;
        float total_time_passed;
        bool use_timer = true;
        

        /*** INTERFACES ***/

        /*** PUBLISHER & SERVICE DECLARATIONS***/
        rclcpp::Publisher<docking_interfaces::msg::ChargingQueue>::SharedPtr queue_pub;
        rclcpp::Service<docking_interfaces::srv::QueueUpdate>::SharedPtr queue_update_service;


        /*** QUEUE UPDATE FUNCTION DECLARATIONS ***/
      
         // add new robot to queue
        bool addNewRobot(std::string id, float distance, float percent);
        bool addNewRobotV2(std::string id, float distance, float percent);

        // When a state of a robot needs to be changes
        bool stateChange(std::string id);
        

        /*** HELPER FUNCTION DECLARATIONS ***/

        // Check priority of queuing robots and adjust queue if needed
        void priorityCheck(std::string state);

        // Send states to robots 
        void sendStates();

        // Rank Update 
        void rankUpdateClient(std::string id);
        void updateRank(std::string id, float distance, float percent);

        /*** FUZZY LOGIC RANK FUNCTION DELCARATION ***/
        int fuzzify_9(float distance, float percent);
        int fuzzy_rank(float distance, float percent);
        



        // Queue Update Service: handles queue updates requests from robots
        void queueUpdateServer(
            const std::shared_ptr<docking_interfaces::srv::QueueUpdate::Request> request,
            const std::shared_ptr<docking_interfaces::srv::QueueUpdate::Response> response);

        // State Update Client: sends states to remote robots
        void stateUpdateClient(std::string id, std::string state, int num);

        // Print current charging queue to terminal
        void print_queue(std::list<Robot> const &queue)
        {
            std::cout << "-----------------------------------\n" << "Charging Queue" << std::endl;

            std::cout << "State\t  ID\t  Rank\t  D\t  %" << std::endl;

            for (auto const &robot: queue) {
                std::cout << robot.state << "\t| " << robot.id << "\t| " << robot.rank << "\t| ";
                std::cout << std::fixed << std::setprecision(2) << robot.distance << "\t| "; 
                std::cout << robot.percent << std::endl;
            }
            std::cout << "-----------------------------------\n";

            docking_interfaces::msg::ChargingQueue queue_msg;
            queue_msg.size = queue.size() - 1;
            queue_msg.send_data = false;

            queue_pub->publish(queue_msg);
        }

};


