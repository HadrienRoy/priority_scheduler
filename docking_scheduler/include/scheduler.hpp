#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <list>
#include <iostream>
#include <string>
#include <iomanip>

#include "docking_interfaces/msg/charging_queue.hpp"
#include "docking_interfaces/srv/queue_update.hpp"
#include "docking_interfaces/srv/state_update.hpp"
#include "docking_interfaces/srv/rank_update.hpp"





using namespace std::chrono_literals;
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
        

        /*** INTERFACES ***/

        /*** PUBLISHER & SERVICE DECLARATIONS***/
        rclcpp::Publisher<docking_interfaces::msg::ChargingQueue>::SharedPtr queue_pub;
        rclcpp::Service<docking_interfaces::srv::QueueUpdate>::SharedPtr queue_update_service;


        /*** FUNCTION DECLARATIONS ***/

        // Main Function to handle scheduling
        void priorityScheduler(std::string id, float distance, float percent);
       
        // add new robot to queue
        bool addNewRobot(std::string id, float distance, float percent);
        
        // Check priority of queuing robots and adjust queue if needed
        void priorityCheck(std::string state);

        // When a state of a robot needs to be changes
        bool stateChange(std::string id);

        
        
        // Send states to robots 
        void sendStates();

        int fuzzify(float distance, float percent);
        int fuzzify_9(float distance, float percent);
        
        // Queue Update Service: handles queue updates requests from robots
        void queueUpdateServer(
            const std::shared_ptr<docking_interfaces::srv::QueueUpdate::Request> request,
            const std::shared_ptr<docking_interfaces::srv::QueueUpdate::Response> response);

        // State Update Client: sends states to remote robots
        void stateUpdateClient(std::string id, std::string state, int num);

        // Rank Update 
        bool addNewRobotV2(std::string id, float distance, float percent);
        void rankUpdateClient(std::string id);
        void updateRank(std::string id, float distance, float percent);
        

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

            docking_interfaces::msg::ChargingQueue queue_msg;
            queue_msg.size = queue.size() - 1;
            queue_msg.send_data = false;

            queue_pub->publish(queue_msg);
        }

};


