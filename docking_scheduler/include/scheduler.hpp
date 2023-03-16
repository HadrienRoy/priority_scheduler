#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <list>
#include <iostream>
#include <string>
#include "docking_interfaces/srv/queue_update.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class SchedulerNode : public rclcpp::Node 
{
    public:
        SchedulerNode() : Node("scheduler")
        {
            /*** SUBSCRIPTION DEFINITIONS***/
            // my_subscriber = this->create_subscription<example_interfaces::msg::Int64>(
            //     "topic_name", 10, std::bind(&SchedulerNode::callbackSub, this, std::placeholders::_1));

            /** CLIENT DEFINITIONS **/

            /** PUBLISHER DEFINITIONS **/
            // my_publisher = this->create_publisher<example_interfaces::msg::Int64>("topic_name", 10);
            
            /** SERVICE DEFINITIONS **/
            queue_update_service = this->create_service<docking_interfaces::srv::QueueUpdate>(
                "scheduler/queue_update_service",
                std::bind(&SchedulerNode::queueUpdateServer, this, _1, _2));

            
            // SERVICE to change CHARGING_COMPLETE -> robot will have client that calls when done
            
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
        // char queue[5][10];
        std::list<Robot> queue;

        const float distance_const = 1;
        const float percent_const = 1;

        bool add_new_robot = false;
        bool state_change = false;

        // bool docking_success = false;
        // bool queuing_success = false;
        // bool charging_success = false;
        

        /*** INTERFACES ***/

        /*** SERVICE DECLARATIONS***/
        rclcpp::Service<docking_interfaces::srv::QueueUpdate>::SharedPtr queue_update_service;


        // Main Function to handle scheduling
        void priorityScheduler(std::string id, float distance, float percent);
       
        // add new robot to queue
        bool addNewRobot(std::string id, float distance, float percent);
        
        // Check priority of queuing robots and adjust queue if needed
        void priorityCheck(std::string id, float distance, float percent);

        // When a state of a robot needs to be changes
        bool stateChange(std::string id);
        
        // Send states to robots 
        void sendStates();
        
        // Queue Update Service
        void queueUpdateServer(
            const std::shared_ptr<docking_interfaces::srv::QueueUpdate::Request> request,
            const std::shared_ptr<docking_interfaces::srv::QueueUpdate::Response> response);



        void print_queue(std::list<Robot> const &queue)
        {
            for (auto const &robot: queue) {
                std::cout << robot.state << "\t| " << robot.id << std::endl;
            }
        }

};


