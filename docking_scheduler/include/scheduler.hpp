#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <list>
#include <iostream>
#include <string>

#include "rclcpp_action/rclcpp_action.hpp"

#include "docking_interfaces/srv/queue_update.hpp"
#include "docking_interfaces/msg/charging_queue.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"


using namespace std::chrono_literals;
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
            queue_pub = this->create_publisher<docking_interfaces::msg::ChargingQueue>("charging_queue", 10);
            
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
        std::list<Robot> queue;

        const float distance_const = 1;
        const float percent_const = 1;

        bool add_new_robot = false;
        bool state_change = false;

        // bool docking_success = false;
        // bool queuing_success = false;
        // bool charging_success = false;
        

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
        
        // Queue Update Service
        void queueUpdateServer(
            const std::shared_ptr<docking_interfaces::srv::QueueUpdate::Request> request,
            const std::shared_ptr<docking_interfaces::srv::QueueUpdate::Response> response);

        // Print current charging queue to terminal
        void print_queue(std::list<Robot> const &queue)
        {
            std::cout << "State\t  ID\t  Rank\t  D\t  %" << std::endl;

            for (auto const &robot: queue) {
                std::cout << robot.state << "\t| " << robot.id << "\t| " << robot.rank << "\t| ";
                std::cout << robot.distance << "\t| " << robot.percent << std::endl;
            }

            docking_interfaces::msg::ChargingQueue queue_msg;
            queue_msg.size = queue.size() - 1;

            queue_pub->publish(queue_msg);
        }

};


