#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <functional>
#include <stdio.h>

using namespace std::chrono_literals;

class UartNode : public rclcpp::Node{
    public:
        UartNode() : Node("uart_node"){ // call parent Node constructor, give name uart_node
            RCLCPP_INFO(this->get_logger(), "UART node started"); // print function. get_logger() gets ROS 2's logger for that node.
            
            publisher_ = this->create_publisher<std_msgs::msg::String>("stm32/cmd", 10); // create publisher that accepts ros2 string type
            subscriber_ = this->create_subscription<std_msgs::msg::String>("stm32/sensor", 10, std::bind(&UartNode::sensor_callback, this, std::placeholders::_1)); // create subscriber
            
            timer_ = this->create_wall_timer(3s, std::bind(&UartNode::timer_callback, this));
    }

    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        int angle = 90;
        int state = 0; // 0 = inc 1 = dec
        
        void timer_callback(){
            char buffer[64];
            
            std_msgs::msg::String msg;

            for (int i = 0; i < 5; i++){
                snprintf(buffer, sizeof(buffer), "FINGER%d %d", i, angle);

                msg.data = buffer;
                publisher_->publish(msg);
            }

            switch (state){
                case 0:
                    angle += 30;
                    if (angle >= 180) state = 1;
                    break;
                case 1:
                    angle -= 30;
                    if (angle <= 0) state = 0;
                    break;
            }
        }

        void sensor_callback(const std_msgs::msg::String::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
        }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize ROS 2

    auto node = std::make_shared<UartNode>(); // Create a UartNode object.

    rclcpp::spin(node); // keeps the node running and waits for ROS 2 events/callbacks.

    rclcpp::shutdown(); // Shut down ROS 2

    return 0;
}