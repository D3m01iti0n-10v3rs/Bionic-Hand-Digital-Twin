#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <functional>
#include <thread>
#include <atomic>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <sys/eventfd.h>

using namespace std::chrono_literals;

class UartNode : public rclcpp::Node{
    public:
        UartNode() : Node("uart_node"){ // call parent Node constructor, give name uart_node
            if (!uart_init()){
                RCLCPP_ERROR(this->get_logger(), "UART initialization failed");
                return;
            }

            stop_fd_ = eventfd(0, EFD_CLOEXEC);

            if (stop_fd_ < 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to create stop event");
                close(uart_fd_);
                uart_fd_ = -1;
                return;
            }

            publisher_ = this->create_publisher<std_msgs::msg::String>("stm32/cmd", 10); // create publisher that accepts ros2 string type
            subscriber_ = this->create_subscription<std_msgs::msg::String>("stm32/sensor", 10, std::bind(&UartNode::sensor_callback, this, std::placeholders::_1)); // create subscriber
            
            timer_ = this->create_wall_timer(3s, std::bind(&UartNode::timer_callback, this));

            rx_thread_ = std::thread(&UartNode::uart_receive, this);

            RCLCPP_INFO(this->get_logger(), "UART node started"); // print function. get_logger() gets ROS 2's logger for that node.
        }

        ~UartNode(){
            running_ = false;

            if (stop_fd_ >= 0){
                uint64_t signal = 1;
                write(stop_fd_, &signal, sizeof(signal));
            }

            if (rx_thread_.joinable()) rx_thread_.join();

            if (uart_fd_ >= 0) close(uart_fd_);
            if (stop_fd_ >= 0) close(stop_fd_);
        }

    private:
        std::thread rx_thread_;
        std::atomic<bool> running_{true};

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;

        int uart_fd_ = -1;
        int stop_fd_ = -1;

        int angle = 90;
        int state = 0; // 0 = inc 1 = dec

        bool uart_init(){
            uart_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);

            if (uart_fd_ < 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to open UART");
                return false;
            }

            termios tty{};

            if (tcgetattr(uart_fd_, &tty) != 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to get UART settings");
                close(uart_fd_);
                uart_fd_ = -1;
                return false;
            }

            cfsetispeed(&tty, B115200);
            cfsetospeed(&tty, B115200);

            tty.c_cflag &= ~PARENB;  // no parity
            tty.c_cflag &= ~CSTOPB;  // 1 stop bit
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;      // 8 data bits
            tty.c_cflag |= CREAD;    // enable receiver
            tty.c_cflag |= CLOCAL;   // ignore modem control

            tty.c_lflag = 0;
            tty.c_iflag = 0;
            tty.c_oflag = 0;

            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 0;

            if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to configure UART");
                close(uart_fd_);
                uart_fd_ = -1;
                return false;
            }

            RCLCPP_INFO(this->get_logger(), "UART initialized");

            return true;
        }

        void uart_send(const std_msgs::msg::String &msg){
            if (uart_fd_ < 0) return;

            write(uart_fd_, msg.data.c_str(), msg.data.size());
            // write(uart_fd_, "\n", 1);
        }

        void uart_receive(){
            char buffer[128];

            struct pollfd fds[2];

            fds[0].fd = uart_fd_;
            fds[0].events = POLLIN;

            fds[1].fd = stop_fd_;
            fds[1].events = POLLIN;

            while (running_){
                int result = poll(fds, 2, -1);

                if (result < 0){
                    if (errno == EINTR) continue;

                    RCLCPP_ERROR(this->get_logger(), "poll() failed");
                    break;
                }

                if (fds[1].revents & POLLIN){
                    break;
                }

                if (fds[0].revents & POLLIN){
                    ssize_t bytes_read = read(uart_fd_, buffer, sizeof(buffer));

                    if (bytes_read > 0){
                        std::string data(buffer, bytes_read);

                        while (!data.empty() && (data.back() == '\r' || data.back() == '\n'))
                            data.pop_back();

                        if (!data.empty())
                            RCLCPP_INFO(this->get_logger(), "STM SENT: %s", data.c_str());
                    }
                }
            }
        }
        
        void timer_callback(){
            char buffer[64];
            
            std_msgs::msg::String msg;

            for (int i = 0; i < 5; i++){
                snprintf(buffer, sizeof(buffer), "FINGER%d %d\r\n", i, angle);

                msg.data = buffer;

                uart_send(msg);
                publisher_->publish(msg); // probably doesnt need a publisher at all
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
