#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int8.hpp"
#include <iostream>
using namespace std;

class ttlcode : public rclcpp::Node
{
public:
    ttlcode() : Node("dual_ps4_node")
    {
        pub_drive = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
        pub_arm = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel2", 10);
        pub_stepper = this->create_publisher<std_msgs::msg::Int8>("/stepper_cmd", 10);
        
        sub_drive = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy1", 10, std::bind(&ttlcode::joy1, this, std::placeholders::_1));
        sub_arm = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy2", 10, std::bind(&ttlcode::joy2, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Dual PS4 control node started!");
    }

private:
    void joy1(sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto v_drive = geometry_msgs::msg::Twist();
        v_drive.linear.x = msg->axes[1];      // left stick vertical
        v_drive.angular.z = (msg->axes[3]);   // right stick horizontal
        pub_drive->publish(v_drive);
    }

    void joy2(sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto v_arm = geometry_msgs::msg::Twist();
        auto stepper_msg = std_msgs::msg::Int8();

        // Stepper control with Square and Circle buttons
        if(msg->buttons[3] != 0)  // Square button - move left
        {
            stepper_msg.data = -1;
            pub_stepper->publish(stepper_msg);
        }
        else if(msg->buttons[1] != 0)  // Circle button - move right
        {
            stepper_msg.data = 1;
            pub_stepper->publish(stepper_msg);
        }
        else if(msg->buttons[2] != 0)  // Triangle led on
        {
            stepper_msg.data = 10;
            pub_stepper->publish(stepper_msg);
        }
        else if(msg->buttons[0] != 0)  // Cross led off
        {
            stepper_msg.data = -10;
            pub_stepper->publish(stepper_msg);
        }

        // Default joystick control when no buttons pressed
        if(msg->buttons[0] == 0 && msg->buttons[1] == 0 && 
           msg->buttons[2] == 0 && msg->buttons[3] == 0)
        {
            v_arm.linear.x = msg->axes[1];       // left stick vertical
            v_arm.angular.z = (-msg->axes[3]);   // right stick horizontal
        }
        
        pub_arm->publish(v_arm);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_drive;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_arm;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_stepper;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_drive;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_arm;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ttlcode>());
    rclcpp::shutdown();
    return 0;
}