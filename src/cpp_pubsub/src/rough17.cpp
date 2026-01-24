#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"

#include <geographic_msgs/msg/geo_point.hpp>
#include <geodesy/utm.h>
#include <cmath>
#include <iostream>
#include <limits>

class gps_controller_cin : public rclcpp::Node
{
public:
    gps_controller_cin() : Node("gps_controller_cin")
    {
        /* -------- GOAL INPUT (ONCE) -------- */
        std::cout << "Enter goal latitude  : ";
        std::cin >> goal_geo_.latitude;
        std::cout << "Enter goal longitude : ";
        std::cin >> goal_geo_.longitude;

        goal_geo_.altitude = 0.0;

        geodesy::UTMPoint goal_utm;
        geodesy::fromMsg(goal_geo_, goal_utm);
        goal_e_ = goal_utm.easting;
        goal_n_ = goal_utm.northing;

        RCLCPP_INFO(get_logger(), "Goal locked, starting navigation");

        /* -------- SUBSCRIBERS -------- */
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/fix", 10,
            std::bind(&gps_controller_cin::gps_callback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<std_msgs::msg::Int32>("/imu_data", 10,
            std::bind(&gps_controller_cin::imu_callback, this, std::placeholders::_1));

        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/ldlidar_node/scan", 10,
            std::bind(&gps_controller_cin::lidar_callback, this, std::placeholders::_1));

        /* -------- PUBLISHERS -------- */
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
        arm_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel2", 10);
        feedback_pub_ = create_publisher<geometry_msgs::msg::Twist>("/navigation_feedback", 10);
    }

private:
    /* -------- ROS HANDLES -------- */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr feedback_pub_;

    /* -------- STATE -------- */
    geographic_msgs::msg::GeoPoint goal_geo_;
    double goal_e_, goal_n_;
    double imu_heading_rad_{0.0};

    float left_min_{10.0};
    float right_min_{10.0};
    float front_min_{10.0};
    bool obstacle_{false};

    // State machine
    enum class State {
        NAVIGATE_TO_GOAL,
        CONE_SEARCH_AND_APPROACH
    };
    State current_state_{State::NAVIGATE_TO_GOAL};

    /* -------- PARAMETERS -------- */
    const float OBSTACLE_THRESHOLD = 1.5;      // Detection distance for navigation
    const float CRITICAL_DISTANCE = 0.5;       // Emergency stop distance
    const float AVOIDANCE_GAIN = 0.9;          // How aggressively to avoid
    const float GOAL_GAIN = 1.1;               // How much to bias toward goal
    const float CONE_SEARCH_RADIUS = 1.0;      // Start cone search within x m
    
    // Cone attraction parameters (from your provided code)
    const double CONE_DETECTION_THRESHOLD = 2.5;  // Detect cone within x m
    const double CONE_STOP_THRESHOLD = 0.5;       // Stop when x m away from cone
    const double CONE_LINEAR_VEL = 0.15;          // Linear velocity towards cone
    const double CONE_ROTATION_VEL = -0.45;         // Rotation velocity for searching
    const double CONE_KPZ = 0.25;                  // Angular gain for turning towards cone
    const double smooth_factor = 0.8;              // overall smoothener for the whole mission

    /* -------- GRIPPER DROP FUNCTION -------- */
    void drop_object()
    {
        RCLCPP_INFO(get_logger(), "Opening gripper to drop object...");
        
        geometry_msgs::msg::Twist gripper_cmd;
        // Set linear.y to open the gripper (motor index 0 - cross button)
        // Positive value opens, negative closes based on your control logic
        gripper_cmd.linear.x = 0.0;
        gripper_cmd.linear.y = 1.0;  // Open gripper
        gripper_cmd.linear.z = 0.0;
        gripper_cmd.angular.x = 0.0;
        gripper_cmd.angular.y = 0.0;
        gripper_cmd.angular.z = 0.0;
        
        // Publish gripper open command for 10-15 seconds
        auto start_time = this->now();
        auto drop_duration = rclcpp::Duration::from_seconds(4.0);  // 12 seconds
        
        rclcpp::Rate rate(10);  // 10 Hz
        while ((this->now() - start_time) < drop_duration)
        {
            arm_cmd_pub_->publish(gripper_cmd);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, 
                "Gripper opening... %.1f seconds remaining",
                (drop_duration - (this->now() - start_time)).seconds());
            rate.sleep();
        }
        
        // Stop gripper
        gripper_cmd.linear.y = 0.0;
        arm_cmd_pub_->publish(gripper_cmd);
        
        RCLCPP_INFO(get_logger(), "Object dropped successfully!");
    }

    /* -------- UTILS -------- */
    double clamp(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    /* -------- CALLBACKS -------- */

    void imu_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        imu_heading_rad_ = msg->data * M_PI / 180.0;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        left_min_  = msg->range_max;
        right_min_ = msg->range_max;
        front_min_ = msg->range_max;

        int total = msg->ranges.size();
        int third = total / 3;

        // Left sector (0 to 1/3)
        for (int i = 0; i < third; i++)
        {
            float d = msg->ranges[i];
            if (d > 0.05 && d < left_min_){
                left_min_ = d;
            }
        }

        // Front sector (1/3 to 2/3)
        for (int i = third; i < 2 * third; i++)
        {
            float d = msg->ranges[i];
            if (d > 0.05 && d < front_min_){
                front_min_ = d;
            }
        }

        // Right sector (2/3 to end)
        for (size_t i = 2 * third; i < total; i++)
        {
            float d = msg->ranges[i];
            if (d > 0.05 && d < right_min_)
            {
                right_min_ = d;
            }
        }

        obstacle_ = (left_min_ < OBSTACLE_THRESHOLD || right_min_ < OBSTACLE_THRESHOLD || front_min_ < OBSTACLE_THRESHOLD);
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;

        /* -------- CURRENT POSITION -------- */
        geographic_msgs::msg::GeoPoint curr_geo;
        curr_geo.latitude  = msg->latitude;
        curr_geo.longitude = msg->longitude;
        curr_geo.altitude  = msg->altitude;

        geodesy::UTMPoint curr_utm;
        geodesy::fromMsg(curr_geo, curr_utm);

        double dx = goal_e_ - curr_utm.easting;
        double dy = goal_n_ - curr_utm.northing;
        double dist = std::hypot(dx, dy);

        /* -------- GOAL HEADING -------- */
        double desired_heading = atan2(dx, dy);
        double angle_error = normalize_angle(-(desired_heading - imu_heading_rad_));

        /* -------- PUBLISH FEEDBACK -------- */
        geometry_msgs::msg::Twist feedback;
        feedback.linear.x = dist;  // Distance to goal in meters
        feedback.angular.z = angle_error;  // Angle error in radians
        feedback_pub_->publish(feedback);

        /* -------- STATE MACHINE -------- */
        
        if (current_state_ == State::NAVIGATE_TO_GOAL)
        {
            // Check if within 2m of goal - start cone search
            if (dist < CONE_SEARCH_RADIUS)
            {
                current_state_ = State::CONE_SEARCH_AND_APPROACH;
                RCLCPP_INFO(get_logger(), "Within 2m of goal. Starting cone search and approach...");
                // Don't return - let it fall through to cone logic
            }
            else
            {
                // Normal navigation with obstacle avoidance
                if (obstacle_)
                {
                    // Calculate repulsive forces from obstacles
                    double avoidance_angle = 0.0;
                    
                    if (front_min_ < OBSTACLE_THRESHOLD) {
                        if (left_min_ < right_min_) {
                            avoidance_angle = -1.0 * smooth_factor;  // Turn right
                        } else {
                            avoidance_angle = 1.0 * smooth_factor;   // Turn left
                        }
                    } else {
                        double left_repulsion = (OBSTACLE_THRESHOLD - left_min_) / OBSTACLE_THRESHOLD;
                        double right_repulsion = (OBSTACLE_THRESHOLD - right_min_) / OBSTACLE_THRESHOLD;
                        avoidance_angle = (right_repulsion - left_repulsion);
                    }

                    double goal_weight = GOAL_GAIN;
                    double avoidance_weight = AVOIDANCE_GAIN;
                    
                    if (front_min_ < CRITICAL_DISTANCE || 
                        left_min_ < CRITICAL_DISTANCE || 
                        right_min_ < CRITICAL_DISTANCE) {
                        goal_weight = 0.2;
                        avoidance_weight = 2.0;
                        cmd.linear.x = 0.05;
                    } else {
                        cmd.linear.x = 0.15;
                    }

                    double goal_turn = clamp(angle_error / M_PI, -1.0, 1.0);
                    cmd.angular.z = (goal_weight * goal_turn + 
                                    avoidance_weight * avoidance_angle);
                    cmd.angular.z = clamp(cmd.angular.z, -1.0, 1.0);

                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "Avoiding | L:%.2f F:%.2f R:%.2f | Goal angle:%.1f° Turn:%.2f",
                        left_min_, front_min_, right_min_, 
                        angle_error * 180.0 / M_PI, cmd.angular.z);
                }
                else
                {
                    double ang = clamp(angle_error / M_PI, -1.0, 1.0);
                    
                    if(dist > 5.0){
                        cmd.linear.x = 1*smooth_factor;
                    } else {
                        cmd.linear.x = clamp(dist / 5.0, 0.0, 1.0);
                    }
                    cmd.angular.z = ang * smooth_factor;
                }
                
                cmd_pub_->publish(cmd);
                return;
            }
        }
        
        if (current_state_ == State::CONE_SEARCH_AND_APPROACH)
        {
            // Use the cone attraction logic from your provided code
            float closest_distance = std::min(left_min_, right_min_);
            
            // Check if cone is detected within detection threshold
            bool cone_detected = (closest_distance < CONE_DETECTION_THRESHOLD);
            
            if (!cone_detected)
            {
                // No cone detected - rotate 360 to search
                cmd.linear.x = 0.0;
                cmd.angular.z = CONE_ROTATION_VEL;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "No cone detected. Searching... L:%.2f R:%.2f", left_min_, right_min_);
            }
            else if (closest_distance <= CONE_STOP_THRESHOLD)
            {
                // Close enough - stop and drop object
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                cmd_pub_->publish(cmd);
                
                RCLCPP_INFO(get_logger(), "Reached cone! Stopping at distance: %.2fm", closest_distance);
                
                // Drop the object before shutting down
                drop_object();
                
                rclcpp::shutdown();
                return;
            }
            else
            {
                // Cone detected but not close enough - move towards it
                cmd.linear.x = CONE_LINEAR_VEL;
                
                // Turn towards the closer obstacle (cone)
                if (left_min_ < right_min_)
                {
                    // Cone on left - turn left (positive angular z)
                    cmd.angular.z = CONE_KPZ * (CONE_DETECTION_THRESHOLD - left_min_);
                }
                else
                {
                    // Cone on right - turn right (negative angular z)
                    cmd.angular.z = -CONE_KPZ * (CONE_DETECTION_THRESHOLD - right_min_);
                }
                
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "Moving towards cone. Left: %.2fm, Right: %.2fm, Closest: %.2fm", 
                    left_min_, right_min_, closest_distance);
            }
        }
        
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gps_controller_cin>());
    rclcpp::shutdown();
    return 0;
}