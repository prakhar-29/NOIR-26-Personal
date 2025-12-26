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
    }

private:
    /* -------- ROS HANDLES -------- */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

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
        CONE_DETECTION
    };
    State current_state_{State::NAVIGATE_TO_GOAL};

    /* -------- PARAMETERS -------- */
    // Navigation parameters
    const float OBSTACLE_THRESHOLD = 1.0;      // Detection distance for navigation
    const float CRITICAL_DISTANCE = 0.3;       // Emergency stop distance
    const float AVOIDANCE_GAIN = 1.3;          // How aggressively to avoid
    const float GOAL_GAIN = 1.0;               // How much to bias toward goal
    const float CONE_SEARCH_RADIUS = 2.0;      // Start cone search within 2m
    
    // Cone detection parameters (from ttlcode)
    const double CONE_DETECTION_THRESHOLD = 2.0;  // Detect cone within 2m
    const double CONE_STOP_THRESHOLD = 0.3;       // Stop when 0.3m away from cone
    const double CONE_LIN_VEL = 0.2;              // Linear velocity when approaching cone
    const double CONE_ROT_VEL = 0.5;              // Rotation velocity for searching
    const double CONE_KPZ = 0.15;                  // Angular gain for turning towards cone (reduced for smoothness)
    const double CONE_ANGLE_THRESHOLD = 0.15;     // Acceptable angular error (radians, ~8.5 degrees)
    
    // Smoothing variables
    double prev_angular_z_{0.0};
    const double ANGULAR_SMOOTHING = 0.3;         // Smoothing factor for angular velocity

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
        
        // Handle cone detection mode
        if (current_state_ == State::CONE_DETECTION)
        {
            cone_detection_behavior(msg);
        }
    }

    void cone_detection_behavior(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto cmd = geometry_msgs::msg::Twist();
        
        // Calculate indices for ±75° field of view
        int stid = ((-30.0 - msg->angle_min) / msg->angle_increment);
        int endid = ((30.0 - msg->angle_min) / msg->angle_increment);
        stid = std::max(0, stid);
        endid = std::min(static_cast<int>(msg->ranges.size()) - 1, endid);
        int mid = stid + (endid - stid) / 2;
        
        float ld = msg->range_max;
        float rd = msg->range_max;
        float center_d = msg->range_max;
        
        // Find minimum distance on left side
        for (int i = stid; i <= mid; i++)
        {
            float instdis = msg->ranges[i];
            if (instdis > 0.0 && instdis < ld)
            {
                ld = instdis;
            }
        }
        
        // Find minimum distance on right side
        for (int j = mid + 1; j <= endid; j++)
        {
            float instdis = msg->ranges[j];
            if (instdis > 0.0 && instdis < rd)
            {
                rd = instdis;
            }
        }
        
        // Check center region (±10 degrees) for more accurate forward distance
        int center_range = static_cast<int>(10.0 / (msg->angle_increment * 180.0 / M_PI));
        for (int i = mid - center_range; i <= mid + center_range; i++)
        {
            if (i >= 0 && i < static_cast<int>(msg->ranges.size()))
            {
                float instdis = msg->ranges[i];
                if (instdis > 0.0 && instdis < center_d)
                {
                    center_d = instdis;
                }
            }
        }
        
        float closest_distance = std::min(ld, rd);
        
        // Check if obstacle (cone) is detected within detection threshold
        bool obstacle_detected = (closest_distance < CONE_DETECTION_THRESHOLD);
        
        if (!obstacle_detected)
        {
            // No obstacle detected - rotate 360 to search
            cmd.linear.x = 0.0;
            cmd.angular.z = CONE_ROT_VEL;
            prev_angular_z_ = cmd.angular.z;
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                "No cone detected. Searching...");
        }
        else if (center_d <= CONE_STOP_THRESHOLD)
        {
            // Close enough - stop
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            RCLCPP_INFO(get_logger(), "Reached cone! Stopping at distance: %.2fm", center_d);
            cmd_pub_->publish(cmd);
            rclcpp::shutdown();
            return;
        }
        else
        {
            // Calculate angular difference to center the cone
            float angular_diff = 0.0;
            
            if (std::abs(ld - rd) < 0.1)
            {
                // Cone is approximately centered
                angular_diff = 0.0;
            }
            else if (ld < rd)
            {
                // Cone is on left - positive angular velocity
                angular_diff = CONE_KPZ * (rd - ld);
            }
            else
            {
                // Cone is on right - negative angular velocity
                angular_diff = -CONE_KPZ * (ld - rd);
            }
            
            // Apply smoothing to angular velocity
            cmd.angular.z = ANGULAR_SMOOTHING * angular_diff + (1.0 - ANGULAR_SMOOTHING) * prev_angular_z_;
            cmd.angular.z = clamp(cmd.angular.z, -0.5, 0.5);
            prev_angular_z_ = cmd.angular.z;
            
            // Adjust linear velocity based on alignment
            if (std::abs(cmd.angular.z) > CONE_ANGLE_THRESHOLD)
            {
                // Need significant turning - slow down
                cmd.linear.x = CONE_LIN_VEL * 0.5;
            }
            else if (closest_distance < 0.8)
            {
                // Very close - slow down
                cmd.linear.x = CONE_LIN_VEL * 0.6;
            }
            else
            {
                // Good alignment and reasonable distance - normal speed
                cmd.linear.x = 1.0*CONE_LIN_VEL;
            }
            
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                "Approaching cone. L:%.2fm R:%.2fm Center:%.2fm | Turn:%.2f", 
                ld, rd, center_d, cmd.angular.z);
        }
        
        cmd_pub_->publish(cmd);
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

        /* -------- STATE MACHINE -------- */
        
        if (current_state_ == State::NAVIGATE_TO_GOAL)
        {
            // Check if within 2m of goal - switch to cone detection mode
            if (dist < CONE_SEARCH_RADIUS)
            {
                current_state_ = State::CONE_DETECTION;
                RCLCPP_INFO(get_logger(), "Within 2m of goal. Switching to cone detection mode...");
                // The cone detection will be handled in lidar_callback
                return;
            }

            // Normal navigation with obstacle avoidance
            if (obstacle_)
            {
                // Calculate repulsive forces from obstacles
                double avoidance_angle = 0.0;
                
                if (front_min_ < OBSTACLE_THRESHOLD) {
                    if (left_min_ < right_min_) {
                        avoidance_angle = -1.0;  // Turn right
                    } else {
                        avoidance_angle = 1.0;   // Turn left
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
                    cmd.linear.x = 1.0;
                } else {
                    cmd.linear.x = clamp(dist / 5.0, 0.0, 1.0);
                }
                cmd.angular.z = ang;
            }
            
            cmd_pub_->publish(cmd);
        }
        // CONE_DETECTION state is handled in lidar_callback
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gps_controller_cin>());
    rclcpp::shutdown();
    return 0;
}