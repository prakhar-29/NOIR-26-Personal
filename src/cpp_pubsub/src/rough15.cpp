#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

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

        // NEW: Subscribe to cone coordinates from camera
        cone_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>("/cone_coordinates", 10,
            std::bind(&gps_controller_cin::cone_callback, this, std::placeholders::_1));

        /* -------- PUBLISHERS -------- */
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
    }

private:
    /* -------- ROS HANDLES -------- */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cone_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    /* -------- STATE -------- */
    geographic_msgs::msg::GeoPoint goal_geo_;
    double goal_e_, goal_n_;
    double imu_heading_rad_{0.0};

    float left_min_{10.0};
    float right_min_{10.0};
    float front_min_{10.0};
    bool obstacle_{false};

    // Camera-based cone detection state
    bool cone_detected_camera_{false};
    float cone_center_x_{0.0};
    float cone_center_y_{0.0};
    float cone_class_id_{0.0};
    rclcpp::Time last_cone_detection_time_;

    // State machine
    enum class State {
        NAVIGATE_TO_GOAL,
        CONE_SEARCH_AND_APPROACH,
        CONE_ALIGN,
        CONE_APPROACH_FINAL
    };
    State current_state_{State::NAVIGATE_TO_GOAL};

    /* -------- PARAMETERS -------- */
    const float OBSTACLE_THRESHOLD = 1.5;      // Detection distance for navigation
    const float CRITICAL_DISTANCE = 0.5;       // Emergency stop distance
    const float AVOIDANCE_GAIN = 1.1;          // How aggressively to avoid
    const float GOAL_GAIN = 1.1;               // How much to bias toward goal
    const float CONE_SEARCH_RADIUS = 1.0;      // Start cone search within 2.5m
    
    // Camera parameters
    const float CAMERA_WIDTH = 640.0;          // Camera resolution width (adjust to your camera)
    const float CAMERA_HEIGHT = 480.0;         // Camera resolution height
    const float CAMERA_CENTER_X = CAMERA_WIDTH / 2.0;
    const float ALIGNMENT_TOLERANCE = 50.0;    // Pixels - cone must be within this range of center
    const float CONE_DETECTION_TIMEOUT = 1.0;  // Seconds - max time since last detection
    
    // Cone approach parameters
    const double CONE_STOP_THRESHOLD = 0.5;   // Stop when 0.45m away from cone (LiDAR based)
    const double CONE_LINEAR_VEL = 0.15;       // Linear velocity towards cone
    const double CONE_ROTATION_VEL = 0.25;      // Rotation velocity for searching
    const double CONE_ALIGN_KP = 0.003;        // Proportional gain for alignment (angular velocity per pixel error)
    const double MAX_ALIGN_ANGULAR_VEL = 0.25;  // Maximum angular velocity during alignment

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

    bool is_cone_detection_valid()
    {
        if (!cone_detected_camera_) return false;
        
        auto now = this->now();
        double time_since_detection = (now - last_cone_detection_time_).seconds();
        
        return time_since_detection < CONE_DETECTION_TIMEOUT;
    }

    double calculate_alignment_error()
    {
        // Calculate pixel error from center
        // Positive error means cone is to the right, need to turn right (negative angular velocity)
        // Negative error means cone is to the left, need to turn left (positive angular velocity)
        return cone_center_x_ - CAMERA_CENTER_X;
    }

    bool is_cone_aligned()
    {
        double error = std::abs(calculate_alignment_error());
        return error < ALIGNMENT_TOLERANCE;
    }

    /* -------- CALLBACKS -------- */

    void cone_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        // Data format: [cx1, cy1, class_id1, cx2, cy2, class_id2, ...]
        // For simplicity, we'll use the first detected cone
        
        if (msg->data.size() >= 3 && msg->data[0] != 0.0)
        {
            cone_detected_camera_ = true;
            cone_center_x_ = msg->data[0];
            cone_center_y_ = msg->data[1];
            cone_class_id_ = msg->data[2];
            last_cone_detection_time_ = this->now();
            
            RCLCPP_DEBUG(get_logger(), "Cone detected at pixel: (%.1f, %.1f), class: %.0f", 
                        cone_center_x_, cone_center_y_, cone_class_id_);
        }
        else
        {
            cone_detected_camera_ = false;
        }
    }

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

        /* -------- STATE MACHINE -------- */
        
        if (current_state_ == State::NAVIGATE_TO_GOAL)
        {
            // Check if within 2.5m of goal - start cone search
            if (dist < CONE_SEARCH_RADIUS)
            {
                current_state_ = State::CONE_SEARCH_AND_APPROACH;
                RCLCPP_INFO(get_logger(), "Within %.1fm of goal. Starting camera-based cone detection...", CONE_SEARCH_RADIUS);
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
                        cmd.linear.x = 1*0.8;
                    } else {
                        cmd.linear.x = clamp(dist / 5.0, 0.0, 1.0);
                    }
                    cmd.angular.z = 1*0.8;
                }
                
                cmd_pub_->publish(cmd);
                return;
            }
        }
        
        if (current_state_ == State::CONE_SEARCH_AND_APPROACH)
        {
            // Use camera to detect cone
            if (is_cone_detection_valid())
            {
                // Cone detected by camera - transition to alignment
                current_state_ = State::CONE_ALIGN;
                RCLCPP_INFO(get_logger(), "Cone detected by camera! Starting alignment...");
            }
            else
            {
                // No cone detected - rotate to search
                cmd.linear.x = 0.0;
                cmd.angular.z = CONE_ROTATION_VEL;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Searching for cone with camera...");
                cmd_pub_->publish(cmd);
                return;
            }
        }
        
        if (current_state_ == State::CONE_ALIGN)
        {
            // Check if cone is still detected
            if (!is_cone_detection_valid())
            {
                // Lost cone - go back to search
                current_state_ = State::CONE_SEARCH_AND_APPROACH;
                RCLCPP_WARN(get_logger(), "Lost cone detection! Returning to search...");
                cmd_pub_->publish(cmd);
                return;
            }
            
            // Check if aligned
            if (is_cone_aligned())
            {
                // Aligned - move to final approach
                current_state_ = State::CONE_APPROACH_FINAL;
                RCLCPP_INFO(get_logger(), "Cone aligned! Starting final approach...");
            }
            else
            {
                // Calculate alignment correction
                double pixel_error = calculate_alignment_error();
                
                // Negative pixel_error means cone is left, need positive angular (turn left)
                // Positive pixel_error means cone is right, need negative angular (turn right)
                cmd.angular.z = -CONE_ALIGN_KP * pixel_error;
                cmd.angular.z = clamp(cmd.angular.z, -MAX_ALIGN_ANGULAR_VEL, MAX_ALIGN_ANGULAR_VEL);
                
                // Slow forward movement while aligning
                cmd.linear.x = 0.05;
                
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "Aligning: Cone at pixel %.1f (error: %.1f), Angular vel: %.3f",
                    cone_center_x_, pixel_error, cmd.angular.z);
                
                cmd_pub_->publish(cmd);
                return;
            }
        }
        
        if (current_state_ == State::CONE_APPROACH_FINAL)
        {
            // Check if cone is still detected
            if (!is_cone_detection_valid())
            {
                // Lost cone - go back to search
                current_state_ = State::CONE_SEARCH_AND_APPROACH;
                RCLCPP_WARN(get_logger(), "Lost cone during approach! Returning to search...");
                cmd_pub_->publish(cmd);
                return;
            }
            
            // Check LiDAR distance
            float closest_distance = std::min({left_min_, right_min_, front_min_});
            
            if (closest_distance <= CONE_STOP_THRESHOLD)
            {
                // Reached cone - stop
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                RCLCPP_INFO(get_logger(), "SUCCESS! Reached cone at distance: %.2fm", closest_distance);
                cmd_pub_->publish(cmd);
                rclcpp::shutdown();
                return;
            }
            else
            {
                // Continue approach with minor alignment corrections
                double pixel_error = calculate_alignment_error();
                
                // Small alignment correction during approach
                cmd.angular.z = -CONE_ALIGN_KP * pixel_error * 0.5; // Reduced correction during approach
                cmd.angular.z = clamp(cmd.angular.z, -MAX_ALIGN_ANGULAR_VEL * 0.5, MAX_ALIGN_ANGULAR_VEL * 0.5);
                
                // Move forward
                cmd.linear.x = CONE_LINEAR_VEL;
                
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "Approaching cone: Distance %.2fm, Pixel %.1f, Linear: %.2f, Angular: %.3f",
                    closest_distance, cone_center_x_, cmd.linear.x, cmd.angular.z);
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