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
#include <vector>

class gps_controller_cin : public rclcpp::Node
{
public:
    gps_controller_cin() : Node("gps_controller_cin")
    {
        /* -------- CONE SELECTION MENU -------- */
        std::cout << "\n========================================\n";
        std::cout << "     CONE DETECTION SELECTION MENU\n";
        std::cout << "========================================\n";
        std::cout << "Select the cone type you want to detect:\n";
        std::cout << "  1 - Yellow cone\n";
        std::cout << "  2 - Blue cone\n";
        std::cout << "  3 - Orange cone\n";
        std::cout << "  4 - Large Orange cone\n";
        std::cout << "  5 - Red cone\n";
        std::cout << "  6 - Green cone\n";
        std::cout << "  7 - Other cones\n";
        std::cout << "  9 - Any cone (all types)\n";
        std::cout << "========================================\n";
        std::cout << "Enter cone index: ";
        
        int cone_selection;
        std::cin >> cone_selection;
        
        // Configure which cone classes to detect
        if (cone_selection == 9) {
            target_cone_classes_ = {1, 2, 3, 4, 5, 6, 7};
            std::cout << "Selected: ANY CONE (all types)\n";
        } else if (cone_selection >= 1 && cone_selection <= 7) {
            target_cone_classes_ = {cone_selection};
            std::string cone_names[] = {"", "Yellow", "Blue", "Orange", 
                                       "Large Orange", "Red", "Green", "Other"};
            std::cout << "Selected: " << cone_names[cone_selection] << " cone (class " 
                     << cone_selection << ")\n";
        } else {
            std::cout << "Invalid selection! Defaulting to Orange cones (classes 3 and 4)\n";
            target_cone_classes_ = {3, 4};
        }
        
        std::cout << "========================================\n\n";

        /* -------- GOAL INPUT -------- */
        std::cout << "Enter goal latitude  : ";
        std::cin >> goal_geo_.latitude;
        std::cout << "Enter goal longitude : ";
        std::cin >> goal_geo_.longitude;

        goal_geo_.altitude = 0.0;

        geodesy::UTMPoint goal_utm;
        geodesy::fromMsg(goal_geo_, goal_utm);
        goal_e_ = goal_utm.easting;
        goal_n_ = goal_utm.northing;

        RCLCPP_INFO(get_logger(), "Goal locked at (%.6f, %.6f)", 
                    goal_geo_.latitude, goal_geo_.longitude);
        RCLCPP_INFO(get_logger(), "Target cone classes: %s", 
                    get_cone_classes_string().c_str());
        RCLCPP_INFO(get_logger(), "Starting navigation...\n");

        /* -------- SUBSCRIBERS -------- */
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>("/fix", 10,
            std::bind(&gps_controller_cin::gps_callback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<std_msgs::msg::Int32>("/imu_data", 10,
            std::bind(&gps_controller_cin::imu_callback, this, std::placeholders::_1));

        lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/ldlidar_node/scan", 10,
            std::bind(&gps_controller_cin::lidar_callback, this, std::placeholders::_1));

        cone_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>("/cone_coordinates", 10,
            std::bind(&gps_controller_cin::cone_callback, this, std::placeholders::_1));

        /* -------- PUBLISHERS -------- */
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel1", 10);
        arm_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel2", 10);
        
        last_cone_detection_time_ = this->now();
    }

private:
    /* -------- ROS HANDLES -------- */
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cone_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr arm_cmd_pub_;

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

    // Target cone classes (configured at startup) - 1-indexed (1-7)
    std::vector<int> target_cone_classes_;

    // State machine
    enum class State {
        NAVIGATE_TO_GOAL,
        CONE_SEARCH_AND_APPROACH,
        CONE_ALIGN,
        CONE_APPROACH_FINAL,
        CONE_DROP_OBJECT
    };
    State current_state_{State::NAVIGATE_TO_GOAL};

    /* -------- PARAMETERS -------- */
    const float OBSTACLE_THRESHOLD = 1.5;
    const float CRITICAL_DISTANCE = 0.5;
    const float AVOIDANCE_GAIN = 1.1;
    const float GOAL_GAIN = 1.1;
    const float CONE_SEARCH_RADIUS = 1.0;
    
    // Camera parameters
    const float CAMERA_WIDTH = 640.0;
    const float CAMERA_HEIGHT = 640.0;
    const float CAMERA_CENTER_X = CAMERA_WIDTH / 2.0;
    const float ALIGNMENT_TOLERANCE = 75.0;
    const float CONE_DETECTION_TIMEOUT = 1.0;
    
    // Cone approach parameters
    const double CONE_STOP_THRESHOLD = 0.5;
    const double CONE_LINEAR_VEL = 0.15;
    const double CONE_ROTATION_VEL = -0.30;
    const double CONE_ALIGN_KP = 0.003;
    const double MAX_ALIGN_ANGULAR_VEL = 0.25;
    const double smooth_factor = 0.6;

    /* -------- GRIPPER DROP FUNCTION -------- */
    void drop_object()
    {
        RCLCPP_INFO(get_logger(), "Opening gripper to drop object...");
        
        geometry_msgs::msg::Twist gripper_cmd;
        gripper_cmd.linear.x = 0.0;
        gripper_cmd.linear.y = 1.0;  // Open gripper
        gripper_cmd.linear.z = 0.0;
        gripper_cmd.angular.x = 0.0;
        gripper_cmd.angular.y = 0.0;
        gripper_cmd.angular.z = 0.0;
        
        // Publish gripper open command for 6 seconds
        auto start_time = this->now();
        auto drop_duration = rclcpp::Duration::from_seconds(4.0);
        
        rclcpp::Rate rate(10);  // 10 Hz
        while (rclcpp::ok() && (this->now() - start_time) < drop_duration)
        {
            arm_cmd_pub_->publish(gripper_cmd);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, 
                "Gripper opening... %.1f seconds remaining",
                (drop_duration - (this->now() - start_time)).seconds());
            
            rclcpp::spin_some(this->get_node_base_interface());
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

    std::string get_cone_classes_string()
    {
        std::string result = "[";
        for (size_t i = 0; i < target_cone_classes_.size(); ++i) {
            result += std::to_string(target_cone_classes_[i]);
            if (i < target_cone_classes_.size() - 1) result += ", ";
        }
        result += "]";
        return result;
    }

    std::string get_cone_name(int class_id)
    {
        std::string cone_names[] = {"Unknown", "Yellow", "Blue", "Orange", 
                                   "Large Orange", "Red", "Green", "Other"};
        if (class_id >= 1 && class_id <= 7) {
            return cone_names[class_id];
        }
        return "Unknown";
    }

    bool is_valid_cone_class(int class_id)
    {
        return std::find(target_cone_classes_.begin(), target_cone_classes_.end(), class_id) 
               != target_cone_classes_.end();
    }

    bool is_cone_detection_valid()
    {
        if (!cone_detected_camera_) return false;
        
        if (!is_valid_cone_class(static_cast<int>(cone_class_id_))) {
            return false;
        }
        
        auto now = this->now();
        double time_since_detection = (now - last_cone_detection_time_).seconds();
        
        return time_since_detection < CONE_DETECTION_TIMEOUT;
    }

    double calculate_alignment_error()
    {
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
        cone_detected_camera_ = false;
    
        if (msg->data.size() < 4) {
            return;
        }
    
        if (msg->data[0] == 0.0 && msg->data[1] == 0.0 && 
            msg->data[2] == 0.0 && msg->data[3] == 0.0) {
            return;
        }
    
        float best_cone_x = 0.0;
        float best_cone_y = 0.0;
        float best_cone_class = 0.0;
        float min_center_distance = std::numeric_limits<float>::max();
    
        for (size_t i = 0; i + 3 < msg->data.size(); i += 4)
        {
            float cone_index = msg->data[i];
            float cx = msg->data[i + 1];
            float cy = msg->data[i + 2];
            float class_id = msg->data[i + 3];
        
            if (is_valid_cone_class(static_cast<int>(class_id)))
            {
                float dist_from_center = std::abs(cx - CAMERA_CENTER_X);
            
                if (dist_from_center < min_center_distance)
                {
                    min_center_distance = dist_from_center;
                    best_cone_x = cx;
                    best_cone_y = cy;
                    best_cone_class = class_id;
                    cone_detected_camera_ = true;
                }
            }
        }
    
        if (cone_detected_camera_)
        {
            cone_center_x_ = best_cone_x;
            cone_center_y_ = best_cone_y;
            cone_class_id_ = best_cone_class;
            last_cone_detection_time_ = this->now();
        
            RCLCPP_INFO(get_logger(), "Valid %s cone (class %.0f) at pixel: (%.1f, %.1f)", 
                        get_cone_name(static_cast<int>(best_cone_class)).c_str(),
                        cone_class_id_, cone_center_x_, cone_center_y_);
        }
    }

    void imu_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        imu_heading_rad_ = msg->data * M_PI / 180.0;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Data is already cropped from param file, use full range
        int start_id = 0;
        int end_id = msg->ranges.size() - 1;
        
        // Calculate midpoint
        int mid = start_id + (end_id - start_id) / 2;
        
        // Initialize distances
        left_min_ = msg->range_max;
        right_min_ = msg->range_max;
        front_min_ = msg->range_max;
        
        // Scan left side (start to mid)
        for (int i = start_id; i <= mid; i++)
        {
            float d = msg->ranges[i];
            if (d > 0.0 && d < left_min_) {
                left_min_ = d;
            }
        }
        
        // Scan right side (mid+1 to end)
        for (int i = mid + 1; i <= end_id; i++)
        {
            float d = msg->ranges[i];
            if (d > 0.0 && d < right_min_) {
                right_min_ = d;
            }
        }
        
        // Front is the minimum of left and right for compatibility
        front_min_ = std::min(left_min_, right_min_);
        
        // Set obstacle flag
        obstacle_ = (left_min_ < OBSTACLE_THRESHOLD || right_min_ < OBSTACLE_THRESHOLD);
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
            if (dist < CONE_SEARCH_RADIUS)
            {
                current_state_ = State::CONE_SEARCH_AND_APPROACH;
                RCLCPP_INFO(get_logger(), "Within %.1fm of goal. Starting camera-based cone detection...", CONE_SEARCH_RADIUS);
                RCLCPP_INFO(get_logger(), "Looking for cone classes: %s", get_cone_classes_string().c_str());
            }
            else
            {
                if (obstacle_)
                {
                    // TurtleBot-style obstacle avoidance with proportional control
                    double kpz = 0.4;  // Proportional gain from TurtleBot code
                    double linvel = 0.4;  // Linear velocity from TurtleBot code
                    
                    cmd.linear.x = linvel;
                    
                    // Apply proportional control based on which side has obstacle
                    if (left_min_ < OBSTACLE_THRESHOLD)
                    {
                        // Obstacle on left, turn right (negative angular velocity)
                        cmd.angular.z = -kpz * left_min_;
                    }
                    else if (right_min_ < OBSTACLE_THRESHOLD)
                    {
                        // Obstacle on right, turn left (positive angular velocity)
                        cmd.angular.z = kpz * right_min_;
                    }
                    else
                    {
                        cmd.angular.z = 0.0;
                    }

                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                        "Avoiding | L:%.2f R:%.2f | Turn:%.2f",
                        left_min_, right_min_, cmd.angular.z);
                }
                else
                {
                    double ang = clamp(angle_error / M_PI, -1.0, 1.0);
                    
                    if(dist > 5.0){
                        cmd.linear.x = 1.0 * smooth_factor;
                    } else {
                        cmd.linear.x = clamp(dist / 5.0, 0.0, 1.0);
                    }
                    cmd.angular.z = ang * smooth_factor;
                    
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Navigating to goal | Distance: %.2fm, Angle error: %.1f°", 
                        dist, angle_error * 180.0 / M_PI);
                }
                
                cmd_pub_->publish(cmd);
                return;
            }
        }
        
        if (current_state_ == State::CONE_SEARCH_AND_APPROACH)
        {
            if (is_cone_detection_valid())
            {
                current_state_ = State::CONE_ALIGN;
                RCLCPP_INFO(get_logger(), "VALID %s CONE (class %.0f) detected! Starting alignment...", 
                           get_cone_name(static_cast<int>(cone_class_id_)).c_str(), cone_class_id_);
            }
            else
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = CONE_ROTATION_VEL;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                    "Searching for target cone with camera...");
                cmd_pub_->publish(cmd);
                return;
            }
        }
        
        if (current_state_ == State::CONE_ALIGN)
        {
            if (!is_cone_detection_valid())
            {
                current_state_ = State::CONE_SEARCH_AND_APPROACH;
                RCLCPP_WARN(get_logger(), "Lost valid cone detection! Returning to search...");
                cmd_pub_->publish(cmd);
                return;
            }
            
            if (is_cone_aligned())
            {
                current_state_ = State::CONE_APPROACH_FINAL;
                RCLCPP_INFO(get_logger(), "Cone aligned! Starting final approach...");
            }
            else
            {
                double pixel_error = calculate_alignment_error();
                
                cmd.angular.z = -CONE_ALIGN_KP * pixel_error;
                cmd.angular.z = clamp(cmd.angular.z, -MAX_ALIGN_ANGULAR_VEL, MAX_ALIGN_ANGULAR_VEL);
                
                cmd.linear.x = 0.05;
                
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "Aligning: %s cone at pixel %.1f (error: %.1f), Angular vel: %.3f",
                    get_cone_name(static_cast<int>(cone_class_id_)).c_str(), 
                    cone_center_x_, pixel_error, cmd.angular.z);
                
                cmd_pub_->publish(cmd);
                return;
            }
        }
        
        if (current_state_ == State::CONE_APPROACH_FINAL)
        {
            if (!is_cone_detection_valid())
            {
                current_state_ = State::CONE_SEARCH_AND_APPROACH;
                RCLCPP_WARN(get_logger(), "Lost cone during approach! Returning to search...");
                cmd_pub_->publish(cmd);
                return;
            }
            
            float closest_distance = std::min({left_min_, right_min_, front_min_});
            
            if (closest_distance <= CONE_STOP_THRESHOLD)
            {
                // Stop robot
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                cmd_pub_->publish(cmd);
                
                RCLCPP_INFO(get_logger(), "SUCCESS! Reached %s cone (class %.0f) at distance: %.2fm", 
                           get_cone_name(static_cast<int>(cone_class_id_)).c_str(),
                           cone_class_id_, closest_distance);
                
                // Transition to drop object state
                current_state_ = State::CONE_DROP_OBJECT;
                RCLCPP_INFO(get_logger(), "Starting object drop sequence...");
                return;
            }
            else
            {
                double pixel_error = calculate_alignment_error();
                
                cmd.angular.z = -CONE_ALIGN_KP * pixel_error * 0.5;
                cmd.angular.z = clamp(cmd.angular.z, -MAX_ALIGN_ANGULAR_VEL * 0.5, MAX_ALIGN_ANGULAR_VEL * 0.5);
                
                cmd.linear.x = CONE_LINEAR_VEL;
                
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                    "Approaching %s cone: Distance %.2fm, Pixel %.1f",
                    get_cone_name(static_cast<int>(cone_class_id_)).c_str(),
                    closest_distance, cone_center_x_);
            }
        }
        
        if (current_state_ == State::CONE_DROP_OBJECT)
        {
            // Execute the drop
            drop_object();
            
            // Mission complete
            RCLCPP_INFO(get_logger(), "Mission completed successfully!");
            rclcpp::shutdown();
            return;
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