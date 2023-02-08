#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "math.h"

/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node"), car_width(0.5)    
    {
        /// TODO: create ROS subscribers and publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 1, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        this->declare_parameter("r_b", 0.3);
        this->declare_parameter("disp_thresh", 0.3);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    float car_width;

    float preprocess_lidar(float* ranges, float theta_min, float theta_max, float theta_increment)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        
        int range_size = sizeof(ranges)/sizeof(ranges[0]);
        float gap_width = 0.0;
        bool gap_flag = true;
        bool obs_flag = true;
        int gap_start;


        for(int i=0; i<range_size; i++)
        {
            if ((ranges[i+1] - ranges[i]) > this->get_parameter("disp_thresh").get_parameter_value().get<float>())  
            {
                // left disparity
                gap_width += ranges[i+1] * theta_increment;
                gap_flag = true;
                obs_flag = false;
                gap_start = i+1;
            }
            else if ((ranges[i] - ranges[i + 1]) > this->get_parameter("disp_thresh").get_parameter_value().get<float>())
            {
                // right disparity
                gap_width = 0.0;
                gap_flag = false;
                obs_flag = true;
            }
            if (gap_flag)
            {
                // incrementing gap without disparity
                gap_width += ranges[i] * theta_increment;
            }
            if((gap_width > car_width) && (obs_flag == false))
            {
                // gap detected
                // go for it
                return theta_min + (i - gap_start) * theta_increment/2.0;
            }

        }

        return 0.0;
    }

   


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 

        // Find the best point in the gap 

        // Publish Drive message
        
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}