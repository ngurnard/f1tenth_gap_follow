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
    ReactiveFollowGap() : Node("reactive_node")   
    {
        /// TODO: create ROS subscribers and publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 1, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        // this->declare_parameter("car_width", 0.7);
        // this->declare_parameter("disp_thresh", 0.1);
        this->declare_parameter("Kp", 0.71);
        this->declare_parameter("speed_turn", 1.5);
        this->declare_parameter("speed_straight", 3.0);
        this->declare_parameter("obs_dist", 1.2);
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    float theta_min, theta_increment;
    float theta;
    // float car_width;
    int number_of_rays;

    float preprocess_lidar(const float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
     
        // Straight logic
        int max_gap = 0;
        int start_gap = 0;
        int end_gap = 0;
        int gap = 0;

        float debug_max_ranges = 0.0;

        for(int i=0; i<number_of_rays; i++)
        {
            if (0
                || std::isinf(ranges[i])
                || std::isnan(ranges[i]))
            {
                    continue; 
            }
            if(ranges[i] < this->get_parameter("obs_dist").get_parameter_value().get<float>())
            {
                // printf("OBSTACLE\n");
                if(gap > max_gap)
                {
                    max_gap = gap;
                    start_gap = (i - gap);
                    end_gap = i;
                }
                gap = 0;
            }
            else
                gap++;
            
            // DEBUG CODE TO REMOVE
            if(ranges[i] > debug_max_ranges)
                debug_max_ranges = ranges[i];
        }

        RCLCPP_INFO(this->get_logger(), "Max range: %f", debug_max_ranges);

        // int ignore_scans = ((-theta_min) - M_PI/2)/theta_increment;
        // float max_gap_depth = 0.0;
        // for(int i=ignore_scans; i<number_of_rays-ignore_scans; i++)
        // for(int i=0; i<number_of_rays; i++)
        // {
        //     if (std::isinf(ranges[i])
        //         || std::isnan(ranges[i]))
        //     {
        //             continue; 
        //     }

        //     if(ranges[i] < this->get_parameter("obs_dist").get_parameter_value().get<float>())
        //     {   
        //         // it is an obstacle
        //         if(gap > max_gap)
        //         {
        //             for (int k = i-gap; k < i; k++) {
        //                 if (ranges[k] > max_gap_depth) {
        //                     max_gap_depth = ranges[k];
        //                     max_gap = gap;
        //                     start_gap = (i - gap) ;
        //                     end_gap = i-1;
        //                 }
        //             }
        //             gap = 0;
        //         }
        //     }
        //     else
        //         gap++;
        // }
        
        RCLCPP_INFO(this->get_logger(), "Max gap: %d, start: %f, end: %f", max_gap, (theta_min + start_gap* theta_increment)*180.0/M_PI, (theta_min +  end_gap* theta_increment)*180.0/M_PI);
        return theta_min + (start_gap + end_gap) * theta_increment/2.0;
    
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


        theta_min = scan_msg->angle_min;
        theta_increment = scan_msg->angle_increment;
        number_of_rays = scan_msg->ranges.size();

        const float *range_data = scan_msg->ranges.data();
        theta = preprocess_lidar(range_data);     // change function name

        RCLCPP_INFO(this->get_logger(), "theta: %f", theta*180.0/M_PI);

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        // drive_msg.header.stamp = this->now();
        drive_msg.drive.steering_angle = this->get_parameter("Kp").get_parameter_value().get<float>() * theta;
        if(theta*180.0/M_PI > 10.0)
            drive_msg.drive.speed = this->get_parameter("speed_turn").get_parameter_value().get<float>();
        else
            drive_msg.drive.speed = this->get_parameter("speed_straight").get_parameter_value().get<float>();
        // drive_msg.drive.speed = this->get_parameter("speed_straight").get_parameter_value().get<float>();
        drive_pub_->publish(drive_msg);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}