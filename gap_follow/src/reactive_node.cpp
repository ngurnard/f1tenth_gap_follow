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

        this->declare_parameter("car_width", 0.5);
        this->declare_parameter("disp_thresh", 0.1);
        this->declare_parameter("Kp", 0.7);
        this->declare_parameter("speed_turn", 0.2);
        this->declare_parameter("speed_straight", 0.5);
        this->declare_parameter("obs_dist", 1.0);
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
        
        // int range_size = sizeof ranges/sizeof(ranges[0]);
        float gap_width = 0.0;
        bool gap_flag = true;
        bool obs_flag = true;
        int gap_start;

        // printf("size of ranges: %d\n", number_of_rays);
        // printf("theta_min: %f\n", theta_min);
        // printf("theta_increment: %f\n", theta_increment);
        
        // Straight logic
        int max_gap = 0;
        int start_gap = 0;
        int end_gap = 0;
        int gap = 0;
        float max_gap_depth = 0.0;
        for(int i=0; i<number_of_rays; i++)
        {
            if(ranges[i] < this->get_parameter("obs_dist").get_parameter_value().get<float>())
            {
                // printf("OBSTACLE\n");

                if(gap > max_gap)
                {
                    float deep_scan = 0.0;
                    for (int k = i-gap; k < i; k++) {
                        if (ranges[k] > max_gap_depth) {
                            max_gap_depth = ranges[k];
                            max_gap = gap;
                            start_gap = (i - gap);
                            end_gap = i;
                        }
                    }
                    gap = 0;
                //     if (deep_scan > max_gap_depth) {
                //         max_gap = gap;
                //         start_gap = (i - gap);
                //         end_gap = i;
                //         max_gap_depth = ranges[int(i-gap/2)];
                //     }  
                // }
                }
            }
            else
                gap++;
        }
        RCLCPP_INFO(this->get_logger(), "Max gap: %d, start: %f, end: %f", max_gap, (theta_min + start_gap* theta_increment)*180.0/M_PI, (theta_min +  end_gap* theta_increment)*180.0/M_PI);
        return theta_min + (start_gap + end_gap) * theta_increment/2.0;
        // float theta_ = this->get_parameter("car_width").get_parameter_value().get<float>() / ranges[end_gap];
        // return theta_min + (end_gap)*theta_increment - theta_;

        // for(int i=number_of_rays-5; i>5; i--)
        // {   
        //     if(ranges[i] > 2.0 || ranges[i-1]>2.0)
        //         continue;
        //     // printf("-----------------------------------------------------------\n");
        //     // printf("theta_min: %f\n", theta_min*180.0/M_PI);
        //     // printf("i: %d\n", i);
        //     // printf("ranges[i]: %f\n", ranges[i]);
        //     // printf("ranges[i-1]: %f\n", ranges[i-1]);
        //     // printf("-----------------------------------------------------------\n");
        //     if ((ranges[i-1] - ranges[i]) > this->get_parameter("disp_thresh").get_parameter_value().get<float>())  
        //     {
        //         // left disparity
        //         RCLCPP_INFO(this->get_logger(), "Left disparity, %f, %f :%f", (theta_min + (i-1)* theta_increment)*180.0/M_PI, (theta_min +  i* theta_increment)*180.0/M_PI, ranges[i-1] - ranges[i]);
        //         gap_width += ranges[i] * theta_increment;
        //         gap_flag = true;
        //         obs_flag = false;
        //         gap_start = i;
        //     }
        //     else if ((ranges[i] - ranges[i-1]) > this->get_parameter("disp_thresh").get_parameter_value().get<float>())
        //     {
        //         // right disparity
        //         RCLCPP_INFO(this->get_logger(), "Right disparity, %f, %f :%f", (theta_min + i* theta_increment)*180.0/M_PI, (theta_min + (i-1)* theta_increment)*180.0/M_PI, ranges[i] - ranges[i-1]);
        //         if(gap_flag==true && obs_flag==true)
        //         {
        //             // First right disparity detected
        //             if((gap_width > this->get_parameter("car_width").get_parameter_value().get<float>()))
        //             {
        //                 // gap detected
        //                 // go for it
        //                 RCLCPP_INFO(this->get_logger(), "Gap width: %f", gap_width);
        //                 RCLCPP_INFO(this->get_logger(), "Gap detected, %f, %f\n", (theta_min + 1080*theta_increment)*180.0/M_PI, (theta_min + i*theta_increment)*180.0/M_PI);
        //                 return theta_min + (1080+i) * theta_increment/2.0;
        //             }
        //         }
        //         gap_width = 0.0;
        //         gap_flag = false;
        //         obs_flag = true;
        //     }
        //     if (gap_flag)
        //     {
        //         // incrementing gap without disparity
        //         gap_width += ranges[i] * theta_increment;
        //     }
        //     if((gap_width > this->get_parameter("car_width").get_parameter_value().get<float>()) && (obs_flag == false))
        //     {
        //         // gap detected
        //         // go for it
        //         RCLCPP_INFO(this->get_logger(), "Gap detected, %f, %f\n", (theta_min + gap_start*theta_increment)*180.0/M_PI, (theta_min + i*theta_increment)*180.0/M_PI);
        //         RCLCPP_INFO(this->get_logger(), "Gap width: %f", gap_width);
        //         return theta_min + (gap_start+i) * theta_increment/2.0;
        //     }
        // }
        // // RCLCPP_INFO(this->get_logger(), "No gap detected");
        // return 0.0;
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
        if(abs(theta)*180.0/M_PI > 20.0)
            drive_msg.drive.speed = this->get_parameter("speed_turn").get_parameter_value().get<float>();
        else
            drive_msg.drive.speed = this->get_parameter("speed_straight").get_parameter_value().get<float>();
        drive_pub_->publish(drive_msg);
        drive_pub_->publish(drive_msg);
        drive_pub_->publish(drive_msg);
        drive_pub_->publish(drive_msg);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}