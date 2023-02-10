#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node"), lidarscan_topic("/scan"), drive_topic("/drive")
    {
        /// create ROS subscribers and publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 1, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        // declare useful params
        this->declare_parameter("range_thresh", 1f);
        this->declare_parameter("disp_thresh", 0.1);
        this->declare_parameter("Kp", 0.7);
        this->declare_parameter("speed", 1.5);
    }

private:
    std::string lidarscan_topic;
    std::string drive_topic;
    /// create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // declare useful vals
    int number_of_rays;
    float theta_increment;
    float theta_min;

    // for disparity (starting indices)
    std::vector<int> obs_idx;
    std::vector<int> gap_idx;
    std::vector<int> shawties; // short scans

    // to check which disparity comes first
    bool first_flag; // 0 is obs, 1 is gap

    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        // clear previous scan
        obs_idx.clear();
        gap_idx.clear();
        shawties.clear();

        // sweep right to left
        for (int i = 1; i < number_of_rays; i++) {
            // store the short scans in case of no disparity
            if (ranges[i] < this->get_parameter("range_thresh").get_parameter_value().get<float>()) {
                shawties.push_back(i);
            }
            // right disparity
            if ((ranges[i] - ranges[i-1]) > this->get_parameter("disp_thresh").get_parameter_value().get<float>())  
            {
                if (obs_idx.size() == 0 && gap_idx.size() == 0) {
                    // if the first disparity is a right disparity, then start of obs is first scan
                    obs_idx.push_back(0);
                    gap_idx.push_back(i);

                    first_flag = 0; // 0 is obs
                } else {
                    gap_idx.push_back(i);
                }
                
            }
            // left disparity
            if ((ranges[i-1] - ranges[i]) > this->get_parameter("disp_thresh").get_parameter_value().get<float>())  
            {
                if (obs_idx.size() == 0 && gap_idx.size() == 0) {
                    // if the first disparity is a left disparity, then start of gap is first scan
                    gap_idx.push_back(0);
                    obs_idx.push_back(i);

                    first_flag = 1; // 1 is gap
                } else {
                    // start of obs
                    obs_idx.push_back(i);
                }
            }

            // zero logic
            if (obs_idx.size() != 0 || gap_idx.size() != 0) {
                if (obs_idx.back() > gap_idx.back()) {
                    // if in an obstacle (most recent scan idx is an obs)
                    ranges[i] == 0;
                } 
            }
        }

        // no disparity
        if (obs_idx.size() == 0 && gap_idx.size() == 0) {
            // implement wall follow later potentially ( i think this would be best)
            
            // biggest arclength?
            // d arc?

            // look above the max thresh for the gap (zero out short scans)
            for (int i = shawties.start(); shawties.end(); i++) {
                ranges[i] = 0;
            }

        } else {
            // check if the first flag is an obs, if so need more zero logic
            if (first_flag == 0) {
                for (int i = obs_idx.start(); i < gap_idx.start(); i++) {
                    ranges[i] = 0;
                }
            }
             
        }
        return;
    }

    void find_max_gap(float* ranges, int* start_idx, int* end_idx)
    {    
        // Return the start index & end index of the max gap in free_space_ranges
        // Straight logic
        float max_gap;
        int gap_iter;
        float gap;
        for(int i = 0; i < number_of_rays; i++)
        {
            if (ranges[i] == 0) { // if an obs
                if (gap > max_gap)
                {
                    // logic for finding biggest gap
                    max_gap = gap;
                    start_idx = i - gap_iter; 
                    end_idx = i;
                }
            gap = 0; // reset the gap
            } else {
                gap += ranges[i] * this->theta_increment; // s = r * theta
                gap_iter++;
            }
        }
        return;
    }

    void find_best_point(float* ranges, int* start_idx, int* end_idx, int* best_idx)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        best_idx = (start_idx + end_idx) / 2; // go to center

        return;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        // need to store some params
        this->number_of_rays = scan_msg->ranges.size();
        this->theta_increment = scan_msg->angle_increment;
        this->theta_min = scan_msg->angle_min;
        const float *range_data = scan_msg->ranges.data();
        int gap_start_idx;
        int gap_end_idx;
        int best_idx;

        // process all of the lidar scans
        preprocess_lidar(range_data);

        // Find max length gap 
        find_max_gap(range_data, &gap_start_idx, &gap_end_idx);

        // Find the best point in the gap 
        find_best_point(range_data, &gap_start_idx, &gap_end_idx, &best_idx);

        // Publish Drive message
        // RCLCPP_INFO(this->get_logger(), "theta: %f", theta*180.0/M_PI);
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = this->get_parameter("Kp").get_parameter_value().get<float>() * (best_idx * theta_increment + theta_min);
        drive_msg.drive.speed = this->get_parameter("speed").get_parameter_value().get<float>();
        drive_pub_->publish(drive_msg);
    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}