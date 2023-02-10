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
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers

    int number_of_rays;

    // for disparity (REMEMBER TO CLEAR)
    std::vector<int> obs_idx;
    std::vector<int> gap_idx;
    std::vector<int> shawties; // short scans

    bool first_flag; // 0 is obs, 1 is gap

    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

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

    void find_max_gap(float* ranges, int* idx)
    {   
        // Return the start index & end index of the max gap in free_space_ranges

        return;
    }

    void find_best_point(float* ranges, int* idx)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        return;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        number_of_rays = scan_msg->ranges.size();

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