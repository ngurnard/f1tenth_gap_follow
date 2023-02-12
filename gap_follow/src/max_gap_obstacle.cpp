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
    ReactiveFollowGap() : Node("max_gap_node"), lidarscan_topic("/scan"), drive_topic("/drive")
    {
        /// create ROS subscribers and publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 1, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        // declare useful params
        this->declare_parameter("range_thresh", 1.);
        this->declare_parameter("disp_thresh", 0.5);
        this->declare_parameter("Kp", 0.6);
        this->declare_parameter("speed", 0.0);
        this->declare_parameter("car_width", 0.5); // s for s=r*theta
        this->declare_parameter("check_thresh", 1.0);
    }

private:
    std::string lidarscan_topic;
    std::string drive_topic;
    /// create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // declare useful vals
    int number_of_rays;
    int ignore_scans;

    float theta_increment;
    float theta_min;

    // for disparity (starting indices)
    std::vector<int> obs_idx;
    std::vector<int> gap_idx;
    std::vector<int> shawties; // short scans

    // to check which disparity comes first
    bool first_flag; // 0 is obs, 1 is gap

    void preprocess_lidar(const float* ranges_og, float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        // clear previous scan
        obs_idx.clear();
        gap_idx.clear();
        shawties.clear();

        // sweep right to left
        std::cout << ignore_scans << std::endl;
        for (int i = ignore_scans+1; i < number_of_rays-ignore_scans; i++)
        {
            // TODO: Filter NaN and infs
            if (std::isinf(ranges_og[i]) || std::isinf(ranges_og[i-1])
                || std::isnan(ranges_og[i]) || std::isnan(ranges_og[i-1])
                || (ranges_og[i] > this->get_parameter("check_thresh").get_parameter_value().get<float>() && ranges_og[i-1] > this->get_parameter("check_thresh").get_parameter_value().get<float>()))
            {
                    continue;
            }
            // store the short scans in case of no disparity
            if (ranges_og[i] < this->get_parameter("range_thresh").get_parameter_value().get<float>()) {
                shawties.push_back(i);
            }
    

            // right disparity
            if ((ranges_og[i] - ranges_og[i-1]) > this->get_parameter("disp_thresh").get_parameter_value().get<float>())  
            {
                RCLCPP_INFO(this->get_logger(), "right disparity at %f", (theta_min+(i*theta_increment))*180.0/M_PI);
                RCLCPP_INFO(this->get_logger(), "ranges_og[i]: %f,; ranges_og[i-1]: %f", ranges_og[i], ranges_og[i-1]);
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
            else if ((ranges_og[i-1] - ranges_og[i]) > this->get_parameter("disp_thresh").get_parameter_value().get<float>())  
            {
                RCLCPP_INFO(this->get_logger(), "left disparity at %f", (theta_min+(i*theta_increment))*180.0/M_PI);
                RCLCPP_INFO(this->get_logger(), "ranges_og[i-1]: %f,; ranges_og[i]: %f", ranges_og[i-1], ranges_og[i]);
                if (obs_idx.size() == 0 && gap_idx.size() == 0) {
                    // if the first disparity is a left disparity, then start of gap is first scan

                    // gap_idx.push_back(0);

                    // this is where the left disp is in radians
                    // obs_idx.push_back(i);
                    int k = i-1;
                    while(true) {
                        float arc = (theta_min+(i*theta_increment)) * ranges_og[i] - 
                                    (theta_min+(k*theta_increment)) * ranges_og[i]; 
                        if(arc > this->get_parameter("car_width").get_parameter_value().get<float>()) {
                            RCLCPP_INFO(this->get_logger(), "Car width gap start: %f (FOR DEBUG)", (theta_min+(k*theta_increment))*180.0/M_PI);
                            gap_idx.push_back(k);
                            obs_idx.push_back(ignore_scans);
                            obs_idx.push_back(i);
                            first_flag = 0;
                            break;
                        } else {
                            k--;
                        }
                        if(k < ignore_scans)
                        {
                            gap_idx.push_back(ignore_scans);
                            first_flag = 1;
                            obs_idx.push_back(i);
                            break;
                        }

                    }
                    // obs_idx.push_back(i);

                    // first_flag = 0; // 1 is gap
                } else {
                    // start of obs
                    obs_idx.push_back(i);
                }
            }

            // zero logic
            if (obs_idx.size() != 0 || gap_idx.size() != 0) {
                if (obs_idx.back() > gap_idx.back()) {
                    // if in an obstacle (most recent scan idx is an obs)
                    ranges[i] = 0;
                } 
            }
        }

        // no disparity
        if (obs_idx.size() == 0 && gap_idx.size() == 0) {
            // look above the max thresh for the gap (zero out short scans)
            for (auto i : shawties) {
                ranges[i] = 0;
            }

        } else {
            // check if the first flag is an obs, if so need more zero logic
            if (first_flag == 0) {
                for (int i = obs_idx[0]; i < gap_idx[0]; i++) {
                    ranges[i] = 0;
                }
            }
        }
        return;
    }

    void find_max_gap(const float* ranges, int* start_idx, int* end_idx)
    {    
        // Return the start index & end index of the max gap in free_space_ranges
        // Straight logic
        float max_gap = 0.0;
        int gap_iter = 0;
        float gap = 0.0;
        for (int i = ignore_scans; i < number_of_rays-ignore_scans; i++)
        {
            if (ranges[i] == 0) { // if an obs
                if (gap > max_gap)
                {
                    // logic for finding biggest gap
                    RCLCPP_INFO(this->get_logger(), "New max gap");
                    max_gap = gap;
                    *start_idx = i - gap_iter; 
                    *end_idx = i;
                    RCLCPP_INFO(this->get_logger(), "Start: %f; End: %f", (theta_min+(*start_idx*theta_increment))*180.0/M_PI, (theta_min+(*end_idx*theta_increment))*180.0/M_PI);
                }
            gap = 0; // reset the gap
            } else {
                gap += ranges[i] * this->theta_increment; // s = r * theta
                gap_iter++;
            }
        }
        return;
    }

    void find_best_point(const float* ranges, int* start_idx, int* end_idx, float* best_theta)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        *best_theta = (*start_idx + *end_idx)*theta_increment/2.0 + theta_min; // go to center
        // s = r*theta
        // float theta = this->get_parameter("car_width").get_parameter_value().get<float>() / ranges[*end_idx];
        // *best_theta = theta_min + (*end_idx)*theta_increment - theta;
        return;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        // need to store some params
        this->number_of_rays = scan_msg->ranges.size();
        this->theta_increment = scan_msg->angle_increment;
        this->theta_min = scan_msg->angle_min;

        ignore_scans = ((-theta_min) - M_PI/2)/theta_increment;

        // this->ranges_min = scan_msg->range_min;
        // this->ranges_max = scan_msg->range_max;
        float const *ranges_data = scan_msg->ranges.data();
        float *ranges_data_copy = new float[this->number_of_rays];
        memcpy(ranges_data_copy, ranges_data, this->number_of_rays * sizeof(float));
        int gap_start_idx;
        int gap_end_idx;
        float best_theta;

        // process all of the lidar scans
        preprocess_lidar(ranges_data, ranges_data_copy);

        // Find max length gap 
        find_max_gap(ranges_data_copy, &gap_start_idx, &gap_end_idx);

        // Find the best point in the gap 
        find_best_point(ranges_data, &gap_start_idx, &gap_end_idx, &best_theta);

        // Publish Drive message
        RCLCPP_INFO(this->get_logger(), "theta: %f", best_theta * 180.0/M_PI);
        ackermann_msgs::msg::AckermannDriveStamped drive_msg; 
        drive_msg.drive.steering_angle = this->get_parameter("Kp").get_parameter_value().get<float>() * best_theta;
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