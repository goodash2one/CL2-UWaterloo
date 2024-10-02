#include <cmath>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

class Safety : public rclcpp::Node {
    // The class that handles emergency braking

   public:
    Safety() : Node("safety_node") {
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("safety", 10);
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Safety::scan_callback, this, _1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("ego_racecar/odom", 10, std::bind(&Safety::odom_callback, this, _1));
    }

   private:
    double speed = 0.0;
    bool is_breaking = false;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        this->speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
        double r;
        std::size_t i;
        double min_r = 0.18; // scan_msg->range_min; // To be tuned in real vehicle
        double forward_r = scan_msg->ranges[scan_msg->ranges.size()/2];

	// AEB escape conditions: enough space to move forward
        if (this->is_breaking) {
            double escape_r = 1.5;  // To be tuned in real vehicle
            if (!std::isnan(forward_r) && escape_r < forward_r) {
                for (i = 0; i < scan_msg->ranges.size(); i++) {
                    r = scan_msg->ranges[i];
                    if (std::isnan(r))
                        continue;
                    if (r < min_r)
                        break;
                }
                if (i == scan_msg->ranges.size()) {
                    RCLCPP_INFO(this->get_logger(), "release AEB");
                    this->is_breaking = false;
                }
            }
        } else {
            int k = 1, l = 1;
            while (std::isnan(forward_r)) {
                RCLCPP_INFO(this->get_logger(), "cannot find forward space: searching...");  // Output to log;
                if (k > 10)
                    return;
                forward_r = scan_msg->ranges[scan_msg->ranges.size()/2 + k*l];
                if (l < 0)
                    k++;
                l *= -1;
            }
            /// calculate TTC
            /// bool emergency_breaking = false;
            for (i = 0; i < scan_msg->ranges.size(); i++) {
                r = scan_msg->ranges[i];
                if (r < min_r) {
                    RCLCPP_INFO(this->get_logger(), "under the safety margin");
                    is_breaking = true;
                    break;
                } else if (std::isnan(r) || r > scan_msg->range_max) {
                    continue;
                }
                double threshold = 0.32;  // To be tuned in real vehicle
                double cos_val = std::cos(scan_msg->angle_min + (double)i * scan_msg->angle_increment);
                if (forward_r < r / std::max(cos_val, 0.0001) && r / std::max(this->speed * cos_val, 0.001) < threshold) {
                    RCLCPP_INFO(this->get_logger(), "AEB triggered");
                    is_breaking = true;
                    break;
                }
            }
        }
	
        // Publish command to brake
        if (is_breaking) {
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.drive.speed = 0.0;
            RCLCPP_INFO(this->get_logger(), "emergency brake engaged at speed '%f'", this->speed);  // Output to log;
            this->publisher_->publish(drive_msg);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
