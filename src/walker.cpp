#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class WalkerBot : public rclcpp::Node {
public:
  WalkerBot() : Node("walker"), count_(0), obstacle_detected_(false) {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&WalkerBot::laser_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        500ms, std::bind(&WalkerBot::timer_callback, this));
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Simulate obstacle detection based on LaserScan data
    auto laser_data=msg->ranges;
    for (int i = 330; i < 330 + 60; i++) {
      if (laser_data[i % 360] < 0.8) {
        obstacle_detected_ = true;
        return; 
      }
    }
    obstacle_detected_ = false;
  }

  void timer_callback() {
    auto twist_msg = geometry_msgs::msg::Twist();

    if (!obstacle_detected_) {
      twist_msg.linear.x = 0.2;
      twist_msg.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Moving forward...");
    } else {
      twist_msg.linear.x = 0.0;
      twist_msg.angular.z = 0.2;
      RCLCPP_INFO(this->get_logger(), "Obstacle detected. Rotating...");
    }

    publisher_->publish(twist_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      laser_subscriber_;
  size_t count_;
  bool obstacle_detected_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkerBot>());
  rclcpp::shutdown();
  return 0;
}