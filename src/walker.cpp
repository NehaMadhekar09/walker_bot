/*
 * @file walker.cpp
 * @brief This file contains the implementation of the WalkerBot class
 * that implement walker algorithm like rumba robot.
 * @author Neha Nitin Madhekar
 * @date 2023
 * @copyright Open Source Robotics Foundation, Inc.
 * @license Apache License, Version 2.0
 *    (you may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0)
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @class WalkerBot
 * @brief A ROS 2 node for a simple walker robot.
 */
class WalkerBot : public rclcpp::Node {
public:
  /**
   * @brief Constructor for the WalkerBot class.
   */
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
  /**
   * @brief Callback function for processing laser scan data.
   * @param msg Laser scan data message.
   */
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

  /**
   * @brief Callback function for the timer to control the robot's movement.
   */
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

/**
 * @brief Main function to initialize and run the ROS 2 node.
 * @param argc Number of command-line arguments.
 * @param argv Command-line arguments.
 * @return Exit code.
 */
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WalkerBot>());
  rclcpp::shutdown();
  return 0;
}