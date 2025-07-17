// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Standard C++ headers
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp" // Allows you to use the most common pieces of ROS2
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// All of these lines above are the node's dependencies
// Dependencies must be added to package.xml and CMakeLists.txt

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node // every this in the code is referring to the node.
{
public:
  MinimalPublisher() // constructor
  : Node("minimal_publisher"), count_(0)
  {
    // publisher initialized with the string message type, topic name: topic
    // and required queue size (10) to limit messages in the event of a backup
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    // timer_ causes the timer_callback function to be executed twice a second
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() // in this function:
                        // the message data is set and the mexs are actually published
                        // RCLPP_INFO macro ensures every published message is on console
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  // Declarations of the timer, publisher and counter fields:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); // initializes ROS 2
  rclcpp::spin(std::make_shared<MinimalPublisher>()); // starts processing data from the node
  rclcpp::shutdown();
  return 0;
}
