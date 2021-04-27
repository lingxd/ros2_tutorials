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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tutorial_interfaces/msg/num.hpp"  //CHANGE 使用自己定义的msg

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<tutorial_interfaces::msg::Num>("topic", 10); //主题名称topic和所需的队列大小来初始化
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this)); //500ms执行一次
  }

private:
  void timer_callback()
  {
    auto message = tutorial_interfaces::msg::Num();                      // CHANGE
    message.num = this->count_++;                                        // CHANGE
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.num);    // CHANGE
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_; //计时器
  rclcpp::Publisher<tutorial_interfaces::msg::Num>::SharedPtr publisher_;         // CHANGE
  size_t count_; //计数器
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());// 开始处理来自节点的数据，包括来自计时器的回调。
  rclcpp::shutdown();
  return 0;
}
