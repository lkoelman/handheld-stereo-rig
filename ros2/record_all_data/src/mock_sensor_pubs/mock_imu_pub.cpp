#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <stdlib.h>

double randZeroToOne()
{
    return rand() / (RAND_MAX + 1.);
}

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MockImuPub : public rclcpp::Node
{
public:
  MockImuPub()
      : Node("mock_imu_pub"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MockImuPub::timer_callback, this));
    
    // Random number generator engine initialisor
  //  mtEngine_(rd_);
    
  }

private:
  void timer_callback()
  {
    sensor_msgs::msg::Imu msgImu;
    pseudo_imu_measurement(msgImu);
    RCLCPP_INFO(this->get_logger(), "Publishing: mock IMU message");
    publisher_->publish(msgImu);

    /*
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
*/
  }

  // Quote from https://stackoverflow.com/questions/31600717/how-to-generate-a-random-quaternion-quickly

  /*
  Choose three points u, v, w ∈ [0,1] uniformly at random.
  A uniform, random quaternion is given by the simple expression:

  q1 = sqrt(1-u) sin(2πv)
  q2 = sqrt(1-u) cos(2πv)
  q3 = sqrt(u) sin(2πw)
  q4 = sqrt(u) cos(2πw)
  */
  void pseudo_imu_measurement(sensor_msgs::msg::Imu &mockMsgImu)
  {

    float u = randZeroToOne(); // std::round(dist_(mtEngine_));
    float v = randZeroToOne(); // std::round(dist_(mtEngine_));
    float w = randZeroToOne(); // std::round(dist_(mtEngine_));

    mockMsgImu.orientation.x = std::sqrt(1 - u) * std::sin(2 * M_PI * v);
    mockMsgImu.orientation.y = std::sqrt(1 - u) * std::cos(2 * M_PI * v);
    mockMsgImu.orientation.z = std::sqrt(u) * std::sin(2 * M_PI * w);
    mockMsgImu.orientation.w = std::sqrt(u) * std::cos(2 * M_PI * w);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  size_t count_;
//  std::random_device rd_;
 // std::mt19937 mtEngine_;
 // std::uniform_real_distribution<> dist_(0, 1);
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockImuPub>());
  rclcpp::shutdown();
  return 0;
}
