#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <stdlib.h>

constexpr int kImageWidth = 640;
constexpr int kImageHeight = 480;
constexpr int kNumberOfPixels = kImageWidth * kImageHeight;

constexpr int kImagePatternLength = 8;

std::vector<unsigned char> binary_pattern_generator(const int &patternLen, const bool &startWithWhite){
  
  // Initial pixels are dark
  std::vector <unsigned char> white_pattern(patternLen/2, 255);
  std::vector <unsigned char> dark_pattern(patternLen/2, 0);  

  std::vector <unsigned char> pattern(dark_pattern);

  if(startWithWhite){
    std::copy(dark_pattern.begin(), dark_pattern.end(), std::back_inserter(pattern));
    std::copy(white_pattern.begin(), white_pattern.end(), std::back_inserter(pattern));
    
  } else {
    std::copy(white_pattern.begin(), white_pattern.end(), std::back_inserter(pattern));
    std::copy(dark_pattern.begin(), dark_pattern.end(), std::back_inserter(pattern));
  }
  
  return pattern;
} 

std::vector<unsigned char> patterned_image_generator(const int patternLen, bool startWithWhite) {
  std::vector <unsigned char> pattern = binary_pattern_generator(patternLen, startWithWhite);

  std::vector<unsigned char> image;
  image.reserve(kNumberOfPixels);
  
  const int numberOfStride = kNumberOfPixels / patternLen;
  
  for(int i = 0; i < numberOfStride; i++){
    std::copy(pattern.begin(), pattern.end(), std::back_inserter(image));
  }

  // For leftover pixels
  if( (numberOfStride * patternLen) < kNumberOfPixels){
    for(int j = 0; j < (kNumberOfPixels - (numberOfStride * patternLen)); j++){
      image.push_back(0);
    }
  }
  return image;
}

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MockCameraPub : public rclcpp::Node
{
public:
  MockCameraPub()
      : Node("mock_camera_pub"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MockCameraPub::timer_callback, this));
    
    // Random number generator engine initialisor
  //  mtEngine_(rd_);
    
  }

private:
  void timer_callback()
  {
    sensor_msgs::msg::Image msgImage;

    rclcpp::Time captureTime = this->get_clock()->now();

    msgImage.header.stamp = captureTime;

    pseudo_image_measurement(msgImage);
    RCLCPP_INFO(this->get_logger(), "Publishing: mock Image message");
    publisher_->publish(msgImage);

    /*
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
*/
  }
  void pseudo_image_measurement(sensor_msgs::msg::Image &mockMsgImage)
  {
    mockMsgImage.height = kImageHeight;
    mockMsgImage.width = kImageWidth;
    mockMsgImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;

    mockMsgImage.data = patterned_image_generator(kImagePatternLength, true);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
//  std::random_device rd_;
 // std::mt19937 mtEngine_;
 // std::uniform_real_distribution<> dist_(0, 1);
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockCameraPub>());
  rclcpp::shutdown();
  return 0;
}
