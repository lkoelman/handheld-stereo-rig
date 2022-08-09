#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <rosbag2_cpp/writer.hpp>

using std::placeholders::_1;

const std::string lidar_topic_name = "/points";
const std::string image_topic_name = "/image";
const std::string imu_topic_name = "/imu";

class BagRecorder : public rclcpp::Node
{
public:
    BagRecorder()
        : Node("record_all_data")
    {
        writer_ = std::make_unique<rosbag2_cpp::Writer>();

        writer_->open("handheld_rig_bag");

        lidar_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic_name, 10, std::bind(&BagRecorder::lidar_topic_callback, this, _1));

        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_name, 10, std::bind(&BagRecorder::image_topic_callback, this, _1));

        imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_name, 10, std::bind(&BagRecorder::imu_topic_callback, this, _1));
    }

private:
    void lidar_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        rclcpp::Time time_stamp = this->now();

        writer_->write(*msg, lidar_topic_name, "sensor_msgs/msg/LaserScan", time_stamp);
    }

    void image_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        rclcpp::Time time_stamp = this->now();

        writer_->write(*msg, image_topic_name, "sensor_msgs/msg/Image", time_stamp);
    }

    void imu_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        rclcpp::Time time_stamp = this->now();

        writer_->write(*msg, imu_topic_name, "sensor_msgs/msg/Imu", time_stamp);
    }

    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr image_subscription_;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr imu_subscription_;
    
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BagRecorder>());
    rclcpp::shutdown();
    return 0;
}
