#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <rosbag2_cpp/writer.hpp>

using std::placeholders::_1;


// ===================  Ouster OS1 parameters  ===================

const std::string lidar_topic_name = "/points";
//
constexpr int lidar_downsampling_rate = 3;

// ===================  OAK-D parameters  ===================
const std::string oakd_color_topic_name = "/color/image";
//
constexpr int oakd_color_downsampling_rate = 3;

const std::string oakd_color_info_topic_name = "/color/camera_info"; 
//
constexpr int oakd_color_info_downsampling_rate = 4;
/*
const std::string oakd_stereo_left_topic_name = "/left/image_rect"; x
const std::string oakd_stereo_left_info_topic_name = "/left/camera_info"; x 

const std::string oakd_stereo_right_topic_name = "/right/image_rect"; x 
const std::string oakd_stereo_right_info_topic_name = "/right/camera_info"; x 
*/

const std::string oakd_imu_topic_name = "/imu";
//
constexpr int oakd_imu_downsampling_rate = 3;

const std::string oakd_stereo_depth_topic_name = "/stereo/converted_depth";
// 
constexpr int oakd_stereo_depth_downsampling_rate = 5;

const std::string oakd_stereo_depth_info_topic_name = "/stereo/camera_info";  
// 
constexpr int oakd_stereo_depth_info_downsampling_rate = 5;

const std::string oakd_stereo_points_topic_name = "/stereo/points";
// 
constexpr int oakd_stereo_points_downsampling_rate = 5;

// ===================  VN100 parameters  ===================

const std::string vn100_imu_topic_name = "/vn100_imu";
// 100 Hz -> 50 Hz
constexpr int vn100_imu_downsampling_rate = 2;

class BagRecorder : public rclcpp::Node
{
public:
    BagRecorder()
        : Node("record_all_data")
    {
        writer_ = std::make_unique<rosbag2_cpp::Writer>();

        writer_->open("handheld_pod_bag");

        lidar_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic_name, 10, std::bind(&BagRecorder::lidar_topic_callback, this, _1));
        // OAK-D
        // Color image
        oakd_color_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            oakd_color_topic_name, 1, std::bind(&BagRecorder::oakd_color_topic_callback, this, _1));
        oakd_color_camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            oakd_color_info_topic_name, 1, std::bind(&BagRecorder::oakd_color_camera_info_topic_callback, this, _1));

        /*
        // Stereo
        oakd_stereo_left_image_subscription_ = create_subscription<ssensor_msgs::msg::Image>(
            oakd_stereo_left_topic_name, 10, std::bind(&BagRecorder::oakd_left_topic_callback, this, _1));
        oakd_left_camera_info_subscription_ = create_subscription<ssensor_msgs::msg::CameraInfo>(
            oakd_stereo_left_info_topic_name, 1, std::bind(&BagRecorder::oakd_stereo_left_info_topic_callback, this, _1));
        oakd_stereo_right_image_subscription_ = create_subscription<ssensor_msgs::msg::Image>(
            oakd_stereo_right_topic_name, 10, std::bind(&BagRecorder::oakd_stereo_right_topic_callback, this, _1));
        oakd_right_camera_info_subscription_ = create_subscription<ssensor_msgs::msg::CameraInfo>(
            oakd_stereo_right_info_topic_name, 1, std::bind(&BagRecorder::oakd_stereo_right_info_topic_callback, this, _1));
*/

        // Depth
        oakd_stereo_depth_image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            oakd_stereo_depth_topic_name, 10, std::bind(&BagRecorder::oakd_stereo_depth_topic_callback, this, _1));
        oakd_stereo_depth_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            oakd_stereo_depth_info_topic_name, 1, std::bind(&BagRecorder::oakd_stereo_depth_info_topic_callback, this, _1));

        oakd_stereo_spatial_points_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            oakd_stereo_points_topic_name, 1, std::bind(&BagRecorder::oakd_stereo_points_topic_callback, this, _1));

        oakd_imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
            oakd_imu_topic_name, 1, std::bind(&BagRecorder::oakd_imu_topic_callback, this, _1));

        vn100_imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
            vn100_imu_topic_name, 10, std::bind(&BagRecorder::vn100_imu_topic_callback, this, _1));

        // Initialise the frame counter
        frame_counter_ = {};
    }

private:
    void lidar_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {   
        if((frame_counter_[0] % lidar_downsampling_rate) == 0){
            rclcpp::Time time_stamp = this->now();
            writer_->write(*msg, lidar_topic_name, "sensor_msgs/msg/LaserScan", time_stamp);
        }
        frame_counter_[0]++;
    }

    void oakd_color_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {   
        if((frame_counter_[1] % oakd_color_downsampling_rate) == 0){
            rclcpp::Time time_stamp = this->now();
            writer_->write(*msg, oakd_color_topic_name, "sensor_msgs/msg/Image", time_stamp);
        }
        frame_counter_[1]++;
    }

    void oakd_color_camera_info_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        if((frame_counter_[2] % oakd_color_info_downsampling_rate) == 0){
            rclcpp::Time time_stamp = this->now();
            writer_->write(*msg, oakd_color_info_topic_name, "sensor_msgs/msg/CameraInfo", time_stamp);
        }
        frame_counter_[2];
    }
/*
    void oakd_stereo_left_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        rclcpp::Time time_stamp = this->now();

        writer_->write(*msg, oakd_stereo_left_topic_name, "sensor_msgs/msg/Image", time_stamp);
    }

    void oakd_stereo_left_info_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        rclcpp::Time time_stamp = this->now();

        writer_->write(*msg, oakd_stereo_left_info_topic_name, "sensor_msgs/msg/CameraInfo", time_stamp);
    }

    void oakd_stereo_right_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        rclcpp::Time time_stamp = this->now();

        writer_->write(*msg, oakd_stereo_right_topic_name, "sensor_msgs/msg/Image", time_stamp);
    }

    void oakd_stereo_right_info_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
    {
        rclcpp::Time time_stamp = this->now();

        writer_->write(*msg, oakd_stereo_right_info_topic_name, "sensor_msgs/msg/CameraInfo", time_stamp);
    }
*/
    void oakd_stereo_depth_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {   
        if((frame_counter_[3] % oakd_stereo_depth_downsampling_rate) == 0){
            rclcpp::Time time_stamp = this->now();
            writer_->write(*msg, oakd_stereo_depth_topic_name, "sensor_msgs/msg/Image", time_stamp);
        }
        frame_counter_[3]++;

    }

    void oakd_stereo_depth_info_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {   
        if((frame_counter_[4] % oakd_stereo_depth_info_downsampling_rate) == 0){
            rclcpp::Time time_stamp = this->now();
            writer_->write(*msg, oakd_stereo_depth_info_topic_name, "sensor_msgs/msg/CameraInfo", time_stamp);
        }
        frame_counter_[4]++;

    }

    void oakd_stereo_points_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {
        if((frame_counter_[5] % oakd_stereo_points_downsampling_rate) == 0){
            rclcpp::Time time_stamp = this->now();
            writer_->write(*msg, oakd_stereo_points_topic_name, "sensor_msgs/msg/PointCloud2", time_stamp);
        }
        frame_counter_[5]++;
    }

    void oakd_imu_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {   
        if((frame_counter_[6] % oakd_imu_downsampling_rate) == 0){
            rclcpp::Time time_stamp = this->now();
            writer_->write(*msg, oakd_imu_topic_name, "sensor_msgs/msg/Imu", time_stamp);
        }
        frame_counter_[6]++;
    }

    void vn100_imu_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
    {   
        if((frame_counter_[7] % vn100_imu_downsampling_rate) == 0){
            rclcpp::Time time_stamp = this->now();
            writer_->write(*msg, vn100_imu_topic_name, "sensor_msgs/msg/Imu", time_stamp);
        }
        frame_counter_[7]++;
    }

    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr lidar_subscription_;
    
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr oakd_color_image_subscription_;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr oakd_color_camera_info_subscription_;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr oakd_stereo_depth_image_subscription_;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr oakd_stereo_depth_info_subscription_;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr oakd_stereo_spatial_points_subscription_;
    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr oakd_imu_subscription_;

    rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr vn100_imu_subscription_;

    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    std::array<int, 8> frame_counter_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BagRecorder>());
    rclcpp::shutdown();
    return 0;
}
