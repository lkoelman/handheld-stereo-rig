#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/file.hpp>

#include <chrono>
#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "vn/compositedata.h"
#include <vn/sensors.h>

#include "sensor_msgs/msg/imu.hpp"

using namespace vn::protocol::uart;
using namespace vn::sensors;
using namespace vn::math;

const std::string kSensorPortName =
    "/dev/serial/by-id/usb-FTDI_USB-RS232_Cable_FT1WD85D-if00-port0";
constexpr int kSensorBaudRate = 115200;

void destroyVN100(VnSensor &vs)
{
  // Register async callback function
  vs.unregisterAsyncPacketReceivedHandler();
  vs.disconnect();
}

uint32_t getUpdateFrequency(VnSensor &vs)
{
  return 50; // static_cast<uint32_t>(vs.getReadingRateFrequency());
}

// Custom user data to pass to packet callback function
struct UserData
{
  vec4f quaternion;
  vec3f angular_rate;
  vec3f acceleration;
  vec3f orientationStdDev;

  bool is_updated = false;

  // ROS header time stamp adjustments
  double average_time_difference{0};
  rclcpp::Time ros_start_time;

  bool adjust_ros_timestamp{false};

  // int frame_id = 0;
  rclcpp::Time packet_timestamp;
};

rclcpp::Time get_time_stamp(vn::sensors::CompositeData &cd, UserData *user_data,
                            const rclcpp::Time &ros_time)
{
  if (!cd.hasTimeStartup() || !user_data->adjust_ros_timestamp)
  {
    return (ros_time); // don't adjust timestamp
  }
  const double sensor_time = cd.timeStartup() * 1e-9; // time in seconds
  if (user_data->average_time_difference == 0)
  { // first call
    user_data->ros_start_time = ros_time;
    user_data->average_time_difference = static_cast<double>(-sensor_time);
  }
  // difference between node startup and current ROS time
  const double ros_dt = (ros_time - user_data->ros_start_time).seconds();
  // difference between elapsed ROS time and time since sensor startup
  const double dt = ros_dt - sensor_time;
  // compute exponential moving average
  const double alpha = 0.001; // average over rougly 1000 samples
  user_data->average_time_difference =
      user_data->average_time_difference * (1.0 - alpha) + alpha * dt;

  // adjust sensor time by average difference to ROS time
  const rclcpp::Time adj_time =
      user_data->ros_start_time +
      rclcpp::Duration(user_data->average_time_difference + sensor_time);
  return (adj_time);
}

void configureVN100(VnSensor &vs)
{
  // Connect to the sensor
  // Default baudrate variable
  int defaultBaudrate;
  // Run through all of the acceptable baud rates until we are connected
  // Looping in case someone has changed the default
  bool baudSet = false;
  // Lets add the set baudrate to the top of the list, so that it will try
  // to connect with that value first (speed initialization up)
  std::vector<unsigned int> supportedBaudrates = vs.supportedBaudrates();
  supportedBaudrates.insert(supportedBaudrates.begin(), kSensorBaudRate);
  while (!baudSet)
  {
    // Make this variable only accessible in the while loop
    static int i = 0;
    defaultBaudrate = supportedBaudrates[i];
    BOOST_LOG_TRIVIAL(info) << "Connecting with default at " << defaultBaudrate;
    // std::cout << "Connecting with default at " << defaultBaudrate << "\n";

    // Default response was too low and retransmit time was too long by default.
    // They would cause errors
    vs.setResponseTimeoutMs(1000); // Wait for up to 1000 ms for response
    vs.setRetransmitDelayMs(50);   // Retransmit every 50 ms

    // Acceptable baud rates 9600, 19200, 38400, 57600, 128000, 115200, 230400,
    // 460800, 921600 Data sheet says 128000 is a valid baud rate. It doesn't
    // work with the VN100 so it is excluded. All other values seem to work
    // fine.
    try
    {
      // Connect to sensor at it's default rate
      if (defaultBaudrate != 128000 && kSensorBaudRate != 128000)
      {
        vs.connect(kSensorPortName, defaultBaudrate);
        // Issues a change baudrate to the VectorNav sensor and then
        // reconnects the attached serial port at the new baudrate.
        vs.changeBaudRate(kSensorBaudRate);
        // Only makes it here once we have the default correct
        BOOST_LOG_TRIVIAL(info) << "Connected baud rate is " << vs.baudrate();
        // std::cout << "Connected baud rate is " << vs.baudrate() << "\n";

        baudSet = true;
      }
    }
    // Catch all oddities
    catch (...)
    {
      // Disconnect if we had the wrong default and we were connected
      vs.disconnect();
      using namespace std::chrono_literals;

      std::this_thread::sleep_for(200ms);
    }
    // Increment the default iterator
    i++;
    // There are only 9 available data rates, if no connection
    // made yet possibly a hardware malfunction?
    if (i > 8)
    {
      break;
    }
  }
  // Now we verify connection (Should be good if we made it this far)
  if (vs.verifySensorConnectivity())
  {
    BOOST_LOG_TRIVIAL(info) << "Device connection established.";
  }

  // Some info dump
  // Query the sensor's model number.
  std::string mn = vs.readModelNumber();
  std::string fv = vs.readFirmwareVersion();
  uint32_t hv = vs.readHardwareRevision();
  uint32_t sn = vs.readSerialNumber();
  BOOST_LOG_TRIVIAL(info) << "Model number: " << vs.readModelNumber() << "\n"
                          << "Firmware version: " << vs.readFirmwareVersion()
                          << "\n"
                          << "Hardware revision: " << vs.readHardwareRevision()
                          << "\n"
                          << "Serial number: " << vs.readSerialNumber() << "\n";

  // Make sure no generic async output is registered
  vs.writeAsyncDataOutputType(VNOFF);

  // Configure binary output message
  BinaryOutputRegister bor(
      ASYNCMODE_PORT1,
      50, // update rate [ms]
      COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE |
          COMMONGROUP_TIMESTARTUP | COMMONGROUP_ACCEL,
      TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE,
      ATTITUDEGROUP_YPRU, //<-- if we want to return yaw pitch roll
                          // uncertainties in the future
      INSGROUP_NONE, GPSGROUP_NONE);

  // An empty output register for disabling output 2 and 3 if previously set
  BinaryOutputRegister bor_none(
      0, 1, COMMONGROUP_NONE, TIMEGROUP_NONE, IMUGROUP_NONE, GPSGROUP_NONE,
      ATTITUDEGROUP_NONE, INSGROUP_NONE, GPSGROUP_NONE);

  vs.writeBinaryOutput1(bor);
  vs.writeBinaryOutput2(bor_none);
  vs.writeBinaryOutput3(bor_none);

  vs.writeSettings();
}

void convertUserMessageToSensorMsg(UserData &user_data,
                                   sensor_msgs::msg::Imu &msgIMU)
{
  // Covariances
  msgIMU.orientation_covariance[0] =
      pow(user_data.orientationStdDev[2] * M_PI / 180,
          2); // Convert to radians Roll
  msgIMU.orientation_covariance[4] =
      pow(user_data.orientationStdDev[1] * M_PI / 180,
          2); // Convert to radians Pitch
  msgIMU.orientation_covariance[8] = pow(
      user_data.orientationStdDev[0] * M_PI / 180, 2); // Convert to radians Yaw

  // Since everything is in the normal frame, no flipping required
  msgIMU.orientation.x = user_data.quaternion[0];
  msgIMU.orientation.y = user_data.quaternion[1];
  msgIMU.orientation.z = user_data.quaternion[2];
  msgIMU.orientation.w = user_data.quaternion[3];

  msgIMU.angular_velocity.x = user_data.angular_rate[0];
  msgIMU.angular_velocity.y = user_data.angular_rate[1];
  msgIMU.angular_velocity.z = user_data.angular_rate[2];

  msgIMU.linear_acceleration.x = user_data.acceleration[0];
  msgIMU.linear_acceleration.y = user_data.acceleration[1];
  msgIMU.linear_acceleration.z = user_data.acceleration[2];

  user_data.is_updated = false;
}

void fillIMUMessage(vn::sensors::CompositeData &cd, UserData *user_data, rclcpp::Time time)
{

  if (cd.hasAttitudeUncertainty())
  {
    vec3f orientationStdDev = cd.attitudeUncertainty();
    user_data->orientationStdDev = orientationStdDev;
  }

  if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasTimeStartup() &&
      cd.hasAcceleration())
  {
    vec4f q = cd.quaternion();
    vec3f ar = cd.angularRate();
    vec3f al = cd.acceleration();

    user_data->quaternion = q;
    user_data->angular_rate = ar;
    user_data->acceleration = al;

    const double sensor_time = cd.timeStartup();

    BOOST_LOG_TRIVIAL(info) << "Time is: " << sensor_time;
    BOOST_LOG_TRIVIAL(info) << "Quaternions are: " << q[0] << " " << q[1] << " "
                            << q[2] << " " << q[3];
    BOOST_LOG_TRIVIAL(info)
        << "Angular rates are: " << ar[0] << " " << ar[1] << " " << ar[2];
  }

}

//
// Callback function to process data packet from sensor
//
void BinaryAsyncMessageReceived(void *userData, Packet &p, size_t index)
{
  vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);

  // evaluate time first, to have it as close to the measurement time as
  // possible
  // const rclcpp::Time ros_time = ros::Time::now();
  rclcpp::Clock clc;
  const rclcpp::Time ros_time = clc.now();

  UserData *user_data = (struct UserData *)userData;
  //UserData *user_data = 
  rclcpp::Time time = get_time_stamp(cd, user_data, ros_time);

  user_data->packet_timestamp = time;

  fillIMUMessage(cd, user_data, time);

  user_data->is_updated = true;
}

/////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  // ROS related things goes here
  rclcpp::init(argc, argv);
  rclcpp::Node node_handle("vn100_device");
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher =
      node_handle.create_publisher<sensor_msgs::msg::Imu>("imu", 10);

  std::shared_ptr<VnSensor> vs = std::make_shared<VnSensor>();
  try
  {
    // Create a VnSensor object and connect to sensor
    RCLCPP_INFO(
        node_handle.get_logger(),
        "VN100 DRIVER - Init node, load params and connect to the device.");

    configureVN100(*vs);

    RCLCPP_INFO(node_handle.get_logger(),
                "VN100 DRIVER - Register callback function");
    // keeping all information passed to callback
    UserData user_data;

    // Register async callback function
    // TODO(jp): Pass the publisher as an input to the callback function
    vs->registerAsyncPacketReceivedHandler(&user_data,
                                          BinaryAsyncMessageReceived);

    uint32_t loopFrequency;
    loopFrequency = getUpdateFrequency(*vs);
    // Increase the waking up frequency more than sensor publish rate
    // To not miss any received packets
    rclcpp::Rate loop_rate(loopFrequency * 2);

    while (rclcpp::ok())
    {
      if (user_data.is_updated)
      {
        sensor_msgs::msg::Imu msgIMU;
        convertUserMessageToSensorMsg(user_data, msgIMU);
        // TODO(jp): publish the package
        publisher->publish(msgIMU);
      }
      loop_rate.sleep();
    }

    destroyVN100(*vs);

    return 0;
  }
  catch (std::exception const &refE)
  {
    RCLCPP_ERROR(node_handle.get_logger(), "VN100 DRIVER - %s", refE.what());
  }

  destroyVN100(*vs);

  return 0;
}
