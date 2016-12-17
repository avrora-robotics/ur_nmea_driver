#include "ros/ros.h"
#include "serial/serial.h"
#include "nmea_driver_node.h"
#include "nmea_driver_config.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ur_nmea_driver_node");
  ros::NodeHandle node_handle;

  nmea_driver_config.init();

  if (!nmea_driver_config.port.length()) {
    ROS_FATAL_STREAM(ros::this_node::getName() << ": Wrong port name: " << nmea_driver_config.port);
    return -1;
  }

  if (nmea_driver_config.baud_rate < 0) {
    ROS_FATAL_STREAM(ros::this_node::getName() << ": Wrong baud rate: " << nmea_driver_config.baud_rate);
    return -1;
  }

  std::shared_ptr<serial::Serial> serial_port = nullptr;
  try {
    serial_port = std::shared_ptr<serial::Serial>(
          new serial::Serial(nmea_driver_config.port, nmea_driver_config.baud_rate, serial::Timeout::simpleTimeout(10000)));

  } catch (serial::IOException) {
    ROS_FATAL_STREAM(ros::this_node::getName() << ": connection error: device is busy. Try to rerun node several seconds later.");
    return -1;
  }

  NMEADriver node(node_handle, serial_port);
  ros::Rate rate(10);

  while (ros::ok() && serial_port->isOpen()) {
    rate.sleep();
    node.update();
    ros::spinOnce();
  }

  return 0;
}
