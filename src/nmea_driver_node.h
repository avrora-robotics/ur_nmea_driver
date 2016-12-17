#ifndef RR_NMEA_DRIVER_H
#define RR_NMEA_DRIVER_H

#include <string>
#include <vector>
#include <map>

#include "ros/ros.h"
#include "serial/serial.h"
#include "nmea_parser.h"

class NMEADriver
{
public:
  NMEADriver(ros::NodeHandle& node_handle, std::shared_ptr<serial::Serial> serial_port);
  ~NMEADriver() { }

  void update();

protected:
  virtual bool processMessage();

private:
  ros::Publisher gps_publisher_;
  std::shared_ptr<serial::Serial> serial_port_;

  NMEAParser parser_;
  std::map<std::string, bool> parser_flags_;

  void clearParserFlags();
};

#endif
