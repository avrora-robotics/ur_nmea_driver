#include "nmea_driver_node.h"

NMEADriver::NMEADriver(ros::NodeHandle& node_handle, std::shared_ptr<serial::Serial> serial_port)
  : serial_port_(serial_port)
{
  gps_publisher_ = node_handle.advertise<gps_common::GPSFix>("/gps_output", 16);

  parser_flags_.insert(std::pair<std::string, bool>("RMC", false));
  parser_flags_.insert(std::pair<std::string, bool>("GSA", false));
  parser_flags_.insert(std::pair<std::string, bool>("GGA", false));

}

void NMEADriver::update()
{
  auto bytes_available = serial_port_->available();
  while (bytes_available--) {
    uint8_t current_byte;
    serial_port_->read(&current_byte, 1);

    char byte = static_cast<char>(current_byte);

    parser_.addByte(byte);

    if (parser_.isReadyToParse() && processMessage()) {
      gps_common::GPSFix message = parser_.getMessage();
      parser_.clear();

      message.header.frame_id = "world";
      message.header.stamp = ros::Time::now();

      gps_publisher_.publish(message);
    }
  }
}

bool NMEADriver::processMessage()
{
  if (parser_.parse()) {
    try {
      parser_flags_.at(parser_.getMessageType()) = true;
    } catch (std::out_of_range) {

      parser_.clear();
      clearParserFlags();
      return false;
    }
  }

  for (auto const& flag : parser_flags_) {
    if (!flag.second) {
      return false;
    }
  }

  clearParserFlags();
  return true;
}

void NMEADriver::clearParserFlags()
{
  for (auto const& flag : parser_flags_) {
    parser_flags_.at(flag.first) = false;
  }
}
