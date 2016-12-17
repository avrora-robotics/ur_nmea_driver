#ifndef UR_NMEA_DRIVER_CONFIG_H
#define UR_NMEA_DRIVER_CONFIG_H

#include "ros/ros.h"

class NMEADriverConfig
{

public:
    void init()
    {
        ros::NodeHandle parameter_node_handle("~");
        parameter_node_handle.param<int>("baud_rate", baud_rate, 115200);
        parameter_node_handle.param<std::string>("port", port, "/dev/ttyUSB0");
    }

    int baud_rate = 115200;
    std::string port = "/dev/ttyUSB0";

};

extern NMEADriverConfig nmea_driver_config;

#endif
