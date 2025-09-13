#pragma once

#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    // IP address and port of the micro-ROS agent
    // Change this to the agent IP and port on your network.
    static const char *ROS_AGENT_IP_ADDR = "10.101.41.119";
    static const size_t ROS_DOMAIN_ID = 69;
    static const int ROS_AGENT_UDP_PORT = 8888;

    // WiFi credentials
    static const char *SSID = "redpucp";
    static const char *PSWD = "C9AA28BA93";

#ifdef __cplusplus
}
#endif
