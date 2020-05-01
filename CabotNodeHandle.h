#ifndef _CABOT_NODEHANDLE_H_
#define _CABOT_NODEHANDLE_H_

#include <ros/node_handle.h>
#include <ArduinoHardware.h>

typedef ros::NodeHandle_<ArduinoHardware, 10, 10, 256, 2048> CabotNodeHandle;

#endif //_CABOT_NODEHANDLE_H_
