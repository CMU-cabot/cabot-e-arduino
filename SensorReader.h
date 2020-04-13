#ifndef ARDUINO_NODE_SENSORREADER_H
#define ARDUINO_NODE_SENSORREADER_H

#include <ros.h>
#include <Arduino.h>
#include "CabotNodeHandle.h"

#define ENCODER_FREQ 20 // How many times in a second

class SensorReader {
protected:
    ros::Publisher pub;
public:
    SensorReader(const char * topic_name, ros::Msg * msg)
        :pub(topic_name, msg)
    {}
    virtual void publish(CabotNodeHandle &nh)=0;
    ros::Publisher& get_publisher(){return pub;}
};


#endif //ARDUINO_NODE_SENSORREADER_H
