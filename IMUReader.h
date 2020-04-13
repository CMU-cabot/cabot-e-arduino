#ifndef ARDUINO_NODE_IMUREADER_H
#define ARDUINO_NODE_IMUREADER_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <sensor_msgs/Imu.h>
#include "SensorReader.h"
#include "CabotNodeHandle.h"

class IMUReader : public SensorReader{
    Adafruit_BNO055 imu = Adafruit_BNO055(55);
    sensor_msgs::Imu imu_msg;
    float initial_offset;
public:
    IMUReader();
    void realInit(float initial_offset = 180);
    void update();
    void publish(CabotNodeHandle &nh);
};


#endif //ARDUINO_NODE_IMUREADER_H
