#ifndef TOUCH_H
#define TOUCH_H

#include <Wire.h>
#include "Adafruit_MPR121.h"
#include "SensorReader.h"
#include "std_msgs/Int16.h"
#include "CabotNodeHandle.h"

class Touch : public SensorReader{
    int16_t touchData;
    std_msgs::Int16 currTouched; //each of 12 channels are represented as 1 bit in message
    Adafruit_MPR121 cap = Adafruit_MPR121();

public:
    Touch();
    bool init();
    void publish(CabotNodeHandle &nh);
    bool getTouched(int pinNum);
    int get_velocity(bool status);
};

#endif //TOUCH_H
