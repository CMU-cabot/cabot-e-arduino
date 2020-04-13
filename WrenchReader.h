#ifndef CABOT_E_WRENCHREADER_H
#define CABOT_E_WRENCHREADER_H

#include <geometry_msgs/WrenchStamped.h>
#include "SensorReader.h"
#include "CabotNodeHandle.h"

#define TORQUE_PIN A0
#define FORCE_PIN A1

class WrenchReader : public SensorReader{
    geometry_msgs::WrenchStamped wrench_msg;
public:
    WrenchReader();
    void realInit();
    void update();
    void publish(CabotNodeHandle &nh);
};

#endif //CABOT_E_WRENCHREADER_H
