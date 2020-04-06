#include "Velocity.h"

Velocity::Velocity()
        : SensorReader("cabot/touch_speed", &vel_msg)
{}

void Velocity::update(bool status) {
  status ? vel_msg.data = 2.0 : vel_msg.data = 0.0;;
}

void Velocity::publish(ros::NodeHandle &nh){
    this->pub.publish( &vel_msg );
}
