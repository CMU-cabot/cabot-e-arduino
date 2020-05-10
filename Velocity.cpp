#include "Velocity.h"

Velocity::Velocity()
        : SensorReader("touch_speed", &vel_msg)
{}

void Velocity::update(bool status) {
  vel_msg.data = status ? 2.0 : 0.0;;
}

void Velocity::publish(ros::NodeHandle &nh){
    this->pub.publish( &vel_msg );
}
