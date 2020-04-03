#include "Velocity.h"

Velocity::Velocity()
        : SensorReader("velocity", &vel_msg)
{}

void Velocity::update(bool status) {
  if (status) {
    vel_msg.linear.x = 20.0;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = 20.0;
  } else {
    vel_msg.linear.x = 0.0;
    vel_msg.linear.y = 0.0;
    vel_msg.linear.z = 0.0;
    vel_msg.angular.x = 0.0;
    vel_msg.angular.y = 0.0;
    vel_msg.angular.z = 0.0;
  }
}

void Velocity::publish(ros::NodeHandle &nh){
    this->pub.publish( &vel_msg );
}
