#ifndef ARDUINO_NODE_VELOCITY_H
#define ARDUINO_NODE_VELOCITY_H

#include <geometry_msgs/Twist.h>
#include "SensorReader.h"

class Velocity : public SensorReader {
 public:
  Velocity ();
  ~Velocity () = default;
  void update(bool status);
  void publish(ros::NodeHandle &nh);
 private:
  geometry_msgs::Twist vel_msg;
};

#endif // ARDUINO_NODE_VELOCITY_H
