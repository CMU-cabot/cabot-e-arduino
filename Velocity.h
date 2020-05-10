#ifndef ARDUINO_NODE_VELOCITY_H
#define ARDUINO_NODE_VELOCITY_H

#include <std_msgs/Float32.h>
#include "SensorReader.h"

class Velocity : public SensorReader {
 public:
  Velocity ();
  ~Velocity () = default;
  void update(bool status);
  void publish(ros::NodeHandle &nh);
 private:
  std_msgs::Float32 vel_msg;
};

#endif // ARDUINO_NODE_VELOCITY_H
