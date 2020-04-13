#include "WrenchReader.h"
#include <math.h>
#include "CabotNodeHandle.h"

WrenchReader::WrenchReader()
    : SensorReader("wrench", &wrench_msg){}

void WrenchReader::realInit(){
    pinMode(TORQUE_PIN, INPUT);
    pinMode(FORCE_PIN, INPUT);
}

void WrenchReader::update(){
    wrench_msg.wrench.torque.x = analogRead(TORQUE_PIN);
    wrench_msg.wrench.force.x = analogRead(FORCE_PIN);
}

void WrenchReader::publish(CabotNodeHandle &nh){
    wrench_msg.header.stamp = nh.now();
    this->pub.publish( &wrench_msg );
}
