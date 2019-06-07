#include <ros.h>
#include <Arduino.h>
#include <Timer.h>
#include "IMUReader.h"
#include "WrenchReader.h"
ros::NodeHandle nh;
Timer t;

#define SENSOR_DELAY (10)
#define HEARTBEAT_DELAY (10)
#define HEARTBEAT_CYCLE (1)
#define LED_PIN (13)

IMUReader imuReader;
WrenchReader wrenchReader;

void hearbeat();
void updateSensors();

void setup()
{
    // set baud rate
    nh.getHardware()->setBaud(500000);
    
    nh.initNode();
    
    nh.advertise(imuReader.get_publisher());
    nh.advertise(wrenchReader.get_publisher());
    
    pinMode(LED_PIN, OUTPUT);
    analogWrite(LED_PIN, 255);
    
    while(!nh.connected()) {nh.spinOnce();}
    
    imuReader.realInit();
    wrenchReader.realInit();
    
    t.every(SENSOR_DELAY, updateSensors);
    t.every(HEARTBEAT_DELAY, heartbeat);
}

void loop()
{
    nh.spinOnce();  
    t.update();
}

void updateSensors()
{
    imuReader.update();
    wrenchReader.update();
    
    nh.spinOnce();
    
    imuReader.publish(nh);
    wrenchReader.publish(nh);
}

void heartbeat(){
    static int status = 0;
    status = status+1;
    
    analogWrite(LED_PIN, sin(6.28*status*HEARTBEAT_CYCLE*HEARTBEAT_DELAY/1000.0)*127 + 127);
}
