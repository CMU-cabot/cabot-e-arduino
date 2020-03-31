#include <ros.h>
#include <Arduino.h>
#include <Timer.h>
#include "IMUReader.h"
#include "WrenchReader.h"
#include "Touch.h"

ros::NodeHandle nh;
Timer t;

#define SENSOR_DELAY (10)
#define HEARTBEAT_DELAY (10)
#define HEARTBEAT_CYCLE (1)
#define LED_PIN (13)

IMUReader imuReader;
WrenchReader wrenchReader;
Touch touchReader;

void hearbeat();
void updateSensors();

bool touch_status = false;

void setup()
{
    // set baud rate
    nh.getHardware()->setBaud(500000);
    
    nh.initNode();
    
    nh.advertise(imuReader.get_publisher());
    nh.advertise(wrenchReader.get_publisher());
    nh.advertise(touchReader.get_publisher());
    
    pinMode(LED_PIN, OUTPUT);
    analogWrite(LED_PIN, 255);
    
    while(!nh.connected()) {nh.spinOnce();}
    
    imuReader.realInit();
    wrenchReader.realInit();
    touch_status = touchReader.init();
    
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
    touchReader.getTouched(5);
    
    nh.spinOnce();
    
    imuReader.publish(nh);
    wrenchReader.publish(nh);
    if(touch_status){
      touchReader.publish(nh);
    }
}

void heartbeat(){
    static int status = 0;
    status = status+1;
    
    analogWrite(LED_PIN, sin(6.28*status*HEARTBEAT_CYCLE*HEARTBEAT_DELAY/1000.0)*127 + 127);
}
