#include <ros.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <Timer.h>
#include "IMUReader.h"
#include "WrenchReader.h"
#include "Touch.h"
#include "Velocity.h"

using namespace std;

ros::NodeHandle nh;
Timer t;

#define IMU_DELAY (20)
#define WRENCH_DELAY (20)
#define TOUCH_DELAY (20)

#define HEARTBEAT_DELAY (10)
#define HEARTBEAT_CYCLE (1)
#define LED_PIN (13)

#define button_pin_1 2 // up
#define button_pin_2 3 // down
#define button_pin_3 4 // left
#define button_pin_4 5 // right

#define PIN_VIB1 11  //front
#define PIN_VIB3 10  //left
#define PIN_VIB4 9   //right
#define PIN_VIB2 6   //back //not using

IMUReader imuReader;
WrenchReader wrenchReader;
Touch touchReader;
Velocity velPublisher;

void num_cb_1( const std_msgs::UInt8& cmd_msg){
    analogWrite(PIN_VIB1, cmd_msg.data);
}

void num_cb_2( const std_msgs::UInt8& cmd_msg){
    analogWrite(PIN_VIB2, cmd_msg.data);
}

void num_cb_3( const std_msgs::UInt8& cmd_msg){
    analogWrite(PIN_VIB3, cmd_msg.data);
}

void num_cb_4( const std_msgs::UInt8& cmd_msg){
    analogWrite(PIN_VIB4, cmd_msg.data);
}

void hearbeat();
void updateIMU();
void updateTouch();
void updateWrench();

ros::Subscriber<std_msgs::UInt8> sub_1("vibrator1", num_cb_1);
ros::Subscriber<std_msgs::UInt8> sub_2("vibrator2", num_cb_2);
ros::Subscriber<std_msgs::UInt8> sub_3("vibrator3", num_cb_3);
ros::Subscriber<std_msgs::UInt8> sub_4("vibrator4", num_cb_4);

std_msgs::Bool pushed_msg_1;
std_msgs::Bool pushed_msg_2;
std_msgs::Bool pushed_msg_3;
std_msgs::Bool pushed_msg_4;

ros::Publisher pub_button_1("pushed_1", &pushed_msg_1);
ros::Publisher pub_button_2("pushed_2", &pushed_msg_2);
ros::Publisher pub_button_3("pushed_3", &pushed_msg_3);
ros::Publisher pub_button_4("pushed_4", &pushed_msg_4);
bool touch_status = false;

void setup()
{
    // set baud rate
    nh.getHardware()->setBaud(500000);
    
    nh.initNode();

    nh.subscribe(sub_1);
    nh.subscribe(sub_2);
    nh.subscribe(sub_3);
    nh.subscribe(sub_4);
    
    nh.advertise(imuReader.get_publisher());
    nh.advertise(wrenchReader.get_publisher());
    nh.advertise(touchReader.get_publisher());
    nh.advertise(touchReader.get_raw_publisher());
    nh.advertise(velPublisher.get_publisher());

    nh.advertise(pub_button_1);
    nh.advertise(pub_button_2);    
    nh.advertise(pub_button_3);
    nh.advertise(pub_button_4);
  
    pinMode(LED_PIN, OUTPUT);
    analogWrite(LED_PIN, 255);

    pinMode(button_pin_1, INPUT_PULLUP);
    pinMode(button_pin_2, INPUT_PULLUP);
    pinMode(button_pin_3, INPUT_PULLUP);
    pinMode(button_pin_4, INPUT_PULLUP);

    pinMode(PIN_VIB1, OUTPUT);
    analogWrite(PIN_VIB1,0);

    pinMode(PIN_VIB2, OUTPUT);
    analogWrite(PIN_VIB2,0);

    pinMode(PIN_VIB3, OUTPUT);
    analogWrite(PIN_VIB3,0);

    pinMode(PIN_VIB4, OUTPUT);
    analogWrite(PIN_VIB4,0);
    
    while(!nh.connected()) {nh.spinOnce();}
    
    imuReader.realInit();
    wrenchReader.realInit();
    touch_status = touchReader.init();

    nh.loginfo(touch_status ? "Touch is working" : "Touch is not working");

    t.every(IMU_DELAY, updateIMU);
    t.every(TOUCH_DELAY, updateTouch);
    t.every(WRENCH_DELAY, updateWrench);
    t.every(HEARTBEAT_DELAY, heartbeat);
}

void loop()
{
  bool reading_1 = !digitalRead(button_pin_1);
  bool reading_2 = !digitalRead(button_pin_2);
  bool reading_3 = !digitalRead(button_pin_3);
  bool reading_4 = !digitalRead(button_pin_4);

  for(int i = 0; i < 10; i++) {
    delay(1);
    reading_1 = reading_1 && !digitalRead(button_pin_1);
    reading_2 = reading_2 && !digitalRead(button_pin_2);
    reading_3 = reading_3 && !digitalRead(button_pin_3);  
    reading_4 = reading_4 && !digitalRead(button_pin_4);  
  }
  
  pushed_msg_1.data = reading_1;
  pushed_msg_2.data = reading_2;
  pushed_msg_3.data = reading_3;
  pushed_msg_4.data = reading_4;
        
  pub_button_1.publish(&pushed_msg_1);
  pub_button_2.publish(&pushed_msg_2);  
  pub_button_3.publish(&pushed_msg_3);
  pub_button_4.publish(&pushed_msg_4); 
  nh.spinOnce();  
  t.update();
  delay(1);
}

void updateIMU()
{
    imuReader.update();    
    imuReader.publish(nh);
}

void updateWrench()
{
    wrenchReader.update();
    wrenchReader.publish(nh);
}

void updateTouch()
{
    touch_status = touchReader.getTouched(0);
    velPublisher.update(touch_status);
    touchReader.publish(nh);
    velPublisher.publish(nh);    
}

void heartbeat(){
    static int status = 0;
    status = status+1;
    analogWrite(LED_PIN, sin(6.28*status*HEARTBEAT_CYCLE*HEARTBEAT_DELAY/1000.0)*127 + 127);
}
