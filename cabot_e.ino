/*******************************************************************************
 * Copyright (c) 2020  Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#include <ros.h>
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif
#include <Timer.h>

#include "BarometerReader.h"
#include "ButtonsReader.h"
#include "Heartbeat.h"
#include "IMUReader.h"
#include "TouchReader.h"
#include "VibratorController.h"

using namespace std;

ros::NodeHandle nh;
Timer timer;

#define BAUDRATE (115200)

#define LED_PIN (13)
#define HEARTBEAT_DELAY (20)

#define BTN1_PIN (2) // up
#define BTN2_PIN (3) // down
#define BTN3_PIN (4) // left
#define BTN4_PIN (5) // right

#define VIB1_PIN (11)  //front
#define VIB3_PIN (10)  //left
#define VIB4_PIN (9)   //right
#define VIB2_PIN (6)   //back //not using

BarometerReader bmpReader(nh);
ButtonsReader buttonsReader(nh, BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN);
IMUReader imuReader(nh);
TouchReader touchReader(nh);
VibratorController vibratorController(nh, VIB1_PIN, VIB2_PIN, VIB3_PIN, VIB4_PIN);

Heartbeat heartbeat(LED_PIN, HEARTBEAT_DELAY);


void setup()
{
  // set baud rate
  nh.getHardware()->setBaud(BAUDRATE);
  nh.initNode();

  while(!nh.connected()) {nh.spinOnce();}
    
  bmpReader.init();
  buttonsReader.init();
  imuReader.init();
  touchReader.init();
  vibratorController.init();

  heartbeat.init();
  
  // wait sensors ready
  delay(1000);

  timer.every(500, [](){
      bmpReader.update();
    });

  timer.every(20, [](){
      heartbeat.update();
      buttonsReader.update();
      touchReader.update();
    });

  timer.every(10, [](){
      imuReader.update();
    });
  
  nh.loginfo("Arduino is ready");
}

void loop()
{
  timer.update();
  nh.spinOnce();
}
