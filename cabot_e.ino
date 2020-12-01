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
#include "Arduino.h"
#include <Timer.h>

#include "BarometerReader.h"
#include "ButtonsReader.h"
#include "Heartbeat.h"
#include "IMUReader.h"
#include "TouchReader.h"
#include "VibratorController.h"

ros::NodeHandle nh;
Timer timer;

// configurations
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

#define TOUCH_BASELINE (128)
#define TOUCH_THRESHOLD_DEFAULT (64)
#define RELEASE_THRESHOLD_DEFAULT (24)

// sensors
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

  // connect to rosserial
  nh.initNode();
  while(!nh.connected()) {nh.spinOnce();}

  int touch_params[3];
  int touch_baseline;
  int touch_threshold;
  int release_threshold;
  if (!nh.getParam("~touch_params", touch_params, 3, 500)) {
    nh.logwarn("Please use touch_params:=[baseline,touch,release] format to set touch params");
    touch_baseline = TOUCH_BASELINE;
    if (nh.getParam("~touch_threshold", &touch_threshold, 1, 500)) {
      nh.logwarn("touch_threshold is depricated");
    } else {
      touch_threshold = TOUCH_THRESHOLD_DEFAULT;
    }
    if (nh.getParam("~release_threshold", &release_threshold, 1, 500)) {
      nh.logwarn("release_threshold is depricated");
    } else {
      release_threshold = RELEASE_THRESHOLD_DEFAULT;
    }

    nh.logwarn(" touched  if the raw value is less   than touch_params[0] - touch_params[1]");
    nh.logwarn(" released if the raw value is higher than touch_params[0] - touch_params[2]");
  } else {
    touch_baseline = touch_params[0];
    touch_threshold = touch_params[1];
    release_threshold = touch_params[2];
  }
  char default_values[128];
  sprintf(default_values, "Using [%d, %d, %d] for touch_params", touch_baseline, touch_threshold, release_threshold);
  nh.loginfo(default_values);

  // initialize
  bmpReader.init();
  buttonsReader.init();
  imuReader.init();
  touchReader.init(touch_baseline, touch_threshold, release_threshold);
  vibratorController.init();
  heartbeat.init();
  
  // wait sensors ready
  delay(100);

  // set timers
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
