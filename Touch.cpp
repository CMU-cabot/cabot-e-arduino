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

#include "Touch.h"

Touch::Touch()
        : SensorReader("touch", &currTouched) {}

bool Touch::init(){
    // Default address is 0x5A, if tied to 3.3V its 0x5B
    // If tied to SDA its 0x5C and if SCL then 0x5D

    //first check if MPR121 is plugged in
    //Wire.beginTransmission(0x5A);
    //uint8_t error = Wire.endTransmission();

    if (cap.begin(0x5A)){ //success, initialize MPR121
        //cap.begin(0x5A);
        return true;
    }
    else{  //unknown error, return failure
        return false;
    }
}

void Touch::publish(ros::NodeHandle &nh){
    currTouched.data = touchData;
    this->pub.publish( &currTouched );
}

bool Touch::getTouched(int pinNum){
    touchData = cap.touched();
    return (touchData >> pinNum) & 0x1;
}

int Touch::get_velocity(bool status) {
  return status ? 20 : 0;
}
