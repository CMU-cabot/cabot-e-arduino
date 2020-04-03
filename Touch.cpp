#include "Touch.h"

Touch::Touch()
        : SensorReader("touch", &currTouched)
{}

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
