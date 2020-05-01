#include "IMUReader.h"
#include "CabotNodeHandle.h"

#define D2R 0.0174532925

static float angle_constrain(float angle){
    while (angle > 180) {
        angle -= 360;
    }
    while (angle < -180) {
    	angle +=360;
    }
   	return angle;
}

IMUReader::IMUReader()
        : SensorReader("imu", &imu_msg)
{}

void IMUReader::realInit(float initial_offset){
    if(!imu.begin())
    {
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    }
    this->initial_offset = initial_offset;
    delay(1000);
    imu.setExtCrystalUse(true);
    imu_msg.orientation_covariance[0] = 0.1;
    imu_msg.orientation_covariance[4] = 0.1;
    imu_msg.orientation_covariance[8] = 0.1;
}

void IMUReader::update(){
    imu::Quaternion q = imu.getQuat();

    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();

    imu::Vector<3> xyz = imu.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    imu_msg.angular_velocity.x = xyz.x()*D2R;
    imu_msg.angular_velocity.y = xyz.y()*D2R;
    imu_msg.angular_velocity.z = xyz.z()*D2R;
    
    xyz = imu.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    imu_msg.linear_acceleration.x = xyz.x();
    imu_msg.linear_acceleration.y = xyz.y();
    imu_msg.linear_acceleration.z = xyz.z();
}

void IMUReader::publish(CabotNodeHandle &nh){
    this->imu_msg.header.stamp = nh.now();
    this->imu_msg.header.frame_id = "imu_frame";

    this->pub.publish( &imu_msg );
}
