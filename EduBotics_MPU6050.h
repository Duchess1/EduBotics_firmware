#ifndef EDUBOTICSMPU6050_H
#define EDUBOTICSMPU6050_H

#include "Arduino.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
volatile bool mpuInterrupt_;    // indicates whether MPU interrupt pin has gone high

class EduBoticsMPU6050 {
public:
	EduBoticsMPU6050(); 
	int initialise(); 
	int update(); 
	int getYawPitchRoll(float &yaw, float &pitch, float &roll);
	int getQuaternion(float &w, float &x, float &y, float &z);
	int getAcceleration(float &x, float &y, float &z); 
	int getWorldAcceleration(float &x, float &y, float &z); 
	int getEuler(float &x, float &y, float &z); 

	static void dmpDataReady(void);
private: 
	MPU6050 mpu_;
	uint16_t fifoCount_;      				// count of all bytes currently in FIFO
	uint8_t mpuIntStatus_;   				// holds actual interrupt status byte from MPU
	uint8_t fifoBuffer_[64]; 				// FIFO storage buffer
	uint16_t packetSize_;    				// expected DMP packet size (default is 42 bytes)

	// orientation/motion vars
	Quaternion q_;           				// [w, x, y, z]         quaternion container
	VectorInt16 aa_;         				// [x, y, z]            accel sensor measurements
	VectorInt16 aaReal_;     				// [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld_;    				// [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity_;    				// [x, y, z]            gravity vector
	float euler_[3];         				// [psi, theta, phi]    Euler angle container
	float ypr_[3];           				// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
};


#endif
