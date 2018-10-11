#ifndef EDUBOTICSMOTORCONTROLLER_H
#define EDUBOTICSMOTORCONTROLLER_H

#include "Arduino.h"

class EduboticsMotorController {
public:
	EduboticsMotorController(int motor_one_dir_pin, int motor_one_pwm_pin, int motor_two_dir_pin, int motor_two_pwm_pin);
	int setSpeed(int speed);
private: 
	int motor_one_dir_pin_;
	int motor_one_pwm_pin_; 
	int motor_two_dir_pin_;
	int motor_two_pwm_pin_;
};

#endif











