#ifndef EDUBOTICSMOTORCONTROLLER_H
#define EDUBOTICSMOTORCONTROLLER_H

#include "Arduino.h"
#include "EduBotics_Encoder.h"

class EduboticsMotorController {
public:
	EduboticsMotorController(int motor_one_dir_pin, int motor_one_pwm_pin, int motor_one_output_a_pin, int motor_one_output_b_pin, int motor_two_dir_pin, int motor_two_pwm_pin, int motor_two_output_a_pin, int motor_two_output_b_pin);
	int setDesiredRPM(double desired_rpm);
	double getWheelOneCurrentRPM(void);
	double getWheelTwoCurrentRPM(void);
	void update(void); 
	void stop(void); 
private: 
	int setWheelOnePWMDuty(int pwm_duty);
	int setWheelTwoPWMDuty(int pwm_duty);

	EduBoticsEncoder * encoderWheelOne;
	EduBoticsEncoder * encoderWheelTwo;

	int motor_one_current_pwm = 0; 
	int motor_two_current_pwm = 0; 
	int motor_one_dir_pin_;
	int motor_one_pwm_pin_; 
	int motor_two_dir_pin_;
	int motor_two_pwm_pin_;
	int min_speed_ = 0; 
};

#endif











