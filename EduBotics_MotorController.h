#ifndef EDUBOTICSMOTORCONTROLLER_H
#define EDUBOTICSMOTORCONTROLLER_H

#include "Arduino.h"
#include "EduBotics_Encoder.h"
#include <PID_v1.h>

class EduboticsMotorController {
public:
	EduboticsMotorController(int motor_one_dir_pin, int motor_one_pwm_pin, int motor_one_output_a_pin, int motor_one_output_b_pin, int motor_two_dir_pin, int motor_two_pwm_pin, int motor_two_output_a_pin, int motor_two_output_b_pin);
	int setDesiredRPM(double desired_rpm);
	double getWheelOneCurrentRPM(void);
	void setMotorGains(double Kp, double Ki, double Kd); 
	double getWheelTwoCurrentRPM(void);
	void update(void); 
	void stop(void);
	void start(void);  
private: 
	int setWheelOnePWMDuty(int pwm_duty);
	int setWheelTwoPWMDuty(int pwm_duty);

	bool pid_enabled = true; 

	PID * motor_one_pid;
	double motor_one_current_pwm = 0; 
	double motor_one_current_rpm = 0;
	PID * motor_two_pid;
	double motor_two_current_pwm = 0; 
	double motor_two_current_rpm = 0;
	double Kp_ = 0; 
	double Ki_ = 0; 
	double Kd_ = 0; 
	double desired_rpm_ = 100; 
	
	EduBoticsEncoder * encoderWheelOne;
	EduBoticsEncoder * encoderWheelTwo;

	
	int motor_one_dir_pin_;
	int motor_one_pwm_pin_; 
	int motor_two_dir_pin_;
	int motor_two_pwm_pin_;
	int min_speed_ = 0; 
};

#endif











