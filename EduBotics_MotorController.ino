#include "EduBotics_MotorController.h"

EduboticsMotorController::EduboticsMotorController(int motor_one_dir_pin, int motor_one_pwm_pin, int motor_one_output_a_pin, int motor_one_output_b_pin, int motor_two_dir_pin, int motor_two_pwm_pin, int motor_two_output_a_pin, int motor_two_output_b_pin){

	motor_one_dir_pin_ = motor_one_dir_pin;
	motor_one_pwm_pin_ = motor_one_pwm_pin;
	motor_two_dir_pin_ = motor_two_dir_pin;
	motor_two_pwm_pin_ = motor_two_pwm_pin;

	pinMode(motor_one_dir_pin_, OUTPUT);
	pinMode(motor_one_pwm_pin_, OUTPUT);
	pinMode(motor_two_dir_pin_, OUTPUT);
	pinMode(motor_two_pwm_pin_, OUTPUT);

	encoderWheelOne = new EduBoticsEncoder(motor_one_output_a_pin, motor_one_output_b_pin);
	encoderWheelTwo = new EduBoticsEncoder(motor_two_output_a_pin, motor_two_output_b_pin);

	motor_one_pid = new PID(&motor_one_current_rpm, &motor_one_current_pwm, &desired_rpm_, Kp_, Ki_, Kd_, DIRECT);
	motor_two_pid = new PID(&motor_two_current_rpm, &motor_two_current_pwm, &desired_rpm_, Kp_, Ki_, Kd_, DIRECT);
	
	motor_one_pid->SetSampleTime(5);
	motor_one_pid->SetOutputLimits(-255, 255);
	motor_one_pid->SetMode(AUTOMATIC);

	motor_two_pid->SetSampleTime(5);
	motor_two_pid->SetOutputLimits(-255, 255);
	motor_two_pid->SetMode(AUTOMATIC);
}

void EduboticsMotorController::update(void) {
	encoderWheelOne->update();
	encoderWheelTwo->update(); 
}

void EduboticsMotorController::setMotorGains(double Kp, double Ki, double Kd) {
	Kp_ = Kp; 
	Ki_	= Ki; 
	Kd_ = Kd;

	motor_one_pid->SetTunings(Kp_, Ki_, Kd_); 
	motor_two_pid->SetTunings(Kp_, Ki_, Kd_); 
}

void EduboticsMotorController::stop(void) {
	pid_enabled = false; 
	setWheelOnePWMDuty(0);
  setWheelTwoPWMDuty(0);
}

void EduboticsMotorController::start(void) {
	pid_enabled = true; 
}

double EduboticsMotorController::getWheelOneCurrentRPM(void) {
	return encoderWheelOne->getRPM(); 
}

double EduboticsMotorController::getWheelTwoCurrentRPM(void) {
	return encoderWheelTwo->getRPM(); 
}

int EduboticsMotorController::setDesiredRPM(double desired_rpm) {
	
  motor_one_current_rpm = encoderWheelOne->getRPM();
  motor_two_current_rpm = encoderWheelTwo->getRPM(); 

  desired_rpm_ = desired_rpm;

  if (pid_enabled == true) {
		motor_one_pid->Compute(); 
	  motor_two_pid->Compute(); 
  }
  else { 
  	motor_one_current_pwm = 0; 
  	motor_two_current_pwm = 0; 
  }
  
  setWheelOnePWMDuty(motor_one_current_pwm);
  setWheelTwoPWMDuty(motor_two_current_pwm);
}

int EduboticsMotorController::setWheelOnePWMDuty(int pwm_duty) {

	// Set the direction of both motors to forwards
	if (pwm_duty > 0) {		
		digitalWrite(motor_one_dir_pin_, HIGH); 
	}

	// Set the direction of both motors to backwards
	else if (pwm_duty < 0) {
		pwm_duty = -pwm_duty;
		digitalWrite(motor_one_dir_pin_, LOW); 
	}

	analogWrite(motor_one_pwm_pin_, pwm_duty);
}

int EduboticsMotorController::setWheelTwoPWMDuty(int pwm_duty) {

	// Set the direction of both motors to forwards
	if (pwm_duty > 0) {
		digitalWrite(motor_two_dir_pin_, HIGH); 
	}

	// Set the direction of both motors to backwards
	else if (pwm_duty < 0) {
		pwm_duty = -pwm_duty;
		digitalWrite(motor_two_dir_pin_, LOW); 
	}

	analogWrite(motor_two_pwm_pin_, pwm_duty);
}
