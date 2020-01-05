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
}

void EduboticsMotorController::update(void) {
	encoderWheelOne->update();
	encoderWheelTwo->update(); 
}

void EduboticsMotorController::stop(void) {
	setWheelOnePWMDuty(0); 
	setWheelTwoPWMDuty(0); 
}

double EduboticsMotorController::getWheelOneCurrentRPM(void) {
	return encoderWheelOne->getRPM(); 
}

double EduboticsMotorController::getWheelTwoCurrentRPM(void) {
	return encoderWheelTwo->getRPM(); 
}

int EduboticsMotorController::setDesiredRPM(double desired_rpm) {
	
  double motor_one_rpm = encoderWheelOne->getRPM();
  double motor_two_rpm = encoderWheelTwo->getRPM(); 

  int motor_one_delta = desired_rpm - motor_one_rpm;
  int motor_two_delta = desired_rpm - motor_one_rpm;  

	if (motor_one_rpm != desired_rpm) {
		motor_one_current_pwm += motor_one_delta * 0.4;
		if (motor_one_current_pwm > 255) {
      motor_one_current_pwm = 255; 
    }
    else if (motor_one_current_pwm < -255) {
      motor_one_current_pwm = -255; 
    }

		setWheelOnePWMDuty(motor_one_current_pwm);
	}

	if (motor_two_rpm != desired_rpm) {
		motor_two_current_pwm += motor_two_delta * 0.4;
    if (motor_two_current_pwm > 255) {
      motor_two_current_pwm = 255; 
    }
    else if (motor_two_current_pwm < -255) {
      motor_two_current_pwm = -255; 
    }
		setWheelTwoPWMDuty(motor_two_current_pwm);
	}
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
