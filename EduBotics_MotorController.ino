#include "EduBotics_MotorController.h"

EduboticsMotorController::EduboticsMotorController(int motor_one_dir_pin, int motor_one_pwm_pin, int motor_two_dir_pin, int motor_two_pwm_pin){

	motor_one_dir_pin_ = motor_one_dir_pin;
	motor_one_pwm_pin_ = motor_one_pwm_pin;
	motor_two_dir_pin_ = motor_two_dir_pin;
	motor_two_pwm_pin_ = motor_two_pwm_pin;

	pinMode(motor_one_dir_pin_, OUTPUT);
	pinMode(motor_one_pwm_pin_, OUTPUT);
	pinMode(motor_two_dir_pin_, OUTPUT);
	pinMode(motor_two_pwm_pin_, OUTPUT);
}

int EduboticsMotorController::setSpeed(int motor_speed) {
	
	static int previous_speed = 0; 


	if (motor_speed > -min_speed_ && motor_speed < min_speed_) {
		motor_speed = 0; 
	}

	if (motor_speed > 0) {
		
		digitalWrite(motor_one_dir_pin_, HIGH); 
		digitalWrite(motor_two_dir_pin_, LOW); 

		// if (previous_speed < 0) {
		// 	motor_speed += 50; 
		// 	previous_speed = motor_speed;
		// }
	}
	else if (motor_speed < 0) {
		motor_speed = -motor_speed;

		// if (previous_speed > 0) {
		// 	motor_speed -= 50; 
		// 	previous_speed = motor_speed; 
		// }
		// if (motor_speed < 0) {
		// 	motor_speed = -motor_speed;
		// }
		digitalWrite(motor_one_dir_pin_, LOW); 
		digitalWrite(motor_two_dir_pin_, HIGH); 
	}

	analogWrite(motor_one_pwm_pin_, motor_speed);
	analogWrite(motor_two_pwm_pin_, motor_speed);
}