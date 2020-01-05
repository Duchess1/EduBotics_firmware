#include "EduBotics_Encoder.h"

EduBoticsEncoder::EduBoticsEncoder(int pin_one, int pin_two) {
  last_time = millis(); 
  
  encoder = new Encoder(pin_one, pin_two);
}

void EduBoticsEncoder::update(void) {
  long new_val = encoder->read();

  double tmp_rpm = calculateRPM(current_position, new_val);
  current_rpm = (tmp_rpm + current_rpm)/2; 
  current_position = new_val;
}

double EduBoticsEncoder::getRPM(void) {
	return current_rpm;
}

long EduBoticsEncoder::getPosition(void) {
	return current_position;
}

double EduBoticsEncoder::calculateRPM(long old_position, long new_position) {

  if (millis() - last_time == 0) {
    return current_rpm;
  }

  double pulse_count = new_position - old_position;
  // If we haven't moved return zero RPM
  if (pulse_count == 0) {
    return 0; 
  }

  double revolutions = pulse_count/counts_per_rev; 
  double speed_rpm = (pulse_count/counts_per_rev) * (60000.0/double(millis() - last_time));

  last_time = millis(); 
 
  return speed_rpm;
}