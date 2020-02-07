#ifndef EDUBOTICSENCODER_H
#define EDUBOTICSENCODER_H

#include "Arduino.h"
#include <Encoder.h>

class EduBoticsEncoder {
public:
  EduBoticsEncoder(int pin_one, int pin_two); 
  void update(void); 
  double getRPM(void);
  long getPosition(void); 

private: 
  Encoder * encoder;

  double calculateRPM(long old_position, long new_position);
  
  long current_position = 0;
  double current_rpm = 0; 
  long last_time = 0;
  double counts_per_rev = 1200;
};


#endif
