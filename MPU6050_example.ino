#include "EduBotics_MPU6050.h"

EduBoticsMPU6050 mpu; 

void setup() {
    Serial.begin(115200); 
    while (!mpu.initialise()) {
        Serial.println("Failed to initialise MPU");
    }
    Serial.println("Initialised MPU");  
}
    
void loop() {
    float yaw, pitch, roll;

   if (mpu.update()) {
        mpu.getYawPitchRoll(yaw, pitch, roll); 

        Serial.print("Pitch: ");
        Serial.print(pitch);
        Serial.print(" Roll: ");
        Serial.println(roll); 
    }
    else {
        Serial.println("Error reading from device"); 
    }
}