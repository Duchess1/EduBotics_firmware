#include "BluetoothComms.h"
#include "EduBotics_MPU6050.h"

EduBoticsMPU6050 mpu; 
BluetoothComms BluetoothSerial(9600);

void sendRollPitch(void) {
	float yaw, pitch, roll;

    if (mpu.update()) {
        mpu.getYawPitchRoll(yaw, pitch, roll); 

        // BluetoothSerial.print(pitch);
        String data = String(pitch) + String(",") + String(roll); 
        BluetoothSerial.println(data); 
        Serial.println(data); 
    }
    else {
        BluetoothSerial.println("Error reading from device"); 
        Serial.println("Error reading from device"); 
    }
}

void setup() {
	// put your setup code here, to run once:
  	Serial.begin(115200); 
	while (!mpu.initialise()) {
	    Serial.println("Failed to initialise MPU");
	}
}

void loop() {
	// put your main code here, to run repeatedly:

	//BluetoothSerial.println("Hello from arduino. This is a slightly larger string");
	if (BluetoothSerial.available()) {
		char rec = BluetoothSerial.read(); 

		switch (rec) {
			case 1: {
				sendRollPitch();
			}
			case -1: {
				break;
			}
			default: {
			 	BluetoothSerial.println("-1");	
			}
		}
	}
	if (Serial.available()) {
		char rec = Serial.read(); 

		switch (rec) {
			case 1: {
				sendRollPitch();
			}
			case -1: {
				break;
			}
			default: {
			 	Serial.println("-1");	
			}
		}
	}
	mpu.update();
}