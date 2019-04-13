#include "BluetoothComms.h"
#include "EduBotics_MPU6050.h"
#include "EduBotics_MotorController.h"
#include <PID_v1.h>

//#define DATA_LOG 1
#define BUFFER_SIZE 300

//PID
static double setpoint = 0;
static double Input = 0;
static double Output= 0;
static double yaw, pitch, roll; 
// No vacuum form gains
 double Kp = 12;
 double Kd = .0192;
 double Ki = 13;


// double Kp = 17;
// double Kd = 0;
// double Ki = 0;

//Vacuum form gains
//double Kp = 10.2;
//double Kd = 0.02125;
//double Ki = 11.76;

#ifdef DATA_LOG
int i_rec = 0;
int16_t recordedSamples[BUFFER_SIZE] = {0};
#endif
PID balancePid(&Input, &Output, &setpoint, Kp, Ki,Kd, DIRECT);

// MPU 
EduBoticsMPU6050 mpu; 

// Motor Controller
EduboticsMotorController motorController(4, 5, 9, 6);

// Bluetooth Comms
BluetoothComms BluetoothSerial(9600);

//===========================================================
//						sendRollPitch
// - Used to send roll and pitch to MATLAB
//===========================================================
void sendRollPitch(void) {

	mpu.getYawPitchRoll(&yaw, &pitch, &roll); 

	String data = String(pitch) + String(",") + String(roll); 
	BluetoothSerial.println(data); 
	Serial.println(data);
}

//===========================================================
//						updateTunings
// - Update PID parameters on the fly 
//===========================================================
int updateTuningsBt(void) {
	BluetoothSerial.println("ACK");
	String response = BluetoothSerial.readLine(); 

	int first_comma = response.indexOf(',');
	int second_comma = response.indexOf(',');

	String kp_str = response.substring(0, first_comma); 
	String kd_str = response.substring(first_comma+1, second_comma); 
	String ki_str = response.substring(second_comma+1, response.length()); 

	Kp = kp_str.toDouble();
	Kd = kd_str.toDouble();
	Ki = ki_str.toDouble();

	balancePid.SetTunings(Kp,Kd,Ki);

	String res = "ACK," + String(Kp) + "," + String(Kd) + "," + String(Ki);
	BluetoothSerial.println(res);
}

int setZero(void) {

	BluetoothSerial.println("ACK");
	String response = BluetoothSerial.readLine(); 

	setpoint = response.toDouble(); 

	String res = "ACK_S0_" + response; 
	BluetoothSerial.println(res);

	return 0; 
}

int updateTuningsSerial() {
	Serial.println("ACK");
	String response = Serial.readStringUntil('\n'); 

	int first_comma = response.indexOf(',');
	int second_comma = response.indexOf(',');

	String kp_str = response.substring(0, first_comma); 
	String kd_str = response.substring(first_comma+1, second_comma); 
	String ki_str = response.substring(second_comma+1, response.length()); 

	Kp = kp_str.toDouble();
	Kd = kd_str.toDouble();
	Ki = ki_str.toDouble();

	balancePid.SetTunings(Kp,Kd,Ki);

	String res = "ACK," + kp_str + "," + kd_str + "," + ki_str;
	Serial.println(res);

}
//===========================================================
//						setup
// - Sets up serial, MPU, bluetooth and PID. 
//===========================================================
void setup() {
	// put your setup code here, to run once:
  	Serial.begin(115200); 
    Serial.println("Initialising MPU"); 
	while (!mpu.initialise()) {
	    Serial.println("Failed to initialise MPU");
	}
  	Serial.println("Successfully initialised MPU"); 

 	//setup PID    
 	setpoint = 2.9; 
	balancePid.SetMode(AUTOMATIC);
	balancePid.SetSampleTime(10);
	balancePid.SetOutputLimits(-255, 255);  

	for (int i = 0; i < 1000; i++) {
		mpu.getYawPitchRoll(&yaw, &pitch, &roll);
		delay(1);
	}	
}

//===========================================================
//						loop
// - Main loop of code 
//===========================================================
void loop() {
	#ifdef DATA_LOG
	long long start_time = millis(); 
	static long long last_time = millis(); 
	#endif
	long tmp = 0;

	// Motor controller code
	if (mpu.getYawPitchRoll(&yaw, &pitch, &roll) == true) { 
		tmp = roll*10;
		Input = double(tmp)/10;
		
	}

	#ifdef DATA_LOG
	start_time = millis(); 
	if (i_rec < BUFFER_SIZE) {
		if((start_time - last_time >= 10) && (i_rec < BUFFER_SIZE)) {

			recordedSamples[i_rec] = int16_t(tmp);
			last_time = start_time;
			i_rec ++;
		}
	}
	#endif
 	
	if (BluetoothSerial.available()) {
		char rec = BluetoothSerial.read(); 

		switch (rec) {
			case 1: {
				sendRollPitch();
       			break;
			}
			case 2: {
				motorController.setSpeed(0); 
				updateTuningsBt(); 
				break;
			}
			case 3: {
				motorController.setSpeed(0); 
				setZero();
				break;

			}
			#ifdef DATA_LOG
			case 4: {
				Serial.println("Start"); 
				for (int i = 0; i < BUFFER_SIZE; i++) {
					BluetoothSerial.println(String(recordedSamples[i]));
				}
				Serial.println("Done"); 
				break;
			}
			#endif
			case -1: {
				break;
			}
			default: {
			 	BluetoothSerial.println("-1");
			 	break;
        break;
			}
		}
	}
	if (Serial.available()) {
		char rec = Serial.read(); 

		switch (rec) {
			case 1: {
				sendRollPitch();
				break;
			}
			case 2: {
				motorController.setSpeed(0); 
				updateTuningsSerial(); 
				break;
			}
			case 3: {
				motorController.setSpeed(0); 
				setZero();
				break;
			}
			case -1: {
				break;
			}
			default: {
			 	Serial.println("-1");	
			 	break;
			}
		}
	}

	if (balancePid.Compute() == true && !(roll > 60 || roll < -60)) {
		motorController.setSpeed(Output); 
	}

	if  (roll > 55 || roll < -55 || pitch > 55 || pitch < -55) {
		motorController.setSpeed(0); 
	}
}
