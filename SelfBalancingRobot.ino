#include "BluetoothComms.h"
#include "EduBotics_MPU6050.h"
#include "EduBotics_MotorController.h"
#include <PID_v1.h>

//PID
static double setpoint = 0;
static double Input = 0;
static double Output= 0;
static double yaw, pitch, roll; 
double Kp = 15;
double Kd = .5;
double Ki = 2;
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
int updateTuningsBt() {
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
 	setpoint = 0; 
	balancePid.SetMode(AUTOMATIC);
	balancePid.SetSampleTime(1);
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

	long long start_time = millis(); 

	// Motor controller code
	if (mpu.getYawPitchRoll(&yaw, &pitch, &roll) == true) { 
		long tmp = roll*10;
		Input = double(tmp)/10;
	}
	//BluetoothSerial.println("Hello from arduino. This is a slightly larger string");
	if (BluetoothSerial.available()) {
		char rec = BluetoothSerial.read(); 

		switch (rec) {
			case 1: {
				sendRollPitch();
       			break;
			}
			case 2: {
				updateTuningsBt(); 
				break;
			}
			case -1: {
				break;
			}
			default: {
			 	BluetoothSerial.println("-1");	
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
				updateTuningsSerial(); 
				break;
			}
			case -1: {
				break;
			}
			default: {
			 	Serial.println("-1");	
			}
		}
	}

	if (balancePid.Compute() == true) {
		motorController.setSpeed(Output); 
	}
}
