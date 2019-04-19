//#include "BluetoothComms.h"
#include "EduBotics_MPU6050.h"
#include "EduBotics_MotorController.h"
#include <PID_v1.h>

#define BUFFER_SIZE 200

// Movement
bool active = true; 

// Datalog 
uint8_t data_log_data[BUFFER_SIZE] = {0};
uint16_t data_log_timestamps[BUFFER_SIZE] = {0};
bool data_log_on = false;
uint16_t data_log_period = 50;     // How regularly measurements are made, ms
unsigned int data_log_counter = 0; 

// PID
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

PID balancePid(&Input, &Output, &setpoint, Kp, Ki,Kd, DIRECT);

// MPU 
EduBoticsMPU6050 mpu; 

// Motor Controller
EduboticsMotorController motorController(4, 5, 9, 6);

//===========================================================
//						sendRollPitch
// - Used to send roll and pitch to MATLAB
//===========================================================
void sendRollPitch(void) {

	mpu.getYawPitchRoll(&yaw, &pitch, &roll); 

	String data = String(pitch) + String(",") + String(roll); 
	Serial.println(data);
}

//===========================================================
//            setZero
// - Set the zero point of the robot
//===========================================================
int setZero(void) {
  motorController.setSpeed(0);

	Serial.println("SET ZERO");
	String response = Serial.readString(); 

	setpoint = response.toDouble(); 

	String res = String(setpoint);
	Serial.println(res);

	return 0; 
}

//===========================================================
//            uploadData
// - Upload detailed data to matlab
//===========================================================
void uploadData(void) {

  Serial.println(String("BEGIN DATA DUMP. Number of readings:") + String(BUFFER_SIZE));
  while(!Serial.available());

  for (unsigned int i = 0; i < BUFFER_SIZE; i++) {
    String data = String(i) + String(',') + String(data_log_timestamps[i]) + String(',') + String(data_log_data[i]);
    Serial.println(data);
    delay(50); 
  }
}

//===========================================================
//            readBatteryVoltage
// - Read battery voltage and send to MATLAB
//===========================================================
void uploadBatteryVoltage(void) {
  int voltage_reading = analogRead(A0);
  double voltage_vale = voltage_reading*(5/1023);

  Serial.println(String(voltage_reading));
}

//===========================================================
//            updateTunings
// - Update PID parameters on the fly 
//===========================================================
int updateTunings() {
  motorController.setSpeed(0);
	Serial.println("SET PID VALUES");
  while(!Serial.available());
	String response = Serial.readString(); 

	uint8_t first_comma = response.indexOf(',');
  uint8_t second_comma = response.indexOf(',', first_comma+1);  

  String kp_str = response.substring(0, first_comma); 
  String ki_str = response.substring(first_comma+1, second_comma); 
  String kd_str = response.substring(second_comma+1, response.length()); 

  Kp = kp_str.toDouble();
  Ki = ki_str.toDouble();
  Kd = kd_str.toDouble();

	balancePid.SetTunings(Kp,Ki,Kd);

	String res = String(Kp) + "," + String(Ki) + "," + String(Kd);
	Serial.println(res);
}
//===========================================================
//            setupWifi
// - Sets up WiFi
//===========================================================
void setupWiFi(void) {
  Serial.println("AT"); 
  if (!Serial.find("OK")) {
    
  }

  Serial.println("AT+CWMODE=3"); 
  if (!Serial.find("OK")) {
    
  }

  Serial.println("AT+CWSAP=\"ESP\",\"password\",1,4"); 
  if (!Serial.find("OK")) {
    
  }
   
  Serial.println("AT+CIPSTART=\"UDP\",\"192.168.4.2\",2233,2233,0"); 
  if (!Serial.find("OK")) {
    
  }

  Serial.println("AT+CIPMODE=1"); 
  if (!Serial.find("OK")) {
    
  }

  Serial.println("AT+CIPSEND"); 
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

    setupWiFi(); 
}

//===========================================================
//						loop
// - Main loop of code 
//===========================================================
unsigned long previous_time = millis();

void loop() {
	long tmp = 0;

	// Get roll pitch yaw readings
	if (mpu.getYawPitchRoll(&yaw, &pitch, &roll) == true) { 
		tmp = roll*10;
		Input = double(tmp)/10;		
	}

  // Run PID code
	if (balancePid.Compute() == true && !(roll > 60 || roll < -60) && active == true) {
		motorController.setSpeed(Output); 
	}

  // If robot has fallen over, stop
	if  (roll > 55 || roll < -55 || pitch > 55 || pitch < -55) {
		motorController.setSpeed(0); 
	}

  // Data log
  if (data_log_on == true) {
    uint16_t timestamp = millis() - previous_time;

    if (timestamp > data_log_period) {
      previous_time = millis(); 
      // If this is the first reading
      if (data_log_counter == 0) {
        timestamp = 0;
        data_log_timestamps[0] = timestamp; 
      }
      else {
        data_log_timestamps[data_log_counter] = data_log_timestamps[data_log_counter-1] + timestamp; // Calculate total run time and save
        float tmp = roll*10;
        data_log_data[data_log_counter] = (uint8_t)tmp;
      }
      data_log_counter += 1; 
      // Once complete stop the robot
      if (data_log_counter == BUFFER_SIZE) {
        motorController.setSpeed(0); 
        active = false; 
        uploadData(); 
        data_log_on = false; 
        data_log_counter = 0;
      }
    }
  }

  // Deal with comms
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      // Send pitch
      case 0: {
          String data = String(pitch);
          Serial.println(data);
          break;
      }
      // Send roll
      case 1: {
          String data = String(roll);
          Serial.println(data);
          break;
      }
      // Send yaw
      case 2: {
          String data = String(yaw);
          Serial.println(data);
          break;
      }
      // Set controller zero point
      case 3: {
        setZero();
        break;
      }
      // Update controller tunings
      case 4: {
        updateTunings(); 
        break;
      }
      // Cut power
      case 5: {
        active = false; 
        motorController.setSpeed(0); 
        break;
      }
      // Start
      case 6: {
        active = true; 
        break;
      }
      case 7: {
        active = true; 
        data_log_on = true; 
        break;
      }
      default: {
        break;
      }
    }
  }
}
