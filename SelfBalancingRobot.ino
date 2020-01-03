//#include "BluetoothComms.h"
#include "EduBotics_MPU6050.h"
#include "EduBotics_MotorController.h"
#include <PID_v1.h>
#include <EEPROM.h>

#define ID "Egglet Mk4"
#define WIFI_SSID "Egglet Mk4 007"

// Bluetooth comms definition 
#define BUFFER_SIZE 200
// ADC definitions 
#define VOLTAGE_DIVIDER_PIN A0
#define R1 10.0
#define R2 11.0

// Movement
bool active = true; 

// Datalog 
uint8_t data_log_data[BUFFER_SIZE] = {0};
uint16_t data_log_timestamps[BUFFER_SIZE] = {0};
bool data_log_on = false;
uint16_t data_log_period = 50;     // How regularly measurements are made, ms
unsigned int data_log_counter = 0; 
unsigned long previous_time = millis();

// PID
static double zero_offset = 0;
static double Input = 0;
static double Output= 0;
static double yaw, pitch, roll; 
// No vacuum form gains
double Kp = 0;
double Ki = 0;
double Kd = 0;

// double Kp = 17;
// double Kd = 0;
// double Ki = 0;

//Vacuum form gains
//double Kp = 10.2;
//double Kd = 0.02125;
//double Ki = 11.76;

PID balancePid(&Input, &Output, &zero_offset, Kp, Ki,Kd, DIRECT);

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
  balancePid.SetMode(MANUAL);

	Serial.println("SET ZERO");
	String response = Serial.readString(); 

	zero_offset = double(response.toFloat());

	String res = String(zero_offset);
  delay(200);
	Serial.println(res);

  // Write zero to EEPROM
  EEPROM.put(192, zero_offset);

  balancePid.SetMode(AUTOMATIC);
	return 0; 
}

//===========================================================
//            uploadData
// - Upload detailed data to matlab
//===========================================================
void uploadData(void) {

  Serial.println(String(BUFFER_SIZE));
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
  double voltage_reading = analogRead(A0);
  double voltage_value = voltage_reading*(5.0/1023);

  voltage_value = voltage_value * (R1 + R2) / R2;

  Serial.println(String(voltage_value));
}

//===========================================================
//            updateTunings
// - Update PID parameters on the fly 
//===========================================================
int updateTunings() {
  motorController.setSpeed(0);                        // Turn off motors 
  balancePid.SetMode(MANUAL);
	Serial.println("SET PID VALUES");                   // Notify user/MATLAB we're in updateTunings mode
  while(!Serial.available());                         // Wait for a response
	String response = Serial.readString();              // Read the response. Format "<P parameter>,<I parameter>,<D parameter>". e.g "12,10,0.0193"

  // Find the delimiters in the command
	uint8_t first_comma = response.indexOf(',');
  uint8_t second_comma = response.indexOf(',', first_comma+1);  

  // Split it up
  String kp_str = response.substring(0, first_comma); 
  String ki_str = response.substring(first_comma+1, second_comma); 
  String kd_str = response.substring(second_comma+1, response.length()); 

  // Convert to numbers 
  Kp = kp_str.toFloat();
  Ki = ki_str.toFloat();
  Kd = kd_str.toFloat();

  // Send the parameters back to the user as confirmation
  delay(200);  
	String res = String(Kp) + "," + String(Ki) + "," + String(Kd);
	Serial.println(res);

  // Set tunings
  balancePid.SetTunings(Kp,Ki,Kd);

  // Write tunings to EEPROM
  EEPROM.put(0, Kp);
  EEPROM.put(64, Ki); 
  EEPROM.put(128, Kd);
  balancePid.SetMode(AUTOMATIC);
}

//=========================================================
//            setupWifi
// - Sets up WiFi
//===========================================================
void setupWiFi(void) {
Serial.println("AT"); 
  if (!Serial.find("OK")) {
    
  }
  Serial.find("OK");
  Serial.println("AT+CWMODE=3"); 
  if (!Serial.find("OK")) {
    
  }
  Serial.println("AT+CWSAP=\"ESP\",\"password\",1,4"); 
  if (!Serial.find("OK")) {
    
  }
  Serial.find("OK");
  String cmd = String("AT+CWSAP=\"") + WIFI_SSID + String("\",\"password\",1,4");
  Serial.println(cmd); 
  Serial.find("OK");
  Serial.println("AT+CIPSTART=\"UDP\",\"192.168.4.2\",2233,2233,0"); 
  if (!Serial.find("OK")) {
    
  }
  Serial.find("OK");
  Serial.println("AT+CIPMODE=1"); 
  if (!Serial.find("OK")) {
    
  }
  Serial.find("OK");
  Serial.println("AT+CIPSEND"); 
}

//===========================================================
//            dataLog
// - Log data
//===========================================================
void dataLog(void) {
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
        float tmp = pitch*10;
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

//===========================================================
//						setup
// - Sets up serial, MPU, bluetooth and PID. 
//===========================================================
void setup() {
    // Setup MPU
  	Serial.begin(115200); 
  	while (!mpu.initialise()) {

  	    Serial.println("Failed to initialise MPU");
  	}
  
   	// setup PID    
   	zero_offset = 0; 
  	balancePid.SetSampleTime(50);
  	balancePid.SetOutputLimits(-255, 255);  

    // Read saved values from EEPROM
    EEPROM.get(0, Kp);  
    EEPROM.get(64, Ki);  
    EEPROM.get(128, Kd);  
    EEPROM.get(192, zero_offset);  

    if (isnan(Kp)) {
      Kp = 0;
      EEPROM.put(0, Kp);
    }
    if (isnan(Ki)) {
      Ki = 0;
      EEPROM.put(64, Ki); 
    }
    if (isnan(Kd)) {
      Kd = 0;
      EEPROM.put(128, Kd);
    }
    if (isnan(zero_offset)) {
      zero_offset = 0;
      EEPROM.put(192, zero_offset);
    }

    // Get a few readings to allow MPU to settle
  	for (int i = 0; i < 500; i++) {
  		mpu.getYawPitchRoll(&yaw, &pitch, &roll);
  		delay(10);
  	}	

    // Setup wifi 
    setupWiFi(); 

    // Setup ADC
    pinMode(VOLTAGE_DIVIDER_PIN, INPUT); 

    Kp = float(Kp); 
    Ki = float(Ki); 
    Kd = float(Kd); 
    zero_offset = float(zero_offset); 

    motorController.setSpeed(0); 
    balancePid.SetTunings(Kp,Ki,Kd);
    balancePid.SetMode(AUTOMATIC);
}

//===========================================================
//						loop
// - Main loop of code 
//===========================================================
void loop() {
	long tmp = 0;
  
	// Get roll pitch yaw readings
	if (mpu.getYawPitchRoll(&yaw, &pitch, &roll) == true) { 
		Input = pitch;		
	}

  // Run PID code
	if (active == true) {
    if (balancePid.Compute() == true) {
  		motorController.setSpeed(Output);
    } 
	}

  // Data log
  if (data_log_on == true) {
    dataLog(); 
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
      // Start datalog
      case 7: {
        active = true; 
        data_log_on = true; 
        previous_time = millis(); 
        break;
      }
      // Get voltage
      case 8: {
        uploadBatteryVoltage(); 
        break;
      }
      // Get ID
      case 9: {
        String res = String(ID) + "," + String(Kp) + "," + String(Ki) + "," + String(Kd) + "," + String(zero_offset);
        Serial.println(res); 
        break;
      }
      // Send datalog data
      case 11: {
          uploadData(); 
          break;
      }
      default: {
        break;
      }
    }
  }
}
