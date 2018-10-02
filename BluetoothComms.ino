#include "BluetoothComms.h"

BluetoothComms::BluetoothComms(int baudrate) {
	BluetoothSerial = new SoftwareSerial(7,8);      //RX, TX  
	BluetoothSerial->begin(9600); 
	//Serial.begin(9600);
	// delay(2000); 

	// // AT+NAMEXXX where XXX is the new name
	// Serial.print("AT+NAMEJOHNDOO");
	// delay(2000);

	// // AT+PINXXXX where XXXX new code (4 number)
	// Serial.print("AT+PIN1999");
	// delay(2000);

	// AT+BAUDX where X from 1 to 8
	// 1 -> 1200 Bauds
	// 2 -> 2400 Bauds
	// 3 -> 4800 Bauds
	// 4 -> 9600 Bauds
	// 5 -> 19200 Bauds
	// 6 -> 38400 Bauds
	// 7 -> 57600 Bauds
	// 8 -> 115200 Bauds
	//BluetoothSerial->print("AT+BAUD8"); 
}

int BluetoothComms::println(String data) {
	BluetoothSerial->println(data);
}

int print(char* data) {
	//BluetoothSerial->print(data); 
} 

String BluetoothComms::readString(void) {
	return BluetoothSerial->readString(); 
}

String BluetoothComms::readLine(void) {
	return BluetoothSerial->readStringUntil('\n'); 
}

char BluetoothComms::read(void) {
	return BluetoothSerial->read(); 
}

int BluetoothComms::available(void) {
	return BluetoothSerial->available(); 
}