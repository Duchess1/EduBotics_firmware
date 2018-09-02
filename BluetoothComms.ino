#include "BluetoothComms.h"

BluetoothComms::BluetoothComms(int baudrate) {
  BluetoothSerial = new SoftwareSerial(7,8);      //RX, TX  
  BluetoothSerial->begin(9600); 
}

int BluetoothComms::println(String data) {
  BluetoothSerial->println(data);
}

String BluetoothComms::readString(void) {
  return BluetoothSerial->readString(); 
}

String BluetoothComms::readLine(void) {
  return BluetoothSerial->readStringUntil('\n'); 
}


