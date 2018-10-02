#ifndef BLUETOOTHCOMMS_H
#define BLUETOOTHCOMMS_H

#include <SoftwareSerial.h>
#include "Arduino.h"

class BluetoothComms {
public:
  BluetoothComms(int baudrate); 
  int println(String data); 
  int print(float data); 
  int print(char* data); 
  String readString(void); 
  String readLine(void);
  char read(void); 
  int available(void);

private: 
  SoftwareSerial * BluetoothSerial; 
};

#endif
