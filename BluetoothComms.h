#ifndef BLUETOOTHCOMMS_H
#define BLUETOOTHCOMMS_H

#include <SoftwareSerial.h>
#include "Arduino.h"

class BluetoothComms {
public:
  BluetoothComms(int baudrate); 
  int println(String data); 
  String readString(void); 
  String readLine(void);

private: 
  SoftwareSerial * BluetoothSerial; 
};

#endif
