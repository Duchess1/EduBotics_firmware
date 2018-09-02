#include "BluetoothComms.h"

BluetoothComms BluetoothSerial(9600);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  BluetoothSerial.println("Hello from arduino. This is a slightly larger string");
  delay(100); 
}
