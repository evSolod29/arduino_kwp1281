#include <SoftwareSerial.h>
#include "KeyWordProtocol.h"

#define MAX_SEND_RETRIES 5
#define NENGINEGROUPS 7

#define pinKLineRX 10
#define pinKLineTX 11
KeyWordProtocol kwp(10,11);

SoftwareSerial serial(2,3);

uint8_t retries = 0;

uint8_t engineGroups[NENGINEGROUPS] = { 2, 3, 20, 31, 118, 115, 15 };

void setup() {
  serial.begin(9600);
}

void loop() {
  if(!kwp.isConnected()){
    kwp.connect(ADR_Engine, 10400);
    return;
  }
  Sensor sensor = kwp.getSensorData(engineGroups[1], 1);
  serial.write(4);
  do{
    serial.write(sensor.type);
    serial.write(sensor.a);
    serial.write(sensor.b);
    serial.write(0x03);
    retries = 5;
  }
  while(serial.read() != (0x03 ^ 0xFF) || retries <= MAX_SEND_RETRIES);
}
