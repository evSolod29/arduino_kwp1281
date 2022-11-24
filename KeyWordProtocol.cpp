#include <Arduino.h>
#include <SoftwareSerial.h>

#include "KeyWordProtocol.h"

KeyWordProtocol::KeyWordProtocol(uint8_t receivePin, uint8_t transmitPin){
  txpin = transmitPin;

  pinMode(transmitPin, OUTPUT);
  digitalWrite(transmitPin, HIGH);

  serial = new SoftwareSerial(receivePin, transmitPin, false);
}

bool KeyWordProtocol::connect(uint8_t addr, int baudrate){
  serial->begin(baudrate);
  sendInitPocket(addr);
  if (read() != 0x55 && read() != 0x01 && readByte() != 0x8A) return false;
  uint8_t ack[1] = { 0x09 };
  while(true){
    uint8_t data[MAX_Pocket_Size];
    if(!readBlock(data, MAX_Pocket_Size)) return false;
    if(!sendBlock(ack, 1)) return false;
    if(data[2] == 0x09) break;
  }
  connected = true;
  return connected;
}

Sensor KeyWordProtocol::getSensorData(uint8_t group, uint8_t id){
  Sensor sensor;
  if(id > SENSORS_PER_GROUP) return sensor;
  uint8_t request[2] = { 0x29, id };
  uint8_t responce[13];
  if(!sendBlock(request, 2)) return sensor;
  if(!readBlock(responce, 13)) return sensor;
  sensor.type = id;
  sensor.a = 2 + (id * 3);
  sensor.b = 3 + (id * 3);
  return sensor;
}

void KeyWordProtocol::disconnect(){
  connected = false;
  serial->end();
}

bool KeyWordProtocol::isConnected(){
  return connected;
}

bool KeyWordProtocol::error(){
  disconnect();
  return false;
}

bool KeyWordProtocol::readBlock(uint8_t data[], uint8_t arraySize){
  uint8_t size;
  size = readByte();
  if(size < 3) return error();
  if(size-2 > arraySize) return error();
  for(uint8_t i = 0; i <= size - 2; i++){
    data[i] = readByte();
  }
  if(readByte() != 0x03) return error();
  counterChanged();
  return true;
}

bool KeyWordProtocol::sendBlock(uint8_t *data, uint8_t arraySize){
  uint8_t size = arraySize + 2;
  if(!sendHeader(size)) return false;
  for(int i = 0; i < arraySize; i++){
    if(!writeByte(data[i])) return false;
  }
  if(!writeByte(0x03)) return false;
  counterChanged();  
  return true;
}

bool KeyWordProtocol::sendHeader(uint8_t size){
  return writeByte(size) && writeByte(counter);
}

void KeyWordProtocol::write(uint8_t data){
  serial->write(data);
}

uint8_t KeyWordProtocol::read(){
  unsigned long timeout = millis() + 1000;
  while (!serial->available()){
    if (millis() >= timeout) {
      return 0;
    }
  }
  uint8_t data = serial->read();
  return data;
}

void KeyWordProtocol::sendInitPocket(uint8_t addr){
  byte bits[10];
  byte even=1;
  bits[0] = 0; 
  for (uint8_t i=1; i < 8; i++){
    bits[i] = (byte) ((addr & (1 << (i-1))) != 0);
    even = even ^ bits[i];
  }
  bits[8] = even;
  bits[9] = 1;
  for (uint8_t i = 0; i < 10; i++) {
    if(bits[i]) digitalWrite(txpin, HIGH);
    else digitalWrite(txpin, LOW);
    delay(200);
  }
  serial->flush(); 
}

bool KeyWordProtocol::writeByte(uint8_t data){
  write(data);
  bool result = read() == (data ^ 0xFF);
  if(result) return true;
  return error();
}

uint8_t KeyWordProtocol::readByte(){
  uint8_t data = read();
  write(data ^ 0xFF);
  return data;
}

void KeyWordProtocol::counterChanged(){
  if(counter > 0xFF)
  {
    counter = 0x00;
    return;
  }
  counter++;
}