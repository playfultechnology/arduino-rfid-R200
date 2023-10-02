#include <Arduino.h>
#include "r200.h"

// ctor
R200::R200() {};

bool R200::begin(Stream &stream){
  _serial = &stream;
  //_serial->setTimeout(50);
  return true;
};

bool R200::loop(){

  uint8_t bytesReceived;
  uint8_t incomingByte;
  uint8_t cmdType;
  uint16_t paramLength;
  if(_serial->available()) {
  do {
    // Use readBytes() rather than just read() because it provides timeout
    int count = _serial->readBytes(&incomingByte, 1);
    // We couldn't read any data
    if(count == 0) {
       return false;
    }
    else {
      bytesReceived++;
      if(incomingByte == 0xAA){
        bytesReceived = 1;
        Serial.print("Frame Header received!" );
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 2){
        // Frame Type
        Serial.print("Frame Type received! ");
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 3){
        cmdType = incomingByte;
        Serial.print("CMD Type received!");
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 4){
        // Param MSB
        paramLength = incomingByte << 8;
        Serial.print("Param MSB received! ");
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 5){
        // Param LSB
        paramLength += incomingByte;
        Serial.print("Param LSB received! ");
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 6){
        // Info Type
        Serial.print("Info Type received! ");
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }      
      else if(cmdType == CMD_GetModuleInfo && bytesReceived >= 7 && bytesReceived < paramLength+6){
        Serial.print((char)incomingByte);
      }
      else if(cmdType == CMD_GetModuleInfo && bytesReceived == paramLength+6){
        // CRC Type
        Serial.print("Checksum received! ");
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }
      else if(incomingByte == R200_FrameEnd){
        Serial.print("Frame End received! ");
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }
      else {
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.print(incomingByte, HEX); //echo
      }

    }
  } while (incomingByte != R200_FrameEnd);

  }

  /*
  if(_serial->available()){
    Serial.println("Response received");
    while(_serial->available() > 0) {
      uint8_t incomingByte = _serial->read();
      Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
      Serial.print((char)incomingByte);
      Serial.print(",");
      delay(10);
    }
  }
  */

/*
  if(_serial->available()){
    uint8_t incomingByte = _serial->read();
    _bytesReceived++;
    if(incomingByte == 0xAA){
        bytesReceived = 1;
        Serial.print("Frame Header received!" );
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 2){
        // Frame Type
        Serial.print("Frame Type received! ");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 3){
        cmdType = incomingByte;
        Serial.print("CMD Type received!");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 4){
        // Param MSB
        paramLength = incomingByte << 8;
        Serial.print("Param MSB received! ");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 5){
        // Param LSB
        paramLength += incomingByte;
        Serial.print("Param LSB received! ");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived >=7 && bytesReceived<(paramLength+6) && incomingByte != R200_FrameEnd){
        Serial.print(incomingByte); //echo
      }
      else if(incomingByte != R200_FrameEnd){
        Serial.write(incomingByte); //echo
      }
      else {
        Serial.write(incomingByte); //echo
      }
  }
  */
}

void R200::getModuleInfo(){

  const unsigned char GetModuleInfo[8] = {0xAA, 0x00, 0x03, 0x00, 0x01, 0x00, 0x04, 0xDD};
  Serial.println("Writing Module Info");
  _serial->write(GetModuleInfo,8);
  for(int i=0; i<8; i++){
    Serial.print(GetModuleInfo[i], HEX);
  }

  /*
  uint8_t commandFrame[MAX_SEND_LENGTH] = {0};

  commandFrame[0] = R200_FrameHeader;
  commandFrame[1] = FrameType_Command;
  commandFrame[2] = CMD_GetModuleInfo;
  commandFrame[3] = 0x00; // ParamLen MSB
  commandFrame[4] = 0x01; // ParamLen LSB
  commandFrame[5] = 0x00;  // Param
  commandFrame[6] = 0x04; // LSB of commandFrame[2] + commandFrame[3] + commandFrame[4] + commandFrame[5]
  commandFrame[7] = R200_FrameEnd;

  _serial->write(commandFrame, 8);

  getResponse();
  */
}

void R200::setMultiplePollingMode(){
  uint8_t commandFrame[MAX_SEND_LENGTH] = {0};

  // {0XAA,0X00,0X27,0X00,0X03,0X22,0XFF,0XFF,0X4A,0XDD};

  commandFrame[0] = R200_FrameHeader;
  commandFrame[1] = FrameType_Command;
  commandFrame[2] = CMD_MultiplePollInstruction;
  commandFrame[3] = 0x00; // ParamLen MSB
  commandFrame[4] = 0x03; // ParamLen LSB
  commandFrame[5] = 0x22;  // Param
  commandFrame[6] = 0xFF;  // Param
  commandFrame[7] = 0xFF;  // Param
  commandFrame[8] = 0x4A; // LSB of commandFrame[2] + commandFrame[3] + commandFrame[4] + commandFrame[5] + [6] + [7]
  // (Full checksum is 0x24A)
  commandFrame[9] = 0xDD;

  _serial->write(commandFrame, 10);

  getResponse();
}


// https://github.com/Makuna/DFMiniMp3/blob/master/src/DFMiniMp3.h
bool R200::getResponse(){

  while(!_serial->available()){
    while(_serial->available()){
      uint8_t incomingByte = _serial->read();
      Serial.print(incomingByte, HEX);
    }
  }



  //uint8_t response[64];
  //int numBytes = _serial->readBytesUntil('\x03', &response, 64);
  //if (numBytes > 0)
  //Serial.write(response, numBytes);

/*
  uint8_t bytesReceived;
  uint8_t incomingByte;
  uint8_t cmdType;
  uint16_t paramLength;
  do {
    // Use readBytes() rather than just read() because it provides timeout
    int count = _serial->readBytes(&incomingByte, 1);
    // We couldn't read any data
    if(count == 0) {
       return false;
    }
    else {
      bytesReceived++;
      if(incomingByte == 0xAA){
        bytesReceived = 1;
        Serial.print("Frame Header received!" );
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 2){
        // Frame Type
        Serial.print("Frame Type received! ");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 3){
        cmdType = incomingByte;
        Serial.print("CMD Type received!");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 4){
        // Param MSB
        paramLength = incomingByte << 8;
        Serial.print("Param MSB received! ");
        Serial.println(incomingByte, HEX);
      }
      else if(bytesReceived == 5){
        // Param LSB
        paramLength += incomingByte;
        Serial.print("Param LSB received! ");
        Serial.println(incomingByte, HEX);
      }
      else if(incomingByte != R200_FrameEnd){
        Serial.write(incomingByte); //echo
      }
    }
  } while (incomingByte != R200_FrameEnd);
  */
}


bool R200::waitForResponse(){
  unsigned long timer = millis();
  while (!available()){
    if (millis() - timer > _timeOutDuration) {
      _receivedIndex = 0;
      Serial.print("Timed out waiting for response!");
    }
    delay(0);
  }
  return true;
}

bool R200::available(){
  return _serial->available();
}

uint8_t R200::calculateCheckSum(uint8_t *buffer){

  uint16_t paramLength = *(buffer+3);
  paramLength <<=8;
  paramLength += *(buffer+4);

  uint16_t sum = 0;
  for (int i=1; i<4+paramLength; i++) {
    sum += buffer[i];
  }
  return -sum;
}

void R200::parseResponse(){
  while(_serial->available()){
    uint8_t incomingByte = _serial->read();
    Serial.print(incomingByte, HEX);
  }
}
