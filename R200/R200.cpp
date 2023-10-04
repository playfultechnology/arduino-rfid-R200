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
  uint8_t checksum;
  uint8_t _PC[2];
  uint8_t _EPC[12];
  uint8_t _CRC[2];

  //if(_serial->available() > 0) {
  //  Serial.print("Bytes available:");
  //  Serial.println(_serial->available());
  while(_serial->available() > 0) {
    // Use readBytes() rather than just read() because it provides timeout
    int count = _serial->readBytes(&incomingByte, 1);
    // We couldn't read any data
    if(count == 0) {
      // Serial.print("Well, this is awkward....");
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



      else if(cmdType == CMD_GetModuleInfo && bytesReceived == 6){
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
        Serial.println("");
        Serial.print("Checksum received! ");
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }


    else if(cmdType == CMD_ExecutionFailure && bytesReceived == 6){
      Serial.print("Execution Failure");
      switch(incomingByte){
        case 0x17:
          Serial.println("Command error");
          break;
        case 0x20:
          Serial.println("FHSS fail");
          break;
        case 0x15:
          Serial.println("Inventory Fail");
          break;
        case 0x16:
          Serial.println("Access Fail");
          break;
        case 0x09:
          Serial.println("Read fail");
          break;
        case 0x10:
          Serial.println("Write fail");
          break;
        case 0x13:
          Serial.println("Lock fail");
          break;
        case 0x12:
          Serial.println("Kill fail");
          break;

      }
    }


      else if(cmdType == CMD_SinglePollInstruction && bytesReceived == 6){
        // RSSI
        Serial.print("RSSI received! ");
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }
      else if(cmdType == CMD_SinglePollInstruction && bytesReceived == 7){
        Serial.print("PC(MSB) received! ");
        _PC[0] = incomingByte;
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }
      else if(cmdType == CMD_SinglePollInstruction && bytesReceived == 8){
        Serial.print("PC(LSB) received! ");
        _PC[1] = incomingByte;
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      }

      else if(cmdType == CMD_SinglePollInstruction && bytesReceived == 9) {
        Serial.print("EPC Received! ");
        _EPC[0] = incomingByte;
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.println(incomingByte, HEX);
      } 
      else if(cmdType == CMD_SinglePollInstruction && bytesReceived > 9 && bytesReceived <= 20) {
        _EPC[bytesReceived-9] = incomingByte;
        Serial.print(incomingByte < 0x10 ? "0" : "");
        Serial.print(incomingByte, HEX);
      }
      else if(cmdType == CMD_SinglePollInstruction && bytesReceived > 20 && bytesReceived <= 22) {
        _CRC[bytesReceived-21] = incomingByte;
        Serial.print(incomingByte < 0x10 ? "0" : "");
        Serial.print(incomingByte, HEX);
      }
      else if(bytesReceived == paramLength+6){
        // CRC Type
        Serial.println("");
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
  //} 
  //while (_serial->available() > 0 && incomingByte != R200_FrameEnd);

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

    // Example response
    // AA 02 22 00 11 C7 30 00 E2 80 68 90 00 00 50 0E 88 C6 A4 A7 11 9B 29 DD 
    // AA:Frame Header
    // 02:Instruction Code
    // 22:Command Parameter
    // 00 11:Instruction data length (0x11 = 17 bytes)
    // C7：RSSI Signal Strength
    // 30 00: Label PC code (factory reg code)
    // E2 80 68 90 00 00 50 0E 88 C6 A4 A7：EPC code
    // 11 9B:CRC check
    // 29: Verification
    // DD: End of frame


void R200::getModuleInfo(){

  const unsigned char GetModuleInfo[8] = {0xAA, 0x00, 0x03, 0x00, 0x01, 0x00, 0x04, 0xDD};
  Serial.print("Requesting Module Info; sending command 0x");
  _serial->write(GetModuleInfo,8);
  for(int i=0; i<8; i++){
    Serial.print(GetModuleInfo[i], HEX);
  }
  Serial.println("");
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



void R200::poll(){
  uint8_t commandFrame[MAX_SEND_LENGTH] = {0};
  // {0xAA0022000022DD};
  commandFrame[0] = R200_FrameHeader;
  commandFrame[1] = FrameType_Command;
  commandFrame[2] = CMD_SinglePollInstruction;
  commandFrame[3] = 0x00; // ParamLen MSB
  commandFrame[4] = 0x00; // ParamLen LSB
  commandFrame[5] = 0x22;  // Checksum
  commandFrame[6] = R200_FrameEnd;

  _serial->write(commandFrame, 7);
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
  commandFrame[9] = R200_FrameEnd;

  _serial->write(commandFrame, 10);

 // getResponse();
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

uint16_t R200::arrayToUint16(uint8_t *array){
  uint16_t value = *array;
  value <<=8;
  value += *(array+1);
  return value;
}