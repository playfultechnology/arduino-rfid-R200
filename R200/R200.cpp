#include <Arduino.h>
#include "r200.h"

// ctor
R200::R200() {};

bool R200::begin(HardwareSerial *serial, int baud, uint8_t RxPin, uint8_t TxPin){
  _serial = serial;
  _serial->begin(baud, SERIAL_8N1, RxPin, TxPin);
  //_serial->setTimeout(50);
  return true;
};

void printHexByte(char* name, uint8_t value){
  Serial.print(name);
  Serial.print(":");
  Serial.print(value < 0x10 ? "0x0" : "0x");
  Serial.println(value, HEX);
}

void printHexBytes(char* name, uint8_t *value, uint8_t len){
  Serial.print(name);
  Serial.print(":");
  Serial.print("0x");
  for(int i=0; i<len; i++){
    Serial.print(value[i] < 0x10 ? "0" : "");
    Serial.print(value[i], HEX);
  }
  Serial.println("");
}

void printHexWord(char* name, uint8_t MSB, uint8_t LSB){
  Serial.print(name);
  Serial.print(":");
  Serial.print(MSB < 0x10 ? "0x0" : "0x");
  Serial.println(MSB, HEX);
  Serial.print(LSB < 0x10 ? "0" : "");
  Serial.println(LSB, HEX);
}


void R200::loop(){
  // Has any new data been received?
  if(dataAvailable()){
    // Attempt to receive a full frame of data
    if(receiveData()){
      dumpReceiveBufferToSerial();
      // If a full frame of data has been received, parse it
      parseData();
    };
  }
}

// Has any data been received from the reader?
bool R200::dataAvailable(){
  return _serial->available() >0;
}

void R200::dumpReceiveBufferToSerial(){
  Serial.print("Dumping buffer...");
  Serial.print("0x");
  for (uint8_t i=0; i< RX_BUFFER_LENGTH; i++){
    Serial.print(_buffer[i] < 0x10 ? "0" : "");
    Serial.print(_buffer[i], HEX);
  }
  Serial.println(". Done.");
}


// Parse data that has been placed in the receive buffer
bool R200::parseData() {
  Serial.println("Hi");
  /*
      switch(_buffer[2]){
        case CMD_GetModuleInfo:
        {
          String info;
          for (uint8_t i = 0; i < 50; i++) {
            info += (char)_buffer[6 + i];
            // Stop when then only two bytes left are the CRC and FrameEnd marker
            if (_buffer[8 + i] == R200_FrameEnd) {
                break;
            }
          }
          Serial.print(info);
        }
          break;
        case CMD_SinglePollInstruction:
          printHexByte("RSSI", _buffer[6]);
          printHexWord("PC", _buffer[7], _buffer[8]);
          printHexBytes("EPC(", &_buffer[9], 12);
          //printHexByte("CRC", _buffer[] )
          break;
      }
      */
}



// Read incoming serial data sent by the reader
// This could either be a response to a command sent, or a notification
// (e.g. when set to automatic polling mode)
// Returns true if a complete frame of data is read within the allotted timeout
bool R200::receiveData(unsigned long timeOut){
  unsigned long startTime = millis();
  uint8_t bytesReceived = 0;
  // Clear the buffer
  memset(_buffer, 0, sizeof _buffer);
  while (_serial->available() || (millis() - startTime) < timeOut) {
    if (_serial->available()) {
      uint8_t b = _serial->read();
      _buffer[bytesReceived] = b;
      bytesReceived++;
      if (b == R200_FrameEnd) {
        break;
      }
    }
  }
  if (_buffer[0] == R200_FrameHeader && _buffer[bytesReceived - 1] == R200_FrameEnd) {
      return true;
  } else {
      return false;
  }
  return false;
}



/*
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
//}

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


/*
* Send single poll command
*/
void R200::poll(){
  uint8_t commandFrame[7] = {0};
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
  uint8_t commandFrame[10] = {0};

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

uint16_t R200::arrayToUint16(uint8_t *array){
  uint16_t value = *array;
  value <<=8;
  value += *(array+1);
  return value;
}