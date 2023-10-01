/**
 * R200 RFID demonstration
 * Copyright (c) 2023 Alastair Aitchison, Playful Technology
 */ 

// INCLUDES
#include "R200.h"

// CONSTANTS
// Command to issue to the R200 to set it to multiple read mode 
const unsigned char ReadMultiCmd[10] = {0XAA,0X00,0X27,0X00,0X03,0X22,0XFF,0XFF,0X4A,0XDD};

// GLOBALS
unsigned long lastResetTime = 0;
// How many bytes have been received in the current response
unsigned int bytesReceived = 0;
// What parts of the current response frame have been received
unsigned int parState = 0;
unsigned int codeState = 0;


R200 rfid;


void setup() {

  // Intitialise Serial connection (for debugging)
  Serial.begin(115200);
  Serial.println(__FILE__ __DATE__);

  // Configure LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialise Serial2 connection to R200 module
  Serial2.begin(115200);
  rfid.begin(Serial2);

  delay(50);

  // Get info
  rfid.getModuleInfo();

 // delay(100);

  //rfid.setMultiplePollingMode();

  
  
  //Serial2.begin(115200);
  //Serial2.write(ReadMultiCmd,10);
}

void loop() {
/*
  // Periodically re-send the read command
  if(millis() - lastResetTime > 1000000L){
      digitalWrite(LED_BUILTIN, HIGH);
      Serial2.write(ReadMultiCmd,10);
      digitalWrite(LED_BUILTIN, LOW);
      lastResetTime = millis();
  }
 
  // If data is available on the serial port
  if(Serial2.available() > 0) {
    // Example response
    // AA 02 22 00 11 C7 30 00 E2 80 68 90 00 00 50 0E 88 C6 A4 A7 11 9B 29 DD 
    /*
    AA:Frame Header
    02:Instruction Code
    22:Command Parameter
    00 11:Instruction data length (0x11 = 17 bytes)
    C7：RSSI Signal Strength
    30 00: Label PC code (factory reg code)
    E2 80 68 90 00 00 50 0E 88 C6 A4 A7：EPC code
    11 9B:CRC check
    29: Verification
    DD: End of frame


    // Read the data
    unsigned int incomingByte = Serial2.read();
    // Is it a new instruction code?
    if((parState == 0) && (incomingByte == 0x02)) {
      parState = 1;
      bytesReceived = 2;
    }
    // Is it a command parameter?
    else if((parState == 1) && (codeState == 0) && (incomingByte == 0x22) ){  
        codeState = 1;
        bytesReceived = 3;
    }
    // It's part of the existing command
    else if(codeState == 1){
      bytesReceived++;
      // RSSI is the 6th byte received
      if(bytesReceived == 6) {
        Serial.print("RSSI:"); 
        Serial.print(incomingByte); 
        }
       // (2-byte) PC code is the 7th and 8th bytes
       else if((bytesReceived == 7) || (bytesReceived == 8)){
        if(bytesReceived == 7){
          Serial.print(", PC:"); 
          Serial.print(incomingByte, HEX);
        }
        else {
           Serial.print(incomingByte, HEX);
        }
       }
       // Receive the EPC (i.e. tag) code
       else if((bytesReceived >= 9) && (bytesReceived <= 20)){
        if(bytesReceived == 9){
          Serial.print(", EPC:"); 
        }
        Serial.print(incomingByte < 0x10 ? "0x0" : "0x");
        Serial.print(incomingByte, HEX);
        if(bytesReceived < 20) (Serial.print(","));
       }
       // Ignore any remaining bytes
       else if(bytesReceived >= 21) {
        Serial.println(" "); 
        bytesReceived = 0;
        parState = 0;
        codeState = 0;
        }
    }
     else {
      bytesReceived= 0;
      parState = 0;
      codeState = 0;
    }
  }
  */
}