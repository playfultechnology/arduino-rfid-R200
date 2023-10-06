/**
 * R200 RFID demonstration
 * Copyright (c) 2023 Alastair Aitchison, Playful Technology
 */ 

// INCLUDES

#include "R200.h"

// GLOBALS
unsigned long lastResetTime = 0;
/*
#include <HardwareSerial.h>
HardwareSerial RFIDSerial(2);
HardwareSerial *mySerial;
*/
R200 rfid;

void setup() {

  // Intitialise Serial connection (for debugging)
  Serial.begin(115200);
  Serial.println(__FILE__ __DATE__);

  // Initialise Serial2 connection to R200 module
  //R200Serial.begin(115200, SERIAL_8N1, 16, 17);
  //Serial2.begin(115200);
  //rfid.begin(&R200Serial, 115200, 16, 17);
  //rfid.begin(RFIDSerial, 115200, 16, 17);

  rfid.begin(&Serial2, 115200, 16, 17);


  //mySerial = &Serial2;
  //rfid.begin(mySerial, 115200, 16, 17);

  delay(50);

  // Get info
  rfid.getModuleInfo();

  delay(1000);
}

void loop() {
  rfid.loop();

  // Periodically re-send the read command
  if(millis() - lastResetTime > 4000){
    //  digitalWrite(LED_BUILTIN, HIGH);
      rfid.poll();
  //rfid.getModuleInfo();
    //  digitalWrite(LED_BUILTIN, LOW);
      lastResetTime = millis();
  }
  
  delay(1000);
}