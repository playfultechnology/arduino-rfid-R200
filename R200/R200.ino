/**
 * R200 RFID demonstration
 * Copyright (c) 2023 Alastair Aitchison, Playful Technology
 */ 

// INCLUDES
#include <HardwareSerial.h>
#include "R200.h"

// GLOBALS
unsigned long lastResetTime = 0;

HardwareSerial R200Serial(2);
R200 rfid;

void setup() {

  // Intitialise Serial connection (for debugging)
  Serial.begin(115200);
  Serial.println(__FILE__ __DATE__);

  // Initialise Serial2 connection to R200 module
  //R200Serial.begin(115200, SERIAL_8N1, 16, 17);
  //Serial2.begin(115200);
  rfid.begin(&Serial2, 115200, 16, 17);

  delay(50);

  // Get info
  rfid.getModuleInfo();
}

void loop() {
  rfid.loop();

  // Periodically re-send the read command
  if(millis() - lastResetTime > 1000L){
      digitalWrite(LED_BUILTIN, HIGH);
     // rfid.poll();
      digitalWrite(LED_BUILTIN, LOW);
      lastResetTime = millis();
  }
}