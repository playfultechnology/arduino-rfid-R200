# arduino-rfid-R200
Arduino/ESP32 code for R200 long-range UHF RFID reader

The R200 is a UHF RFID module based on the EPC Gen-2 (ISO18000-6C) protocol. What does that mean in practice?
Well, it can read up to 60 tags per second, at a range of 20m...

The board I bought uses a serial interface, so it's pretty easy to control from an ESP32/Arduino/RaspPi. But, I found it a little hard to find much detailed information on the commands or format of the response; which is what I will now document in this respository.


