# arduino-rfid-R200
Arduino/ESP32 code for R200 long-range UHF RFID reader

The R200 is a UHF RFID module based on the EPC Gen-2 (ISO18000-6C) protocol. What does that mean in practice?
Well, it can read up to 60 tags per second, at a range of 20m...

The board I bought uses a serial interface, so it's pretty easy to control from an ESP32/Arduino/RaspPi. But, I found it a little hard to find much detailed information on the commands or format of the response; which is what I will now document in this respository.

This is the module I bought, from https://www.aliexpress.com/item/4000281733851.html
<img src="docs/R200_module.jpg" />
Note that the board uses an external ceramic antenna, and the read range will vary depending on the physical size (and therefore dbi) of the antenna. I chose the 60x70x7mm 4dbi antenna, which claims a read range of 0-3m.
