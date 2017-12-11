# FreddynFriends 1st prototype

These are our first 2 versions of the freddynfriends game controller

## Hardware used:
* Adafruit Feather M0 Bluefruit LE 
https://learn.adafruit.com/adafruit-feather-m0-bluefruit-le?view=all
* Adafruit BNO055 Absolute Orientation Sensor
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview
* 2 basic buttons
* 1 one/off switch

## Version 1: USB game controller (techfest version)
This version uses the data from the orientation sensor and translates it into keyboard and mouse commands sent via USB.


## Version 2: BLE game controller (based on techfest version)
This version uses the data from the orientation sensor and translates it only to keyboard commands sent via BLE. (Mouse commands where too laggy)
