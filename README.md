# Freddyn'Friends
Freddyn'Friends is a detachable sensor device that can turn everyday objects into game controllers.
It was developed during [Techfest Munich](http://x.unternehmertum.de/events/techfest-munich-17/) '17 and is now being further developed.
To get an idea of what we're talking about, check out our [website](http://freddynfriends.com/) and follow us on [instagram](https://www.instagram.com/freddynfriends/)


## 1st prototype

These are our first 2 versions of the freddynfriends game controller

### Hardware used:
* Adafruit Feather M0 Bluefruit LE 
https://learn.adafruit.com/adafruit-feather-m0-bluefruit-le?view=all
* Adafruit BNO055 Absolute Orientation Sensor
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview
* 2 basic buttons
* 1 on/off switch
* battery for bluetooth version - e.g. this https://www.adafruit.com/product/1578
* USB cable


![First prototype](setup/image1.jpg?raw=true "image 1")
![First prototype](setup/image2.jpg?raw=true "image 2")
![First prototype](setup/image3.jpg?raw=true "image 3")

### Version 1: USB game controller (techfest version)
This version uses the data from the orientation sensor and translates it into keyboard and mouse commands sent via USB.


### Version 2: BLE game controller (based on techfest version)
This version uses the data from the orientation sensor and translates it only to keyboard commands sent via BLE. (Mouse commands where too laggy)

The code considers the part of the usb connection to be the "Front" of the device. If you use a different setup you might have to change some lines in the code.

