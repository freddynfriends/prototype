/*********************************************************************
  Freddy n' Friends first prototype
  This is the slightly different version of the prototype we built at Techfest '17

  The data from the orientation sensor will be sent via USB as keyboard and mouse commands


*********************************************************************/

#include <Arduino.h>
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h> // the orientation sensor we use
#include <utility/imumaths.h>

#include "Keyboard.h"
#include "Mouse.h"


// create sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// button that turns data transmission on/off
int controlSwitchPin = 12;

// button used for throwing stuff in mariokart
int shootingButtonPin = 10;
// button used for drifting in mariokart
int driftingButtonPin = 9;

float mouseScaling = 0.9;

// is the sensor initialized?
boolean initialized = false;
// data transmission on/off
boolean volatile controlsActive = false;
boolean volatile shootingPressed = false;

// variables for debouncing the buttons:

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceControls = 0;// the last time the output pin was toggled
unsigned long lastDebounceShooting = 0;

unsigned long debounceDelay = 50;// the debounce time


// gyro values and offsets
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{

  Serial.begin(9600);
  Serial.println(F("Testing Sensor Movement to Keyboard and Mouse output"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);

  bno.setExtCrystalUse(true);

  Serial.println("BNO055 running");

  // Set up everything needed for the Freddy controller
  // input_pullup mode means that the pin is LOW if the button is pressed, and HIGH if not pressed
  pinMode(controlSwitchPin, INPUT_PULLUP);
  pinMode(shootingButtonPin, INPUT_PULLUP);
  pinMode(driftingButtonPin, INPUT_PULLUP);

  controlsActive = !digitalRead(controlSwitchPin);

  // Attach an interrupt to turn controls on
  attachInterrupt(controlSwitchPin, toggleControls, CHANGE);

  attachInterrupt(shootingButtonPin, shoot, FALLING);

  Keyboard.begin();
  Mouse.begin();

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void) {
  int loopCount = 0;
  getSensorData();


// only if controls are active we will send keyboard and mouse commands
  if (controlsActive) {


    int thresh = 15; // threshold for sensitivity

    if (gyroY < -thresh) {
      // going backwards
      Keyboard.release('w');
      Keyboard.press('s');
    } else if (gyroY > thresh) {
      // going forward
      Keyboard.release('s');
      Keyboard.press('w');
    } else {
      // there is no tilt front/back, we're gonna release W and S
      Keyboard.release('w');
      Keyboard.release('s');
    }

// Mouse movements:
    if (gyroZ > thresh) {
      // going right
      Mouse.move(gyroZ * mouseScaling, 0, 0);
    } else if (gyroZ < -thresh) {
      // going left
      Mouse.move(gyroZ * mouseScaling, 0, 0);
    }

// drifting
    if (!digitalRead(driftingButtonPin)) {
      Serial.println("drifting like fuck");
      Keyboard.press('v');
    }else{
      Keyboard.release('v');
    }

  }

  // print sensor data
    if (loopCount % 20 == 0) {
      Serial.print("gyro x: "); Serial.print(gyroX);
      Serial.print(" - gyro y: "); Serial.print(gyroY);
      Serial.print(" - gyro z: "); Serial.print(gyroZ);
      Serial.print(" - active: "); Serial.println(controlsActive); // Serial.print(" - acc. y: "); Serial.println(accel_y);
    }


  loopCount += 1;

  delay(20);

}



void toggleControls() {
  boolean reading = !digitalRead(controlSwitchPin);
  unsigned int interruptTime = millis();


  if ((interruptTime - lastDebounceControls) > debounceDelay) {
    lastDebounceControls = interruptTime;
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != controlsActive) {
      controlsActive = reading;
      // every time we switch on/off the controls, we want to "calibrate"
      // again
      initialized = false;


    }
  }
}

void shoot() {
  boolean reading = !digitalRead(shootingButtonPin);
  unsigned int interruptTime = millis();


  if ((interruptTime - lastDebounceShooting) > 200) {
    lastDebounceShooting = interruptTime;
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    if (reading) {
      shootingPressed = true;
      Serial.println("shooting things");
      Keyboard.write('c');
    }

  }
}

void initializeSensors() {
  Serial.println("initializing the gyros");
  offsetY = gyroY;
  offsetZ = gyroZ;
  offsetX = gyroX;

  initialized = true;
}

void getSensorData() {
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

// get the x y z orientation
  gyroY = event.orientation.y;
  gyroZ = event.orientation.z;
  gyroX = event.orientation.x;

// if not initialized, initialize sensor (setting the offset)
  if (!initialized) {
    initializeSensors();
  }
  // substract offset:
  gyroX = gyroX - offsetX;
  gyroY = gyroY - offsetY;
  gyroZ = gyroZ - offsetZ;
}



