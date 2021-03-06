/*********************************************************************
  Freddy n' Friends first BLE prototype

  The data from the orientation sensor will be sent via BLE as keyboard commands


  Using code from the adafruit BluefruitLE nRF51 atcommand example


*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h> // the orientation sensor we use
#include <utility/imumaths.h>


/*=========================================================================
    APPLICATION SETTINGS

      FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
     
                                Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                                running this at least once is a good idea.
     
                                When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                                Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
         
                                Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
/*=========================================================================*/


// Create the bluefruit object

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


// create sensor object
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// button that turns data transmission on/off
int controlSwitchPin = 12;

// button used for throwing stuff in mariokart
int shootingButtonPin = 10;
// button used for drifting in mariokart
int driftingButtonPin = 9;



// is the sensor initialized?
boolean initialized = false;
// data transmission on/off
boolean volatile controlsActive = false;

boolean volatile shootingPressed = false;
boolean volatile driftingPressed = false;

// variables for debouncing the buttons:

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceControls = 0;// the last time the output pin was toggled
unsigned long lastDebounceShooting = 0;
unsigned long lastDebounceDrifting = 0;

unsigned long debounceDelay = 50;// the debounce time

// String passed as ATcommand via BLE
String bleString = "";
String lastbleString = "";

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
  Serial.println(F("Testing Sensor Movement to Keyboard output via bluetooth"));
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
  attachInterrupt(driftingButtonPin, toggleDriftingButton, CHANGE);

  /* Initialise the bluetooth module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'freddy': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Freddy" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard): "));
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    //if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) { //enable bluetooth HID --> needed for mouse movement
    //  error(F("Could not enable HID"));
    //}
    if (! ble.sendCommandCheckOK(F( "AT+BleKeyboardEn=On"  ))) { //enable bluetooth keyboard
      error(F("Could not enable Keyboard"));
    }
  }
  ble.println("AT+BLEPOWERLEVEL=4"); //set transmit power to maximum, this will drain the battery faster

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void) {
  int loopCount = 0;
  getSensorData();


  if (controlsActive) {
    // Mouse movements, didn't work so well, mouse movements were lagging..
    /*
      if (gyro_z1 > 15) {
      // this is a tilt left
      ble.print("AT+BLEHIDMOUSEMOVE=");
      ble.println(-gyro_z1*mouse_scaling,0);
      } else if (gyro_z1 < -15) {
      ble.print("AT+BLEHIDMOUSEMOVE=");
      ble.println(-gyro_z1*mouse_scaling,0);
      }*/

    // call function that creates the string to be sent via bLE
    createBLEString();
    if (! lastbleString.equals(bleString)) {
      ble.println(bleString);
    }


  }
  // print sensor data
  if (loopCount % 30 == 0) {
    Serial.print("gyro x: "); Serial.print(gyroX);
    Serial.print(" - gyro y: "); Serial.print(gyroY);
    Serial.print(" - gyro z: "); Serial.print(gyroZ);
    Serial.print(" - active: "); Serial.println(controlsActive); // Serial.print(" - acc. y: "); Serial.println(accel_y);
  }


  loopCount += 1;
  shootingPressed = false;
  driftingPressed = false;
  delay(100);

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


  if ((interruptTime - lastDebounceShooting) > debounceDelay) {
    lastDebounceShooting = interruptTime;
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:
    if (reading) {
      shootingPressed = true;

    }

  }
}

void toggleDriftingButton() {
  boolean reading = !digitalRead(controlSwitchPin);
  unsigned int interruptTime = millis();


  if ((interruptTime - lastDebounceDrifting) > debounceDelay) {
    lastDebounceDrifting = interruptTime;
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != driftingPressed) {
      driftingPressed = reading;
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

// if not initialized, initialize sensor (will be called when controls are switched on/off
  if (!initialized) {
    initializeSensors();
  }
  // substract offset:
  gyroX = gyroX - offsetX;
  gyroY = gyroY - offsetY;
  gyroZ = gyroZ - offsetZ;
}

void createBLEString() {
  lastbleString = bleString;
  
  bleString = "AT+BLEKEYBOARDCODE=00-00"; // string always has to start with 00-00, first byte: modifier (00), second byte always 00
  // using hex-codes for keyboard commands
  // A = 0x04
  // C = 0x06
  // D = 0x07
  // S = 0x16
  // V = 0x19
  // W = 0x1A


  // to release all keys: 00-00
  // up to 6 keys can be pressed simultaneously: eg: 00-00-04-05-07-16-1A-08 --> acdswe

  // gyro values:
  // y for forward and backwards (in this prototype positive values = backward negative = forward)
  // z for left and right (in this prototype positive = right, negative = left)

  int thresh = 30; // threshold for sensitivity

  // this depends on how you put the sensor on the breadboard. In our case tilting the board down to the front means negative y values
  // and tilting down to the back means positive y values
  if (gyroY > thresh) {
    // going backwards
    bleString = bleString + "-16"; // add s to the string
  } else if (gyroY < -thresh) {
    // going forward
    bleString = bleString + "-1A"; // add w to the string
  }

  // this depends on how you put the sensor on the breadboard. In our case tilting the sensor down to the right
  // means z values between -360 and -270, and tilting it down to the left = z values between 0 and -90
  if (gyroZ < -thresh && gyroZ > -90) {
    // going left
    bleString = bleString + "-07"; // add d to the string
  } else if (gyroZ > -360 - thresh && gyroZ < -270 ) {
    // going right
    bleString = bleString + "-04"; // add a to the string
  }

  if (shootingPressed) {
    Serial.println("shooting things");
    bleString = bleString + "-06";
    //shooting_button_pressed = false;
  }
  if (driftingPressed) {
    Serial.println("drifting like fuck");
    bleString = bleString + "-19";
  }
  

}

