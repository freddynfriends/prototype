/*********************************************************************
 Freddy n' Friends first BLE prototype

 The data from the orientation sensor will be sent via BLE as keyboard commands

Using code from the adafruit BluefruitLE nRF51 atcommand example

TODO:
    not releasing keys all the time? 
    drifting and shooting button stuff..
    
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
int control_toggle_button_pin = 12;

// button used for throwing stuff in mariokart
int shooting_button_pin = 10;
// button used for drifting in mariokart
int drifting_button_pin = 9;

int loop_count = 0;

// is the sensor initialized?
boolean initialized = false; 
// data transmission on/off
boolean controls_active = false;

boolean shooting_button_pressed = false;
boolean drifting_button_pressed = false;

// AT command string that will be sent via BLE
String bleString = "";


// gyro values and offsets
float gyro_x1 = 0;
float gyro_y1 = 0;
float gyro_z1 = 0;
float offset_x1 = 0;
float offset_y1 = 0;
float offset_z1 = 0;

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
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
    
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 running");

  // Set up everything needed for the Freddy controller
  
  pinMode(control_toggle_button_pin, INPUT_PULLUP);
  controls_active = !digitalRead(control_toggle_button_pin);

  // Attach an interrupt to turn controls on
  attachInterrupt(control_toggle_button_pin, toggle_controls, CHANGE);

  pinMode(shooting_button_pin, INPUT_PULLUP);
  pinMode(drifting_button_pin, INPUT_PULLUP);
  attachInterrupt(shooting_button_pin, queue_shooting_button, RISING);
  attachInterrupt(drifting_button_pin, toggle_drifting_button, FALLING);

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
    if ( ! ble.factoryReset() ){
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
void loop(void){
  
  getSensorData();
  

  if(controls_active) {   
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
     ble.println(bleString);

  }
  // print sensor data
  if(loop_count % 8 == 0) {
    Serial.print("gyro x1: "); Serial.print(gyro_x1);
    Serial.print(" - gyro y1: "); Serial.print(gyro_y1); 
    Serial.print(" - gyro z1: "); Serial.print(gyro_z1); 
    Serial.print(" - active: "); Serial.println(controls_active); // Serial.print(" - acc. y: "); Serial.println(accel_y);
  }
  
  
  loop_count += 1;

  delay(80);

}



void toggle_controls() {
  controls_active = !digitalRead(control_toggle_button_pin);
  initialized = false;
}

void queue_shooting_button() {
  shooting_button_pressed = true;
}
void toggle_drifting_button() {
  drifting_button_pressed = !drifting_button_pressed;
}

void initializeSensors(){
  Serial.println("initializing the gyros");
      
  // offset_x1 = gyro_x1;
  offset_y1 = gyro_y1;
  offset_z1 = gyro_z1;
  offset_x1 = gyro_x1;
  
  initialized = true;
}
void getSensorData(){
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  // gyro_x1 = event.orientation.x;
  gyro_y1 = event.orientation.y;
  gyro_z1 = event.orientation.z;
  gyro_x1 = event.orientation.x;

   if(!initialized){
      initializeSensors();
   }
  // substract offset:
  gyro_x1 = gyro_x1 - offset_x1;
  gyro_y1 = gyro_y1 - offset_y1;
  gyro_z1 = gyro_z1 - offset_z1;
}

void createBLEString(){

  bleString = "AT+BLEKEYBOARDCODE=00-00"; // string always has to start with 00-00, first byte: modifier (00), second byte always 00
  // using hex-codes for keyboard commands
  // A = 0x04
  // C = 0x05
  // D = 0x07
  // S = 0x16
  // V = 0x19
  // W = 0x1A
  
  
  // to release all keys: 00-00
  // up to 6 keys can be pressed simultaneously: eg: 00-00-04-05-07-16-1A-08 --> acdswe

  // gyro values:
  // y for forward and backwards (in this prototype positive values = backwards, negative = forwards)
  // z for left and right (in this prototype positive = right, negative = left)

  int thresh = 15; // threshold for sensitivity

  if (gyro_y1 > thresh){
    // going backwards
    bleString = bleString + "-16"; // add s to the string
  }else if(gyro_y1 < -thresh){
    // going forward
    bleString = bleString + "-1A"; // add w to the string
  }

  if (gyro_z1 > thresh){
    // going right
    bleString = bleString + "-07"; // add d to the string
  }else if (gyro_z1 < -thresh){
    bleString = bleString + "-04"; // add a to the string
  }

  // todo: this is super weird shit... make this better
  
  if(digitalRead(shooting_button_pin)==LOW) {
    Serial.println("shooting things");
    bleString = bleString + "-06";
    shooting_button_pressed = false;
  }
  if(digitalRead(drifting_button_pin)==LOW) {
    Serial.println("drifting like fuck");
    bleString = bleString + "-19";
   }
   
}

