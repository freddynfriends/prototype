#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);

#include "Keyboard.h"
#include "Mouse.h"

int control_toggle_button_pin = 12;

int player_a_button1_pin = 10;
int player_a_button2_pin = 9;

int loop_count = 0;

boolean initialized = false;
boolean controls_active = false;

boolean player_a_button1_pressed = false;
boolean player_a_button2_pressed = false;

float mouse_scaling = 0.9;

float gyro_x1 = 0;
float gyro_y1 = 0;
float gyro_z1 = 0;
float offset_x1 = 0;
float offset_y1 = 0;
float offset_z1 = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println("starting this shit");

  /* Initialise the sensor */
  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
    
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 running");

  pinMode(control_toggle_button_pin, INPUT_PULLUP);
  controls_active = !digitalRead(control_toggle_button_pin);

  // Attach an interrupt to turn controls on
  attachInterrupt(control_toggle_button_pin, toggle_controls, CHANGE);

  pinMode(player_a_button1_pin, INPUT_PULLUP);
  pinMode(player_a_button2_pin, INPUT_PULLUP);
  attachInterrupt(player_a_button1_pin, queue_player_a_button1, RISING);
  attachInterrupt(player_a_button2_pin, toggle_player_a_button2, FALLING);

  Keyboard.begin();
  Mouse.begin();
}

void loop()
{
  if(loop_count % 5 == 0) {
    /* Get a new sensor event */ 
    sensors_event_t event; 
    bno.getEvent(&event);
    
    // gyro_x1 = event.orientation.x;
    gyro_y1 = event.orientation.y;
    gyro_z1 = event.orientation.z;
    gyro_x1 = event.orientation.x;
    
    if(!initialized) {
      Serial.println("initializing the gyros");
      
      // offset_x1 = gyro_x1;
      offset_y1 = gyro_y1;
      offset_z1 = gyro_z1;
      offset_x1 = gyro_x1;
      
      initialized = true;
    }

    gyro_x1 = gyro_x1 - offset_x1;
    gyro_y1 = gyro_y1 - offset_y1;
    gyro_z1 = gyro_z1 - offset_z1;
    
  }

  if(controls_active) {

    // Serial.print("Z position:  "); Serial.println(gyro_z);
    // check whether a direction needs to be pressed. if so, press.
    
    if (gyro_z1 > 15) {
      // this is a tilt left, we're gonna press S and release W if pressed
      Mouse.move(-gyro_z1*mouse_scaling, 0, 0);
    } else if (gyro_z1 < -15) {
      // this is a tilt right, we're gonna move the mouse to the right
      Mouse.move(-gyro_z1*mouse_scaling, 0, 0);
    }
  
    if((loop_count) % 10 == 0) {
    // keyboard stuff
    
      // Serial.print("Y position: "); Serial.println(gyro_y);
      // check whether a direction needs to be pressed. if so, press.
      if (gyro_y1 > 15) {
        // this is a tilt to the back, we're gonna press S and release W if pressed
        Keyboard.release('w');
        Keyboard.press('s');
      } else if (gyro_y1 < -15) {
        // this is a tilt to the front, we're gonna press W and release S if pressed
        Keyboard.release('s');
        Keyboard.press('w');
      } else {
        // there is no tilt front/back, we're gonna release A and D
        Keyboard.release('w');
        Keyboard.release('s');
      }
    }
    
//    if((loop_count+2) % 4) {
      if(digitalRead(player_a_button1_pin)==LOW) {
        Serial.println("shooting things");
        unsigned long time_buff = millis();
        while(millis() - time_buff < 100){
        Keyboard.write('c');
        }
        player_a_button1_pressed = false;
      }
      if(digitalRead(player_a_button2_pin)==LOW) {
        Serial.println("drifting like fuck");
        Keyboard.press('v');
    } else {
      Keyboard.release('v');
    }
  //}
}else{
  // TODO gibts soas wie release all?
  Keyboard.release('w');
  Keyboard.release('s');
  Keyboard.release('c');
  Keyboard.release('v');
  
}

  
  
  if(loop_count % 20 == 0) {
    //Serial.print("gyro x1: "); Serial.print(gyro_x1);
    Serial.print("gyro y1: "); Serial.print(gyro_y1); 
    Serial.print(" - gyro z1: "); Serial.print(gyro_z1); 
    Serial.print("- gyro x1: "); Serial.print(gyro_x1);
    Serial.print(" - active: "); Serial.println(controls_active); // Serial.print(" - acc. y: "); Serial.println(accel_y);
  }
  
  
  loop_count += 1;

  delay(20);
}

void toggle_controls() {
  controls_active = !digitalRead(control_toggle_button_pin);
  initialized = false;
}

void queue_player_a_button1() {
  player_a_button1_pressed = true;
}
void toggle_player_a_button2() {
  player_a_button2_pressed = !player_a_button2_pressed;
}
