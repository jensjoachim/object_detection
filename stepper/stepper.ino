#include <LIDARLite.h>
#include <LIDARLite_v3HP.h>
#include <LIDARLite_v4LED.h>
#include <stdint.h>
#include <Wire.h>
#include "LIDARLite_v4LED.h"
#include "SparkFun_ProDriver_TC78H670FTG_Arduino_Library.h" 

#include <avr/wdt.h>

/*

  LIDARLite Arduino Library

  Connections:
  LIDAR-Lite 5 VDC   (pin 1) to Arduino 5V
  LIDAR-Lite Ground  (pin 2) to Arduino GND
  LIDAR-Lite I2C SDA (pin 3) to Arduino SDA
  LIDAR-Lite I2C SCL (pin 4) to Arduino SCL
  LIDAR-Lite GPIOA   (pin 5) to Arduino Digital 13
  LIDAR-Lite GPIOB   (pin 6) to Arduino Digital 12
   
  ProDriver  TC78H670FTG Arduino Library

  Connections:
  ARDUINO --> PRODRIVER
  D8 --> STBY
  D7 --> EN
  D6 --> MODE0
  D5 --> MODE1
  D4 --> MODE2
  D3 --> MODE3
  D2 --> ERR

*/

const int VERSION_SCRIPT = 1;

uint8_t DBG_LEVEL = 3;

// Define LIDAR
//#define EN_LIDAR
#ifdef EN_LIDAR
LIDARLite_v4LED myLidarLite;
#define FAST_I2C
#define MonitorPin    12
#define TriggerPin    13
#endif

// Stepper Configuration
const unsigned int STEP_SELECT = 7;
const unsigned int STEP_MULT = 1<<(STEP_SELECT-1);
const unsigned int STEPS_PHYS = 400;
const unsigned int STEPS_LOGIC = STEPS_PHYS * STEP_MULT;

//uint16_t step_index = 0;
uint16_t step_index = STEPS_LOGIC/2;

// Define Stepper ProDriver
PRODRIVER myProDriver; 

// Programs
const uint8_t PROGRAM_NONE = 0;
const uint8_t PROGRAM_SINGLE_SCAN = 1;
const uint8_t PROGRAM_ENABLE_STEPPER = 2;
const uint8_t PROGRAM_DISABLE_STEPPER = 3;
const uint8_t PROGRAM_PRINT_DATA = 4;
const uint8_t PROGRAM_OTHER = 5;

void setup() {

  // Initialize Arduino serial port (for display of ASCII output to PC)
  Serial.begin(115200);

  // Initialize Arduino I2C (for communication to LidarLite)
  Wire.begin();
  #ifdef FAST_I2C
    #if ARDUINO >= 157
      Wire.setClock(400000UL); // Set I2C frequency to 400kHz (for Arduino Due)
    #else
      TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
    #endif
  #endif

  // Configure LIDAR
  #ifdef EN_LIDAR
  digitalWrite(SCL, LOW);
  digitalWrite(SDA, LOW);
  pinMode(MonitorPin, INPUT);
  pinMode(TriggerPin, OUTPUT);
  digitalWrite(TriggerPin, LOW);
  myLidarLite.configure(0); // Mode
  #endif

  // ----------------------------------------------------------------------
  // ** One method of increasing the measurement speed of the
  //    LIDAR-Lite v4 LED is to adjust the number of acquisitions taken
  //    per measurement from the default of 20. For max speed we
  //    set this value to 0x00 in this example.
  // ** Note that when reducing the number of acquisitions taken per
  //    measurement from the default, repeatability of measurements is
  //    reduced.
  // ----------------------------------------------------------------------
  //uint8_t dataByte = 0x00;
  //uint8_t dataByte = 0x15;
  //uint8_t dataByte = 0x3F;
  #ifdef EN_LIDAR
  uint8_t dataByte = 0xFF;
  myLidarLite.write(0xEB, &dataByte, 1, 0x62); // Turn off high accuracy mode
  #endif

  // Print Version
  Serial.print("Camera Stepper Version v");
  Serial.println(VERSION_SCRIPT);
  Serial.println("--");

  // Configure Stepper driver
  if ( STEP_SELECT == 1) {
    myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_FULL;
  } else if ( STEP_SELECT == 2) {
    myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_2; // 1/2 step
  } else if ( STEP_SELECT == 3) {
    myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_4; // 1/4 step
  } else if ( STEP_SELECT == 4) {
    myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_8; // 1/8 step
  } else if ( STEP_SELECT == 5) {
    myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_16; // 1/16 step
  } else if ( STEP_SELECT == 6) {
    myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_32; // 1/32 step
  } else if ( STEP_SELECT == 7) {
    myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_64; // 1/64 step
  } else if ( STEP_SELECT == 8) {
    myProDriver.settings.stepResolutionMode = PRODRIVER_STEP_RESOLUTION_FIXED_1_128; // 1/128 step
  } else {
    Serial.print("ERROR: ");
    Serial.print("Wrong Step Select: ");
    Serial.println(STEP_SELECT);
  }
  Serial.println("Stepper Motor Configuration");
  Serial.print("STEP_SELECT: ");
  Serial.println(STEP_SELECT);
  Serial.print("STEP_MULT:   ");
  Serial.println(STEP_MULT);
  Serial.print("STEPS_PHYS:  ");
  Serial.println(STEPS_PHYS);
  Serial.print("STEPS_LOGIC: ");
  Serial.println(STEPS_LOGIC);
  Serial.println("--");
  
  // Start stepper driver
  myProDriver.begin(); // adjust custom settings before calling this
  myProDriver.enable();

  // Indicate to PC that Arduino is ready
  Serial.println("ready ");
}

String str;
int return_message_enable = 1;
int val_int;
uint16_t distance_val;
uint16_t * distance;
int measurement_ongoing = 0;

void loop() {
  
  // Read Serial
  if (Serial.available()) {
    // Read string
    str = Serial.readStringUntil('\n');
    // Resturn String/Message
    if (return_message_enable == 1) {
      Serial.print("return: ");
      Serial.println(str);
    }
    // Decode message
    if (str == "reset") {
      reboot();
    } else if (str == "return_enable") {
      return_message_enable = 1;
    } else if (str == "return_disable") {
      return_message_enable = 0;
    } else if (str == "re-center") {
      set_step_index(STEPS_LOGIC/2);
      Serial.print("Step: ");
      Serial.println(get_step_index());
    } else if (str.indexOf("set_step:") != -1) {
      str = str.substring(10);
      val_int = str.toInt();
      set_step_index(val_int);
      Serial.print("Step: ");
      Serial.println(get_step_index());
    } else { // Rotate Head
      val_int = str.toInt();
      if (val_int < 0) {
        val_int = abs(val_int);
        step_simple_step(val_int, 0, 1);
      } else {
        step_simple_step(val_int, 1, 1);
      }
      Serial.print("Step: ");
      Serial.println(get_step_index());
    }
  } 

  #ifdef EN_LIDAR
  // Handle distance measument
  if (measurement_ongoing == 0) {
    // Trigger measurement
    myLidarLite.takeRangeGpio(TriggerPin, MonitorPin);
    // Wait for pin to go high
    while (digitalRead(MonitorPin) == 0) {}
    // Indicate ongoing measurement
    measurement_ongoing = 1;
  } else {
    // Check if measurement is done
    if (digitalRead(MonitorPin) == 0) {
      // Read distance
      getDist(&distance_val);
      Serial.print("Dist: ");
      Serial.println(distance_val);
      // Indicate stopped measurement
      measurement_ongoing = 0;
    }
  }
  #endif
  
}

#ifdef EN_LIDAR
void getDist(uint16_t * distance)
{
  *distance = myLidarLite.readDistance();
}
#endif

void reboot() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}

void update_step_index ( bool direction ) {
  if (direction == 0) {
    if (step_index == 0) {
      step_index = STEPS_LOGIC-1;
    } else {
      step_index--;
    }
  } else {
    if (step_index == STEPS_LOGIC-1) {
      step_index = 0;
    } else {
      step_index++;
    }
  }
}

uint16_t get_step_index () {
  return step_index;
}

void set_step_index (uint16_t step_index_in) {
  step_index = step_index_in;
}

void step_set_dir ( bool direction ) 
{
  // set CW-CWW pin (aka mode3Pin) to the desired direction
  // CW-CCW pin controls the rotation direction of the motor. 
  // When set to H, the current of OUT_A is output first, with a phase difference of 90°. 
  // When set to L, the current of OUT_B is output first with a phase difference of 90°
  if(direction == true)
  {
    pinMode(myProDriver.settings.mode3Pin, INPUT); // let on-board external pullup to 3.3V pull this pin HIGH
  }
  else{
    pinMode(myProDriver.settings.mode3Pin, OUTPUT);
    digitalWrite(myProDriver.settings.mode3Pin, LOW);
  }
}

void step_simple_step( uint16_t steps, bool direction, uint8_t clockDelay )
{
  // Set direction 
  step_set_dir(direction);
  
  // step the motor the desired amount of steps
  // each up-edge of the CLK signal (aka mode2Pin) 
  // will shift the motor's electrical angle per step.
  for(uint16_t i = 0 ; i < steps ; i++)
  {
    pinMode(myProDriver.settings.mode2Pin, OUTPUT);
    digitalWrite(myProDriver.settings.mode2Pin, LOW);
    delayMicroseconds(1); // even out the clock signal, error check takes about 2uSec
    //delay(clockDelay);
    pinMode(myProDriver.settings.mode2Pin, INPUT); // let on-board external pullup to 3.3V pull this pin HIGH
    delay(clockDelay);
    update_step_index(direction);
    // Break if any new value incoming
    if (Serial.available()) {
      break;
    }
  }
}
