#include <Servo.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35
Servo myservo;  // create servo object to control a servo
Servo drum;
// twelve servo objects can be created on most boards
const byte SX1509_ADDRESS = 0x3F;
const uint8_t sensorCountL0 = 2;
const uint8_t sensorCountL1 = 4;
const uint8_t xshutPinsL0[sensorCountL0] = {1,2};
const uint8_t xshutPinsL1[sensorCountL1] = {4,5,6,7};
float state_w, distance, distance_left, distance_right, tof_distance,front_tof;
int sensor1,sensor2,sensor3,sensor4,sensor5,sensor6,sensor7,tof_holder,tof_holder_2 = 0;
int weights_collected = 0;
int sensor = A7;
int third_slot  = 180;   
int second_slot  = 135;
int first_slot = 93;
int third_slot_discard = 76;
int second_slot_discard = 36;        
int first_slot_discard = 0 ;  
int state_holder = 1;
unsigned long Timer1,currTimer;  
unsigned long prevTimer = 0;
SX1509 io;
VL53L0X sensorsL0[sensorCountL0];
VL53L1X sensorsL1[sensorCountL1];
//#define LONG_RANGE
#define HIGH_SPEED
//#define HIGH_ACCURACY
// The number of sensors in your system.


void rotateDrum(int start,int dest) {
  int pos = 0;   
  if (start < dest){
  for (pos = start; pos <= dest; pos += 2) {
    myservo.write(pos);    
    delay(15);                      
  }         
}
  else if (start > dest){
  for (pos = start; pos >= dest; pos -= 2  ) {
    myservo.write(pos);   
    delay(15);                      
  }       
}
}

void discard_all_weights() {

  rotateDrum(third_slot,third_slot_discard); 
  delay(2500);
  rotateDrum(third_slot_discard,second_slot_discard); 
  delay(2500);
  rotateDrum(second_slot_discard,first_slot_discard);                             
  delay(2500); 
  rotateDrum(first_slot_discard,first_slot);                             
  delay(2500); 
}


void init_tof() {
   Serial.begin(115200);
  io.begin(SX1509_ADDRESS);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  // L0 Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCountL0; i++)
  {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
  }

  // L1 Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCountL1; i++)
  {
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }

  // L0 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCountL0; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPinsL0[i], HIGH);
    delay(10);

    sensorsL0[i].setTimeout(500);
    if (!sensorsL0[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L0 ");
      Serial.println(i);
      while (1);
    }
      #if defined LONG_RANGE
    // lower the return signal rate limit (default is 0.25 MCPS)
    sensorsL0[i].setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    sensorsL0[i].setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    sensorsL0[i].setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    #endif
    #if defined HIGH_SPEED
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensorsL0[i].setMeasurementTimingBudget(20000);
    #elif defined HIGH_ACCURACY
    // increase timing budget to 200 ms
    sensorsL0[i].setMeasurementTimingBudget(200000);
    #endif
    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);

    sensorsL0[i].startContinuous(50);
  }

  // L1 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCountL1; i++)
  {
    // Stop driving this sensor's XSHUT low. This should allow the carrier
    // board to pull it high. (We do NOT want to drive XSHUT high since it is
    // not level shifted.) Then wait a bit for the sensor to start up.
    //pinMode(xshutPins[i], INPUT);
    io.digitalWrite(xshutPinsL1[i], HIGH);
    delay(10);

    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init())
    {
      Serial.print("Failed to detect and initialize sensor L1 ");
      Serial.println(i);
      while (1);
    }
    sensorsL1[i].setROISize(5, 5);
    sensorsL1[i].setROICenter(197);
   
    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);

    sensorsL1[i].startContinuous(50);
  }
}


void setup() {
  myservo.attach(30);  // attaches the servo on pin 9 to the servo object
  myservo.write(93);  
  drum.attach(29);  // attaches the servo on pin 9 to the servo object
  drum.writeMicroseconds(1950);  
  init_tof();
 
}


void loop() {
  
  //discard_all_weights();
  
  for (uint8_t i = 0; i < sensorCountL0; i++)
  {
    if (sensorsL0[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    else{
    tof_holder = sensorsL0[i].readRangeSingleMillimeters();
    switch (i){
      case 0:
        sensor1 = tof_holder;//TTM
        break;
      case 1:
        sensor2 = tof_holder;//BBR
        break;
      // case 2:
      //   sensor3 = tof_holder;//BBL
      //   break;
    }
    }
  }

  for (uint8_t i = 0; i < sensorCountL1; i++)
  {
    if (sensorsL1[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    tof_holder_2 = sensorsL1[i].readRangeContinuousMillimeters();
   switch (i){
      case 0:
        sensor4 = tof_holder_2;//
        break;
      case 1:
        sensor5 = tof_holder_2;//
        break;
      case 2:
        sensor6 = tof_holder_2;//
        break;
      case 3:
        sensor7 = tof_holder_2;//
        break;
   }
  
  }
  int state = digitalRead(sensor);
  if ((state == LOW)) {
    int state_holder = 0;
  }
  else {
    state_holder = 1;
  }
  front_tof = sensor2;
  Serial.println(state_holder);
  if (front_tof < 90) {
    Timer1 = millis();
    state = digitalRead(sensor);
    if ((state == LOW)) {
    state_holder = 0;
  }
  
    if ((state_holder == 0)) {
      Serial.print(millis()-Timer1);
      switch (weights_collected) {
        case 0:
          delay(1000);
          Serial.println("First real weight detected");
          delay(500);
          rotateDrum(first_slot,second_slot);
          delay(1000);
          weights_collected += 1;
          state_holder = 1;
          break;
        case 1:
          delay(1000);
          Serial.println("2nd real weight detected");
          delay(500);
          rotateDrum(second_slot,third_slot);
          delay(1000);
          weights_collected += 1;
          state_holder = 1;
          break;
        case 2:
          delay(1000);
          Serial.println("3rd real weight detected...Go home");
          delay(500);
          state_holder = 1;
          delay(2500);
          delay(2500);
          discard_all_weights();
          weights_collected = 0;
          break;
      }
     
    }
    while  ((millis()-Timer1)<500){
     if ((millis()-Timer1)>450){
      Serial.print(millis()-Timer1);
      switch (weights_collected) {
      case 0:
        delay(1000);
        Serial.println("Fake weight detected1");
        delay(500);
        rotateDrum(first_slot,first_slot_discard);
        delay(1000);
        rotateDrum(first_slot_discard,first_slot);
        delay(500);
        break;
      case 1:
        delay(1000);
        Serial.println("Fake weight detected2");
        delay(500);
        rotateDrum(second_slot,second_slot_discard);
        delay(1000);
        rotateDrum(second_slot_discard,second_slot);
        delay(500);
        break;
      case 2:
        delay(1000);
        Serial.println("Fake weight detected3");
        delay(500);
        rotateDrum(third_slot,third_slot_discard);
        delay(1000);
        rotateDrum(third_slot_discard,third_slot);
        delay(500);
        break;
    }
  
     }
    }
  }
    
  
}
  

  
 
  
