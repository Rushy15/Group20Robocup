/*
This example shows how to set up and read multiple VL53L1X sensors connected to
the same I2C bus. Each sensor needs to have its XSHUT pin connected to a
different Arduino pin, and you should change sensorCount and the xshutPins array
below to match your setup.

For more information, see ST's application note AN4846 ("Using multiple VL53L0X
in a single design"). The principles described there apply to the VL53L1X as
well.
*/

#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

const byte SX1509_ADDRESS = 0x3F;
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35
//#define LONG_RANGE
//#define HIGH_SPEED
//#define HIGH_ACCURACY
// The number of sensors in your system.
const uint8_t sensorCountL0 = 3;
const uint8_t sensorCountL1 = 4;
int sensor1,sensor2,sensor3,sensor4,sensor5,sensor6,sensor7,tof_holder,tof_holder_2 = 0;

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL0[sensorCountL0] = {1,2};
const uint8_t xshutPinsL1[sensorCountL1] = {4,5,6,7};

SX1509 io; // Create an SX1509 object to be used throughout
VL53L0X sensorsL0[sensorCountL0];
VL53L1X sensorsL1[sensorCountL1];

void setup()
{
  
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
     
   
    // reduce timing budget to 20 ms (default is about 33 ms)
    sensorsL0[i].setMeasurementTimingBudget(20000);
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
    sensorsL1[i].setROISize(4, 4);
    sensorsL1[i].setROICenter(195);
    sensorsL1[i].setDistanceMode(VL53L1X::Short);
    sensorsL1[i].setMeasurementTimingBudget(20000);
   
    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);

    sensorsL1[i].startContinuous(50);
  }
}

void loop()
{
  int start = millis();
  for (uint8_t i = 0; i < sensorCountL0; i++)
  {
    if (sensorsL0[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    else{
    tof_holder = sensorsL0[i].readRangeContinuousMillimeters();
    switch (i){
      case 0:
        sensor1 = tof_holder;//TTM
        break;
      case 1:
        sensor2 = tof_holder;//BBR
        break;
      case 2:
        sensor3 = tof_holder;//BBL
        break;
      
     


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
  
  // if ((sensor6 - sensor3)>50){
  //   Serial.print("weight detected on left)");
  // }
  // else{
  //   Serial.print("No weight detected      ");
  // }
  
   Serial.print(sensor1);
    Serial.print(" ");
    Serial.print(sensor2);
    Serial.print(" ");
    Serial.print(sensor3);
    Serial.print(" ");
    Serial.print(sensor4);
    Serial.print(" ");
    Serial.print(sensor5);
    Serial.print(" ");
    Serial.print(sensor6);
    Serial.print(" ");
    Serial.print(sensor7);
    Serial.print(" ");
    Serial.print(" Time taken millis: ");
  Serial.println(millis()-start);

  //   if ((sensor7 - sensor2)>50){
  //   Serial.println("     weight detected on right)");
  // }
  // else{
  //   Serial.println("     No weight detected      ");
  // }
  
 
}
