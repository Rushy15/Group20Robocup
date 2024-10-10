#include "sensors.h"

#define IRQ_PIN 2
#define XSHUT_PIN 3
#define VL53L0X_ADDRESS_START 0x30
#define VL53L1X_ADDRESS_START 0x35
#define SAMPLE_SIZE 5
//#define LONG_RANGE
#define HIGH_SPEED
//#define HIGH_ACCURACY
Sensors *sensor = nullptr;

movingAvg lUS_Avg(SAMPLE_SIZE), rUS_Avg(SAMPLE_SIZE); // For Ultrasound 
movingAvg mTOF_Avg(SAMPLE_SIZE), entry_Avg(SAMPLE_SIZE), barrel_Avg(SAMPLE_SIZE),entry2_Avg(SAMPLE_SIZE); // For VL53L0X
movingAvg trTOF_Avg(SAMPLE_SIZE), tlTOF_Avg(SAMPLE_SIZE), brTOF_Avg(SAMPLE_SIZE), blTOF_Avg(SAMPLE_SIZE); // For VL53L1X

void Sensors::srTOF_Setup()
{
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
      Serial.print("NOT INITED");
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

  
    mTOF_Avg.begin();
    entry_Avg.begin();
    entry2_Avg.begin();
    barrel_Avg.begin();
  }
}

void Sensors::lrTOF_Setup()
{
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
      //Serial.println(i);
      while (1);
    }
    if (i == 0) {
    sensorsL1[i].setROISize(4, 4);
    sensorsL1[i].setROICenter(91);//No 116 consider:90, 82
    }
    if (i == 1) {
    sensorsL1[i].setROISize(4, 4);
    sensorsL1[i].setROICenter(241);
    }
    if ((i == 3)||(i == 4)) {
    sensorsL1[i].setROISize(4, 4);
    sensorsL1[i].setROICenter(220);
    }
    if ((i == 2)||(i == 5)) {
    sensorsL1[i].setROISize(4, 4);
    sensorsL1[i].setROICenter(164);
    }

    sensorsL1[i].setDistanceMode(VL53L1X::Short);
    sensorsL1[i].setMeasurementTimingBudget(20000);
    
   
    // Each sensor must have its address changed to a unique value other than
    // the default of 0x29 (except for the last one, which could be left at
    // the default). To make it simple, we'll just count up from 0x2A.
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);

    sensorsL1[i].startContinuous(50);

    trTOF_Avg.begin(); 
    tlTOF_Avg.begin();
    brTOF_Avg.begin(); 
    blTOF_Avg.begin();
  }
}

void Sensors::usSetup()
{
  // US 
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);

  lUS_Avg.begin();
  rUS_Avg.begin();
}

void Sensors::us_Values()
{
    lUS = ping(trigPinl, echoPinl); // Left US Sensor Reading
    rUS = ping(trigPinr, echoPinr); // Right US Sensor Reading
}

void Sensors::srTOF_Values()
{
  for (uint8_t i = 0; i < sensorCountL0; i++)
  {
    if (sensorsL0[i].timeoutOccurred()) {
       Serial.print(" TIMEOUT"); 
       sensor -> sensor_setup();
       }
    else {
      switch (i){
        case 0:
          srTOF_holder1 = sensorsL0[i].readRangeSingleMillimeters();
          //srTOF_holder1 = mTOF_Avg.reading(sensorsL0[i].readRangeSingleMillimeters());
          mTOF = &srTOF_holder1;// Middle tof reading
          break;
        case 1:
          srTOF_holder2 = sensorsL0[i].readRangeSingleMillimeters();
          //srTOF_holder2 = entry_Avg.reading(sensorsL0[i].readRangeSingleMillimeters());
          barrel = &srTOF_holder2;// Entry channel tof reading
          break;
      }
    }
  }
}

void Sensors::lrTOF_Values()
{
  for (uint8_t i = 0; i < sensorCountL1; i++)
  {
    if (sensorsL1[i].timeoutOccurred()) { 
      Serial.print(" TIMEOUT"); 
      sensor -> sensor_setup();
      }
    switch (i){
      case 0:
        lrTOF_holder1 = ((sensorsL1[i].readRangeContinuousMillimeters()));
        // lrTOF_holder1 =  entry2_Avg.reading(sensorsL1[i].readRangeContinuousMillimeters());
        entry2 = &lrTOF_holder1; // Entry tof reading
        break;
      case 1:
        lrTOF_holder2 = ((sensorsL1[i].readRangeContinuousMillimeters()));
        // lrTOF_holder2 =  entry_Avg.reading(sensorsL1[i].readRangeContinuousMillimeters());
        entry = &lrTOF_holder2; // Entry tof reading
        break;
      case 2:
        lrTOF_holder3 = ((sensorsL1[i].readRangeContinuousMillimeters()));
        //lrTOF_holder1 = blTOF_Avg.reading(sensorsL1[i].readRangeSingleMillimeters());
        blTOF = &lrTOF_holder3; // Bottom left tof reading
        break;
      case 3:
        lrTOF_holder4 = ((sensorsL1[i].readRangeContinuousMillimeters()));
        //lrTOF_holder2 = trTOF_Avg.reading(sensorsL1[i].readRangeSingleMillimeters());
        trTOF = &lrTOF_holder4; // Top right tof reading
        break;
      case 4:
        lrTOF_holder5 = ((sensorsL1[i].readRangeContinuousMillimeters()));
        //lrTOF_holder4 = tlTOF_Avg.reading(sensorsL1[i].readRangeSingleMillimeters());
        brTOF = &lrTOF_holder5; // Bottom right tof reading
        break;
      case 5:
        lrTOF_holder6 = ((sensorsL1[i].readRangeContinuousMillimeters()));
        //lrTOF_holder4 = tlTOF_Avg.reading(sensorsL1[i].readRangeSingleMillimeters());
        tlTOF = &lrTOF_holder6; // Top left tof reading
      break;
   }
  }
}

void allTOFReadings()
{  
  sensor->srTOF_Values();
  sensor->lrTOF_Values();
}

void allUSValues()
{
  sensor->us_Values();
}

/* Function for reading and writing from ultrasonic sensors */
float Sensors::ping(int32_t trigPin, int32_t echoPin) {
  digitalWrite(trigPin, LOW);
  int start = micros();
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  start = micros();
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration * .0343) / 2;
  return distance;
}

/* Final Setup Function for Sensors */
void Sensors::sensor_setup()
{
  // Start of Init
  Serial.begin(115200);
  io.begin(SX1509_ADDRESS);
  Wire.begin();
  Wire.setClock(400000);
  
  srTOF_Setup();
  lrTOF_Setup();
  usSetup();
}

/* Functions for getting sensor values */
int get_lUS()
{
  return sensor->lUS;
}

int get_rUS()
{
  return sensor->rUS;
}

int get_mTOF()
{
  return *(sensor->mTOF);
}

int get_entry() 
{
  return *(sensor->entry);
}

int get_entry2() 
{
  return *(sensor->entry2);
}

int get_barrel()
{
  return *(sensor->barrel);
}

int get_brTOF()
{
  return *(sensor->brTOF);
}

int get_trTOF()
{
  return *(sensor->trTOF);
}

int get_blTOF()
{
  return *(sensor->blTOF);
}

int get_tlTOF()
{
  return *(sensor->tlTOF);
}