#ifndef SENSORS_H
#define SENSORS_H

#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

#include <Arduino.h>
#include <Wire.h>
#include <movingAvg.h>

#include <cstdlib>
#include <cstdint>

class Sensors {
  protected:
    static const byte SX1509_ADDRESS = 0x3F;
    static const uint8_t sensorCountL0 = 4;
    static const uint8_t sensorCountL1 = 4;
    const uint8_t xshutPinsL0[sensorCountL0] = {0,1,2,3};
    const uint8_t xshutPinsL1[sensorCountL1] = {4,5,6,7};

    SX1509 io;
    VL53L0X sensorsL0[sensorCountL0];
    VL53L1X sensorsL1[sensorCountL1];

    // static constexpr int sampleSize = 10;
    // movingAvg lUS_Avg, rUS_Avg; // For Ultrasound 
    // movingAvg mTOF_Avg, entry_Avg, barrel_Avg; // For VL53L0X
    // movingAvg trTOF_Avg, tlTOF_Avg, brTOF_Avg, blTOF_Avg; // For VL53L1X

    float duration, distance, distance_left, distance_right, front_tof;
    // static int sensor1,sensor2,sensor3,sensor4,sensor5,sensor6,sensor7,tof_holder,tof_holder_2 = 0;
    
    int srTOF_holder1;
    int srTOF_holder2;
    int srTOF_holder3;
    int srTOF_holder4;
    
    int lrTOF_holder1;
    int lrTOF_holder2;
    int lrTOF_holder3;
    int lrTOF_holder4;

    void lrTOF_Setup();
    void srTOF_Setup();
    void usSetup();

  public:
  static const int echoPinr = 2;
    static const int trigPinr = 3;
    static const int echoPinl = 4;
    static const int trigPinl = 5;
    
    int lUS, rUS;
    int* mTOF;
    int* blTOF; 
    int* tlTOF;
    int* brTOF;
    int* trTOF; // VL53L1X for detceting if there are weights 

    int* barrel; // Short-range TOF for weight detection in the barrel
    int* entry; // Short-range TOF for weight detection in the channel
    int* entry2; //second Short-range TOF for weight detection in the channel
    int ps = 1; // Proximity Sensor value

    float ping(int32_t trigPin, int32_t echoPin);
    void sensor_setup();
    void srTOF_Values();
    void lrTOF_Values();
    void us_Values();
};

void allTOFReadings();
void allUSValues();


/*
Getter Functions for the distance sensor values
*/
int get_lUS();
int get_rUS();

int get_mTOF();
int get_entry();
int get_entry2();
int get_barrel();

int get_brTOF();
int get_trTOF();
int get_blTOF();
int get_tlTOF();

extern Sensors *sensor;

#endif // SENSORS_H