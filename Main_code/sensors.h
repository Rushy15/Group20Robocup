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
    static const uint8_t sensorCountL0 = 2;
    static const uint8_t sensorCountL1 = 6;
    const uint8_t xshutPinsL0[sensorCountL0] = {0,1};
    const uint8_t xshutPinsL1[sensorCountL1] = {2,3,4,5,6,7};

    SX1509 io;
    VL53L0X sensorsL0[sensorCountL0];
    VL53L1X sensorsL1[sensorCountL1];

    // static constexpr int sampleSize = 10;
    // movingAvg lUS_Avg, rUS_Avg; // For Ultrasound 
    // movingAvg mTOF_Avg, entry_Avg, barrel_Avg; // For VL53L0X
    // movingAvg trTOF_Avg, tlTOF_Avg, brTOF_Avg, blTOF_Avg; // For VL53L1X

    float distance, duration;
    
    uint16_t srTOF_holder1;
    uint16_t srTOF_holder2; // Entry Sensor Value Holder
    uint16_t srTOF_holder3; // Barrel Sensor Value Holder
    uint16_t srTOF_holder4; // Entry Sensor Value Holder
    
    uint16_t lrTOF_holder1;
    uint16_t lrTOF_holder2;
    uint16_t lrTOF_holder3;
    uint16_t lrTOF_holder4;
    uint16_t lrTOF_holder5;
    uint16_t lrTOF_holder6;

    void lrTOF_Setup();
    void srTOF_Setup();
    void usSetup();

  public:
  static const int echoPinr = 2;
    static const int trigPinr = 3;
    static const int echoPinl = 4;
    static const int trigPinl = 5;
    
    int lUS, rUS;
    uint16_t* mTOF;
    uint16_t* blTOF; 
    uint16_t* tlTOF;
    uint16_t* brTOF;
    uint16_t* trTOF; // VL53L1X for detceting if there are weights 

    uint16_t* barrel; // Short-range TOF for weight detection in the barrel
    uint16_t* entry; // Short-range TOF for weight detection in the channel
    uint16_t* entry2; //second Short-range TOF for weight detection in the channel
    uint8_t ps = 1; // Proximity Sensor value

    float ping(int32_t trigPin, int32_t echoPin);
    void sensor_setup(); 
    void srTOF_Values();
    void lrTOF_Values();
    void us_Values();
};

/*
Reading and updating all TOF sensor values
*/
void allTOFReadings(); 

/*
Reading and updating all ultrasonic sensor values
*/
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