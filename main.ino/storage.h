#ifndef STORAGE_H
#define STORAGE_H

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>

#include "sensors.h"
#include "navigation.h"

class Storage {
  protected:
    Servo myservo;  // create servo object to control a servo
    Servo drum;
    static const uint8_t third_slot  = 180;   
    static const uint8_t second_slot  = 135;
    static const uint8_t first_slot = 93;
    static const uint8_t third_slot_discard = 76;
    static const uint8_t second_slot_discard = 36;        
    static const uint8_t first_slot_discard = 0 ;  

    
    int front_tof;

  public:
    static const int sensor = A7;
    Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
    // int psState_reading;
    uint8_t psState; // Final Value used within the FSM

    unsigned long Timer1, currTimer;  
    unsigned long prevTimer = 0;
    uint8_t weights_collected = 0;
    uint8_t finished_storing = 0;

    uint16_t clear, red, green, blue;
    
    // uint16_t r, g, b; // Final values to be used

    void colour_sensor_setup();
    void storage_setup();
    void rotateDrum(int start,int dest);
    void discard_all_weights();
    void storeWeights();
    void removeWeights(int Timer1); 
};

int read_psState();
void storing(uint8_t proximityState);
bool max_capacity();
void reset_capacity();

void updateColourValues();
uint16_t getR();
uint16_t getG();
uint16_t getB();

extern Storage *storage;

#endif // STORAGE_H