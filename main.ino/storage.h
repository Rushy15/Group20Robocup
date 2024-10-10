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
  
    int psState; // Final Value used within the FSM
    bool robotLeaveBase;

    unsigned long Timer1, currTimer;  
    unsigned long prevTimer = 0;
    uint8_t weights_collected = 0;
    uint8_t finished_storing = 0;

    uint16_t clear, red, green, blue;
    uint16_t red_homebase, green_homebase, blue_homebase;
    uint16_t red_enemy, green_enemy, blue_enemy;
    
    // uint16_t r, g, b; // Final values to be used

    void colour_sensor_setup();
    void storage_setup();
    void rotateDrum(int start,int dest);
    void discard_all_weights();
    void storeWeights();
    void removeWeights(int Timer1); 
};

int read_psState();
void set_psState(int state);
int get_psState();
int get_weightsCollected();

void storing(uint8_t proximityState);
bool max_capacity();
void reset_capacity();

/*
Updates the current RGB Values
*/
void updateColourValues();

/*
One time function run to collect home base data at startup
*/
void collectingColourData();

/*
Assigns RGB values for Enemy Base
*/
void assignEnemyBaseRGB(); 

/*
Checks to see if the robot is in the home base
*/
bool inHomeBase();

/*
Checks to see if the robot is in the enemy base
*/
bool inEnemyBase();

/*
Returns 1 if the home base is blue else returns 0 if the home base is green
*/
int homeBaseColour();

uint16_t getR();
uint16_t getG();
uint16_t getB();

/*
Returns true when the robot has collected one weight 
*/
bool get_robotLeaveBase();

/*
sets to true when the robot has collected one weight and resets when the robot dumps weights
*/
void set_robotLeaveBase(bool state);

extern Storage *storage;
#endif // STORAGE_H