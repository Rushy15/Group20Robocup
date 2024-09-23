#ifndef STORAGE_H
#define STORAGE_H

#include <Servo.h>

#include "sensors.h"
#include "navigation.h"

class Storage {
  protected:
    Servo myservo;  // create servo object to control a servo
    Servo drum;
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
    // int front_tof;

  public:
    void storage_setup();
    void continueOperation();
    void rotateDrum(int start,int dest);
    void discard_all_weights();
    void continue_Operation();
    void storing();
};

extern Storage *storage;

#endif // STORAGE_H