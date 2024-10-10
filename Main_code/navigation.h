#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "sensors.h"
#include <Servo.h>

class Navigation {
  protected:
    
  public:
    Servo Rservo;
    Servo Lservo;
    bool isRemovingWeight = false;
    bool walldetected_bool = false;
    bool robotstuck_bool = false;
    bool weight_detcted_bool = false;

    int start_weight_detection;

    void navigation_setup();
};


void go_straight();
void stop();
void reverse();
void turn_left();
void roll_left();
void turn_left_slow();
void turn_right();
void turn_right_slow();
void roll_right();

void walldetected();
void wallFollowingRight();
void wallFollowingLeft();
bool reachedDesiredHeadingAngle(int desiredAngle);
int angleToTurn(int currentHeadingAngle, int angleToTurn, int directionToTurn);

bool get_weight_detected_bool();
void set_weight_detected_bool(bool state);

void set_wall_detected_bool(bool state);

bool get_isRemovingWeight_bool();
void set_isRemovingWeight_bool(bool state);

void weight_entered_entry();

void general_navigation();
void weightDetection(bool direction);
void nav_loop(bool weight_detected);

extern Navigation *navigation; 
#endif // NAVIGATION_H