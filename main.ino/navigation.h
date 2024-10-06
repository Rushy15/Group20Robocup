#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "sensors.h"
#include <Servo.h>

class Navigation {
  protected:
    
  public:
    Servo Rservo;
    Servo Lservo;

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
void turn_right();
void roll_right();

void walldetected();
void wallFollowing();

bool reachedDesiredHeadingAngle(int desiredAngle);
int angleToTurn(int currentHeadingAngle, int angleToTurn);

bool get_weight_detected_bool();
void set_weight_detected_bool(bool state);

void general_navigation();
void weightDetection(bool direction);
void nav_loop(bool weight_detected);

#endif // NAVIGATION_H