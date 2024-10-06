#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "sensors.h"
#include <Servo.h>

class Navigation {
  protected:
    Servo Rservo;
    Servo Lservo;
    
  public:
    bool walldetected_bool = false;
    bool robotstuck_bool = false;
    bool weight_detcted_bool = false;

    int start_weight_detection;

    void go_straight();
    void turn_left();
    void turn_right();
    void reverse();
    void stop();
    void roll_right();
    void roll_left();

    void general_navigation();
    void weightDetection(bool direction);
    void navigation_setup();
};

// bool endOfWall = false;

int angleToTurn(int currentHeadingAngle, int angleToTurn);
void nav_loop(bool weight_detected);
void wallFollowing();
void walldetected();
bool reachedDesiredHeadingAngle(int desiredAngle);

extern Navigation *navigation;

#endif // NAVIGATION_H