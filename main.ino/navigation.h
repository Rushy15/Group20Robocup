#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "sensors.h"
#include <Servo.h>

class Navigation {
  protected:
    Servo Rservo;
    Servo Lservo;
    
  public:
    void go_straight();
    void turn_left();
    void turn_right();
    void reverse();
    void stop();

    void general_navigation();
    void weightDetection(bool direction);
    void navigation_setup();
};

// bool endOfWall = false;

void nav_loop();
// void wallFollowing();

extern Navigation *navigation;

#endif // NAVIGATION_H