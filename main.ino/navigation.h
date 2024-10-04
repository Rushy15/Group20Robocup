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

void nav_loop();
void wallFollowing();
void walldetected();
extern Navigation *navigation;

#endif // NAVIGATION_H