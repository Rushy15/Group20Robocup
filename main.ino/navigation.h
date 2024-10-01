#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "sensors.h"
#include <Servo.h>

class Navigation {
  protected:
    Servo Rservo;
    Servo Lservo;

    bool endOfWall = false;
    
    void turn_left();
    void turn_right();
    
    
  public:
    void go_straight();
    void reverse();
    void stop();
    void general_navigation();
    void weightDetection(bool direction);
    void navigation_setup();
    void wallFollowing();
};

void nav_loop();

extern Navigation *navigation;

#endif // NAVIGATION_H