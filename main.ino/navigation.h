#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "sensors.h"
#include <Servo.h>

class Navigation {
  protected:
    Servo Rservo;
    Servo Lservo;
    
    void turn_left();
    void turn_right();
    void go_straight();

  public:
    void general_navigation();
    void weightDetection(bool direction);
    void navigation_setup();
};

void nav_loop();

extern Navigation *navigation;

#endif // NAVIGATION_H