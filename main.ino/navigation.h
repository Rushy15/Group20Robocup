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
    void navigation_setup();
    void loop();
};

extern Navigation *navigation;

#endif // NAVIGATION_H