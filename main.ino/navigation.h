#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "sensors.h"

class Navigation: public Sensors {
  public:
    Navigation();
    void loop();
  
  private:
    void turn_left();
    void turn_right();
    void go_straight()
}

#endif // NAVIGATION_H