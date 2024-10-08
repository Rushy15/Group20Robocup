#ifndef COLLECTION_H
#define COLLECTION_H

#include <Servo.h>
#include "sensors.h"

class Collection {
  public:
    Servo drum;
    void collection_setup();
};

void reverseDrum();
void stopDrum();
void spinDrum();

extern Collection *collection;

#endif // COLLECTION_H