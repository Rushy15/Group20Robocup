#include "collection.h"

Collection *collection = nullptr;

void Collection::collection_setup()
{
    drum.attach(29);  // attaches the servo on pin 9 to the servo object
}

void reverseDrum()
{
  (collection->drum).writeMicroseconds(1050);
}

void stopDrum()
{
  (collection->drum).writeMicroseconds(1500);
}

void spinDrum()
{
  (collection->drum).writeMicroseconds(1950);
}