#include "navigation.h"

Navigation navigation;

void setup() {
  // put your setup code here, to run once:
  sensor.setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  navigation.loop();
}
