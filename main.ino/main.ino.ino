#include "navigation.h"

Navigation navigation;

void setup() {
  // put your setup code here, to run once:
  
 
  navigation.setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("set up");
  navigation.loop();
}
