#include "navigation.h"
#include "sensors.h"

#define frontTOFLimit 300
#define frontTOFMinimum 500
#define rUSLimit 20 
#define lUSLimit 20

#define N 1500 // Neutral Speed

Navigation *navigation = nullptr;

void Navigation::navigation_setup()
{
  // Motor Setup
  Rservo.attach(28);
  Lservo.attach(8);
}

void Navigation::turn_left() {
    Rservo.writeMicroseconds(1950);  
    Lservo.writeMicroseconds(1950);
}

void Navigation::turn_right() {
    Rservo.writeMicroseconds(1050);  
    Lservo.writeMicroseconds(1050); 
}

void Navigation::go_straight() {
  Rservo.writeMicroseconds(1900);  
  Lservo.writeMicroseconds(1100);  
}

void Navigation::loop() {
  if (*(sensor->mTOF) < frontTOFLimit){///units in mm
    if (sensor->rUS < sensor->lUS){
      while (*(sensor->mTOF) < frontTOFMinimum){
        sensor -> allTOFReadings();
        // for (uint8_t i = 0; i < sensorCount; i++){
        //     front_tof = sensors[i].readRangeContinuousMillimeters();
        //     if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
        //     }
        turn_left();
      }
    }
    else if (sensor->rUS > sensor->lUS){
      while (*(sensor->mTOF) < frontTOFMinimum){
        sensor -> allTOFReadings();
        // for (uint8_t i = 0; i < sensorCount; i++) {
        //     if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }   
      // }
        turn_right();
      }
    }
  }
  else if (sensor->rUS < lUSLimit) {///units in cm
    turn_left();
  }

   else if (sensor->lUS < rUSLimit) {///units in cm
    turn_right();
  }

  else {
    go_straight(); 
  }
  
  delay(50);
}