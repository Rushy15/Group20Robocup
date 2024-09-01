#include "navigation.h"

#define frontTOFLimit 300
#define frontTOFMinimum 500
#define rUSLimit 20 
#define lUSLimit 20

#define N 1500 // Neutral Speed

Navigation::Navigation()

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
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    distance_left = ping(trigPinl, echoPinl); // Left US Sensor Reading
    
    front_tof = sensors[i].readRangeContinuousMillimeters();

    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    distance_right = ping(trigPinr, echoPinr); // Right US Sensor Reading

    Serial.print(" Left: ");
    Serial.print(distance_left);
    Serial.print('\t');
    
    Serial.print(" Middle: ");
    Serial.print(front_tof);
    Serial.print('\t');
    
    Serial.print(" Right: ");
    Serial.println(distance_right);
    Serial.print('\t');
  }

  if (front_tof < frontTOFLimit){///units in mm
    if (distance_right < distance_left){
      while (front_tof < frontTOFMinimum){
        for (uint8_t i = 0; i < sensorCount; i++){
            front_tof = sensors[i].readRangeContinuousMillimeters();
            if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
            }
      turn_left();
      }
    }
    else if (distance_right > distance_left){
      while (front_tof < frontTOFMinimum){
        for (uint8_t i = 0; i < sensorCount; i++) {
            front_tof = sensors[i].readRangeContinuousMillimeters();
            if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }   
      }
      turn_right();
      }
    }
  }
  else if (distance_right < lUSLimit) {///units in cm
    turn_left();
  }

   else if (distance_left < rUSLimit) {///units in cm
    turn_right();
  }

  else {
    go_straight();
  }
  
  delay(50);
}