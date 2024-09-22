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

void Navigation::general_navigation()
{
  int mTOF = *(sensor->mTOF);
  int l_us =  sensor->lUS;
  int r_us = sensor->rUS;
  if (mTOF < frontTOFLimit){///units in mm
    if (r_us < l_us){
      while (mTOF < frontTOFMinimum){
        mTOF = *(sensor->mTOF);
        l_us =  sensor->lUS;
        r_us = sensor->rUS;
        sensor -> allTOFReadings();

        turn_left();
      }
    }
    else if (r_us > l_us){
      while (mTOF < frontTOFMinimum){
        mTOF = *(sensor->mTOF);
        l_us =  sensor->lUS;
        r_us = sensor->rUS;
        sensor -> allTOFReadings();

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

void Navigation::weightDetection(bool direction)
{
  int tr = *(sensor->trTOF);
  int br = *(sensor->brTOF);
  int tl = *(sensor->tlTOF);
  int bl = *(sensor->blTOF);

  if (direction) {
    while ((tr - br) > 200) {
      tr = *(sensor->trTOF);
      br = *(sensor->brTOF);
      sensor->allTOFReadings();
      turn_right();
    } 
  } else {
    while ((tl - bl) > 200) {
      tl = *(sensor->tlTOF);
      bl = *(sensor->blTOF);
      sensor -> allTOFReadings();
      turn_left();
    }
  }
}

void Navigation::loop() 
{
  int tr = *(sensor->trTOF);
  int br = *(sensor->brTOF);
  int tl = *(sensor->tlTOF);
  int bl = *(sensor->blTOF);

  if (((tr - br) > 200) && (br < 1000)) {
    Serial.print("Gotcha1");
    weightDetection(1);
  } else if (((tl - bl) > 200) && (bl < 1000)) {
    Serial.print("Gotcha2");
    weightDetection(0);
  } else {
    general_navigation();
  }
}