#include "navigation.h"
#include "sensors.h"
#include "storage.h"

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
  int mTOF = get_mTOF();
  int l_us =  get_lUS();
  int r_us = get_rUS();

  if (mTOF < frontTOFLimit){///units in mm
    if (r_us < l_us){
      while (mTOF < frontTOFMinimum){
        mTOF = get_mTOF();
        l_us =  get_lUS();
        r_us = get_rUS();
        //storage->storing();
        turn_left();
        allTOFReadings();
        allUSValues();
      }
    }
    else if (r_us > l_us){
      while (mTOF < frontTOFMinimum){
        mTOF = get_mTOF();
        l_us =  get_lUS();
        r_us = get_rUS();

        turn_right();
        allTOFReadings();
        allUSValues();
      }
    }
  }
  else if (r_us < rUSLimit) {///units in cm
    turn_left();
  }

   else if (l_us < lUSLimit) {///units in cm
    turn_right();
  }

  else {
    go_straight(); 
  }
  
  delay(50);
}

void Navigation::weightDetection(bool direction)
{
  int tr = get_trTOF();
  int br = get_brTOF();
  int tl = get_tlTOF();
  int bl = get_blTOF();

  if (direction) {
    while ((tr - br) > 100) {
      tr = get_trTOF();
      br = get_brTOF();
      allTOFReadings();
      //storage->storing();
      turn_right();
      allTOFReadings();
      allUSValues();
    } 
  } else {
    while ((tl - bl) > 100) {
      tl = get_tlTOF();
      bl = get_blTOF();
      
      //storage->storing();
      turn_left();
      allTOFReadings();
      allUSValues();
    }
  }
}

void nav_loop() 
{
  int tr = get_trTOF();
  int br = get_brTOF();
  int tl = get_tlTOF();
  int bl = get_blTOF();

  if (((tr - br) > 100) && (br < 1000)) {
    //Serial.print("Gotcha1");
    navigation -> weightDetection(1);
  } else if (((tl - bl) > 100) && (bl < 1000)) {
    //Serial.print("Gotcha2");
    navigation->weightDetection(0);
  } else {
    navigation->general_navigation();
  }
}