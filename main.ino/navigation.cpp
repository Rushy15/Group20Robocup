#include "navigation.h"
#include "sensors.h"
#include "storage.h"

#define frontTOFLimit 300
#define frontTOFMinimum 500
#define rUSLimit 16
#define lUSLimit 16

#define N 1500 // Neutral Speed

Navigation *navigation = nullptr;

void Navigation::navigation_setup()
{
  // Motor Setup
  Rservo.attach(28);
  Lservo.attach(8);
}

void Navigation::stop()
{
  Rservo.writeMicroseconds(1500);  
  Lservo.writeMicroseconds(1500);
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
  Rservo.writeMicroseconds(1950);  
  Lservo.writeMicroseconds(1050);  
}

void Navigation::reverse() {
  Rservo.writeMicroseconds(1050);  
  Lservo.writeMicroseconds(1950);  
}
void Navigation::general_navigation()
{
  // allTOFReadings();
  // allUSValues();
  int mTOF = get_mTOF();
  int l_us =  get_lUS();
  int r_us = get_rUS();
  if (mTOF < frontTOFLimit){///units in mm
    if (r_us < l_us){
      while (mTOF < frontTOFMinimum){
        allTOFReadings();
        allUSValues();
        mTOF = get_mTOF();
        l_us =  get_lUS();
        r_us = get_rUS();
        //storage->storing();
        turn_left();
      }
    }
    else if (r_us > l_us){
      while (mTOF < frontTOFMinimum){
        allTOFReadings();
        allUSValues();
        mTOF = get_mTOF();
        l_us =  get_lUS();
        r_us = get_rUS();
        turn_right();
      }
    }
  }
  else if (get_trTOF() < 150) {///units in cm
    turn_left();
  }

   else if (get_tlTOF() < 150) {///units in cm
    turn_right();
  }
  else if (get_rUS() < rUSLimit) {///units in cm
    turn_left();
  }

   else if (get_lUS() < lUSLimit) {///units in cm
    turn_right();
  }

  else {
    go_straight(); 
  }
}

void Navigation::weightDetection(bool direction)
{
  int tr = get_trTOF();
  int br = get_brTOF();
  int tl = get_tlTOF();
  int bl = get_blTOF();

  if (direction) {
    while ((tr - br) > 65) {
      turn_right();
      allTOFReadings();
      allUSValues();
      tr = get_trTOF();
      br = get_brTOF();
      //storage->storing();
      
    }
  } else {
    while ((tl - bl) > 65) {
      turn_left();
      allTOFReadings();
      allUSValues();
      tl = get_tlTOF();
      bl = get_blTOF();
      //storage->storing();
    }
  }
}

void nav_loop() 
{
  int tr = get_trTOF();
  int br = get_brTOF();
  int tl = get_tlTOF();
  int bl = get_blTOF();

  if (((tr - br) > 65) && (br < 1300)) {
    //Serial.print("Gotcha1");
    navigation -> weightDetection(1);
  } else if (((tl - bl) > 65) && (bl < 1300)) {
    //Serial.print("Gotcha2");
    navigation->weightDetection(0);
  } else {
    navigation->general_navigation();
  }
}