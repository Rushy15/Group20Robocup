#include "navigation.h"
#include "sensors.h"
#include "storage.h"

#define frontTOFLimit 300
#define frontTOFMinimum 500
#define rUSLimit 16
#define lUSLimit 16
#define weightDetectingDistance 65
#define topLevel_longRangeTOFLimit 100

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
  //int l_us =  get_lUS();
  //int r_us = get_rUS();
  int tr_tof = get_trTOF();
  int tl_tof = get_tlTOF();

  if (mTOF < frontTOFLimit){ //units in mm
    if (tr_tof < tl_tof){
      while (mTOF < frontTOFMinimum){
        allTOFReadings();
        //allUSValues();
        mTOF = get_mTOF();
        //l_us =  get_lUS();
        //r_us = get_rUS();
        turn_left();
      }
    }
    else if (tr_tof > tl_tof){
      while (mTOF < frontTOFMinimum){
        allTOFReadings();
        //allUSValues();
        mTOF = get_mTOF();
        //l_us =  get_lUS();
        //r_us = get_rUS();
        turn_right();
      }
    }
  }

  else if (get_trTOF() < topLevel_longRangeTOFLimit) {///units in cm
    turn_left();
  }

   else if (get_tlTOF() < topLevel_longRangeTOFLimit) {///units in cm
    turn_right();
  }
  else if (tr_tof < rUSLimit) {///units in cm
    turn_left();
  }

   else if (tl_tof < lUSLimit) {///units in cm
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
    while ((tr - br) > weightDetectingDistance) {
      turn_right();
      allTOFReadings();
      allUSValues();
      tr = get_trTOF();
      br = get_brTOF();
      //storage->storing();
      
    }
  } else {
    while ((tl - bl) > weightDetectingDistance) {
      turn_left();
      allTOFReadings();
      allUSValues();
      tl = get_tlTOF();
      bl = get_blTOF();
      //storage->storing();
    }
  }
}

void Navigation::wallFollowing()
{
  // allTOFReadings();
  // allUSValues();
  int mTOF = get_mTOF();
  int l_us =  get_lUS();
  int r_us = get_rUS();

  /*
  if the mTOF and rUS both are less than some distance, 
  we want to turn left 

  if the mTOF is sensing over some distance, and the rUS is detecting a wall on the right, 
  then when the right rUS is not detetcing the wall anymore, turn right. 
  */

  if ((mTOF < frontTOFLimit) && (r_us < rUSLimit)) {
    turn_left();
  } else if ((mTOF > frontTOFLimit) && (r_us < rUSLimit)) {
    endOfWall = false;
    while (endOfWall == false) {
      go_straight();

      allTOFReadings();
      allUSValues();
      mTOF = get_mTOF();
      l_us =  get_lUS();
      r_us = get_rUS();

      if (r_us > rUSLimit) {
        turn_right();
        endOfWall = true;
      }
  }
  } else if ((mTOF < frontTOFLimit) && (l_us < lUSLimit)){
    while (l_us < lUSLimit) {
      turn_left();

      allTOFReadings();
      allUSValues();
      mTOF = get_mTOF();
      l_us =  get_lUS();
      r_us = get_rUS();
    }
  } else {
    go_straight();
  }
}


void nav_loop() 
{
  int tr = get_trTOF();
  int br = get_brTOF();
  int tl = get_tlTOF();
  int bl = get_blTOF();

  // if ((((tr - br) > 65) && (br < 1300))||(((tl - bl) > 65) && (bl < 1300))){
  //   while ((get_entry() > 190)||(get_mTOF() > frontTOFLimit)) {
  //       allTOFReadings();
  //       tr = get_trTOF();
  //       br = get_brTOF();
  //       tl = get_tlTOF();
  //       bl = get_blTOF();
        if (((tr - br) > 65) && (br < 1300)) {
          //Serial.print("Gotcha1");
          navigation -> weightDetection(1);
        } else if (((tl - bl) > 65) && (bl < 1300)) {
          //Serial.print("Gotcha2");
          navigation->weightDetection(0);
        }
        else {
          navigation->general_navigation();
        }
    
  // }
   
  
  // navigation->wallFollowing();
  // navigation->wallFollowing();
}