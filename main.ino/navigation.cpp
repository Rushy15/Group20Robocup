#include "navigation.h"
#include "sensors.h"
#include "storage.h"

#define frontTOFLimit 300
#define frontTOFMinimum 600
#define rUSLimit 15
#define lUSLimit 15
#define weightDetectingDistance 130// Difference between long range TOFs to turn the robot if a weight is detected
#define weightDetectingDistanceMax 1200
#define topLevel_longRangeTOFLimit 170
#define WallfollowingLimit 25


#define FWR_FULL 1950
#define FWL_FULL 1050
#define BWR_FULL 1050  
#define BWL_FULL 1950

#define FWR 1850
#define FWL 1150
#define BWR 1150  
#define BWL 1850

#define FWR_SLOW 1780
#define FWL_SLOW 1220
#define BWR_SLOW 1220  
#define BWL_SLOW 1780

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
  Rservo.writeMicroseconds(N);  
  Lservo.writeMicroseconds(N);
}

void Navigation::turn_left() {
    Rservo.writeMicroseconds(FWR);  
    Lservo.writeMicroseconds(BWL);
}

void Navigation::turn_right() {
    Rservo.writeMicroseconds(BWR);  
    Lservo.writeMicroseconds(FWL); 
}

void Navigation::go_straight() {
  Rservo.writeMicroseconds(FWR);  
  Lservo.writeMicroseconds(FWL);  
}

void Navigation::reverse() {
  Rservo.writeMicroseconds(BWR_SLOW);  
  Lservo.writeMicroseconds(BWL_SLOW);  
}

void Navigation::roll_right()
{
  Rservo.writeMicroseconds(BWR_FULL);  
  Lservo.writeMicroseconds(FWL);  
}

void Navigation::general_navigation()
{
  // allTOFReadings();
  // allUSValues();
  int mTOF = get_mTOF();
  int tr_tof = get_trTOF();
  int tl_tof = get_tlTOF();

  if (mTOF < frontTOFLimit){ //units in mm
    if (tr_tof < tl_tof){
      while (mTOF < frontTOFMinimum){
        allTOFReadings();
        mTOF = get_mTOF();
        turn_left();
      }
    }
    else if (tr_tof > tl_tof){
      while (mTOF < frontTOFMinimum){
        allTOFReadings();
        mTOF = get_mTOF();
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
  // else if (tr_tof < rUSLimit) {///units in cm
  //   turn_left();
  // }

  //  else if (tl_tof < lUSLimit) {///units in cm
  //   turn_right();
  // }

  else {
    go_straight(); 
  }
}

void Navigation::weightDetection(bool direction)
{
  uint16_t tr = get_trTOF();
  uint16_t br = get_brTOF();
  uint16_t tl = get_tlTOF();
  uint16_t bl = get_blTOF();

  if (direction) {
    while ((tr - br) > weightDetectingDistance) {
      turn_right();
      // delay(600);
      // go_straight();
      allTOFReadings();
      allUSValues();
      tr = get_trTOF();
      br = get_brTOF();      
    }
  } else {
    while ((tl - bl) > weightDetectingDistance) {
      turn_left();
      // delay(600);
      // go_straight();
      allTOFReadings();
      allUSValues();
      tl = get_tlTOF();
      bl = get_blTOF();
    }
  }
}

void walldetected() {
  if (get_mTOF() < frontTOFLimit) {
    navigation -> walldetected_bool = true;
  }
}

void wallFollowing()
{
  allTOFReadings();
  allUSValues();
  uint16_t mTOF = get_mTOF();
  // uint32_t l_us = get_lUS();
  uint32_t r_us = get_rUS();
  uint16_t trTOF = get_trTOF();

  /*
  if the mTOF and rUS both are less than some distance, 
  we want to turn left 

  if the mTOF is sensing over some distance, and the rUS is detecting a wall on the right, 
  then when the right rUS is not detetcing the wall anymore, turn right. 

  when wall following the number of same turns in a row you can have is two
  */
  //Following wall on the right
  while (navigation -> walldetected_bool == false) {
    walldetected();
    Serial.println("Stuck Here");
    allTOFReadings();
    allUSValues();
    navigation -> go_straight();
  }

  if (navigation->walldetected_bool == true) {
    if ((mTOF < frontTOFMinimum) || ((mTOF < frontTOFMinimum) && (r_us <= rUSLimit))) {
    while (mTOF < frontTOFLimit){
      Serial.println("Stuck Here 1");
        allTOFReadings();
        allUSValues();
        mTOF = get_mTOF();
        r_us = get_rUS();
        navigation -> turn_left();
      }
  }
  
  else if ((mTOF > frontTOFLimit) && (r_us > rUSLimit) && (trTOF > topLevel_longRangeTOFLimit)) {
    while ((r_us > WallfollowingLimit) || (trTOF > frontTOFLimit)) {
      Serial.println("Stuck Here 2");
      allTOFReadings();
      allUSValues();

      mTOF = get_mTOF();
      r_us = get_rUS();
      trTOF = get_trTOF();
      // navigation -> turn_right(); 
      navigation -> roll_right();
    }

  } else if (get_trTOF() < topLevel_longRangeTOFLimit) {///units in cm
    navigation->turn_left();
  } else {
    navigation -> go_straight();
  }
  }

  // if ((mTOF < frontTOFLimit) && (r_us < rUSLimit)) {
  //   navigation->turn_left();
  // } else if ((mTOF > frontTOFLimit) && (r_us < rUSLimit)) {
  //   endOfWall = false;
  //   while (endOfWall == false) {
  //     navigation->go_straight();
      
  //     allTOFReadings();
  //     allUSValues();
  //     mTOF = get_mTOF();
  //     l_us =  get_lUS();
  //     r_us = get_rUS();

  //     if (r_us > rUSLimit) {
  //       navigation->turn_right();
  //       endOfWall = true;
  //     }
  // }
  // } else if ((mTOF < frontTOFLimit) && (l_us < lUSLimit)){
  //   while (l_us < lUSLimit) {
  //     navigation->turn_left();

  //     allTOFReadings();
  //     allUSValues();
  //     mTOF = get_mTOF();
  //     l_us =  get_lUS();
  //     r_us = get_rUS();
  //   }
  // } else {
  //   navigation->go_straight();
  // }
}


void nav_loop() 
{
  uint16_t tr = get_trTOF();
  uint16_t br = get_brTOF();
  uint16_t tl = get_tlTOF();
  uint16_t bl = get_blTOF();

  // if ((((tr - br) > 65) && (br < 1300))||(((tl - bl) > 65) && (bl < 1300))){
  //   while ((get_entry() > 190)||(get_mTOF() > frontTOFLimit)) {
  //       allTOFReadings();
  //       tr = get_trTOF();
  //       br = get_brTOF();
  //       tl = get_tlTOF();
  //       bl = get_blTOF();
        if (((tr - br) > weightDetectingDistance) && (br < weightDetectingDistanceMax)) {
          //Serial.print("Gotcha1");
          navigation -> weightDetection(1);
        } else if (((tl - bl) > weightDetectingDistance) && (bl <  weightDetectingDistanceMax)) {
          //Serial.print("Gotcha2");
          navigation -> weightDetection(0);
        }
        else {
          navigation->general_navigation();
        }
    
  // }
   
  
  // navigation->wallFollowing();
  // navigation->wallFollowing();
}