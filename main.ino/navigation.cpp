#include "navigation.h"
#include "sensors.h"
#include "storage.h"
#include "imu.h"

#define N 1500 // Neutral Speed

#define FWR 1850
#define FWL 1150
#define BWR 1150  
#define BWL 1850

#define FWR_SLOW 1780
#define FWL_SLOW 1220
#define BWR_SLOW 1220  
#define BWL_SLOW 1780

#define FWR_FULL 1900
#define FWL_FULL 1100
#define BWR_FULL 1100
#define BWL_FULL 1900

#define STUCK_THRESHOLD 3000 // Time in milliseconds to consider stuck
#define REVERSE_DURATION 2000 // Duration to reverse in milliseconds
#define HOME_BASE_THRESHOLD 5000 // Time in milliseconds to stay at home base

#define rUSLimit 15
#define lUSLimit 15
#define frontTOFLimit 300
#define frontTOFWFLimit 350 // Wall following limit
#define frontTOFMinimum 500
#define frontTOFWFMinimum 500
#define WallfollowingLimit 25
#define weightDetectingDistance 130 // Difference between long range TOFs to turn the robot if a weight is detected
#define maxStraightLineTravelTime 3000
#define topLevel_longRangeTOFLimit 170
#define weightDetectingDistanceMax 1200
#define topLevel_longRangeTOFWFLimit 200
#define angleToTurnDuringWallFollowing 90
#define angleToTurnDuringFindingWeight 340

unsigned long stuckStartTime = 0;
unsigned long homeBaseStartTime = 0;
bool isStuck = false;
bool atHomeBase = false;

Navigation *navigation = nullptr;

void Navigation::navigation_setup()
{
  // Motor Setup
  Rservo.attach(28);
  Lservo.attach(8);
}

void stop()
{
  navigation->Rservo.writeMicroseconds(N);  
  navigation->Lservo.writeMicroseconds(N);
}

void turn_left() {
    navigation->Rservo.writeMicroseconds(FWR);  
    navigation->Lservo.writeMicroseconds(BWL);
}

void turn_right() {
    navigation->Rservo.writeMicroseconds(BWR);  
    navigation->Lservo.writeMicroseconds(FWL); 
}

void go_straight() {
  navigation->Rservo.writeMicroseconds(FWR);  
  navigation->Lservo.writeMicroseconds(FWL);  
}

void reverse() {
  navigation->Rservo.writeMicroseconds(BWR_SLOW);  
  navigation->Lservo.writeMicroseconds(BWL_SLOW);  
}

void roll_right()
{
  navigation->Rservo.writeMicroseconds(BWR_FULL);  
  navigation->Lservo.writeMicroseconds(FWL_FULL);
  delay(10); 
  navigation->Rservo.writeMicroseconds(N); 
  delay(20); 
}

void roll_left()
{
  navigation->Rservo.writeMicroseconds(FWR_FULL);  
  navigation->Lservo.writeMicroseconds(BWL_FULL);
  delay(55); 
  navigation->Lservo.writeMicroseconds(N); 
}

int angleToTurn(int currentHeadingAngle, int angleToTurn)
{
  if ((currentHeadingAngle + angleToTurn) <= 360) {
    return currentHeadingAngle + angleToTurn;
  } else {
    return ((currentHeadingAngle + angleToTurn) - 360);
  }
  return 0;
}

void walldetected() {
  if ((get_mTOF() < frontTOFLimit)||(get_tlTOF() < topLevel_longRangeTOFLimit)) {
    while ((get_mTOF() < frontTOFMinimum)||(get_tlTOF() < topLevel_longRangeTOFLimit)){
      Serial.println("Stuck Here 1");
      allTOFReadings();
      turn_left();
    }
    set_weight_detected_bool(true);
  }
}

bool reachedDesiredHeadingAngle(int desiredAngle)
{
  int current_angle = get_headingAngle(0); // Getting the current heading angle in the x direction (0) - y-direction = 1, z-direction = 2
  int angle_to_turn = abs(current_angle - desiredAngle);
  if (angle_to_turn < 10) {
    return true;
  }
  return false;
}

void check_stuck_condition() {
    // Check if robot is stuck in the corner
    //Serial.print("Stuck");
    uint16_t frontLeft = get_tlTOF();
    uint16_t frontRight = get_trTOF();
    uint16_t middle = get_mTOF();

    // Check if all front sensors detect an object close
    if (frontLeft < topLevel_longRangeTOFLimit && frontRight < topLevel_longRangeTOFLimit) {
        if (!isStuck) {
            stuckStartTime = millis(); // Start timer
            Serial.println("Stuck1");
            isStuck = true;
        } else if (millis() - stuckStartTime >= STUCK_THRESHOLD) {
            reverse();
            delay(2000);
            Serial.println("Stuck2");
            isStuck = false; // Reset stuck state after reversing
        }
    } else {
        isStuck = false; // Reset if not stuck
    }
}

void general_navigation()
{
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
  else {
    go_straight(); 
  }
}

void weightDetection(bool direction)
{
  uint16_t tr = get_trTOF();
  uint16_t br = get_brTOF();
  uint16_t tl = get_tlTOF();
  uint16_t bl = get_blTOF();

  if (direction) {
    while ((tr - br) > weightDetectingDistance) {
      turn_right();

      allTOFReadings();
      allUSValues();
      tr = get_trTOF();
      br = get_brTOF();      
    }
  } else {
    while ((tl - bl) > weightDetectingDistance) {
      turn_left();

      allTOFReadings();
      allUSValues();
      tl = get_tlTOF();
      bl = get_blTOF();
    }
  }
}

void wallFollowing()
{
  allTOFReadings();
  allUSValues();

  uint16_t mTOF = get_mTOF();
  uint32_t r_us = get_rUS();
  uint16_t trTOF = get_trTOF();

  /* Following wall on the right */
  while (get_weight_detected_bool() == false) {
    walldetected();
    Serial.println("Stuck Here");
    allTOFReadings();
    allUSValues();
    go_straight();
  }

  if (navigation->walldetected_bool == true) {
    if ((mTOF < frontTOFMinimum) || ((mTOF < frontTOFMinimum) && (r_us <= rUSLimit)) ||
        (get_trTOF() < topLevel_longRangeTOFWFLimit)) {
      int desired_angle = angleToTurn(get_headingAngle(0), angleToTurnDuringWallFollowing);
      while (reachedDesiredHeadingAngle(desired_angle) == false) {
        Serial.println("Stuck Here 1");
        allTOFReadings();
        //allUSValues();
        mTOF = get_mTOF();
        r_us = get_rUS();
        // int current_angle = get_headingAngle(0); // Getting the current heading angle in the x direction (0) - y-direction = 1, z-direction = 2
        // int desired_angle = angleToTurn(current_angle, angleToTurnDuringWallFollowing);
        turn_left();
        imu_loop();
        
      }
    }
    else if ((mTOF > frontTOFLimit) && (r_us > rUSLimit) && (trTOF > topLevel_longRangeTOFLimit)) {
      int desired_angle = angleToTurn(get_headingAngle(0), angleToTurnDuringWallFollowing);
      while (reachedDesiredHeadingAngle(desired_angle) == false) {
        // (r_us > WallfollowingLimit) && (trTOF > topLevel_longRangeTOFWFLimit) && (mTOF > frontTOFLimit)
        Serial.println("Stuck Here 2");
        
        allTOFReadings();
        allUSValues();

        mTOF = get_mTOF();
        r_us = get_rUS();
        trTOF = get_trTOF();
        turn_right();
        imu_loop();
      }
    } else {
      go_straight();
    }
  }
}

void nav_loop(bool weight_detected)
{
  uint16_t tr = get_trTOF();
  uint16_t br = get_brTOF();
  uint16_t tl = get_tlTOF();
  uint16_t bl = get_blTOF();
  Serial.print("Checking Conditions");
  // check_stuck_condition();
  if (((tr - br) > weightDetectingDistance) && (br < weightDetectingDistanceMax)) {
    //Serial.print("Gotcha1");
    weightDetection(1);
    set_weight_detected_bool(true);
  } else if (((tl - bl) > weightDetectingDistance) && (bl <  weightDetectingDistanceMax)) {
    //Serial.print("Gotcha2");
    weightDetection(0);
    set_weight_detected_bool(true);
  }
  else {
    if (weight_detected == false) {
      navigation->start_weight_detection = millis();
      set_weight_detected_bool(true);
      // Serial.println("Timer Started");
    } else {
      int end_weight_detection = millis();
      // Serial.print("Timer Ended");
      // Serial.print("\t");
      // Serial.println(end_weight_detection - navigation->start_weight_detection);
      if (end_weight_detection - navigation->start_weight_detection >  maxStraightLineTravelTime) {
        int desired_angle = angleToTurn(get_headingAngle(0), angleToTurnDuringFindingWeight);
        
        // Serial.print("Finding Desired Angle:");
        // Serial.print("\t");
        // Serial.print(desired_angle);
        // Serial.print("\t");
        // Serial.print("Current Angle:");
        // Serial.print("\t");
        // Serial.print(get_headingAngle(0));
        // Serial.print("\t");
        // Serial.print("Has it performed a turn?: ");
        // Serial.print("\t");
        // Serial.println(reachedDesiredHeadingAngle(desired_angle));

        while (reachedDesiredHeadingAngle(desired_angle) == false) {
          if (((tr - br) > weightDetectingDistance) && (br < weightDetectingDistanceMax)) {
            //Serial.print("Gotcha1");
            set_weight_detected_bool(true);
            break;
          } else if (((tl - bl) > weightDetectingDistance) && (bl <  weightDetectingDistanceMax)) {
            //Serial.print("Gotcha2");
            set_weight_detected_bool(true);
            break;
          } else {
            // Serial.println("Scanning...");
            turn_right();
            
            allTOFReadings();
            tr = get_trTOF();
            br = get_brTOF();
            tl = get_tlTOF();
            bl = get_blTOF();

            imu_loop();
          }
        }
        set_weight_detected_bool(false);
        turn_right();
      }
    }
    general_navigation();
  }
}

void set_weight_detected_bool(bool state)
{
  navigation->weight_detcted_bool = state;
}

bool get_weight_detected_bool()
{
  return navigation->weight_detcted_bool;
}