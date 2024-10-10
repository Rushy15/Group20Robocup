#include "navigation.h"
#include "sensors.h"
#include "storage.h"
#include "imu.h"
#include "collection.h"

#define N 1500 // Neutral Speed

#define FWR 1850
#define FWL 1150
#define BWR 1150  
#define BWL 1850

#define FWR_SLOW 1750
#define FWL_SLOW 1250
#define BWR_SLOW 1250  
#define BWL_SLOW 1750

#define FWR_FULL 1950
#define FWL_FULL 1050
#define BWR_FULL 1050
#define BWL_FULL 1950

#define ENTRY_MIN 50
#define ENTRY_MAX 150
#define ENTRY2_MIN 30
#define ENTRY2_MAX 100

#define STUCK_THRESHOLD 2500 // Time in milliseconds to consider stuck
#define REVERSE_DURATION 2000 // Duration to reverse in milliseconds
#define HOME_BASE_THRESHOLD 5000 // Time in milliseconds to stay at home base

#define rUSLimit 20
#define lUSLimit 15
#define frontTOFLimit 300
#define frontTOFWFLimit 350 // Wall following limit
#define frontTOFMinimum 400
#define frontTOFWFMinimum 450
#define WallfollowingLimit 25
#define weightDetectingDistance 130 // Difference between long range TOFs to turn the robot if a weight is detected
#define maxStraightLineTravelTime 4000
#define weightDetectingDistanceMax 1200

#define topLevel_longRangeTOFLimit 170
#define topLevel_longRangeTOFWFLimit 300

#define angleTurningTolerance 6
#define angleToTurnDuringWallFollowing 90
#define angleToTurnDuringFindingWeight 345

unsigned long stuckStartTime = 0;
unsigned long homeBaseStartTime = 0;
bool isStuck = false;
bool atHomeBase = false;

Navigation *navigation = nullptr;

void Navigation::navigation_setup()
{
  // Motor Setup
  Rservo.attach(28);
  Lservo.attach(7);
}

void stop()
{
  navigation->Rservo.writeMicroseconds(N);  
  navigation->Lservo.writeMicroseconds(N);
}

void turn_left() {
    navigation->Rservo.writeMicroseconds(FWR_FULL);  
    navigation->Lservo.writeMicroseconds(BWL_FULL);
}

void turn_left_slow() {
    navigation->Rservo.writeMicroseconds(FWR_SLOW);
    navigation->Lservo.writeMicroseconds(BWL_SLOW);
}

void turn_right() {
    navigation->Rservo.writeMicroseconds(BWR_FULL);  
    navigation->Lservo.writeMicroseconds(FWL_FULL); 
}

void turn_right_slow() {
    navigation->Rservo.writeMicroseconds(BWR_SLOW);  
    navigation->Lservo.writeMicroseconds(FWL_SLOW); 
}

void go_straight() {
  navigation->Rservo.writeMicroseconds(FWR);  
  navigation->Lservo.writeMicroseconds(FWL);
}

void go_straight_full() {
  navigation->Rservo.writeMicroseconds(FWR_FULL);  
  navigation->Lservo.writeMicroseconds(FWL_FULL);
}

void reverse() {
  navigation->Rservo.writeMicroseconds(BWR);  
  navigation->Lservo.writeMicroseconds(BWL);  
}

void reverse_full() {
  navigation->Rservo.writeMicroseconds(BWR_FULL);  
  navigation->Lservo.writeMicroseconds(BWL_FULL);  
}

void roll_right()
{
  navigation->Rservo.writeMicroseconds(BWR_FULL);  
  navigation->Lservo.writeMicroseconds(FWL_FULL);
  delay(55); 
  navigation->Rservo.writeMicroseconds(N);
  // delay(20);
}

void roll_left()
{
  navigation->Rservo.writeMicroseconds(FWR_FULL);  
  navigation->Lservo.writeMicroseconds(BWL_FULL);
  delay(55); 
  navigation->Lservo.writeMicroseconds(N); 
}

void shake()
{
  reverse_full();
  delay(80);
  go_straight_full();
  delay(80);
  turn_left();
  delay(80);
  turn_right();
  delay(80);
}

int angleToTurn(int currentHeadingAngle, int angleToTurn, int directionOfTurn)
{
  if (directionOfTurn == 0) { // Robot is turning left
    int difference = (currentHeadingAngle - angleToTurn);
    if (difference > 0) {
      return difference;
    } else {
      return 360 - abs(difference);
    }
  }

  if (directionOfTurn == 1) { // Robot Turning Right
    int difference = currentHeadingAngle + angleToTurn;
    if (difference <= 359) {
    return difference;
  } else {
      return (difference - 360);
    }
  }

  return currentHeadingAngle;
}

int angleToTurn_WF(int currentHeadingAngle, int angleToTurn, int directionOfTurn)
{
  imu_loop();
  if ((currentHeadingAngle > 5) && (currentHeadingAngle < 85)) {
    if (directionOfTurn == 0) { // Robot is turning left
      return 90;
    } else {
      return 0;
    }
  } else if ((currentHeadingAngle > 95) && (currentHeadingAngle < 175)) {
    if (directionOfTurn == 0) { // Robot is turning left
      return 180;
    } else {
      return 90;
    }
  } else if ((currentHeadingAngle > 185) && (currentHeadingAngle < 265)) {
    if (directionOfTurn == 0) { // Robot is turning left
      return 270;
    } else {
      return 180;
    }
  } else if ((currentHeadingAngle > 275) && (currentHeadingAngle < 355)) {
    if (directionOfTurn == 0) { // Robot is turning left
      return 0;
    } else {
      return 270;
    }
  }
}

bool perpendicularToWall(int currentHeadingAngle)
{
  if (
     ((currentHeadingAngle >= 355) && (currentHeadingAngle <= 5)) ||
     ((currentHeadingAngle >= 85) &&  (currentHeadingAngle <= 95)) ||
     ((currentHeadingAngle >= 175) && (currentHeadingAngle <= 185))||
     ((currentHeadingAngle >= 265) && (currentHeadingAngle <= 275))
     ) {
      return true;
  } else {
    return false;
  }
}

void walldetected() 
{
  if ((get_mTOF() < frontTOFLimit)||(get_tlTOF() < topLevel_longRangeTOFLimit)||(get_trTOF() < topLevel_longRangeTOFLimit)) {
    while ((get_mTOF() < frontTOFWFLimit)||(get_tlTOF() < topLevel_longRangeTOFWFLimit)){
      Serial.println("Have not detcted a wall for wall following yet");
      allTOFReadings();
      turn_left();
    }
    set_wall_detected_bool(true);
  }
}

bool reachedDesiredHeadingAngle(int desiredAngle)
{
  imu_loop();
  int current_angle = get_headingAngle(0); // Getting the current heading angle in the x direction (0) - y-direction = 1, z-direction = 2
  int angle_to_turn = abs(current_angle - desiredAngle);
  if (angle_to_turn < angleTurningTolerance) {
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

void weight_entered_entry() {
  if ((((get_entry() < ENTRY_MAX) && (get_entry() > ENTRY_MIN)) || ((get_entry2() < ENTRY2_MAX) && (get_entry2() > ENTRY2_MIN))) 
        && !(navigation -> isRemovingWeight)) {  /* Only check if not currently removing */
      navigation -> isRemovingWeight = true;
      go_straight();
      delay(500);
      stop();
      delay(500);
      int start = millis();
      int end;
      while (get_barrel() > 100) {
          allTOFReadings();
          spinDrum();
          int current_psState = read_psState();
          set_psState(current_psState);
          // Serial.print(get_psState());
          end = millis();
          if ((end - start) > 14000) {  /* Check to see if nothing has been collected in 14 seconds */
            while ((end - start) < 20000) {
              end = millis();
              shake();
              Serial.print("Jam it around");
              int current_psState = read_psState();
              set_psState(current_psState);
              if (get_barrel() < 100) {
                storing(get_psState());
              }
              allTOFReadings();
            }
            end = millis();
            while ((end - start) < 22000) { /* Reverse the drum and robot for (14 - 12) = 2 seconds */
              allTOFReadings();
              end = millis();
              reverseDrum();
              reverse();
              navigation -> isRemovingWeight = false;
              Serial.print("Reversing");
            }
            navigation -> isRemovingWeight = false;
            stopDrum();
            break;
          }
      }
  }
}

void general_navigation()
{
  int mTOF = get_mTOF();
  int tr_tof = get_trTOF();
  int tl_tof = get_tlTOF();

  if (mTOF < frontTOFLimit){ //units in mm
    navigation->start_weight_detection = millis();
    if (tr_tof < tl_tof){
      while (mTOF < frontTOFMinimum){
        // weight_entered_entry();
        allTOFReadings();
        mTOF = get_mTOF();
        turn_left();
      }
    }
    else if (tr_tof > tl_tof){
      while (mTOF < frontTOFMinimum){
        // weight_entered_entry();
        allTOFReadings();
        mTOF = get_mTOF();
        turn_right();
      }
    }
  } 
  else if (get_trTOF() < topLevel_longRangeTOFLimit) {///units in cm
    navigation->start_weight_detection = millis();
    turn_left();
  } 
  else if (get_tlTOF() < topLevel_longRangeTOFLimit) {///units in cm
    navigation->start_weight_detection = millis();
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
      weight_entered_entry();
      turn_right();
      allTOFReadings();
      allUSValues();
      tr = get_trTOF();
      br = get_brTOF();      
    }
  } else {
    while ((tl - bl) > weightDetectingDistance) {
      weight_entered_entry();
      turn_left();
      allTOFReadings();
      allUSValues();
      tl = get_tlTOF();
      bl = get_blTOF();
    }
  }
}

void wallFollowingRight()
{
  allTOFReadings();
  allUSValues();

  uint16_t mTOF = get_mTOF();
  uint32_t r_us = get_rUS();
  uint16_t trTOF = get_trTOF();

  /* Following wall on the right */
  while (navigation->walldetected_bool == false) {
    walldetected();
    allTOFReadings();
    // allUSValues();
    
    go_straight();
    Serial.println("Stuck Here");
  }
  stopDrum();

  if (navigation->walldetected_bool == true) {
    allTOFReadings();
    if ((perpendicularToWall(get_headingAngle(0))) && (get_mTOF() > frontTOFWFLimit)) {
      go_straight();
      // Serial.println("Going straight during Wall Following (1)");
    }
    else {
        allTOFReadings();
        if ((mTOF < frontTOFLimit) || ((mTOF < frontTOFLimit) && (r_us <= rUSLimit))) {
          int desired_angle = angleToTurn_WF(get_headingAngle(0), angleToTurn, 0);
            while ((reachedDesiredHeadingAngle(desired_angle) == false) && (get_mTOF() < frontTOFMinimum)) {
              // if (perpendicularToWall(get_headingAngle(0))) {
              //   break;
              // }
              turn_left();
              imu_loop();
              allTOFReadings();

              Serial.print("Desired Heading Angle: ");
              Serial.print("\t");
              Serial.print(desired_angle);
              Serial.print("\t");
              Serial.print("Current Heading Angle: ");
              Serial.print("\t");
              Serial.print(get_headingAngle(0));
              Serial.print("\t");
              Serial.println("Turning left during Wall Following");
            }
            // stop();
            delay(500);
            // imu_loop();
        } else if ((mTOF > frontTOFLimit) && (r_us > rUSLimit) && (trTOF > topLevel_longRangeTOFLimit)) {
          // go_straight();
          // delay(1000);
          int desired_angle = angleToTurn_WF(get_headingAngle(0), angleToTurn, 1);
          while (reachedDesiredHeadingAngle(desired_angle) == false) {
            // if (perpendicularToWall(get_headingAngle(0))) {
            //     break;
            //   }
            turn_right();
            imu_loop();
            allTOFReadings();

            Serial.print("Desired Heading Angle: ");
            Serial.print("\t");
            Serial.print(desired_angle);
            Serial.print("\t");
            Serial.print("Current Heading Angle: ");
            Serial.print("\t");
            Serial.print(get_headingAngle(0));
            Serial.print("\t");
            Serial.println("Turning right during Wall Following");
          }
          stop();
          delay(500);
          // imu_loop();
        } else {
          go_straight();
          // Serial.print("Desired Heading Angle: ");
          // Serial.print("\t");
          // Serial.print(desired_angle);
          // Serial.print("\t");
          // Serial.print("Current Heading Angle: ");
          // Serial.print("\t");
          // Serial.print(get_headingAngle(0));
          // Serial.print("\t");
          // Serial.println("Going straight during Wall Following (2)");
        }
    }
  }
}

void nav_loop(bool weight_detected)
{
  uint16_t tr = get_trTOF();
  uint16_t br = get_brTOF();
  uint16_t tl = get_tlTOF();
  uint16_t bl = get_blTOF();
  
  if (((tr - br) > weightDetectingDistance) && (br < weightDetectingDistanceMax)) {
    //Serial.print("Gotcha1");
    weightDetection(1);
    set_weight_detected_bool(true);
    check_stuck_condition();
    navigation->start_weight_detection = millis();
  } else if (((tl - bl) > weightDetectingDistance) && (bl <  weightDetectingDistanceMax)) {
    //Serial.print("Gotcha2");
    weightDetection(0);
    set_weight_detected_bool(true);
    check_stuck_condition();
    navigation->start_weight_detection = millis();
  }
  else {
    if (weight_detected == false) {
      navigation->start_weight_detection = millis();
      set_weight_detected_bool(true);
    } else {
      int end_weight_detection = millis();
      
      if (end_weight_detection - navigation->start_weight_detection >  maxStraightLineTravelTime) {
        int desired_angle = angleToTurn(get_headingAngle(0), angleToTurnDuringFindingWeight, 1);
        while (reachedDesiredHeadingAngle(desired_angle) == false) {
          if (((tr - br) > weightDetectingDistance) && (br < weightDetectingDistanceMax)) {
            set_weight_detected_bool(true);
            break;
          } else if (((tl - bl) > weightDetectingDistance) && (bl <  weightDetectingDistanceMax)) {
            set_weight_detected_bool(true);
            break;
          } else {
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
      }
    }
    general_navigation();
    check_stuck_condition();
  }
}

void set_wall_detected_bool(bool state)
{ 
  navigation->walldetected_bool = state;
}

void set_weight_detected_bool(bool state)
{
  navigation->weight_detcted_bool = state;
}

bool get_weight_detected_bool()
{
  return navigation->weight_detcted_bool;
}

void set_isRemovingWeight_bool(bool state)
{
  navigation -> isRemovingWeight = state;
}

bool get_isRemovingWeight_bool()
{
  return navigation -> isRemovingWeight;
}