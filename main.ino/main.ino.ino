#include "navigation.h"
#include "sensors.h"
#include "storage.h"
#include "collection.h"
#include "imu.h"

// bool isRemovingWeight = false;
bool colourDataCollected = false;
int start_collecting;

#define frontTOFLimit 300
#define frontTOFMinimum 450
#define ENTRY_MIN 50
#define ENTRY_MAX 150
#define ENTRY2_MIN 30
#define ENTRY2_MAX 100

#define WEIGHT_IN_BARREL_LIMIT 100

#define MAX_COLLECTING_TIME 14000
#define REVERSE_SYSTEM_TIME 16000

#define ANGLE_TO_TURN_DURING_UNLOADING 135  // Can assign a max value of 180
#define ANGLE_TO_TURN_IF_IN_ENEMYBASE 180   // Can assign a max value of 180


/* Printing functions for Serial */
void printingSensorValues() {
  int l_us = get_lUS();
  int r_us = get_rUS();
  int m_tof = get_mTOF();
  int bl_tof = get_blTOF();
  int tl_tof = get_tlTOF();
  int br_tof = get_brTOF();
  int tr_tof = get_trTOF();
  int entry = get_entry();
  int entry2 = get_entry2();
  int barrel = get_barrel();

  Serial.print("Middle TOF: ");
  Serial.print(m_tof);
  Serial.print('\t');
  Serial.print("Entry: ");
  Serial.print(entry);
  Serial.print('\t');
  Serial.print("Entry2: ");
  Serial.print(entry2);
  Serial.print('\t');
  Serial.print("Barrel: ");
  Serial.print(barrel);
  Serial.print('\t');
  Serial.print("Bottom Left: ");
  Serial.print(bl_tof);
  Serial.print('\t');
  Serial.print("Top Left: ");
  Serial.print(tl_tof);
  Serial.print('\t');
  Serial.print("Bottom Right: ");
  Serial.print(br_tof);
  Serial.print('\t');
  Serial.print("Top Right: ");
  Serial.print(tr_tof);
  Serial.print('\t');
  Serial.print("Left US: ");
  Serial.print(l_us);
  Serial.print('\t');
  Serial.print("Right US: ");
  Serial.println(r_us);
}

void printingColourData() {
  Serial.print("R: ");
  Serial.print(getR());
  Serial.print("\t");
  Serial.print("G: ");
  Serial.print(getG());
  Serial.print("\t");
  Serial.print("B: ");
  Serial.print(getB());
  Serial.print("\t");
  Serial.print("RH: ");
  Serial.print(storage->red_homebase);
  Serial.print("\t");
  Serial.print("GH: ");
  Serial.print(storage->green_homebase);
  Serial.print("\t");
  Serial.print("BH: ");
  Serial.print(storage->blue_homebase);
  Serial.print("\t");
  Serial.print("RE: ");
  Serial.print(storage->red_enemy);
  Serial.print("\t");
  Serial.print("GE: ");
  Serial.print(storage->green_enemy);
  Serial.print("\t");
  Serial.print("BE: ");
  Serial.println(storage->blue_enemy);
}

void printingIMUData() {
  if (imu_ptr->printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    Serial.print("Heading: ");
    Serial.println(imu_ptr->orientationData.orientation.x);

    imu_ptr->printCount = 0;
  } else {
    imu_ptr->printCount = imu_ptr->printCount + 1;
  }
}

void disposingWeightsLoop(int direction) {
  int current_angle = get_headingAngle(0);  // Getting the current heading angle in the x direction (0) - y-direction = 1, z-direction = 2
  int desired_angle = angleToTurn(current_angle, ANGLE_TO_TURN_DURING_UNLOADING, direction);
  go_straight();
  delay(500);
  stop();
  delay(500);
  // while (reachedDesiredHeadingAngle(desired_angle) == false) {
  //   turn_left();
  //   imu_loop();
  // }
  int mTOF = get_mTOF();
  int tr_tof = get_trTOF();
  int tl_tof = get_tlTOF();

  // if (mTOF < frontTOFLimit){ //units in mm // mTOF < frontTOFMinimum
  while (reachedDesiredHeadingAngle(desired_angle) == false) {
    // allTOFReadings();
    // mTOF = get_mTOF();
    turn_left_slow();
    imu_loop();
    // }
  }
  stop();
  delay(500);
  reset_capacity();
}

void getOutOfEnemyBase()
{
  int current_angle = get_headingAngle(0);          // Getting the current heading angle in the x direction (0) - y-direction = 1, z-direction = 2
  int direction = 1 ? (homeBaseColour() == 0) : 0;  // Returns 0 if homebase is
  int desired_angle = angleToTurn(current_angle, ANGLE_TO_TURN_IF_IN_ENEMYBASE, direction);
  stop();
  reverseDrum();
  delay(500);
  reverse();
  delay(1500);
  while (reachedDesiredHeadingAngle(desired_angle) == false) {
    if (direction == 0) {
      turn_left();
    } else {
      turn_right();
    }
    Serial.print("IN ENEMY BASE");
    imu_loop();
  }
  stopDrum();
}

void getOutOfHomeBase()
{
  int current_angle = get_headingAngle(0);   // Getting the current heading angle in the x direction (0) - y-direction = 1, z-direction = 2
  int direction = homeBaseColour();  // Returns 0 if homebase is green base, returns 1 if blue base
  int desired_angle = angleToTurn(current_angle, ANGLE_TO_TURN_IF_IN_ENEMYBASE, direction);
  stop();
  reverseDrum();
  delay(500);
  reverse();
  delay(1500);
  while (reachedDesiredHeadingAngle(desired_angle) == false) {
    if (direction == 0) {
      turn_left();
    } else {
      turn_right();
    }
    Serial.print("IN ENEMY BASE");
    imu_loop();
  }
  stopDrum();
}

void setup() {
  // put your setup code here, to run once:
  if (sensor == nullptr) {
    sensor = new Sensors();
  }

  if (navigation == nullptr) {
    navigation = new Navigation();
  }

  if (storage == nullptr) {
    storage = new Storage();
  }

  if (collection == nullptr) {
    collection = new Collection();
  }

  if (imu_ptr == nullptr) {
    imu_ptr = new IMU();
  }

  sensor->sensor_setup();
  navigation->navigation_setup();
  storage->storage_setup();
  collection->collection_setup();
  imu_ptr->imu_setup();

  Serial.println("Goodbye");
}

void loop() {
  allTOFReadings();
  updateColourValues();
  imu_loop();
  // printingSensorValues();
  // printingIMUData();

  /* Getting colour of home base - One time loop at startup */
  int start_collecting = (millis()) ? (colourDataCollected == false) : 0;
  while (colourDataCollected == false) {
    int end_collecting = millis();
    collectingColourData();
    printingColourData();
    if ((end_collecting - start_collecting) > 3000) {
      colourDataCollected = true;
      assignEnemyBaseRGB();
    }
  }

  /* State Machine for the robot */
  nav_loop(navigation->weight_detcted_bool);

  /* Checking to see if a weight has entered the channel of the robot */
  weight_entered_entry();

  /* Checking to see if the weight has entered the barrel */
  if (get_barrel() < 100 && get_isRemovingWeight_bool()) {
    stop();
    stopDrum();
    storing(get_psState());
    set_isRemovingWeight_bool(false); /* Reset flag once the barrel has returned */
  }

  if (inHomeBase()) {
    if (get_weightsCollected() == 0) {
      getOutOfHomeBase;
    } else if (get_weightsCollected() >= 1) {
      disposingWeightsLoop(homeBaseColour());  // Returns 0 if homebase is green base, returns 1 if blue base
    }
  }

  if (inEnemyBase()) {
    getOutOfEnemyBase();
  }

  // /* Checking to see if the robot has collected three weights and is at full capacicty*/
  while (max_capacity()) {
    imu_loop();
    wallFollowingRight();
    updateColourValues();
    // printingColourData();
    if (inHomeBase()) {
      stopDrum();
      disposingWeightsLoop(0);
      imu_loop();
    }
  }

  // updateColourValues();
  // printingColourData();
  // printingSensorValues();
  // wallFollowingRight();
}
