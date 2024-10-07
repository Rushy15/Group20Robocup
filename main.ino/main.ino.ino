#include "navigation.h"
#include "sensors.h"
#include "storage.h"
#include "collection.h"
#include "imu.h"

bool isRemovingWeight = false;
bool colourDataCollected = false;

#define ENTRY_MIN 50
#define ENTRY_MAX 150
#define ENTRY2_MIN 30
#define ENTRY2_MAX 100

#define ANGLE_TO_TURN_DURING_UNLOADING 90 // Can assign a max value of 180 

void printingSensorValues()
{
  int l_us = get_lUS();
  int r_us =  get_rUS();
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

void printingColourData()
{
  uint16_t r = getR();
  uint16_t g = getG();
  uint16_t b = getB();

  Serial.print("\tR:\t"); 
  Serial.print(r);
  Serial.print("\tG:\t"); 
  Serial.print(g);
  Serial.print("\tB:\t"); 
  Serial.println(b);
}

void printingIMUData()
{
  if (imu_ptr->printCount * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) {
    //enough iterations have passed that we can print the latest data
    Serial.print("Heading: ");
    Serial.println(imu_ptr->orientationData.orientation.x);

    imu_ptr->printCount = 0;
  }
  else {
    imu_ptr->printCount = imu_ptr->printCount + 1;
  }
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

  navigation -> navigation_setup();
  sensor -> sensor_setup();
  storage -> storage_setup();
  collection -> collection_setup();
  imu_ptr -> imu_setup();

  Serial.println("Goodbye");
  
}

void loop() {
  allTOFReadings();
  imu_loop();

  //printingSensorValues();
  printingIMUData();

  /* Getting colour of home base - One time loop at startup*/
  int start_collecting = (millis()) ? (colourDataCollected == false) : 0;
  while(colourDataCollected == false) {
    int end_collecting = millis();
    collectingColourData();
    printingColourData();
    if ((end_collecting - start_collecting) > 3000) {
      colourDataCollected = true;
    }
  }
  
  /* State Machine for the robot */  
  // nav_loop(navigation->weight_detcted_bool);

  
  /* Checking to see if a weight has entered the channel of the robot */
  if ((((get_entry() < ENTRY_MAX) && (get_entry() > ENTRY_MIN)) || ((get_entry2() < ENTRY2_MAX) && (get_entry2() > ENTRY2_MIN))) 
        && !isRemovingWeight) {  /* Only check if not currently removing */
      isRemovingWeight = true;
      // navigation -> go_straight();
      delay(500);
      // navigation -> stop();
      delay(500);
      int start = millis();
      int end;
      while (get_barrel() > 100) {
          allTOFReadings();
          spinDrum();
          storage->psState = read_psState();
          end = millis();
          if ((end - start) > 14000) {  /* Check to see if nothing has been collected in 14 seconds */
            while ((end - start) < 16000) { /* Reverse the drum and robot for (14 - 12) = 2 seconds */
              allTOFReadings();
              end = millis();
              reverseDrum();
              // navigation -> reverse();
              isRemovingWeight = false;
            }
            isRemovingWeight = false;
            stopDrum();
            break;
          }
      }
  }

  /* Checking to see if the weight has entered the barrel */
  if (get_barrel() < 100 && isRemovingWeight) {
      navigation -> stop();
      stopDrum();
      storing(storage->psState);
      Serial.print("passing");
      isRemovingWeight = false;  /* Reset flag once the barrel has returned */
  }

  // /* Checking to see if the robot has collected three weights and is at full capacicty*/
  while (max_capacity()) {
    wallFollowing();
    updateColourValues();
    printingColourData();
    if (inHomeBase()) {
      navigation->go_straight();
      delay(1000);
      int current_angle = get_headingAngle(0); // Getting the current heading angle in the x direction (0) - y-direction = 1, z-direction = 2
      int desired_angle = angleToTurn(current_angle, ANGLE_TO_TURN_DURING_UNLOADING);
      int angle_to_turn = abs(current_angle - desired_angle);

      // Serial.print("Current Angle");
      // Serial.print(current_angle);
      // Serial.print('\t');
      // Serial.print("Desired Angle");
      // Serial.print(desired_angle);
      // Serial.print('\t');
      // Serial.print("Difference");
      // Serial.println(angle_to_turn);

      navigation -> stop();
      while (reachedDesiredHeadingAngle(desired_angle) == false) {
        navigation->turn_left();
        imu_loop();
        // current_angle = get_headingAngle(0);
        // angle_to_turn = abs(current_angle - desired_angle);
        
        // Serial.print("Current Angle in Loop");
        // Serial.println(current_angle);
      } 
      navigation -> stop();
      reset_capacity();
    }
  }
}