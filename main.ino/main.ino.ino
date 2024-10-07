#include "navigation.h"
#include "sensors.h"
#include "storage.h"
#include "collection.h"
#include "imu.h"

/* Enum that stores different states for the FSM */
typedef enum {
  IDLE = 0,
  NAVIGATION,
  COLLECTION,
  STORAGE,
  WALL_FOLLOWING
} RobotState_t;

RobotState_t currentState = IDLE; /* Initialise the current state of the FSM to IDLE */

bool isRemovingWeight = false;
bool colourDataCollected = false;

int start_collecting;

#define ENTRY_MIN 50
#define ENTRY_MAX 150
#define ENTRY2_MIN 30
#define ENTRY2_MAX 100

#define WEIGHT_IN_BARREL_LIMIT 100

#define MAX_COLLECTING_TIME 14000
#define REVERSE_SYSTEM_TIME 16000

#define ANGLE_TO_TURN_DURING_UNLOADING 90 // Can assign a max value of 180 


                                              /* Printing functions for Serial */
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

                                              /* Callback functions for tasks */

void updateColourData()
{
  /* Getting colour of home base - One time loop at startup*/
  start_collecting = millis();
  while(colourDataCollected == false) {
    Serial.println("Updating Colours");
    int end_collecting = millis();
    collectingColourData();
    printingColourData();
    Serial.print(end_collecting - start_collecting);
    if ((end_collecting - start_collecting) > 3000) {
      colourDataCollected = true;
    }
  }
}

void wallFollowingCallback()
{
  while (max_capacity()) {
    wallFollowing();
    updateColourValues();
    // printingColourData();
    if (inHomeBase()) {
      go_straight();
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

      stop();

      while (reachedDesiredHeadingAngle(desired_angle) == false) {
        turn_left();
        imu_loop();
        // current_angle = get_headingAngle(0);
        // angle_to_turn = abs(current_angle - desired_angle);
        
        // Serial.print("Current Angle in Loop");
        // Serial.println(current_angle);
      } 
      stop();
      reset_capacity();
    }
  }
}

/*
FSM for dealing with the different states the robot can be in
*/
void FSMHandler()
{
  switch (currentState) {
    case IDLE: // IDLE - One time call, used to read colour sensor data and initialise a homebase
      if (colourDataCollected == false) {
        updateColourData();
      }
      currentState = IDLE;
      break;
    
    case NAVIGATION: // NAVIGATION - Calls general navigation and checks for the condition where a weight has entered the channel
      nav_loop(get_weight_detected_bool());
      Serial.print("NAVIGATION");
      if ((((get_entry() < ENTRY_MAX) && (get_entry() > ENTRY_MIN)) || ((get_entry2() < ENTRY2_MAX) && (get_entry2() > ENTRY2_MIN))) 
        && !isRemovingWeight) {  /* Only check if not currently removing */
        isRemovingWeight = true;
        Serial.print("Condition Hit");
        go_straight();
        delay(500);
        stop();
        delay(500);
        
        currentState = COLLECTION;
        break;
      } else {
        Serial.print("Enabling General Navigation");
        currentState = NAVIGATION;
        Serial.println("Navigating 1");
        break;
      }
      Serial.println("Navigating 2");
      break;
    
    case COLLECTION: // COLLECTION - Used to spin the drum and has error checking incase the weight is stuck
      int start = millis();
      int end;
      while (get_barrel() > WEIGHT_IN_BARREL_LIMIT) {
        end = millis();
        allTOFReadings();
        spinDrum();
        set_psState(read_psState());
        if ((end - start) > MAX_COLLECTING_TIME) {  /* Check to see if nothing has been collected in 14 seconds */
          while ((end - start) < REVERSE_SYSTEM_TIME) { /* Reverse the drum and robot for (14 - 12) = 2 seconds */
            allTOFReadings();
            end = millis();
            reverseDrum();
            reverse();
            
            isRemovingWeight = false;
          }
          isRemovingWeight = false;
          stopDrum();
          currentState = NAVIGATION;
          break;
        }  
      }
      currentState = STORAGE;
      break;
    
    case STORAGE: // STORAGE - Stores/Removes the weight that has entered the barrel
      if (get_barrel() < WEIGHT_IN_BARREL_LIMIT && isRemovingWeight) {
        stop();
        stopDrum();
        storing(read_psState());
        Serial.print("passing");
        isRemovingWeight = false;  /* Reset flag once the barrel has returned */
      }
      if (max_capacity()) { // Only enters wall following mode when it is at max_capacity
        currentState = WALL_FOLLOWING;
        break;
      }
      currentState = NAVIGATION;
      break;
    
    case WALL_FOLLOWING:  // WALL_FOLLOWING - Peforms wall-following until the homebase is reached
      wallFollowingCallback();
      currentState = NAVIGATION;
      break;

    default:
      currentState = IDLE;
      break;
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
  
  sensor -> sensor_setup();
  navigation -> navigation_setup();
  storage -> storage_setup();
  collection -> collection_setup();
  imu_ptr -> imu_setup();

  Serial.println("Goodbye");
}

void loop() {
  Serial.print("Executing Task: ");
  Serial.println(currentState);
  delay(500);
  FSMHandler();
  delay(500);
  // printingSensorValues();
  // printingIMUData();
}