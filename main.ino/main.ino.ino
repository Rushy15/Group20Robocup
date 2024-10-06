#include <TaskScheduler.h> // https://github.com/arkhipenko/TaskScheduler/tree/master

#include "navigation.h"
#include "sensors.h"
#include "storage.h"
#include "collection.h"
#include "imu.h"

/* Enum that stores different states for the FSM */
enum State {
  IDLE = 0,
  NAVIGATION,
  COLLECTION,
  STORAGE,
  WALL_FOLLOWING,
};

State currentState = IDLE; /* Initialise the current state of the FSM to IDLE */
Scheduler ts; /* Creating a task scheduler variable */

bool isRemovingWeight = false;
bool colourDataCollected = false;

#define ENTRY_MIN 50
#define ENTRY_MAX 150
#define ENTRY2_MIN 30
#define ENTRY2_MAX 100

#define WEIGHT_IN_BARREL_LIMIT 100

#define MAX_COLLECTING_TIME 14000
#define REVERSE_SYSTEM_TIME 16000

#define ANGLE_TO_TURN_DURING_UNLOADING 90 // Can assign a max value of 180 

/* Creating local pointers to the different classes */
Sensors *sensor_ptr;
Navigation *navigation_ptr;
Storage *storage_ptr;
Collection *collection_ptr;
IMU *imu_ptr;

/*
--------------------------------------------------------------------------------------------------------------------------
| FORMAT: Task tTaskName(period, iterations, callbackFunction, scheduler, enabled, onEnableCallback, onDisableCallback); |
--------------------------------------------------------------------------------------------------------------------------
period (in milliseconds):
-------------------------
The interval or delay between task executions. If period = 10000L, the task will run every 10 seconds (10000 milliseconds).
If period = 0, the task runs as fast as possible (continuous execution).
You can modify this later with setInterval().

iterations (number of times the task runs):
-------------------------------------------
This indicates how many times the task will execute before stopping.
TASK_FOREVER means the task will keep running indefinitely.
TASK_ONCE means the task will run only once and then stop.
You can also specify a specific number like 5 to run the task 5 times.

callbackFunction (the function to execute):
-------------------------------------------
This is a pointer to the function that will be called each time the task is run. It should match the required signature void callbackFunction().
If no function is provided, you can pass NULL, which is often the case when using onEnable or onDisable methods to control the task’s behavior.

scheduler (the task scheduler object):
--------------------------------------
The scheduler object that manages the task. All tasks must be associated with a Scheduler instance, in this case, &ts.
This is important for ensuring that tasks are properly managed and can run concurrently or sequentially.

enabled (whether the task starts enabled):
------------------------------------------
A boolean (true or false) indicating if the task should start enabled when the program begins.
true: The task will start running immediately when the program starts.
false: The task will be created but not start until manually enabled via enable() or restart().

onEnableCallback (optional callback function when task is enabled):
-------------------------------------------------------------------
This function is called once when the task is enabled. It is optional but allows you to set up the task’s behavior when it first starts or restarts.
If no callback is needed, you can pass NULL.

onDisableCallback (optional callback function when task is disabled):
---------------------------------------------------------------------
This function is called when the task finishes or is disabled. It is used to clean up, stop hardware, or reset values.
If no callback is needed, you can pass NULL.
*/

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

void updateColourDataCallback()
{
  /* Getting colour of home base - One time loop at startup*/
  int start_collecting = millis();
  while(colourDataCollected == false) {
    int end_collecting = millis();
    collectingColourData();
    printingColourData();
    if ((end_collecting - start_collecting) > 3000) {
      colourDataCollected = true;
    }
  }
}

void generalNavCallback()
{
  /* State Machine for the robot */
  nav_loop(get_weight_detected_bool());
}

void collectWeightCallback()
{
  allTOFReadings();
  spinDrum();
  set_psState(read_psState());
}

void storeWeightCallback()
{
  stop();
  stopDrum();
  storing(read_psState());
  Serial.print("passing");
  isRemovingWeight = false;  /* Reset flag once the barrel has returned */
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
  -> Defining the different tasks used within the scheduler
  -> Each task is defined as a 'Callback' function that will be called at a specific interval
  -> Look at above description to understand the format for initialising tasks *
*/

Task updateTOFData(300, TASK_FOREVER, &allTOFReadings, &ts, true);
Task updateUSData(1000, TASK_FOREVER, &allUSValues, &ts, false);
Task updateIMUData(300, TASK_FOREVER, &imu_loop, &ts, false);
Task updateColourData(3000, TASK_ONCE, &updateColourDataCallback, &ts, true);

Task generalNav(3000, TASK_FOREVER, &generalNavCallback, &ts, false);
Task wallFollowingNav(10000, TASK_FOREVER, &wallFollowingCallback, &ts, false);

Task collectWeight(16000, TASK_FOREVER, &collectWeightCallback, &ts, false);
Task storeWeight(2000, TASK_FOREVER, &storeWeightCallback, &ts, false);

/*
Helper Function for disabling all tasks
*/
void disableAllTasks() {
    // updateTOFData.disable();
    updateUSData.disable();
    updateIMUData.disable();
    updateColourData.disable();

    generalNav.disable();
    wallFollowingNav.disable();

    collectWeight.disable();
    storeWeight.disable();
}

/*
Helper Function for disabling all tasks except for a specified task - 'task'
*/
void disableAllTasksExcept(Task &task) {
    disableAllTasks();
    task.enable();
}

/*
FSM for dealing with the different states the robot can be in
*/
void FSMHandler()
{
  switch (currentState) {
    case 0: { // IDLE - One time call, used to read colour sensor data and initialise a homebase
      disableAllTasks();
      if (colourDataCollected == false) {
        updateColourData.enable();
      }
      currentState = NAVIGATION;
      disableAllTasks();
      break;
    }
    case 1: { // NAVIGATION - Calls general navigation and checks for the condition where a weight has entered the channel
      disableAllTasksExcept(generalNav);

      if ((((get_entry() < ENTRY_MAX) && (get_entry() > ENTRY_MIN)) || ((get_entry2() < ENTRY2_MAX) && (get_entry2() > ENTRY2_MIN))) 
        && !isRemovingWeight) {  /* Only check if not currently removing */
        isRemovingWeight = true;
        
        go_straight();
        delay(500);
        stop();
        delay(500);
        
        currentState = COLLECTION;
        disableAllTasks();
        break;
    }
    case 2: { // COLLECTION - Used to spin the drum and has error checking incase the weight is stuck
      int start = millis();
      int end;
      while (get_barrel() > WEIGHT_IN_BARREL_LIMIT) {
        end = millis();
        collectWeight.enable();
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
      disableAllTasks();
      break;
    }
    case 3: { // STORAGE - Stores/Removes the weight that has entered the barrel
      disableAllTasks();
      if (get_barrel() < WEIGHT_IN_BARREL_LIMIT && isRemovingWeight) {
        storeWeight.enable();
      }
      disableAllTasks();
      if (max_capacity()) { // Only enters wall following mode when it is at max_capacity
        currentState = WALL_FOLLOWING;
        break;
      }
      currentState = NAVIGATION;
      break;
    }
    case 4: { // WALL_FOLLOWING - Peforms wall-following until the homebase is reached
      disableAllTasksExcept(wallFollowingNav);
      currentState = NAVIGATION;
      break;
    }
  }
}
}

void setup() {
  // put your setup code here, to run once:
  sensor_ptr -> sensor_setup();
  navigation_ptr->navigation_setup();
  storage_ptr -> storage_setup();
  collection_ptr -> collection_setup();
  imu_ptr -> imu_setup();

  ts.addTask(updateUSData);
  ts.addTask(updateIMUData);
  ts.addTask(updateColourData);
  ts.addTask(generalNav);
  ts.addTask(wallFollowingNav);
  ts.addTask(collectWeight);
  ts.addTask(storeWeight);

  Serial.println("Goodbye");
}

void loop() {
  FSMHandler();

  ts.execute();
  
  // printingSensorValues();
  // printingIMUData();
}