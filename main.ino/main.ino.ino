#include "navigation.h"
#include "sensors.h"
#include "storage.h"
#include "collection.h"

bool isRemovingWeight = false;

#define ENTRY_MIN 50
#define ENTRY_MAX 150
#define ENTRY2_MIN 30
#define ENTRY2_MAX 100

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
  // int start = millis();
  // nav_loop();
  // if (weight_entered()) {
  //   collect_weight();
  //   storing();
  //   if (weightInBarrel()) {
  //     if (ps == 1) {
  //       store_weight();
  //     } else {
  //       remove_weight();
  //     }
  //   }  
  // }
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

  navigation -> navigation_setup();
  sensor -> sensor_setup();
  storage -> storage_setup();
  collection -> collection_setup();
  Serial.println("Goodbye");
  
}

void loop() {
  allTOFReadings();
  //allUSValues();
  
  printingSensorValues();
  
  // State Machine for the robot
  nav_loop();
  
  
  if ((((get_entry() < ENTRY_MAX) && (get_entry() > ENTRY_MIN)) || ((get_entry2() < ENTRY2_MAX) && (get_entry2() > ENTRY2_MIN))) 
        && !isRemovingWeight) {  // Only check if not currently removing
      isRemovingWeight = true;
      navigation -> go_straight();
      delay(1000);
      navigation -> stop();
      delay(500);
      int start = millis();
      int end;
      while (get_barrel() > 100) {
          allTOFReadings();
          spinDrum();
          //nav_loop();
          storage->psState = read_psState();
          end = millis();
          if ((end - start) > 14000) { // Check to see if nothing has been collected in 12 seconds
            while ((end - start) < 16000) { // Reverse the drum and robot for (14 - 12) = 2 seconds
              end = millis();
              reverseDrum();
              navigation -> reverse();
              isRemovingWeight = false;
            }
            stopDrum();
            break;
          }
      }
  }

  if (get_barrel() < 100 && isRemovingWeight) {
      navigation -> stop();
      stopDrum();
      storing(storage->psState);
      Serial.print("passing");
      isRemovingWeight = false;  // Reset flag once the barrel has returned
  }

  
  
  
  
  // while (max_capacity()) {
  //   wallFollowing();
  //   if (/*ADD COLOUR SENSOR CONDITION CODE*/) {
  //     reset_capacity();
  //   }
  // }
  
  // updateColourValues();

  // uint16_t r = getR();
  // uint16_t g = getR();
  // uint16_t b = getR();
  // Serial.print("\tR:\t"); 
  // Serial.print(r);
  // Serial.print("\tG:\t"); 
  // Serial.print(g);
  // Serial.print("\tB:\t"); 
  // Serial.println(b);
  // if (get_barrel() < 100 && isRemovingWeight) {
  //     navigation -> stop();
  //     stopDrum();
  //     storing(psState);
  //     Serial.print("passing");
  //     isRemovingWeight = false;  // Reset flag once the barrel has returned
  // }
}
