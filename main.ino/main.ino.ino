#include "navigation.h"
#include "sensors.h"
#include "storage.h"
#include "collection.h"
bool isRemovingWeight = false;
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
  Serial.println(barrel);
  Serial.println('\t');
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
  // Serial.println("Goodbye");
}

void loop() {
  // put your main code here, to run repeatedly:
  allTOFReadings();
  allUSValues();
  // Serial.print(weight);
  printingSensorValues();
  //navigation -> go_straight();
  /* State Machine for the Robot - Slow, but working */
  nav_loop();
  isRemovingWeight = false; 
  if ((((get_entry() < 180)&&(get_entry() > 50))||((get_entry2() < 180)&&(get_entry2() > 20))) && !isRemovingWeight){  // Only check if not currently removing
      isRemovingWeight = true;
      delay(1000);
      navigation -> stop();
      int start = millis();
      int end;
      while (get_barrel() > 100) {
          allTOFReadings();
          spinDrum();
          psState = read_psState();
          end = millis();
          if ((end - start) > 12000) {
            while ((end - start) < 14000) {
              end = millis();
              reverseDrum();
              navigation -> reverse();
            }
            stopDrum();
            break;
          }
      }
  }

  if (get_barrel() < 100 && isRemovingWeight) {
      navigation -> stop();
      stopDrum();
      storing(psState);
      Serial.print("passing");
      isRemovingWeight = false;  // Reset flag once the barrel has returned
  }
}
