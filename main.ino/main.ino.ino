#include "navigation.h"
#include "sensors.h"

void setup() {
  // put your setup code here, to run once:
  if (sensor == nullptr) {
    sensor = new Sensors();
  }

  if (navigation == nullptr) {
    navigation = new Navigation();
  } 

  navigation -> navigation_setup();
  sensor -> sensor_setup();
  Serial.println("Goodbye");

}

void loop() {
  // put your main code here, to run repeatedly:
  sensor -> allTOFReadings();
  sensor -> us_Values();

  // int l_us = sensor_main -> lUS;
  // int r_us = sensor_main -> rUS;

  int m_tof = *(sensor -> mTOF);
  int bl_tof = *(sensor -> blTOF); 
  int tl_tof = *(sensor -> tlTOF); 
  int br_tof = *(sensor -> brTOF); 
  int tr_tof = *(sensor -> trTOF);

  int entry = *(sensor -> entry);
  int barrel = *(sensor -> barrel);

  Serial.print(m_tof);
  Serial.print('\t');
  Serial.print(entry);
  Serial.print('\t');
  Serial.print(barrel);
  Serial.print('\t');
  
  Serial.print(bl_tof);
  Serial.print('\t');
  Serial.print(tl_tof);
  Serial.print('\t');
  Serial.print(br_tof);
  Serial.print('\t');
  Serial.println(tr_tof);
  // navigation.loop();
}
