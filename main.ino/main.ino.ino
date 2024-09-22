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

  int l_us = sensor -> lUS;
  int r_us = sensor -> rUS;

  int m_tof = *(sensor -> mTOF);
  int bl_tof = *(sensor -> blTOF); 
  int tl_tof = *(sensor -> tlTOF); 
  int br_tof = *(sensor -> brTOF); 
  int tr_tof = *(sensor -> trTOF);

  int entry = *(sensor -> entry);
  int barrel = *(sensor -> barrel);

  Serial.print("Middle TOF: ");
  Serial.print(m_tof);
  Serial.print('\t');
  Serial.print("Entry: ");
  Serial.print(entry);
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

  navigation->loop();
}
