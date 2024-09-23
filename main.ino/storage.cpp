#include "sensors.h"
#include "storage.h"
// #include "navigation.h"

Storage *storage = nullptr;

void Storage::storage_setup() {
  myservo.attach(30);  // attaches the servo on pin 9 to the servo object
  myservo.write(93);  
  drum.attach(29);  // attaches the servo on pin 9 to the servo object
  drum.writeMicroseconds(1950);  
  
 
}

void Storage::rotateDrum(int start,int dest) {
  int pos = 0;   
  if (start < dest){
  for (pos = start; pos <= dest; pos += 2) {
    myservo.write(pos);    
    delay(15);                      
  }         
}
  else if (start > dest){
  for (pos = start; pos >= dest; pos -= 2  ) {
    myservo.write(pos);   
    delay(15);                      
  }       
}
}

void Storage::discard_all_weights() {

  rotateDrum(third_slot,third_slot_discard); 
  delay(2500);
  rotateDrum(third_slot_discard,second_slot_discard); 
  delay(2500);
  rotateDrum(second_slot_discard,first_slot_discard);                             
  delay(2500); 
  rotateDrum(first_slot_discard,first_slot);                             
  delay(2500); 
}

// void continueOperation()
// {
//   navigation->loop();
//   // sensor -> allTOFReadings();  
//   // sensor -> us_Values();
// }

void Storage::storing()
{
  int state = digitalRead(sensor);
  if ((state == LOW)) {
    int state_holder = 0;
  }
  else {
    state_holder = 1;
  }
  int enter_sensor = get_entry();
  Serial.println(state_holder);
  if (enter_sensor < 90) {
    Timer1 = millis();
    state = digitalRead(sensor);
    if ((state == LOW)) {
    state_holder = 0;
  }
  
    if ((state_holder == 0)) {
      Serial.print(millis()-Timer1);
      switch (weights_collected) {
        case 0:
          delay(1000);
          Serial.println("First real weight detected");
          delay(500);
          rotateDrum(first_slot,second_slot);
          delay(1000);
          weights_collected += 1;
          state_holder = 1;
          break;
        case 1:
          delay(1000);
          Serial.println("2nd real weight detected");
          delay(500);
          rotateDrum(second_slot,third_slot);
          delay(1000);
          weights_collected += 1;
          state_holder = 1;
          break;
        case 2:
          delay(1000);
          Serial.println("3rd real weight detected...Go home");
          delay(500);
          state_holder = 1;
          delay(2500);
          delay(2500);
          discard_all_weights();
          weights_collected = 0;
          break;
      }
     
    }
    while  ((millis()-Timer1)<500){
     if ((millis()-Timer1)>450){
      Serial.print(millis()-Timer1);
      switch (weights_collected) {
      case 0:
        delay(1000);
        Serial.println("Fake weight detected1");
        delay(500);
        rotateDrum(first_slot,first_slot_discard);
        delay(1000);
        rotateDrum(first_slot_discard,first_slot);
        delay(500);
        break;
      case 1:
        delay(1000);
        Serial.println("Fake weight detected2");
        delay(500);
        rotateDrum(second_slot,second_slot_discard);
        delay(1000);
        rotateDrum(second_slot_discard,second_slot);
        delay(500);
        break;
      case 2:
        delay(1000);
        Serial.println("Fake weight detected3");
        delay(500);
        rotateDrum(third_slot,third_slot_discard);
        delay(1000);
        rotateDrum(third_slot_discard,third_slot);
        delay(500);
        break;
    }
  
     }
    }
  }
}

