#include <Servo.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>
#define VL53L0X_ADDRESS_START 0x30
Servo myservo;  // create servo object to control a servo
Servo drum;
// twelve servo objects can be created on most boards
const byte SX1509_ADDRESS = 0x3F;
const uint8_t sensorCount = 1;
const uint8_t xshutPins[1] = {2};
float state_w, distance, distance_left, distance_right, tof_distance,front_tof;
int weights_collected = 0;
int sensor = A7;
int third_slot  = 180;   
int second_slot  = 135;
int first_slot = 89;
int third_slot_discard = 76;
int second_slot_discard = 36;        
int first_slot_discard = 0 ;  
int state_holder = 1;
unsigned long Timer1,Timer2;  
SX1509 io;
VL53L0X sensors[sensorCount];



void rotateDrum(int start,int dest) {
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

void discard_all_weights() {

  rotateDrum(third_slot,third_slot_discard); 
  delay(2500);
  rotateDrum(third_slot_discard,second_slot_discard); 
  delay(2500);
  rotateDrum(second_slot_discard,first_slot_discard);                             
  delay(2500); 
  rotateDrum(first_slot_discard,first_slot);                             
  delay(2500); 
}


void init_tof() {
  Serial.begin(115200);
  pinMode(sensor, INPUT);
  io.begin(SX1509_ADDRESS);
  Wire.begin();
  Wire.setClock(400000);
   for (uint8_t i = 0; i < sensorCount; i++)
  {
    io.pinMode(xshutPins[i], OUTPUT);
    io.digitalWrite(xshutPins[i], LOW);
  }
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    io.digitalWrite(xshutPins[i], HIGH);
    delay(10);
    sensors[i].setTimeout(500);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }
    sensors[i].setAddress(VL53L0X_ADDRESS_START + i);
    sensors[i].startContinuous(50);
  }
}
void setup() {
  myservo.attach(30);  // attaches the servo on pin 9 to the servo object
  myservo.write(89);  
  drum.attach(29);  // attaches the servo on pin 9 to the servo object
  drum.writeMicroseconds(1950);  
  init_tof();
 
}


void loop() {
  
  //discard_all_weights();
  
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    
    Serial.print(" Distance: Middle ");
    front_tof = sensors[i].readRangeContinuousMillimeters();
    Serial.println(front_tof);
    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    
  }
  int state = digitalRead(sensor);
  if ((state == LOW)) {
    int state_holder = 0;
  }
  else {
    state_holder = 1;
  }
  Serial.print(state_holder);
  if (front_tof < 90) {
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
  

  
 
  
