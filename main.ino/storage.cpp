#include "sensors.h"
#include "storage.h"
// #include "navigation.h"

Storage *storage = new Storage();

void Storage::colour_sensor_setup()
{
  Serial.println("Color View Test!");

  if (tcs.begin()) 
  Wire1.begin();

  if (tcs.begin(0x29, &Wire1)){
    Serial.println("Found sensor");
  } else 
  {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void Storage::storage_setup() {
  colour_sensor_setup();
  myservo.attach(30);  // attaches the servo on pin 9 to the servo object
  myservo.write(93);//93  
  // drum.writeMicroseconds(1950);  
  weights_collected = 0;
  //attachInterrupt(0, storing(), CHANGE);
}

void Storage::rotateDrum(int start, int dest) {
  int pos; //0   
  // Serial.println("Looping");
  if (start < dest){
  for (pos = start; pos <= dest; pos += 2) {
    myservo.write(pos);    
    delay(15);                      
  }         
}
  else if (start > dest){
  for (pos = start; pos >= dest; pos -= 2  ) {
    // Serial.println(pos);
    myservo.write(pos);   
    delay(15);                      
  }       
}
  
}

void Storage::discard_all_weights() {

  rotateDrum(third_slot,third_slot_discard); 
  delay(1000);
  rotateDrum(third_slot_discard,second_slot_discard); 
  delay(1000);
  rotateDrum(second_slot_discard,first_slot_discard);                             
  delay(1000); 
  rotateDrum(first_slot_discard,first_slot);                             
  delay(1000); 
}

void Storage::storeWeights()
{
      Serial.print(millis()-Timer1);
      switch (weights_collected) {
        case 0:
          delay(500);
          Serial.println("First real weight detected");
          rotateDrum(first_slot, second_slot);
          delay(500);
          weights_collected += 1;
          psState = 1;
          finished_storing = 1;
          break;
        case 1:
          delay(500);
          Serial.println("2nd real weight detected");
          rotateDrum(second_slot, third_slot);
          delay(500);
          weights_collected += 1;
          psState = 1;
          break;
        case 2:
          delay(500);
          Serial.println("3rd real weight detected...Go home");
          delay(500);
          psState = 1;
          delay(2500);
          // discard_all_weights();
          weights_collected = 3;
          break;
      }
}

void Storage::removeWeights(int Timer1) 
{
  while  ((millis() - Timer1) < 500){
     if ((millis()-Timer1) > 450){
      Serial.print(millis()-Timer1);
      switch (weights_collected) {
      case 0:
        delay(1000);
        Serial.println("Fake weight detected1");
        rotateDrum(first_slot,first_slot_discard);
        delay(1000);
        rotateDrum(first_slot_discard,first_slot);
        delay(1500);
        finished_storing = 1;
        break;
      case 1:
        delay(1000);
        Serial.println("Fake weight detected2");
        rotateDrum(second_slot,second_slot_discard);
        delay(1000);
        rotateDrum(second_slot_discard,second_slot);
        delay(500);
        break;
      case 2:
        delay(1000);
        Serial.println("Fake weight detected3");
        rotateDrum(third_slot,third_slot_discard);
        delay(1000);
        rotateDrum(third_slot_discard,third_slot);
        delay(500);
        break;
    }
    }
    }
}

void collectingColourData()
{
  updateColourValues();

  storage->red_homebase = storage->red;
  storage->blue_homebase = storage->blue;
  storage->green_homebase = storage->green;
}

void updateColourValues()
{
  storage->tcs.setInterrupt(false);      // turn on LED
  delay(60);  // takes 50ms to read 
  storage->tcs.getRawData(&(storage->red), &(storage->green), &(storage->blue), &(storage->clear));
  storage->tcs.setInterrupt(true);  // turn off LED
}

bool inHomeBase()
{
  // Measured colour values
  uint16_t r = storage->red;
  uint16_t b = storage->blue;
  uint16_t g = storage->green;
  // Desired colour values of Homebase
  uint16_t rh = storage->red_homebase;
  uint16_t bh = storage->blue_homebase;
  uint16_t gh = storage->green_homebase;

  if (((r >= (rh - 10)) && (r <= (rh + 10))) &&
     ((b >= (bh - 10)) && (b <= (bh + 10))) &&
     ((g >= (gh - 10)) && (g <= (gh + 10)))) {
      return true;
     } else {
      return false;
     }
}

bool max_capacity()
{
  if (storage->weights_collected == 3) {
    return true;
  } else {
    return false;
  }
}

void reset_capacity()
{
  storage->discard_all_weights();
  storage->weights_collected = 0;
  set_weight_detected_bool(false);
}

void storing(uint8_t proximityState)
{ 
  storage->Timer1 = millis();
  int Timer1 = storage->Timer1; 
  storage->finished_storing = 0;
  // Serial.print(psState);
  
  if (storage->psState == 0) {
    // navigation -> stop();
    // nav_loop();
    storage->storeWeights();
  } else {
    // navigation -> stop();
    // nav_loop();
    storage->removeWeights(Timer1);
  }
}

uint16_t getR()
{
  // Serial.print("\tR:\t"); Serial.print(storage->r);
  return storage->red;
} 

uint16_t getG()
{
  // Serial.print("\tG:\t"); Serial.print(storage->g);
  return storage->green;
} 

uint16_t getB()
{
  // Serial.println("\tB:\t"); Serial.print(storage->b);
  return storage->blue;
}

int read_psState()
{
  uint8_t state = digitalRead(storage->sensor); // Gets the state that the proximity sensor senses
  if ((state == LOW)) {
    state = 0;
  }
  else {
    state = 1;
  }
  return state;
}

void set_psState(int state)
{
  storage->psState = state;
}

uint8_t get_psState()
{
  return storage->psState;
}