
#include <Servo.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>

#define IRQ_PIN 2
#define XSHUT_PIN 3
#define VL53L0X_ADDRESS_START 0x30

#define frontTOFLimit 300
#define frontTOFMinimum 500
#define rUSLimit 20 
#define lUSLimit 20

#define N 1500

const int trigPinr = 3;
const int echoPinr = 2;
const int trigPinl = 5;
const int echoPinl = 4;

bool turn_direction = NULL;
int turn_counter = 0;
uint32_t my_time = 0;

const byte SX1509_ADDRESS = 0x3F;
const uint8_t sensorCount = 1;
const uint8_t xshutPins[sensorCount] = {0};
float duration, distance, distance_left, distance_right, tof_distance,front_tof;

Servo Rservo;
Servo Lservo;
SX1509 io;
VL53L0X sensors[sensorCount];

void setup() {
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);
  Rservo.attach(28);
  Lservo.attach(8);
  // Rservo.writeMicroseconds(1500);  
  // Lservo.writeMicroseconds(1500);  
  Serial.begin(115200);
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

//===============================================================================//
//                                    Logic                                      //
//===============================================================================//

// Function used to read ultrasonic readings
float ping(int32_t trigPin, int32_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  return distance;
}


/*
================================================|
 Right Motor:         | Left Motor:             |
--------------        | -------------           |
FW Full-Power = 1950  | FW Full-Power = 1050    |
REV Full-Power = 1050 | REV Full-Power = 1950   |
================================================|
*/

void get_my_time() {
  if (turn_counter == 1) {
    my_time = millis();
  }
}

bool stuck() {
  if (turn_counter > 5) {
    int end_time = millis();
    if ((end_time - my_time) < 3000) {
      return 1;
    }
  } 
  return 0;
}

void reverse_left() {
  Lservo.writeMicroseconds(1950);
  Rservo.writeMicroseconds(1950);   
}

void reverse_right() {
  Rservo.writeMicroseconds(1050);
  Lservo.writeMicroseconds(1200);
}

void turn_left() {
    if (turn_direction == 1) {
      turn_counter += 1;
    }
    Rservo.writeMicroseconds(1950);  
    Lservo.writeMicroseconds(1950); 
    turn_direction = 0;
    // Rservo.writeMicroseconds(N);  
    // Lservo.writeMicroseconds(N);  
}

void turn_right() {
  if (turn_direction == 0) {
      turn_counter += 1;
    }
    Rservo.writeMicroseconds(1050);  
    Lservo.writeMicroseconds(1050);
    turn_direction = 1;
    // Rservo.writeMicroseconds(N);  
    // Lservo.writeMicroseconds(N);   
}

void go_straight() {
  Rservo.writeMicroseconds(1900);  
  Lservo.writeMicroseconds(1100);
  // Rservo.writeMicroseconds(N);  
  // Lservo.writeMicroseconds(N);    
}

/*
================= 
  MAIN FUNCTION 
=================
*/
void loop() {
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    distance_left = ping(trigPinl, echoPinl); // Left US Sensor Reading
    front_tof = sensors[i].readRangeContinuousMillimeters();
    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    distance_right = ping(trigPinr, echoPinr); // Right US Sensor Reading
    // Serial.print(" Left: ");
    // Serial.print(distance_left);
    // Serial.print('\t');
    
    // Serial.print(" Middle: ");
    // Serial.print(front_tof);
    // Serial.print('\t');
    
    // Serial.print(" Right: ");
    // Serial.println(distance_right);
    // Serial.print('\t');
  }
  
  get_my_time();

  if (stuck()) {
    if (turn_direction == 0) {
      reverse_left();
      delay(1000);
    } else {
      reverse_right();
      delay(1000);
    }
  }

  if (front_tof < frontTOFLimit){///units in mm
    if (distance_right < distance_left){
      while (front_tof < frontTOFMinimum){
        for (uint8_t i = 0; i < sensorCount; i++){
            front_tof = sensors[i].readRangeContinuousMillimeters();
            if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
            }
      turn_left();
      }
    }
    else if (distance_right > distance_left){
      while (front_tof < frontTOFMinimum){
        for (uint8_t i = 0; i < sensorCount; i++) {
            front_tof = sensors[i].readRangeContinuousMillimeters();
            if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }   
      }
      turn_right();
      }
    }
  }
  else if (distance_right < lUSLimit) {///units in cm
    turn_left();
  }

   else if (distance_left < rUSLimit) {///units in cm
    turn_right();
  }

  else {
    go_straight();
    turn_counter = 0;
  }
  
  delay(50);
  
}