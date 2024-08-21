
#include <Servo.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>

#define IRQ_PIN 2
#define XSHUT_PIN 3
#define VL53L0X_ADDRESS_START 0x30

const int trigPinr = 3;
const int echoPinr = 2;
const int trigPinl = 5;
const int echoPinl = 4;

const byte SX1509_ADDRESS = 0x3F;
const uint8_t sensorCount = 1;
const uint8_t xshutPins[1] = {0};
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
  Rservo.attach(29);
  Lservo.attach(28);
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

void turn_left() {
    Rservo.writeMicroseconds(1950);  
    Lservo.writeMicroseconds(1950); 
}

void turn_right() {
    Rservo.writeMicroseconds(1050);  
    Lservo.writeMicroseconds(1050); 
}

void go_straight_slow() {
  Rservo.writeMicroseconds(1800);  
  Lservo.writeMicroseconds(1200);  
}
void loop() {
  // Serial.print(" Middle: ");
  // Serial.print(tof_distance);
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    distance_left = ping(trigPinl, echoPinl);
    Serial.print(" Distance: Left ");
    Serial.print(distance_left);
    Serial.print(" Distance: Middle ");
    front_tof = sensors[i].readRangeContinuousMillimeters();
    Serial.print(front_tof);
    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    distance_right = ping(trigPinr, echoPinr);
    Serial.print(" Right: ");
    Serial.println(distance_right);
  }
  
  if (front_tof < 250){///units in mm
    if (distance_right < distance_left){
      while (front_tof < 450){
        for (uint8_t i = 0; i < sensorCount; i++){
            front_tof = sensors[i].readRangeContinuousMillimeters();
            if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
            }
      turn_left();
      }
    }
    else if (distance_right > distance_left){
      while (front_tof < 450){
        for (uint8_t i = 0; i < sensorCount; i++) {
            front_tof = sensors[i].readRangeContinuousMillimeters();
            if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }   
      }
      turn_right();
      }
    }
  }
  else if (distance_right < 20) {///units in cm
    turn_left();
  }

   else if (distance_left < 20) {///units in cm
    turn_right();
  }

  else {
    go_straight_slow();
  }
  
  delay(50);
  
}