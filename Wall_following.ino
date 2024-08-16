/*
 * HC-SR04 example sketch
 *
 * https://create.arduino.cc/projecthub/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-036380
 *
 * by Isaac100
 */
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
float duration, distance, distance_left, distance_right, tof_distance;

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
  Lservo.attach(29);
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
    Rservo.writeMicroseconds(1750);  
    Lservo.writeMicroseconds(1750); 
}

void slight_turn_left() {
    Rservo.writeMicroseconds(1675);  
    Lservo.writeMicroseconds(1675); 
}

void slight_turn_right() {
    Rservo.writeMicroseconds(1350);  
    Lservo.writeMicroseconds(1350); 
 }
void go_straight_slow() {
  Rservo.writeMicroseconds(1670);  
  Lservo.writeMicroseconds(1330);  
}
void loop() {
  
  delay(50);
  
  // Serial.print(" Middle: ");
  // Serial.print(tof_distance);
  
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    distance_left = ping(trigPinl, echoPinl);
    Serial.print(" Distance: Left ");
    Serial.print(distance_left);
    Serial.print(" Distance: Middle ");
    Serial.print(sensors[i].readRangeContinuousMillimeters());
    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    distance_right = ping(trigPinr, echoPinr);
    Serial.print(" Right: ");
    Serial.println(distance_right);
  }
  Rservo.writeMicroseconds(1950);  
  Lservo.writeMicroseconds(1950);  
  delay(1500);
  Rservo.writeMicroseconds(1950);  
  Lservo.writeMicroseconds(1050); 
  delay(1500);
  // if ((tof_distance <= 175)&&(tof_distance > -1)){
  //   turn_left();
  // }
  // else if (distance_right <= 30 ){
  //   slight_turn_left();
  // }
  // else if (distance_left <= 25){
  //   slight_turn_right();
  // }
  // else {
  //   go_straight_slow();
  // }
  // Rservo.writeMicroseconds(1500);  
  // Lservo.writeMicroseconds(1500);  
  // delay(5000);
  // Rservo.writeMicroseconds(1655);  
  // Lservo.writeMicroseconds(1350);  
  // delay(5000);
  
}