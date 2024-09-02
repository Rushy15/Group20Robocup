#include "navigation.h"

#define IRQ_PIN 2
#define XSHUT_PIN 3
#define VL53L0X_ADDRESS_START 0x30

#define frontTOFLimit 300
#define frontTOFMinimum 500
#define rUSLimit 20 
#define lUSLimit 20

#define N 1500

static const uint8_t sensorCount = 1;

// Constructor
// Navigation::Navigation();// NOT SURE ABOUT THIS

// Setup Function
void Navigation::setup(){
  Serial.print("set up 0");
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);
  Serial.print("set up 1");
  Rservo.attach(20);
  Lservo.attach(8);

  Serial.begin(115200);
  io.begin(SX1509_ADDRESS);
  Wire.begin();
  Wire.setClock(400000);
  Serial.print("set up 2");
  for (uint8_t i = 0; i < sensorCount; i++) {
    io.pinMode(xshutPins[i], OUTPUT);
    io.digitalWrite(xshutPins[i], LOW);
  }
  Serial.println("set up 3");
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    io.digitalWrite(xshutPins[i], HIGH);
    delay(10);
    sensors[i].setTimeout(500);
    Serial.print("set up 4");
    Serial.print(i);
    if (!sensors[i].init())
    {
      Serial.print("Failed to detect and initialize sensor ");
      Serial.println(i);
      while (1);
    }
    Serial.print("set up 5");
    sensors[i].setAddress(VL53L0X_ADDRESS_START + i);
    sensors[i].startContinuous(50);
  }
  
}

// Function for reading and writing from ultrasonic sensors 
float Navigation::ping(int32_t trigPin, int32_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration * .0343) / 2;
  return distance;
}

void Navigation::turn_left() {
    Rservo.writeMicroseconds(1950);  
    Lservo.writeMicroseconds(1950); 
}

void Navigation::turn_right() {
    Rservo.writeMicroseconds(1050);  
    Lservo.writeMicroseconds(1050); 
}

void Navigation::go_straight() {
  Rservo.writeMicroseconds(1900);  
  Lservo.writeMicroseconds(1100);  
}

void Navigation::loop() {
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    distance_left = ping(trigPinl, echoPinl); // Left US Sensor Reading
    
    front_tof = sensors[i].readRangeContinuousMillimeters();

    if (sensors[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    distance_right = ping(trigPinr, echoPinr); // Right US Sensor Reading

    Serial.print(" Left: ");
    Serial.print(distance_left);
    Serial.print('\t');
    
    Serial.print(" Middle: ");
    Serial.print(front_tof);
    Serial.print('\t');
    
    Serial.print(" Right: ");
    Serial.println(distance_right);
    Serial.print('\t');
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
  }
  
  delay(50);
}