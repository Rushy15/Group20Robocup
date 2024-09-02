#include "sensors.h"

#define IRQ_PIN 2
#define XSHUT_PIN 3
#define VL53L0X_ADDRESS_START 0x30

// Setup Function
void Navigation::setup(){
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);
  
  Rservo.attach(28);
  Lservo.attach(8);

  Serial.begin(115200);
  io.begin(SX1509_ADDRESS);
  Wire.begin();
  Wire.setClock(400000);
  
  for (uint8_t i = 0; i < sensorCount; i++) {
    io.pinMode(xshutPins[i], OUTPUT);
    io.digitalWrite(xshutPins[i], LOW);
  }
  
  for (uint8_t i = 0; i < sensorCount; i++)
  {
    io.digitalWrite(xshutPins[i], HIGH);
    delay(10);
    sensors[i].setTimeout(500);
    
    Serial.print(i);
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

