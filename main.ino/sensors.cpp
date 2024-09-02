#include sensors.h

#define IRQ_PIN 2
#define XSHUT_PIN 3
#define VL53L0X_ADDRESS_START 0x30

Sensors::Sensors() : io(SX1509_ADDRESS) {}

Sensors::setup(){
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);

  Rservo.attach(20);
  Lservo.attach(8);

  Serial.begin(115200);
  io.being(SX1509_ADDRESS);
  Wire.begin();
  Wire.setClock(400000);

  for (uint8_t i = 0; i < sensorCount; i++) {
    io.pinMode(xshutPins[i], OUTPUT);
    io.digitalWrite(xshutPins[i], LOW);
  }

  for (uint8_t i = 0; i < sensorCount; i++) {
    io.digitalWrite(xshutPins[i], HIGH);
    delay(10);
    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.print("Failed to detect and initialise sensor");
      Serial.println(i);
      while(1);
    }
    sensor[i].setAddress(VL53L0X_ADDRESS_START + i);
    sensors[i].startContinuous(50);
  }
}

// Function for reading and writing from ultrasonic sensors 
float Sensors::ping(int32_t trigPin, int32_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicrosconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration * .0343) / 2;
  return distance;
}

