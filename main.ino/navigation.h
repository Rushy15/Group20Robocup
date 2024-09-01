#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Servo.h>
#include <VL53L0X.h>
#include <SparkFunSX1509.h>

#include <Arudino.h>
#include <Wire.h>

#include <cstdlib>
#include <cstdint>

class Navigation {
  public:
    Navigtation();
    void setup();
    void loop();
  
  private:
    static const int echoPinr = 2;
    static const int trigPinr = 3;
    static const int echoPinl = 4;
    static const int trigPinl = 5;
    
    float duration, distance, distance_left, distance_right, front_tof;

    static const byte SX1059_ADDRESS = 0x3F;
    static const uint8_t sensorCount = 1;
    static const uint8_t xshutPins[sensorCount] = {0};

    Servo Rservo;
    Servo Lservo;
    SX1509 io;
    VL53L0X sensors[sensorCount];

    float ping(int32_t trigPin, int32_t echoPin);

    void turn_left();
    void turn_right();
    void go_straight_slow()
}

#endif // NAVIGATION_H