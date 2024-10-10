#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

static uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
static uint16_t PRINT_DELAY_MS = 50; // how often to print the data

// static double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

class IMU {
  protected:

  public:
    uint16_t printCount = 0; //counter to avoid printing every 10MS sample
    sensors_event_t orientationData;
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

    void imu_setup();
};

void imu_loop();
int get_headingAngle(int direction);

extern IMU *imu_ptr;
#endif // IMU_H