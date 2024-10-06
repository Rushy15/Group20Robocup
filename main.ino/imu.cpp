#include "imu.h"

/* Check I2C device address and correct line below (by default address is 0x29 or 0x28)
                                   id, address 
*/
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

IMU *imu_ptr_1 = new IMU();

void IMU::imu_setup() {
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
    while (1);
  }

  delay(1000);
}

void imu_loop(void)
{
  unsigned long tStart = micros();
  imu_ptr_1->bno.getEvent(&(imu_ptr_1->orientationData), Adafruit_BNO055::VECTOR_EULER);
  // bno.getEvent(&angVelData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  while ((micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
  {
    //poll until the next sample is ready
  }
}

int get_headingAngle(int direction)
{
  switch(direction) {
    case 0:
      return imu_ptr_1->orientationData.orientation.x;
    case 1:
      return imu_ptr_1->orientationData.orientation.y;
    case 2:
      return imu_ptr_1->orientationData.orientation.z;
  }
  return 0;
}