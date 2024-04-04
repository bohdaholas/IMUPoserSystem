#ifndef PERIPH_IMU_H
#define PERIPH_IMU_H

#include "BNO055ESP32.h"

class Imu {
public:
  void init();

  Imu();

    bno055_calibration_t getCalibration();

    bno055_vector_t getVectorEuler();

private:
  BNO055 bno;
};

inline Imu imu;

#endif //PERIPH_IMU_H
