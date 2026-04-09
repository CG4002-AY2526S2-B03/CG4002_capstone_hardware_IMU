#include "LSM6DSRSensor.h"

// Struct forward declaration
struct IMU_Data;

// -------- CONSTANTS --------
extern const float accel_bias[3];
extern const float gyro_bias[3];
extern const float accel_correction[3][3];
extern const float imu_pos[3];
extern LSM6DSRSensor AccGyr;

// -------- FUNCTIONS --------
void calibrateAccel(const float accel[3], float calib[3]);
void computeRacketVelocity(float q0, float q1, float q2, float q3,
                           const float accel[3],
                           const float gyro[3],
                           const float pos[3],
                           const float dt,
                           IMU_Data *data);
float getDT();
void reinitIMU();
void resetI2CBus();