#include "imu_processing.h"
#include "mqtt.h" 

// ------------ CONSTANTS ------------
const float accel_bias[3] = { 9.625, -14.477, 13.271 };   // from Magneto
const float gyro_bias[3] = { 337.21, -694.25, -365.58 };  // from calibration code
const float accel_correction[3][3] = {                    // A⁻¹ (correction matrix) from Magneto
  { 1.015291, 0.001008, 0.001879 },
  { 0.001008, 1.009009, 0.004430 },
  { 0.001879, 0.004430, 1.011325 }
};
const float imu_pos[3] = { 0.0, -0.15, 0.0 };  // IMU position relative to racket center (in m)
#define DRIFT_DECAY 0.995                      // Dright decay (HPF) - reduces integration drift for velocity calculation

// ============ QUATERNION ROTATION FUNCTIONS ============
void quatRotate(float qw, float qx, float qy, float qz,
                float vx, float vy, float vz,
                float out[3]) {
  // Quaternion multiplication: v' = q * v * q_conj
  float px = qw * vx + qy * vz - qz * vy;
  float py = qw * vy + qz * vx - qx * vz;
  float pz = qw * vz + qx * vy - qy * vx;
  float pw = -qx * vx - qy * vy - qz * vz;

  out[0] = px * qw + pw * -qx + py * -qz - pz * -qy;
  out[1] = py * qw + pw * -qy + pz * -qx - px * -qz;
  out[2] = pz * qw + pw * -qz + px * -qy - py * -qx;
}
void quatRotateConjugate(float qw, float qx, float qy, float qz,
                         float vx, float vy, float vz,
                         struct IMU_Data *data) {
  // For inverse rotation, use conjugate: q_conj = (qw, -qx, -qy, -qz)
  float temp[3];
  quatRotate(qw, -qx, -qy, -qz, vx, vy, vz, temp);

  // Copy to struct
  data->velocity.x_vel = temp[2];
  data->velocity.y_vel = temp[0];
  data->velocity.z_vel = temp[1];
}
// Accelerometer calibration function from Magneto
void calibrateAccel(const float accel[3], float calib[3]) {
  // Subtract bias
  float tmp[3];
  tmp[0] = accel[0] - accel_bias[0];
  tmp[1] = accel[1] - accel_bias[1];
  tmp[2] = accel[2] - accel_bias[2];

  // Apply correction matrix
  calib[0] = accel_correction[0][0] * tmp[0] + accel_correction[0][1] * tmp[1] + accel_correction[0][2] * tmp[2];
  calib[1] = accel_correction[1][0] * tmp[0] + accel_correction[1][1] * tmp[1] + accel_correction[1][2] * tmp[2];
  calib[2] = accel_correction[2][0] * tmp[0] + accel_correction[2][1] * tmp[1] + accel_correction[2][2] * tmp[2];
}

void computeRacketVelocity(float q0, float q1, float q2, float q3,
                           const float accel[3],
                           const float gyro[3],
                           const float pos[3],
                           const float dt,
                           struct IMU_Data *data) {

  static float v_world[3] = { 0, 0, 0 };  // Velocity in world frame at IMU

  // --- Step 1: Convert accel from g to m/s² ---
  float accel_ms2[3];
  accel_ms2[0] = accel[0] * 9.81f;
  accel_ms2[1] = accel[1] * 9.81f;
  accel_ms2[2] = accel[2] * 9.81f;

  // --- Step 2: Convert gyro from deg/s to rad/s ---
  float gyro_rad[3];
  gyro_rad[0] = gyro[0] * DEG_TO_RAD;  // 0.0174533f
  gyro_rad[1] = gyro[1] * DEG_TO_RAD;
  gyro_rad[2] = gyro[2] * DEG_TO_RAD;

  // --- Step 3: Rotate acceleration from body to world frame ---
  float a_world[3];
  quatRotate(q0, q1, q2, q3, accel_ms2[0], accel_ms2[1], accel_ms2[2], a_world);

  float gravity_body[3];
  // Rotate world gravity into body frame using conjugate
  quatRotate(q0, -q1, -q2, -q3, 0.0f, 0.0f, 9.81f, gravity_body);
  // Subtract in body frame before rotation
  float accel_corrected[3] = {
      accel_ms2[0] - gravity_body[0],
      accel_ms2[1] - gravity_body[1],
      accel_ms2[2] - gravity_body[2]
  };
  quatRotate(q0, q1, q2, q3, 
            accel_corrected[0], accel_corrected[1], accel_corrected[2], 
            a_world);

  // --- Step 4: Integrate to get world velocity with drift decay ---
  v_world[0] = v_world[0] * DRIFT_DECAY + a_world[0] * dt;
  v_world[1] = v_world[1] * DRIFT_DECAY + a_world[1] * dt;
  v_world[2] = v_world[2] * DRIFT_DECAY + a_world[2] * dt;

  // Zero velocity detection
  float accel_mag = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
  if (fabs(accel_mag - 1.0f) < 0.04f) {  // Within 0.04g of 1g = stationary
    v_world[0] *= 0.5f;
    v_world[1] *= 0.5f;
    v_world[2] *= 0.5f;
  }

  // --- Step 5: Lever arm effect (body frame) ---
  // v_rot = ω × r (using rad/s)
  float v_rot_body_x = gyro_rad[1] * pos[2] - gyro_rad[2] * pos[1];
  float v_rot_body_y = gyro_rad[2] * pos[0] - gyro_rad[0] * pos[2];
  float v_rot_body_z = gyro_rad[0] * pos[1] - gyro_rad[1] * pos[0];

  // Rotate rotational velocity to world frame
  float v_rot_world[3];
  quatRotate(q0, q1, q2, q3, v_rot_body_x, v_rot_body_y, v_rot_body_z, v_rot_world);

  // --- Step 6: Combine velocities in world frame ---
  float v_racket_world_x = v_world[0] + v_rot_world[0];
  float v_racket_world_y = v_world[1] + v_rot_world[1];
  float v_racket_world_z = v_world[2] + v_rot_world[2];

  // --- Step 7: Transform back to body frame (relative to IMU) ---
  quatRotateConjugate(q0, q1, q2, q3,
                      v_racket_world_x, v_racket_world_y, v_racket_world_z,
                      data);
}

float getDT() {
  static unsigned long last = micros();
  unsigned long now = micros();
  float dt = (now - last) / 1000000.0;
  last = now;
  if (dt > 0.1) dt = 0.01;  // Prevent spikes after pause
  return dt;
}

void reinitIMU() {
    if (AccGyr.begin() != LSM6DSR_OK) {
    Serial.println("error!!");
  }
  AccGyr.Enable_X();
  AccGyr.Enable_G();
}

void resetI2CBus() {
  pinMode(SDA, OUTPUT_OPEN_DRAIN);
  pinMode(SCL, OUTPUT_OPEN_DRAIN);

  digitalWrite(SDA, HIGH);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);

  // Clock out 9 pulses to release stuck slave
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL, HIGH);
    delayMicroseconds(5);
  }

  // Generate STOP condition
  digitalWrite(SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA, HIGH);

  // Restore I2C
  Wire.begin();
  Wire.setClock(400000);
}