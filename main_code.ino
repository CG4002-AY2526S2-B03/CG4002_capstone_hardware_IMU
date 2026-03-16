// Includes
#include "LSM6DSRSensor.h"
#include "MahonyAHRS.h"
#include "Mqtt.h"
#include "config.h"

#define DEBUG  // Enable debug output

// -------- PIN CONFIGURATIONS -------
#define IMU_INT_PIN 14
const int buttonPins[4] = { 25, 26, 27, 13 };
#define NMOS_GATE_PIN 2  // NMOS gate control pin

// ------------ CONSTANTS ------------
const float accel_bias[3] = { 9.625, -14.477, 13.271 };   // from Magneto
const float gyro_bias[3] = { 232.28, -536.55, -255.03 };  // from calibration code
const float accel_correction[3][3] = {                    // A⁻¹ (correction matrix) from Magneto
  { 1.015291, 0.001008, 0.001879 },
  { 0.001008, 1.009009, 0.004430 },
  { 0.001879, 0.004430, 1.011325 }
};
const float imu_pos[3] = { 0.0, -0.10, 0.0 };  // IMU position relative to racket center (in m)
#define DRIFT_DECAY 0.995                      // Dright decay (HPF) - reduces integration drift for velocity calculation
#define DEBOUNCE_DELAY 200

// --------- GLOBAL VARIABLES ----------
#define SerialPort Serial
LSM6DSRSensor AccGyr(&Wire, LSM6DSR_I2C_ADD_L);
Mahony filter;
volatile int buttonPressed = -1;

// ----------- QUEUE HANDLES -----------
QueueHandle_t imuQueue;
QueueHandle_t buttonQueue;

// ---------- DATA STRUCTURES ----------
struct Position {
  float pitch;
  float yaw;
  float roll;
};
struct Velocity {
  float x_vel;
  float y_vel;
  float z_vel;
};
struct IMU_Data {
  struct Position position;
  struct Velocity velocity;
};

// ---------- TASK HANDLES -------------
TaskHandle_t imuTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;

// ------- FUNCTION DECLARATIONS --------
void IRAM_ATTR imu_isr();
void calibrateAccel(const float raw[3], float calib[3]);
void computeRacketVelocity(float q0, float q1, float q2, float q3,
                           const float accel[3],
                           const float gyro[3],
                           const float pos[3],
                           const float dt,
                           struct IMU_Data *data);
float getDT();
void IRAM_ATTR imu_isr();
void IRAM_ATTR handleButton0();
void IRAM_ATTR handleButton1();
void IRAM_ATTR handleButton2();
void IRAM_ATTR handleButton3();

void mqttTask(void *pvParameters) {
  IMU_Data data;
  int buttonEvent;

  while (true) {
    if (xQueueReceive(imuQueue, &data, 0) == pdTRUE) {
      Serial.print("Orientation: ");
      Serial.print(data.position.pitch);
      Serial.print(", ");
      Serial.print(data.position.yaw);
      Serial.print(", ");
      Serial.println(data.position.roll);

      Serial.print("Velocity: ");
      Serial.print(data.velocity.x_vel);
      Serial.print(", ");
      Serial.print(data.velocity.y_vel);
      Serial.print(", ");
      Serial.println(data.velocity.z_vel);

      // if (mqttClient.isConnected()) {
      //     std::string payload = formatPayload(data);
      //     mqttClient.publish(playerEspPublishTopic, payload, 0, false);
      // }
    }

    if (xQueueReceive(buttonQueue, &buttonEvent, 0) == pdTRUE) {
      // if (mqttClient.isConnected()) {
      //     std::string payload = formatPayload(pos);
      //     mqttClient.publish(playerEspPublishTopic, payload, 0, false);
      // }
      Serial.print("Button ");
      Serial.print(buttonEvent + 1);
      Serial.println(" pressed");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void buttonTask(void *pvParameters) {
  static unsigned long lastPressTime = 0;
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    unsigned long now = millis();
    if (now - lastPressTime > DEBOUNCE_DELAY) {
      int pressed = buttonPressed;
      buttonPressed = -1;
#ifdef DEBUG
      Serial.print("Button ");
      Serial.print(pressed + 1);
      Serial.println(" pressed");
#endif
      // Toggle NMOS gate
      if (pressed == 0) {
        int currentState = digitalRead(NMOS_GATE_PIN);
        digitalWrite(NMOS_GATE_PIN, !currentState);
#ifdef DEBUG
        Serial.print("NMOS gate is now: ");
        Serial.println(!currentState ? "ON" : "OFF");
        Serial.println(digitalRead(NMOS_GATE_PIN));
#endif
      }
      xQueueSend(buttonQueue, &pressed, 0);
      lastPressTime = now;
    }
  }
}

void imuTask(void *pvParameters) {
  float accel_raw[3] = { 0, 0, 0 };   // raw accelerometer readings (mg)
  int32_t gyro_raw[3] = { 0, 0, 0 };  // raw gyroscope readings (mdps)

  float accel_calib[3] = { 0, 0, 0 };  // calibrated accelerometer (g)
  float gyro_calib[3] = { 0, 0, 0 };   // calibrated gyroscope (deg/s)

  struct IMU_Data data;
  static uint16_t sample_count = 0;

  while (true) {
    // wait for IMU interrupt
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // ============ 1. READ IMU DATA ============
    AccGyr.Get_X_Axes(accel_raw);
    AccGyr.Get_G_Axes(gyro_raw);

    // ============ 2. CALIBRATE IMU DATA ============
    // Magneto calibration for accel (in mg)
    calibrateAccel(accel_raw, accel_calib);
    // Data conversion
    for (int i = 0; i < 3; i++) {
      accel_calib[i] /= 1000.0f;                               // Accelerometer: mg → g
      gyro_calib[i] = (gyro_raw[i] - gyro_bias[i]) / 1000.0f;  // Gyroscope: mdps → deg/s
    }

    // ============ 3. UPDATE MAHONY FILTER ============
    float dt = getDT();
    filter.updateIMU(gyro_calib[0], gyro_calib[1], gyro_calib[2], accel_calib[0], accel_calib[1], accel_calib[2], dt);

    // ============ 4. GET DATA FROM MAHONY FILTER ============
    // get quarternion
    float q0, q1, q2, q3;
    filter.getQuaternion(q0, q1, q2, q3);

    // get orientation
    data.position.roll = filter.getRoll();
    data.position.pitch = filter.getPitch();
    data.position.yaw = filter.getYaw();

    // ============ 5. COMPUTE RACKET VELOCITY ============
    computeRacketVelocity(q0, q1, q2, q3,
                          accel_calib,
                          gyro_calib,
                          imu_pos,
                          dt,
                          &data);

#ifdef DEBUG
    sample_count++;
    if (sample_count >= 100) {  // print once every 100 samples
      sample_count = 0;
      Serial.print("Orientation: ");
      Serial.print(data.position.pitch);
      Serial.print(", ");
      Serial.print(data.position.yaw);
      Serial.print(", ");
      Serial.println(data.position.roll);

      Serial.print("Velocity: ");
      Serial.print(data.velocity.x_vel);
      Serial.print(", ");
      Serial.print(data.velocity.y_vel);
      Serial.print(", ");
      Serial.println(data.velocity.z_vel);
    }
#endif
    xQueueSend(imuQueue, &data, 0);
  }
}

void setup() {
  Serial.begin(115200);
  SerialPort.println("Setup started...");

  Wire.begin();
  Wire.setClock(400000);  // 400kHz

  if (AccGyr.begin() != LSM6DSR_OK) {
    Serial.println("error!!");
  }
  AccGyr.Enable_X();
  AccGyr.Enable_G();

  // Queues
  imuQueue = xQueueCreate(10, sizeof(IMU_Data));
  buttonQueue   = xQueueCreate(10, sizeof(int));

  // ===== HANDLE MQTT =====
  wifiConnect();
  mqttClient.setMqttClientName(clientID);
  mqttClient.enableLastWillMessage("/will", "esp32-client-player went offline", false);
  
  String mqttBrokerURL = String(mqtt_broker);
  mqttClient.setURL(mqttBrokerURL.c_str(), 8883, "", "");
  mqttClient.setCaCert(caCert);
  mqttClient.setClientCert(clientCert);
  mqttClient.setKey(clientKey);
  mqttClient.loopStart();

  // ===== CREATE TASKS =====
  xTaskCreatePinnedToCore(
    imuTask,
    "IMU Task",
    8192,
    NULL,
    2,
    &imuTaskHandle,
    1);
  xTaskCreatePinnedToCore(
    buttonTask,
    "Button Task",
    4096,
    NULL,
    1,
    &buttonTaskHandle,
    1);
  xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 4096, NULL, 1, NULL, 1);

  // Attach interrupt
  pinMode(IMU_INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), imu_isr, RISING);

  // buttons
  for (int i = 0; i < 4; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  pinMode(NMOS_GATE_PIN, OUTPUT);
  digitalWrite(NMOS_GATE_PIN, LOW);

  attachInterrupt(buttonPins[0], handleButton0, FALLING);
  attachInterrupt(buttonPins[1], handleButton1, FALLING);
  attachInterrupt(buttonPins[2], handleButton2, FALLING);
  attachInterrupt(buttonPins[3], handleButton3, FALLING);
}

void loop() {
}

/* ---------------------------------------------------------------------------------- */
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
  data->velocity.x_vel = temp[0];
  data->velocity.y_vel = temp[1];
  data->velocity.z_vel = temp[2];
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

  // Remove gravity in world frame
  a_world[2] -= 9.81f;

  // --- Step 4: Integrate to get world velocity with drift decay ---
  v_world[0] = v_world[0] * DRIFT_DECAY + a_world[0] * dt;
  v_world[1] = v_world[1] * DRIFT_DECAY + a_world[1] * dt;
  v_world[2] = v_world[2] * DRIFT_DECAY + a_world[2] * dt;

  // Zero velocity detection
  float accel_mag = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
  if (fabs(accel_mag - 1.0f) < 0.05f) {  // Within 0.05g of 1g = stationary
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

void IRAM_ATTR imu_isr() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(
    imuTaskHandle,
    &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR handleButtonISR(int btn) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  buttonPressed = btn;
  vTaskNotifyGiveFromISR(buttonTaskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR handleButton0() {
  handleButtonISR(0);
}
void IRAM_ATTR handleButton1() {
  handleButtonISR(1);
}
void IRAM_ATTR handleButton2() {
  handleButtonISR(2);
}
void IRAM_ATTR handleButton3() {
  handleButtonISR(3);
}