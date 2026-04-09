// Includes
#include "LSM6DSRSensor.h"
#include "MahonyAHRS.h"
#include "Mqtt.h"
#include "config.h"
#include "IMU_processing.h"

// #define DEBUG // enables print statements for debugging 
#define IMU_NOTIFY_TIMEOUT_MS 500   // 500 ms timeout

// -------- PIN CONFIGURATIONS -------
#define IMU_INT_PIN 14
const int buttonPins[4] = { 25, 26, 27, 13 };
#define NMOS_GATE_PIN 2  // NMOS gate control pin

// ------------ CONSTANTS ------------
#define DEBOUNCE_DELAY 200
#define VIBRATION_TIME 500

// --------- GLOBAL VARIABLES ----------
#define SerialPort Serial
LSM6DSRSensor AccGyr(&Wire, LSM6DSR_I2C_ADD_L);
Mahony filter;
bool hasGameStarted = true;

// ----------- QUEUE HANDLES -----------
QueueHandle_t imuQueue;
QueueHandle_t buttonQueue;
QueueHandle_t calibrationQueue;
QueueHandle_t motorQueue;

// ---------- TASK HANDLES -------------
TaskHandle_t imuTaskHandle = NULL;

// ------- FUNCTION DECLARATIONS --------
void IRAM_ATTR imu_isr();
void IRAM_ATTR handleButton0();
void IRAM_ATTR handleButton1();
void IRAM_ATTR handleButton2();
void IRAM_ATTR handleButton3();

void mqttTask(void *pvParameters) {
  IMU_Data data;
  int buttonEvent;

  static unsigned long lastPressTime;

  while (true) {
    // ---- IMU ----
    if (xQueueReceive(imuQueue, &data, 0) == pdTRUE) {
      // #ifdef DEBUG
      // Serial.print("Orientation: ");
      // Serial.print(data.position.pitch);
      // Serial.print(", ");
      // Serial.print(data.position.yaw);
      // Serial.print(", ");
      // Serial.println(data.position.roll);

      Serial.print("Velocity: ");
      Serial.print(data.velocity.x_vel);
      Serial.print(", ");
      Serial.print(data.velocity.y_vel);
      Serial.print(", ");
      Serial.println(data.velocity.z_vel);
      // #endif

      if (mqttClient.isConnected() && hasGameStarted) {
        std::string payload = formatImuPayload(data);
        mqttClient.publish(paddleEspPublishTopic, payload, 0, false);
      }
    }

    // ---- BUTTON ----
    if (xQueueReceive(buttonQueue, &buttonEvent, 0) == pdTRUE) {
      unsigned long now = millis();

      if (now - lastPressTime > DEBOUNCE_DELAY) {
        lastPressTime = now;

        if (mqttClient.isConnected() && hasGameStarted) {
          std::string payload = formatButtonPayload(buttonEvent + 1);
          mqttClient.publish(paddleEspPublishTopic, payload, 0, false);
        }
        #ifdef DEBUG
        Serial.printf("Button %d pressed\n", buttonEvent + 1);
        #endif
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void motorTask(void *pvParameters) {
  bool trigger;

  while (true) {
    if (xQueueReceive(motorQueue, &trigger, portMAX_DELAY)) {
      digitalWrite(NMOS_GATE_PIN, HIGH);  // Turn motor ON

      // Non-blocking delay using RTOS
      vTaskDelay(VIBRATION_TIME / portTICK_PERIOD_MS);

      digitalWrite(NMOS_GATE_PIN, LOW); // Turn motor OFF
    }
  }
}

void imuTask(void *pvParameters) {
  float accel_raw[3] = { 0, 0, 0 };   // raw accelerometer readings (mg)
  int32_t gyro_raw[3] = { 0, 0, 0 };  // raw gyroscope readings (mdps)

  float accel_calib[3] = { 0, 0, 0 };  // calibrated accelerometer (g)
  float gyro_calib[3] = { 0, 0, 0 };   // calibrated gyroscope (deg/s)

  struct IMU_Data data;

  float yaw_offset = 0.0f;
  bool calibrateRequest = false;

    // --- timeout settings ---
  const uint32_t imuTimeoutUs = 5000;  // 5ms timeout (adjust to your IMU ODR)
  uint32_t lastIMUReadTime = micros();

  while (true) {

    // Check if calibration requested
    if (xQueueReceive(calibrationQueue, &calibrateRequest, 0) == pdTRUE) {
        yaw_offset = filter.getYaw();  // capture current yaw
    #ifdef DEBUG
        Serial.println("[IMU] Yaw calibrated!");
    #endif
    }
    // wait for IMU interrupt
    BaseType_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_NOTIFY_TIMEOUT_MS));

    if (notified == 0) {
      resetI2CBus();  
      // Timeout expired → IMU did not respond
      Serial.println("[IMU] Timeout! Reinitializing...");
      AccGyr.Write_Reg(LSM6DSR_CTRL3_C, 0x01); // SW_RESET
      delay(50); // wait for reset to complete
      reinitIMU();
      continue; // skip this iteration
    }
    
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
    data.position.pitch = -filter.getPitch();
    
    float calibratedYaw = filter.getYaw() - yaw_offset;
    // wrap to [-180, 180]
    if (calibratedYaw > 180) calibratedYaw -= 360;
    if (calibratedYaw < -180) calibratedYaw += 360;
    data.position.yaw = -calibratedYaw;

    // ============ 5. COMPUTE RACKET VELOCITY ============
    computeRacketVelocity(q0, q1, q2, q3,
                          accel_calib,
                          gyro_calib,
                          imu_pos,
                          dt,
                          &data);

    xQueueSend(imuQueue, &data, 0);
  }
}

void setup() {
  Serial.begin(115200);
  SerialPort.println("Setup started...");

  Wire.begin();
  Wire.setClock(400000);  // 400kHz

  reinitIMU();

  // Queues
  imuQueue = xQueueCreate(10, sizeof(IMU_Data));
  buttonQueue = xQueueCreate(10, sizeof(int));
  calibrationQueue = xQueueCreate(1, sizeof(bool));
  motorQueue = xQueueCreate(1, sizeof(bool));

  // ===== HANDLE MQTT =====
  wifiConnect();
  mqttClient.setMqttClientName(clientID);
  mqttClient.enableLastWillMessage("/will", "esp32-client-paddle went offline", false);

  String mqttBrokerURL = String(mqtt_broker);
  mqttClient.setURL(mqttBrokerURL.c_str(), 1883, "", "");

  // String mqttBrokerURL = String(mqtt_broker);
  // mqttClient.setURL(mqttBrokerURL.c_str(), 8883, "", "");
  // mqttClient.setCaCert(caCert);
  // mqttClient.setClientCert(clientCert);
  // mqttClient.setKey(clientKey);
  mqttClient.loopStart();

  // ===== CREATE TASKS =====
  xTaskCreatePinnedToCore(imuTask, "IMU Task", 8192, NULL, 2, &imuTaskHandle, 1);
  xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(motorTask, "Motor Task", 2048, NULL, 1, NULL, 1);

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
void IRAM_ATTR imu_isr() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(
    imuTaskHandle,
    &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR handleButtonISR(int btn) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(buttonQueue, &btn, &xHigherPriorityTaskWoken);
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