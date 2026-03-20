// Includes
#include "LSM6DSRSensor.h"
#include "MahonyAHRS.h"
#include "Mqtt.h"
#include "config.h"
#include "IMU_processing.h"

// -------- PIN CONFIGURATIONS -------
#define IMU_INT_PIN 14
const int buttonPins[4] = { 25, 26, 27, 13 };
#define NMOS_GATE_PIN 2  // NMOS gate control pin

// ------------ CONSTANTS ------------
#define DEBOUNCE_DELAY 200

// --------- GLOBAL VARIABLES ----------
#define SerialPort Serial
LSM6DSRSensor AccGyr(&Wire, LSM6DSR_I2C_ADD_L);
Mahony filter;
bool hasGameStarted = false;

// ----------- QUEUE HANDLES -----------
QueueHandle_t imuQueue;
QueueHandle_t buttonQueue;

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
        Serial.printf("Button %d pressed\n", buttonEvent + 1);

        if (buttonEvent == 0) {
          int currentState = digitalRead(NMOS_GATE_PIN);
          digitalWrite(NMOS_GATE_PIN, !currentState);
          Serial.print("NMOS gate is now: ");
          Serial.println(!currentState ? "ON" : "OFF");
        }
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
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
  buttonQueue = xQueueCreate(10, sizeof(int));

  // ===== HANDLE MQTT =====
  wifiConnect();
  mqttClient.setMqttClientName(clientID);
  mqttClient.enableLastWillMessage("/will", "esp32-client-paddle went offline", false);
  
  String mqttBrokerURL = String(mqtt_broker);
  mqttClient.setURL(mqttBrokerURL.c_str(), 8883, "", "");
  mqttClient.setCaCert(caCert);
  mqttClient.setClientCert(clientCert);
  mqttClient.setKey(clientKey);
  mqttClient.loopStart();

  // ===== CREATE TASKS =====
  xTaskCreatePinnedToCore(imuTask, "IMU Task", 8192, NULL, 2, &imuTaskHandle, 1);
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
  // if (WiFi.status() != WL_CONNECTED) {
  //   wifiConnect();
  // }

  // if (mqttClient.isConnected()) {
  //   // Test IMU packet
  //   IMU_Data testData;
  //   testData.position.pitch = 1.1;
  //   testData.position.yaw   = 2.2;
  //   testData.position.roll  = 3.3;
  //   testData.velocity.x_vel = 0.5;
  //   testData.velocity.y_vel = 0.6;
  //   testData.velocity.z_vel = 0.7;

  //   std::string imuPayload = formatImuPayload(testData);
  //   mqttClient.publish(paddleEspPublishTopic, imuPayload, 0, false);
  //   Serial.printf("[TEST] Published IMU: %s\n", imuPayload.c_str());

  //   delay(2000);

  //   // Test button packet
  //   std::string buttonPayload = formatButtonPayload(1);
  //   mqttClient.publish(paddleEspPublishTopic, buttonPayload, 0, false);
  //   Serial.printf("[TEST] Published Button: %s\n", buttonPayload.c_str());

  // } else {
  //   Serial.println("[TEST] MQTT not connected...");
  // }

  // delay(3000);
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