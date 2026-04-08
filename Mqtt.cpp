#include "mqtt.h"

ESP32MQTTClient mqttClient;

void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ESP32-PADDLE] Attempting network connection.");
    delay(500);
  }
  Serial.println("[ESP32-PADDLE] Connection to network successful.");
}

void onMqttConnect(esp_mqtt_client_handle_t client) {
  if (mqttClient.isMyTurn(client)) {
    for (const auto& topic : paddleEspSubscribeTopics) {
      mqttClient.subscribe(topic, [topic](const std::string &payload) {
        Serial.printf("[%s] Received: %s\n", topic.c_str(), payload.c_str());

        if (topic == "/system/signal") {
            if (payload == "START") {
              Serial.println("[SYSTEM] Game started.");
              hasGameStarted = true;
            } else if (payload == "STOP") {
              Serial.println("[SYSTEM] Game ended.");
              hasGameStarted = false;
            }
        } else if (topic == "/paddleCalibration") {
            bool trigger = true;
            xQueueSend(calibrationQueue, &trigger, 0);
        } else if (topic == "/hitAck") {
            bool trigger = true;
            xQueueSend(motorQueue, &trigger, 0);
            // Serial.println("[MQTT] motor");

        }
      });
    }
    
    // publish READY to system-coordinator
    mqttClient.publish("/status/esp32-paddle-client", "READY", 1, false);
    Serial.println("[SYSTEM] Published READY");
  }
}

void onMqttEvent(esp_mqtt_event_handle_t event) {
  mqttClient.onEventCallback(event);
}

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
esp_err_t handleMQTT(esp_mqtt_event_handle_t event) {
  mqttClient.onEventCallback(event);
  return ESP_OK;
}
#else
void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  auto *event = static_cast<esp_mqtt_event_handle_t>(event_data);
  mqttClient.onEventCallback(event);
}
#endif

std::string formatPayload(const IMU_Data &data, int button = 0) {
    JsonDocument doc;
    doc["client-id"] = clientID;

    doc["position"]["roll"]  = data.position.roll;
    doc["position"]["pitch"] = data.position.pitch;
    doc["position"]["yaw"]   = data.position.yaw;

    doc["velocity"]["x"] = data.velocity.x_vel;
    doc["velocity"]["y"] = data.velocity.y_vel;
    doc["velocity"]["z"] = data.velocity.z_vel;

    doc["button"] = button;

    std::string jsonPayload;
    serializeJson(doc, jsonPayload);
    return jsonPayload;
}

// IMU packet
std::string formatImuPayload(const IMU_Data &data) {
    JsonDocument doc;
    doc["client-id"] = clientID;
    doc["type"] = "imu";

    doc["position"]["roll"]  = data.position.roll;
    doc["position"]["pitch"] = data.position.pitch;
    doc["position"]["yaw"]   = data.position.yaw;

    doc["velocity"]["x"] = data.velocity.x_vel;
    doc["velocity"]["y"] = data.velocity.y_vel;
    doc["velocity"]["z"] = data.velocity.z_vel;

    std::string jsonPayload;
    serializeJson(doc, jsonPayload);
    return jsonPayload;
}

// Button packet
std::string formatButtonPayload(int buttonState) {
    JsonDocument doc;
    doc["client-id"] = clientID;
    doc["type"] = "button";
    doc["button"] = buttonState;

    std::string jsonPayload;
    serializeJson(doc, jsonPayload);
    return jsonPayload;
}