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
    mqttClient.subscribe(paddleEspSubscribeTopic, [](const std::string &payload) {
      Serial.printf("[%s] Received: %s\n", paddleEspSubscribeTopic.c_str(), payload.c_str());
    });
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

std::string formatPayload(float x, float y) {
  JsonDocument doc;
  doc["clientID"] = clientID;
  doc["position"]["x"] = x;
  doc["position"]["y"] = y;

  std::string jsonPayload;
  serializeJson(doc, jsonPayload);
  return jsonPayload;
}