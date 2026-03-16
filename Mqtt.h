#pragma once

#include <WiFi.h>
#include <ArduinoJson.h>
#include "ESP32MQTTClient.h"
#include "config.h"

extern ESP32MQTTClient mqttClient;

// Exposed functions
void wifiConnect();
void onMqttConnect(esp_mqtt_client_handle_t client);
void onMqttEvent(esp_mqtt_event_handle_t event);
std::string formatPayload(float x, float y);

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
esp_err_t handleMQTT(esp_mqtt_event_handle_t event);
#else
void handleMQTT(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
#endif