#include <Arduino.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include <AsyncMqttClient.h>
#include <SimpleDHT.h>

#define WIFI_SSID ""
#define WIFI_PASSWORD ""

#define MQTT_HOST IPAddress(192, 168, 178, 110)
#define MQTT_PORT 1883
#define MQTT_PUB_TEMP "woonkamer/Temperatuur"


AsyncMqttClient mqttclient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

unsigned long previousMillis = 0;
const long intervalMillis = 10000;

int pinDHT22 = 3;
SimpleDHT22 dht22(pinDHT22);


void connectToWifi() {
  Serial.println("Connecting to wifi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttclient.connect();
}

void WiFiEvent(WiFiEvent_t event){
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.printf("\nLocal ESP32 IP: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Wifi disconnected");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent){
  Serial.println("Connected to MQTT server.");
  Serial.print("Session present ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason){
  Serial.println("Disconnected from MQTT server");
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }


  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttclient.onConnect(onMqttConnect);
  mqttclient.onDisconnect(onMqttDisconnect);
  mqttclient.onPublish(onMqttPublish);
  mqttclient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= intervalMillis) {
    previousMillis = currentMillis;

    byte temperature = 0;
    byte humidity = 0;
    int err = SimpleDHTErrSuccess;
    if ((err = dht22.read(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
      Serial.print("Read DHT22 failed, err="); Serial.print(SimpleDHTErrCode(err));
      Serial.print(","); Serial.println(SimpleDHTErrDuration(err)); delay(2000);
      return;
    } else {
        uint16_t packetIdPubt = mqttclient.publish(MQTT_PUB_TEMP, 1, true, String(temperature).c_str());
        Serial.printf("Publishing on topic %s at Qos 1, packetId: ",  MQTT_PUB_TEMP);
        Serial.println(packetIdPubt);
        Serial.printf("Message: %d \n", temperature);
    }
  }
}
