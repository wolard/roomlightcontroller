/*
This example uses FreeRTOS softwaretimers as there is no built-in Ticker library
*/


#include <WiFi.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
//#include <NeoPixelBus.h>
#include <NeoPixelBrightnessBus.h> // instead of NeoPixelBus.h
int red;
int blue;
int green;
const uint16_t PixelCount = 150; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 2;  // make sure to set this to the correct pin, ignored for Esp8266

#define colorSaturation 255

// three element pixels, in different order and speeds
NeoPixelBus<NeoGrbwFeature, NeoSk6812Method> strip(PixelCount, PixelPin);
RgbColor violet(218, 10, 255);

RgbColor white(colorSaturation);
RgbColor black(0);
#include <ArduinoJson.h>

#define WIFI_SSID "Archer_c5"
#define WIFI_PASSWORD "kopo2008"

#define MQTT_HOST IPAddress(192, 168, 1, 201)
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
StaticJsonDocument<200> doc;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
        xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe("leds", 0);
  uint16_t packetIdSub2 = mqttClient.subscribe("ledstatus", 0);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  //mqttClient.publish("test/lol", 0, true, "test 1");
  //Serial.println("Publishing at QoS 0");
  
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
   Serial.print("  payload: ");
  Serial.println(payload);
   DeserializationError error = deserializeJson(doc, payload);
   
   int r = doc["r"];
  int g = doc["g"];
  int b = doc["b"];
   int a = doc["a"];
   Serial.println(a);
   RgbwColor color(r, g, b,(a*255));
   for(int i=0;i<150;i++)
   {
    strip.SetPixelColor(i, color);
    
   }
   strip.Show();
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  
  Serial.begin(9600);
  Serial.println();
  Serial.println();
  strip.Begin();
  RgbwColor initcolor(196, 0, 255,0);
   for(int i=0;i<150;i++)
   {
    strip.SetPixelColor(i, initcolor);
    strip.Show();
   }
  strip.Show();

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
const uint8_t c_MinBrightness = 8; 
const uint8_t c_MaxBrightness = 255;

int8_t direction; // current direction of dimming
  connectToWifi();
}

void loop() 
{

}
  

