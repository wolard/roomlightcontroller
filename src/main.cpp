/*
This example uses FreeRTOS softwaretimers as there is no built-in Ticker library
*/


#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>



extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
//#include <NeoPixelBus.h>
#include <NeoPixelBrightnessBus.h> // instead of NeoPixelBus.h
#include <ArduinoJson.h>
const uint16_t PixelCount = 426; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 2;  // make sure to set this to the correct pin, ignored for Esp8266
char buffer[4];   //buffer for long to char conversion
//std::unique_ptr<char[]> mqttPayloadBuffer;  //payloadbuffer
char mqttbuffer[25000];
// three element pixels, in different order and speeds
NeoPixelBus<NeoGrbwFeature, NeoSk6812Method> strip(PixelCount, PixelPin);


#define WIFI_SSID "Archer_c5"
#define WIFI_PASSWORD "kopo2008"

#define MQTT_HOST IPAddress(192, 168, 1, 201)
#define MQTT_PORT 1883

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
StaticJsonDocument<50000> doc;
bool effect1=false;
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
  uint16_t packetIdSub3 = mqttClient.subscribe("led", 0);
  uint16_t packetIdSub4 = mqttClient.subscribe("effect", 0);
  uint16_t packetIdSub5 = mqttClient.subscribe("/rgb", 0);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  mqttClient.publish("initial", 0, true, "1");
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
//  Serial.println("Publish received.");
//  Serial.print("  topic: ");
//  Serial.println(topic);
 
  if (strcmp(topic,"led") == 0 )
  {
 // Serial.println("only one led"); 
  //  Serial.print("  payload: ");
 // Serial.println(payload);
   DeserializationError error = deserializeJson(doc, payload);
   
   int r = doc["r"];
   int g = doc["g"];
   int b = doc["b"];
   int a = doc["a"];
   long num = doc["n"];
  // Serial.println(a);
   RgbwColor color(r, g, b,(a*255));
    strip.SetPixelColor(num, color);
    strip.Show();
    //mqttClient.publish("ledstatus", 0, false, ltoa(num,buffer,10));
    
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  }
   if (strcmp(topic,"leds") == 0 )
  {
  Serial.println("whole strip"); 
  
  Serial.print("  payload: ");
  Serial.println(payload);
   DeserializationError error = deserializeJson(doc, payload);
   
   int r = doc["r"];
  int g = doc["g"];
  int b = doc["b"];
   int a = doc["a"];
   Serial.println(a);
   RgbwColor color(r, g, b,a);
   for(int i=0;i<426;i++)
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
   if (strcmp(topic,"effect") == 0 )
   
   
   {
     char new_payload[len+1];
new_payload[len] = '\0';
strncpy(new_payload, payload, len);
      
     if (strcmp(new_payload,"1") == 0 )
     {
       effect1=false;
     }
     else effect1=true;
   
   }
     if (strcmp(topic,"/rgb") == 0 )
   {
   //  if (mqttPayloadBuffer == nullptr || index == 0) mqttPayloadBuffer = std::unique_ptr<char[]>(new char[total+1]); // empty the buffer


memcpy(mqttbuffer + index, payload, len); // copy the content into it

if (index + len != total) return; 
Serial.print("printing rgbarray");

   DeserializationError error = deserializeJson(doc, mqttbuffer);
  for (int i=0;i<426;i++)
  {
    int r = doc[i]["r"];
   int g = doc[i]["g"];
   int b = doc[i]["b"];
   int a = doc[i]["a"];
   int num=doc[i]["n"];
   RgbwColor color(r, g, b,a);
  
     strip.SetPixelColor(num, color);
  }
   strip.Show();
   Serial.print("show rgbstrip");
   memset(mqttbuffer, 0, sizeof mqttbuffer);
     if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
     }
   //mqttClient.publish("ledstatus", 0, false, mqttPayloadBuffer);
  




   }
  
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
 
  Serial.begin(9600);
 
  strip.Begin();
  RgbwColor red(255, 0, 0,0);
  RgbwColor green(0, 255, 0,0);
  RgbwColor blue(0, 0, 255,0);
  /*
   for(int i=0;i<426;i++)
   {
    strip.SetPixelColor(i, red);
    strip.Show();
   
   }
*/

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);
  
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
 // mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
 // mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();
   ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  
}

void loop() 
{
  

 ArduinoOTA.handle();
 if(effect1==true)
 {
   strip.RotateLeft(1);
strip.Show();
 }
  if(effect1==false)
 {
   strip.RotateLeft(0);
strip.Show();
 }
}
  

