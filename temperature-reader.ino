#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <OLED.h>
#include <DHT.h>
#include <ThingSpeak.h>
#include "temperature_reader_private.h"

// ThingSpeak variables
unsigned long lastUpdate = 0;

// Declare OLED display
// display(SDA, SCL);
// SDA and SCL are the GPIO pins of ESP8266 that are connected to respective pins of display.
OLED display(9, 10);

// Starting dht sensor
DHT dht(D4, DHT11);

// Wifi client for ThingSpeak
WiFiClient  client;

void setup() {

  //********** OLED Display ************
  // Initialize display
  display.begin();
  
  Serial.begin(115200);
  Serial.println("Booting");
  display.print("Booting...");

  //************** Arduino OTA **********
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    display.print("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
    display.clear();
    display.print("Starting OTA...");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    display.print("Done", 4);
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    char buf[40];
    sprintf(buf, "Progress: %u%%", (progress / (total / 100)));
    Serial.println(buf);
    display.print(buf, 2);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    char buf[40];
    sprintf(buf, "Error[%u]: ", error);
    Serial.println(buf);
    display.print(buf, 3);
    
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  display.print("Ready", 1);
  display.print("IP address:", 2);
  char buf[128];
  IPAddress myIp = WiFi.localIP();
  sprintf(buf, "%d.%d.%d.%d", myIp[0], myIp[1], myIp[2], myIp[3]);
  display.print(buf, 3);

  //***************** DHT Sensor *****************
  dht.begin();

  //***************** ThingSpeak *****************
  ThingSpeak.begin(client);
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);

  // Clearing screen before starting
  delay(5000);
  display.clear();
}

void loop() {
  ArduinoOTA.handle();

  float h = 0;
  float t = 0;
  float hindex = 0;
  do {
    delay(2000);
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    h = dht.readHumidity();
    // Read temperature as Celsius (the default)
    t = dht.readTemperature();
    // Read head index
    hindex = dht.computeHeatIndex(t, h, false);
  } while( isnan(h) || isnan(t) || isnan(hindex) );

  long rssi = WiFi.RSSI();

  char float_temp[6];
  char buf[48];
  
  dtostrf(h, 2, 0, float_temp);
  sprintf(buf, "Humedad:    %s", float_temp);
  Serial.println(buf);
  display.print(buf);

  dtostrf(t, 2, 0, float_temp);
  sprintf(buf, "Temp:       %s C", float_temp);
  Serial.println(buf);
  display.print(buf, 1);

  dtostrf(hindex, 2, 0, float_temp);
  sprintf(buf, "Index Heat: %s C", float_temp);
  Serial.println(buf);
  display.print(buf, 2);

  sprintf(buf, "RSSI:      %ld", rssi);
  Serial.println(buf);
  display.print(buf, 3);

  int lightRaw = 0;
  unsigned long currTime = millis();
  
  sprintf(buf, "%ld", lastUpdate + (5 * 60 * 1000));
  Serial.println(buf);
  display.print(buf, 4);
  sprintf(buf, "%ld", currTime);
  Serial.println(buf);
  display.print(buf, 5);
  
  if(currTime - lastUpdate > (5 * 60 * 1000) || lastUpdate == 0) {
    ThingSpeak.setField(1, lightRaw);
    ThingSpeak.setField(2, t);
    ThingSpeak.setField(3, h);
    ThingSpeak.setField(4, hindex);
    
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

    lastUpdate = currTime;

    digitalWrite(1, HIGH);
    delay(200);
    digitalWrite(1, LOW);
  }
}

