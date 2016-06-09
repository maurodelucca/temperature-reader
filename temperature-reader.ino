#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <brzo_i2c.h> // Only needed for Arduino 1.6.5 and earlier
#include <SSD1306Brzo.h>
#include <DHT.h>
#include <ThingSpeak.h>
#include "temperature_reader_private.h"

#define PUBLISH_INTERVAL (5 * 60 * 1000)

// ThingSpeak variables
unsigned long lastUpdate = 0;

// Initialize the OLED display using brzo_i2c
SSD1306Brzo display(0x3c, 9, 10);

// Starting dht sensor
DHT dht(D4, DHT11);

// Wifi client for ThingSpeak
WiFiClient  client;

void setup() {

  //********** OLED Display ************
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  
  Serial.begin(115200);
  Serial.println("Booting...");
  
  display.drawString(0, 0, "Booting...");
  display.display();

  //************** Arduino OTA **********
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    display.drawString(0, 10, "Connection Failed! Rebooting...");
    display.display();
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
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int prog, unsigned int total) {
    int progress = (double(prog) / double(total)) * 100;

    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "Updating OTA...");
    
    // draw the progress bar
    display.drawProgressBar(0, 32, 120, 10, progress);
  
    // draw the percentage as String
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 15, String(progress) + "%");
    display.display();
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    char buf[40];
    sprintf(buf, "Error[%u]: ", error);
    Serial.println(buf);
    
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
    
    display.drawString(0, 50, buf);
    display.display();
  });
  
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  display.drawString(0, 10, "Ready");
  
  char buf[128];
  IPAddress myIp = WiFi.localIP();
  sprintf(buf, "IP: %d.%d.%d.%d", myIp[0], myIp[1], myIp[2], myIp[3]);
  display.drawString(0, 20, buf);
  display.display();

  //***************** DHT Sensor *****************
  dht.begin();

  //***************** ThingSpeak *****************
  ThingSpeak.begin(client);
  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);

  // Clearing screen before starting
  delay(3000);
  display.clear();
}

void printResults(float h, float t, float hindex, long rssi) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);

  char float_temp[6];
  char buf[48];

  dtostrf(t, 2, 0, float_temp);
  sprintf(buf, "Temp: %s C", float_temp);
  Serial.println(buf);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Temp:");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(125, 0, float_temp);
  
  dtostrf(h, 2, 0, float_temp);
  sprintf(buf, "Humedad: %s", float_temp);
  Serial.println(buf);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 16, "Humedad:");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(125, 16, float_temp);

  dtostrf(hindex, 2, 0, float_temp);
  sprintf(buf, "Idx Heat: %s C", float_temp);
  Serial.println(buf);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 32, "Idx Heat:");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(125, 32, float_temp);

  sprintf(buf, "RSSI: %ld", rssi);
  Serial.println(buf);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 48, "RSSI:");
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(125, 48, String(rssi));

  unsigned long currTime = millis();
  if(currTime > lastUpdate) {
    int publishStatus = (((double)currTime - (double)lastUpdate) / (double) PUBLISH_INTERVAL) * 63.0;
    display.drawLine(127, 0, 127, publishStatus);
  }

  display.display();
}

void publishResults(float h, float t, float hindex, long rssi) {
  int lightRaw = 0;
  unsigned long currTime = millis();
  
  if(currTime - lastUpdate > PUBLISH_INTERVAL || lastUpdate == 0 || currTime < lastUpdate) {
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

void loop() {
  ArduinoOTA.handle();

  //********* Collect results **********
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

  //********* Print results **********
  printResults(h, t, hindex, rssi);

  //********* Publish results **********
  publishResults(h, t, hindex, rssi);
}

