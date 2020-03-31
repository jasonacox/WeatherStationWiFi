/*
    Solar Powered WiFi Weather Station
    ESP8266 NodeMCU Based Weather Station - Record Temperature (x2), Humidity, Pressure, Rain, and Voltage

    Send results to web or mqtt server
    
    Author: Jason A. Cox - @jasonacox - https://github.com/jasonacox/WeatherStationWiFi

    Date: 20 March 2020
    
*/

// Header includes
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <OneWire.h>
#include <DallasTemperature.h> /* For one-wire temp probe */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "secrets.h"          // WiFi ssid and password

// Globals
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

/* WiFi credentials */
#ifndef WIFISSID
#define WIFISSID "SSID"
#define WIFIPASS "password"
#endif

#define HOSTNAME "WeatherStation"
#define SENSORID 101   // Serial Number - Unqiue for IoT Device

/* Pins */
#define LED_ESP 16     // Built-in LED on NodeMCU Mainboard GPI16 = D0
#define LED_WIFI 2     // Built-in LED on ESP-12 WiFi Module GPIO2 = D4
#define LED 14         // LED on GPIO14 = D5
#define WATER_1 13     // Water Sensor NPN Transitor on GPIO13 = D7
#define ONEWIRE 12     // OneWire Bus on GPIO12 = D6
#define VOLTAGE 0      // Voltage Reader on Analog Input = A0

/* States */
#define DRY 0
#define WET 1
#define NIL -1

/* Debug Mode = Verbose Output to Serial Port */
#define DEBUG 0

/* Wifi Settings */
const char* ssid     = WIFISSID;
const char* password = WIFIPASS;

/* Server location to send alerts and status */
const char* host = "10.0.1.10";
const uint16_t port = 80;
#define URIPREFIX "/sensor.php"  // URI to send updates
const char* mqtt = "10.0.1.10";
const uint16_t mqttport = 1883;

/* globals */
int state;
int count;
ESP8266WiFiMulti WiFiMulti;           // ESP8266 Wifi
OneWire oneWire(ONEWIRE);             // DS18B20 One-Wire Temp Probe
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.
DeviceAddress Thermometer;            // variable to hold device addresses
int deviceCount = 0;                  // Number of One-Wire Devices Found
// uint8_t rainsensor[8] = { 0x28, 0xFF, 0xAB, 0x06, 0x81, 0x14, 0x02, 0xA0  };
bool justboot = true;

/*
 * SETUP - Runs on Startup ONLY
 */
void setup() {
  Serial.begin(115200);
  bool status;

  delay(400);
  Serial.println("WeatherStation WiFi v0.1 - https://github.com/jasonacox/WeatherStationWiFi");
  Serial.println();
  Serial.println("> Booting...");
   
  // initialize GPIOs
  pinMode(LED_ESP, OUTPUT);
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(WATER_1, INPUT_PULLUP);

  // booting - turn on LEDs to show startup 
  digitalWrite(LED_ESP, LOW);     // turn on the mainboard blue LED
  digitalWrite(LED, HIGH);        // turn on the red LED  
  digitalWrite(LED_WIFI, HIGH);   // turn off the wifi LED  

  // Initialize BME280 - Temp, Humidity and Pressure sensor
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("  Could not find a valid BME280 sensor, check wiring!");
    while (1) {
      // Din't find it so pulse red LED to show error and try again
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED, LOW);
      delay(200);
      status = bme.begin(0x76);  
      if (status) break;
    }
  }
  digitalWrite(LED, LOW);         // turn off red LED

  // We start by connecting to a WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.hostname(HOSTNAME);
  WiFiMulti.addAP(ssid, password);

  Serial.println();
  Serial.print("  Connecting to WiFi");

  // Try to connect to WiFi
  while (WiFiMulti.run() != WL_CONNECTED) {
    // Not connected - flash wifi LED
    digitalWrite(LED_WIFI, LOW);
    delay(100);
    digitalWrite(LED_WIFI, HIGH);
    delay(100);
    digitalWrite(LED_WIFI, LOW); 
    delay(100);
    digitalWrite(LED_WIFI, HIGH);
    Serial.print(".");
    delay(300);
  }
  digitalWrite(LED_WIFI, LOW);

  Serial.println("");
  Serial.println("  WiFi connected!");
  Serial.print("  IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("  MAC address: %s\n", WiFi.macAddress().c_str());
  Serial.printf("  RSSI: %d dBm\n", WiFi.RSSI());
    
  Serial.print("  Publishing to host: ");
  Serial.print(host);
  Serial.print(':');
  Serial.println(port);

  digitalWrite(LED_WIFI, HIGH);      // turn off wifi LED

  state = NIL;
  count = 0;

  /* fire up one-wire temp probe and get list of devices available */
  // Start up the library for DS18B20 One-Wire Temp Probe
  sensors.begin();

  // locate devices on the bus
  Serial.println("  Locating devices...");
  Serial.print("  Found ");
  deviceCount = sensors.getDeviceCount();
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");
  Serial.println("");
  
  Serial.println("  Printing addresses...");
  for (int i = 0;  i < deviceCount;  i++)
  {
    Serial.print("  Sensor ");
    Serial.print(i+1);
    Serial.print(" : ");
    sensors.getAddress(Thermometer, i);
    printAddress(Thermometer);
  }

  Serial.println();
  Serial.println("> Starting Main loop...");

  digitalWrite(LED_ESP, HIGH);    // turn off main LED 
}

/*
 * MAIN LOOP 
 */
void loop() {

  int rainsensor = 0;
  int watchdog = 0;
  float voltage = 0.0;
  float bmeTemp = 0.0;
  float bmePressure = 0.0;
  float bmeHumidity = 0.0;

  /* Force heartbeat every 300 cycles ~= 5m */
  count++;
  if(count > 300) {
     count = 0;
     state = NIL;
  }

  // Make sure we are connected to WiFi
  while (WiFiMulti.run() != WL_CONNECTED) {
    // Not connected - flash wifi LED 
    Serial.print("Reconnecting to WiFi");
    digitalWrite(LED_WIFI, LOW);
    delay(100);
    digitalWrite(LED_WIFI, HIGH);
    delay(100);
    digitalWrite(LED_WIFI, LOW); 
    delay(100);
    digitalWrite(LED_WIFI, HIGH);
    Serial.print(".");
    delay(300);
  }
  
  // Heartbeat - flash onboard LED
  digitalWrite(LED_ESP, LOW);  
  delay(10);                       
  digitalWrite(LED_ESP, HIGH); 

  // Check Voltage
  voltage = analogRead(A0) * (4.69 / 907.0);
  if(DEBUG) {
      Serial.print("Voltage: ");
      Serial.println(voltage);
  }
  
  // Capture current temp
  sensors.requestTemperatures(); 
  float temperatureC = sensors.getTempCByIndex(0);
  while(temperatureC <= -55.0 and watchdog < 10) {
    if(DEBUG) Serial.println("WARN: Bad data from temp probe, trying again.");
    delay(100);
    sensors.requestTemperatures(); 
    temperatureC = sensors.getTempCByIndex(0);
    watchdog = watchdog + 1;
  }
  if(DEBUG) {
    Serial.print("     : Temp = ");
    Serial.print(temperatureC);
    Serial.print("ºC / ");
    float temperatureF = sensors.getTempFByIndex(0);
    Serial.print(temperatureF);
    Serial.println("ºF");
  }

  // Poll for humidity and pressure
  bmeTemp = bme.readTemperature();            // in C
  bmePressure = bme.readPressure() / 100.0F;  // in hPa
  bmeHumidity = bme.readHumidity();           // in % humidity
  
  // Poll water sensors
  rainsensor = digitalRead(WATER_1); // waterlevel alert 1
  if(DEBUG) {
    Serial.print("STATE: ");
    Serial.print(state, DEC);
    Serial.print(" - Rain Sensor: ");  
    Serial.println(rainsensor, DEC);
  }
  
  // Detected water at rainsensor 
  if(rainsensor == 0 and state < WET) {
    digitalWrite(LED, HIGH); // turn on red LED
    if(DEBUG) Serial.println("**  Rain Detected **");
    state = WET;

    // Push update to server
    sendStatus(1, SENSORID, temperatureC, bmeTemp, bmePressure, bmeHumidity, voltage);
  }

  // rainsensor clear 
  if(rainsensor == 1 and state != DRY) {
    digitalWrite(LED, LOW); // turn off red LED
    if(DEBUG) Serial.println("**  No Rain Detected **");
    state = DRY;
   
    // Push update to server
    sendStatus(0, SENSORID, temperatureC, bmeTemp, bmePressure, bmeHumidity, voltage);
  }
  
  // Wait about a second
  delay(990);
}

/*
 * Prints OneWire Bus Device Address
 */
void printAddress(DeviceAddress deviceAddress)
{ 
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println("");
}

/*
 * Send Status to Web Server - water level and temperature
 *    waterlevel: 1=wet, 0=dry
 *    temp: floating point degrees celcius
 */
void sendStatus(int waterlevel, int id, float temp, float bmeTemp, float bmePressure, float bmeHumidity, float voltage)
{
    int count = 0;

    // Flash wifi LED
    digitalWrite(LED_WIFI, LOW);
    
    // Use WiFiClient class to create TCP connections
    WiFiClient client;

    // Send via HTTP GET
    while (!client.connect(host, port)) {
      count = count + 1;
      if (count > 5) {
          Serial.println("ERROR: Unable to connect - restarting...");
          digitalWrite(LED, HIGH);
          delay(200);
          digitalWrite(LED, LOW);
          ESP.restart();
      }
      Serial.printf("ERROR: Connection failed - Waiting 5 seconds and retrying %d...\n",count);
      delay(5000);
    }
    client.print("GET ");
    client.print(URIPREFIX);
    client.print("?waterlevel=");
    client.print(waterlevel);
    client.print("&id=");
    client.print(id);
    client.print("&temp=");
    client.print(temp);
    client.print("&temp2=");
    client.print(bmeTemp);
    client.print("&pressure=");
    client.print(bmePressure);
    client.print("&humidity=");
    client.print(bmeHumidity);
    client.print("&voltage=");
    client.print(voltage);
    if (justboot) {
      client.print("&note=poweron");
      justboot = false;
    }
    client.println(" HTTP/1.0");
    client.println();

    //read back one line from server
    if(DEBUG) {
      Serial.println("receiving from remote server");
      String line = client.readStringUntil('\r');
      Serial.println(line);
      Serial.println("closing connection");      
    }
    client.stop();

    // Send via MQTT
    // TBD

    // turn off wifi LED
    digitalWrite(LED_WIFI, HIGH);
}

void printBMEValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");*/
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}
