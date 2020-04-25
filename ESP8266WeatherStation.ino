/*
    Solar Powered WiFi Weather Station
    ESP8266 NodeMCU Based Weather Station
    Records: Temperature, Humidity, Pressure, Rain, Wind and Voltage

    Send results to web server

    Author: Jason A. Cox - @jasonacox - https://github.com/jasonacox/WeatherStationWiFi

    Date: 20 March 2020

*/

// Header includes
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
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
#define SENSORID 102   // Serial Number - Unqiue for IoT Device

/* Pins */
#define LED_ESP 16     // Built-in LED on NodeMCU Mainboard GPIO16 = D0
#define LED_WIFI 2     // Built-in LED on ESP-12 WiFi Module GPIO2 = D4
#define LED 14         // LED on GPIO14 = D5
#define WATER_1 13     // Water Sensor NPN Transitor on GPIO13 = D7
#define ONEWIRE 12     // OneWire Bus on GPIO12 = D6
#define VOLTAGE 0      // Voltage Reader on Analog Input = A0
#define ANEMOMETER 12  // Rotation sensor on Anemometer GPIO12 = D6

/* States */
#define DRY 0
#define WET 1
#define NIL -1

/* Cycles in ms */
#define SENSORSCAN 1000       // Run sensor scan every 1s
#define SENDDATA 5*60*1000    // Publish sensor data every 5m
#define LEDFLASH 5000         // Flash LED every 5s
#define WINDCHECK 10          // Check anemometer every 10ms
#define POWERSAVE 60*60*1000  // Delay cycles during power saving mode - 1hr

/* Debug Mode = Uncomment for Verbose Output to Serial Port */
// #define DEBUG 1

/* Wifi Settings */
const char* ssid     = WIFISSID;
const char* password = WIFIPASS;

/* Network Server location to send alerts and status */
const char* host = "10.0.1.10";
const uint16_t port = 80;
#define URIPREFIX "/sensor.php"  // URI to send updates
const char* mqtt = "10.0.1.10";
const uint16_t mqttport = 1883;

/* Global variables */
int state;
int count;
unsigned long currentMillis = 0;
unsigned long heartbeatMillis = 0;
unsigned long rotationMillis = 0;
unsigned long cycleMillis = 0;
unsigned long anemometerMillis = 0;
unsigned long ledMillis = 0;
bool anemometerState = false;         // Signal state from Anemometer
unsigned long i = 0;
ESP8266WiFiMulti WiFiMulti;           // ESP8266 Wifi
int deviceCount = 0;                  // Number of One-Wire Devices Found
bool justboot = true;                 // Flag for booting
bool lowpower = false;                // Flag for lowpower mode

/*
   SETUP - Runs on Startup ONLY
*/
void setup() {
  Serial.begin(115200);
  bool status;

  Serial.println();
  delay(400);
  Serial.println("WeatherStation WiFi v0.2 - https://github.com/jasonacox/WeatherStationWiFi");
  Serial.println();
  Serial.println("> Booting...");

  // initialize GPIOs
  pinMode(LED_ESP, OUTPUT);
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(WATER_1, INPUT_PULLUP);
  pinMode(ANEMOMETER, INPUT_PULLUP);

  // booting - turn on LEDs to show startup
  digitalWrite(LED_ESP, LOW);     // turn on the mainboard blue LED
  digitalWrite(LED, HIGH);        // turn on the red LED
  digitalWrite(LED_WIFI, HIGH);   // turn off the wifi LED

  // Initialize BME280 - Temp, Humidity and Pressure sensor
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("  ERROR: BME280 sensor is not responding - Retrying...");
    while (1) {
      // Pulse red LED to show error and try again
      digitalWrite(LED_ESP, LOW);
      digitalWrite(LED, HIGH);
      delay(200);
      digitalWrite(LED_ESP, HIGH);
      digitalWrite(LED, LOW);
      delay(200);
      status = bme.begin(0x76);
      if (status) break;
    }
  }
  digitalWrite(LED, LOW);         // turn off red LED

  // We start by connecting to a WiFi network
  WiFi.mode(WIFI_STA);
  //WiFi.setAutoReconnect(false); 
  WiFi.hostname(HOSTNAME);
  WiFiMulti.addAP(ssid, password);

  Serial.println();
  Serial.print("  Connecting to WiFi");

  // Try to connect to WiFi
  while (WiFiMulti.run() != WL_CONNECTED) {
    // Not connected - flash wifi LED
    digitalWrite(LED_WIFI, LOW);
    blinkLED(LED, 200, true);
    blinkLED(LED, 200, true);
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
  
  // Turn off WiFi to save power
  WiFi.forceSleepBegin();
  delay(1);

  state = NIL;
  count = 0;

  Serial.println();
  Serial.println("> Starting Main loop...");

  digitalWrite(LED_ESP, HIGH);    // turn off main LED

}

/*
   MAIN LOOP
*/
void loop() {

  // Sensor Data
  int rainsensor = 0;
  int watchdog = 0;
  float voltage = 0.0;
  float bmeTemp = 0.0;
  float bmePressure = 0.0;
  float bmeHumidity = 0.0;
  float wind = 0.0;
  float rps = 0.0;

  currentMillis = millis();

  /* LED heartbeat - flash onboard LED... */
  if (currentMillis >= ledMillis) {
    ledMillis = currentMillis + LEDFLASH;
    digitalWrite(LED_ESP, LOW);
    delay(10);
    digitalWrite(LED_ESP, HIGH);
  }

  /* Check anemometer sensor every WINDCHECK ms to identify rotation */
  if (currentMillis >= anemometerMillis) {
    anemometerMillis = currentMillis + WINDCHECK;
    if (digitalRead(ANEMOMETER) == HIGH) {
      // Anemometer detects magnet 
      if (anemometerState == false) {
        // Signal state change means that we want to count as rotation.
        anemometerState = true;
        i = i + 1;
      }
    }
    else {
      // Anemometer is not detecting magnet
      anemometerState = false;  // Singal change = count
    }
  }

  /* Scan Sensors every SENSORSCAN ms */
  if (currentMillis >= cycleMillis) {
    // Set next cycle update
    cycleMillis = currentMillis + SENSORSCAN;
#ifdef DEBUG
    Serial.print(">>>> currentMillis: ");
    Serial.print(currentMillis);
    Serial.print(" - cycleMillis: ");
    Serial.print(cycleMillis);
    Serial.print(" - heartbeatMillis: ");
    Serial.println(heartbeatMillis);
#endif

    // Check Voltage
    voltage = analogRead(A0) * (4.69 / 907.0);
#ifdef DEBUG
    Serial.print("Voltage: ");
    Serial.println(voltage);
#endif

    // Send update to server every SENDDATA ms
    if (currentMillis >= heartbeatMillis) {

      heartbeatMillis = currentMillis + SENDDATA; 
      state = NIL;  // Flag to send data

      // Check for register overflow and restart.
      if (currentMillis > heartbeatMillis) {
        ESP.restart();
      }

      // Calculate and sum up anemometer data
      if (i == 0) {
        wind = 0.0;
      }
      else {
        // The value i represents the number of rotation
        // over the past SENDDATA ms (5 min). You can adjust
        // this to give kph, mph, etc.
        wind = i;
      }
      i = 0;

      // Check to see if battery is depleted and switch to low power mode
      if (voltage <= 3.4) {
        lowpower = true;
      }
      else {
        lowpower = false;
      }
    }

    // Poll for humidity and pressure
    bmeTemp = bme.readTemperature();            // in C
    bmePressure = bme.readPressure() / 100.0F;  // in hPa
    bmeHumidity = bme.readHumidity();           // in % humidity
    float temperatureC = bmeTemp;

    // Poll water sensors
    rainsensor = digitalRead(WATER_1); // waterlevel alert 1
#ifdef DEBUG
    Serial.print("STATE: ");
    Serial.print(state, DEC);
    Serial.print(" - Rain Sensor: ");
    Serial.println(rainsensor, DEC);
#endif

    // Detected water at rainsensor
    if (rainsensor == 0 and state < WET) {
#ifdef DEBUG
      digitalWrite(LED, HIGH); // turn on red LED
      Serial.println("**  Rain Detected **");
#endif
      state = WET;

      // Push update to server
      sendStatus(1, SENSORID, temperatureC, bmeTemp, bmePressure, bmeHumidity, voltage, wind);
    }

    // Rainsensor clear or heartbeat update
    if (rainsensor == 1 and state != DRY) {
#ifdef DEBUG
      digitalWrite(LED, LOW); // turn off red LED
      Serial.println("**  No Rain Detected **");
#endif
      state = DRY;

      // Push update to server
      sendStatus(0, SENSORID, temperatureC, bmeTemp, bmePressure, bmeHumidity, voltage, wind);
    }

    // Low power mode - slow down updates and if critical power pause for POWERSAVE (1hr)
    if (lowpower) {
      if (voltage >= 3.0) {
        heartbeatMillis = currentMillis + SENDDATA * 3; // Reduce updates 3x
      }
      else {
        delay(POWERSAVE); // wait an extended period of time
      }
    }

  } // sensor poll loop
  
} // loop

/*
   LED PWM Blink for Fade in and out for ESP8266 NodeMCU
*/
void blinkLED(int pin, long duration, bool pwm)
{
  long fade;
  switch (pin) {
    case 4:
    case 12:
    case 14:
    case 15:
      // PWM capable pins
      if (pwm) {
        // PWM Fade in and out
        fade = (duration * 1000) / 2;
        for (int brightness = 0; brightness < PWMRANGE; brightness++) {
          analogWrite(pin, brightness);
          delayMicroseconds(fade / PWMRANGE);
        }
        for (int brightness = 0; brightness < PWMRANGE; brightness++) {
          analogWrite(pin, PWMRANGE - brightness);
          delayMicroseconds(fade / PWMRANGE);
        }
      }
      else {
        // Digital blink
        digitalWrite(pin, HIGH);
        delay(duration);
        digitalWrite(pin, LOW);
      }
      break;
    case 2:
    case 16:
      // Digital blink - reverse polarity
      digitalWrite(pin, LOW);
      delay(duration);
      digitalWrite(pin, HIGH);
      break;
    case 1:
    case 3:
    case 11:
    default:
      // Digital blink
      digitalWrite(pin, HIGH);
      delay(duration);
      digitalWrite(pin, LOW);
      break;
  }
}

/*
   Send Status to Web Server - water level and temperature
      waterlevel: 1=wet, 0=dry
      temp: floating point degrees celcius
      bmePressure: hPa
      bmeHumidity: percentage
      voltage: volts
      wind: rotation counts
*/
void sendStatus(int waterlevel, int id, float temp, float bmeTemp, float bmePressure, float bmeHumidity, float voltage, float wind)
{
  int count = 0;

  // Light wifi LED
  digitalWrite(LED_WIFI, LOW);
  digitalWrite(LED, HIGH);

  // Wake up WiFi
  WiFi.forceSleepWake();
  delay(1);

  // Make sure we are connected to WiFi.
  while (WiFiMulti.run() != WL_CONNECTED) {
    // Not connected - flash wifi LED and try to reconnect...LED
    digitalWrite(LED_WIFI, LOW);
    blinkLED(LED, 200, true);
    digitalWrite(LED_WIFI, HIGH);
    Serial.print(".");
    delay(300);
  }

  // Use WiFiClient class to create TCP connections
  WiFiClient client;

  // Send via HTTP GET
  while (!client.connect(host, port)) {
    count = count + 1;
    if (count > 5) {
      Serial.println("ERROR: Unable to connect - restarting...");
      for (int x = 0; x < 4; x++) {
        blinkLED(LED, 100, false);
        delay(100);
      }
      ESP.restart();
    }
    Serial.printf("ERROR: Connection failed - Waiting 5 seconds and retrying %d...\n", count);
    delay(5000);
  }
  client.print("GET ");
  client.print(URIPREFIX);
  client.print("?id=");
  client.print(id);
  client.print("&waterlevel=");
  client.print(waterlevel);
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
  client.print("&wind=");
  client.print(wind);
  if (justboot) {
    client.print("&note=poweron");
    justboot = false;
  }
  if (lowpower) {
    client.print("&warn=lowpower");
  }
  client.println(" HTTP/1.0");
  client.println();

  //read back one line from server
#ifdef DEBUG
  Serial.println("receiving from remote server");
  String line = client.readStringUntil('\r');
  Serial.println(line);
  Serial.println("closing connection");
#endif
  client.stop();

  // Send via MQTT
  // TBD

  // turn off wifi LED
  digitalWrite(LED_WIFI, HIGH);
  digitalWrite(LED, LOW);

  // Turn off WiFi to save power
  WiFi.forceSleepBegin();
  delay(1);

}