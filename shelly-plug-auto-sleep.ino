#define DEBUG 0
#define MONITOR_SERIAL Serial
#define RADAR_SERIAL Serial1
#define RADAR_RX_PIN 32
#define RADAR_TX_PIN 33
#define MAX_DISTANCE 90
#define READ_DELAY 2500
//                 millis * seconds * minutes
#define MINIMUM_TIME_OFF 1000 * 30 // 30 seconds

#if DEBUG
  #define POWER_OFF_DELAY 1000 * 60 * 1 // 1 minute
  #define DEBUG_PRINT(...) (Serial.print("DEBUG: "), Serial.print(__VA_ARGS__))
  #define DEBUG_PRINTLN(...) (Serial.print("DEBUG: "), Serial.println(__VA_ARGS__))
  #define DEBUG_PRINTFLN(...) (Serial.print("DEBUG: "), Serial.printf(__VA_ARGS__), Serial.println(""))
#else
  #define POWER_OFF_DELAY 1000 * 60 * 60 // 1 hour
  #define DEBUG_PRINT(...) ((void)0)
  #define DEBUG_PRINTLN(...) ((void)0)
  #define DEBUG_PRINTFLN(...) ((void)0)
#endif

#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <ld2410.h>
#include <TinyPICO.h>
#include <time.h>
#include "secrets.h"

/* 
 *  Copy secrets.h.example to secrets.h and then change the macros in secrets.h 
 *  SSID, PASS, IP
 */
const char* ssid     = SSID;
const char* password = PASS;

const String ip = IP;

/*
 *  End of variables you need to change.
 *  You may want to change variables below ¯\_(ツ)_/¯
 */

ld2410 radar;
WiFiMulti wifiMulti;

// Pins for switch
const uint8_t S_OFF = 14;
const uint8_t S_ON = 15;

TinyPICO tp = TinyPICO();

// NTP Settings
const char* ntpServer        = "pool.ntp.org";
const long gmtOffset_sec     = -8 * 60 * 60;   // GMT -8
const int daylightOffset_sec = 60 * 60;        // 1 hour DST offset

uint32_t offTime     = 0;
uint32_t lastMotion  = 0;
uint32_t lastReading = 0;

bool ledOff = false;

/* state
 * 0 = Init
 * 1 = On
 * 2 = Off
 */

int8_t state = 0;

void rest_api_action(int action) {
  HTTPClient http;
  int32_t httpCode;
  String url;

  if ((wifiMulti.run() == WL_CONNECTED)) {
    switch (action) {
      case 0: // turn on the device
        url = "http://" + ip + "/rpc/Switch.Set?id=0&on=true";
        break;
      case 1:
        url = "http://" + ip + "/rpc/Switch.Set?id=0&on=false";
        break;
    }
    http.begin(url);

    MONITOR_SERIAL.print("WEB: ");
    if (http.GET() == HTTP_CODE_OK) {
      MONITOR_SERIAL.printf("Successfully sent %s request.\n", status(action));
    } else {
      MONITOR_SERIAL.println(F("Failed to send request."));
    }

    http.end();
  }
}

const char* status(int action) {
  return action ? "off" : "on";
}

void power_off(void) {
  state = 2;
  DEBUG_PRINTLN(F("Powering device off."));
  tp.DotStar_SetPixelColor(0xff0000);
  #if !DEBUG
    rest_api_action(1);
  #endif
  offTime = millis();
}

void power_on(void) {
  state = 1;
  DEBUG_PRINTLN(F("Powering on."));
  tp.DotStar_SetPixelColor(0x00ff00);
  ledOff = false;
  #if !DEBUG
    rest_api_action(0);
  #endif
}


bool minOffTimeMet(int t) {
  return state == 0 || (millis() - offTime >= t);
}

void detectMotion(void) {
  DEBUG_PRINTLN(F("[+] Detecting movement..."));
  if (radar.movingTargetDetected() && radar.movingTargetDistance() <= MAX_DISTANCE) {
    DEBUG_PRINTLN(F("[!] Moving target detected."));
    DEBUG_PRINTFLN("[!!] Distance: %d", radar.movingTargetDistance());
    lastMotion = millis();
    return;
  }

  uint32_t d0;
  d0 = radar.stationaryTargetDistance();
  delay(1000);
  if (d0 <= MAX_DISTANCE && d0 != radar.stationaryTargetDistance()) {
    DEBUG_PRINTLN(F("[!] Stationary target detected."));
    lastMotion = millis();
  }
}

void setup(void) {
  tp.DotStar_SetBrightness(10);
  tp.DotStar_SetPixelColor(0x0000ff);
  MONITOR_SERIAL.begin(115200);
  MONITOR_SERIAL.println("Booting...");
  delay(500);

  MONITOR_SERIAL.println("Configure toggle switch...");
  pinMode(S_ON, INPUT_PULLUP);
  pinMode(S_OFF, INPUT_PULLUP);
  delay(500);

  MONITOR_SERIAL.println("Setting up WiFi...");
  wifiMulti.addAP(ssid, password);
  delay(500);  

  MONITOR_SERIAL.println("Configuring Sensor...");
  RADAR_SERIAL.begin(256000, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  delay(500);
  MONITOR_SERIAL.print(F("\nConnect LD2410C radar TX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_RX_PIN);
  MONITOR_SERIAL.print(F("Connect LD2410 radar RX to GPIO:"));
  MONITOR_SERIAL.println(RADAR_TX_PIN);
  MONITOR_SERIAL.print(F("LD2410 radar sensor initialising: "));
  if(radar.begin(RADAR_SERIAL))
  {
    MONITOR_SERIAL.println(F("OK"));
    MONITOR_SERIAL.print(F("LD2410 firmware version: "));
    MONITOR_SERIAL.print(radar.firmware_major_version);
    MONITOR_SERIAL.print('.');
    MONITOR_SERIAL.print(radar.firmware_minor_version);
    MONITOR_SERIAL.print('.');
    MONITOR_SERIAL.println(radar.firmware_bugfix_version, HEX);
  }
  else
  {
    MONITOR_SERIAL.println(F("not connected"));
  }
  delay(500);

  if (wifiMulti.run() == WL_CONNECTED) {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  }

  MONITOR_SERIAL.println("Booted.");
}

void loop() {
  radar.read();
  if (millis() - lastReading > READ_DELAY) {
    // Read device every READ_DELAY while device is connected.
    lastReading = millis();
    
    if (state == 2 && minOffTimeMet(MINIMUM_TIME_OFF) && !ledOff) {
      // if the device is off and the led has been on longer than MINIMUM_TIME_OFF
      // then turn off the led
      DEBUG_PRINTFLN("Off for more than %d.", MINIMUM_TIME_OFF);
      DEBUG_PRINTFLN("Light off millis(): %d", millis());
      tp.DotStar_Clear();
      ledOff = true;
    }

    if (digitalRead(S_OFF) == LOW && digitalRead(S_ON) == HIGH) {
      // Switch set to always off.
      DEBUG_PRINTLN(F("Switch is off."));
      if (state != 2) {
        power_off();
      }
    } else if (digitalRead(S_OFF) == HIGH && digitalRead(S_ON) == LOW) {
      // Switch set to always on.
      DEBUG_PRINTLN(F("Switch is on."));
      if (state != 1) {
        power_on();
      }
    } else if (digitalRead(S_OFF) == HIGH && digitalRead(S_ON) == HIGH) {
      DEBUG_PRINTLN(F("Switch is neutral."));
      if (radar.isConnected()) {
        if (radar.presenceDetected() && radar.stationaryTargetDetected()) {
          // Presence detected
          detectMotion();

          if (state != 1 && minOffTimeMet(MINIMUM_TIME_OFF) && radar.stationaryTargetDistance() <= MAX_DISTANCE) {
            // stationary target is less than 100cm away, device is not powered on, and device has been off for at least MINIUM_TIME_OFF.
            power_on();
          } else if (state != 2 && (millis() - lastMotion >= POWER_OFF_DELAY) && (radar.stationaryTargetDistance() > MAX_DISTANCE)) {
            // stationary target is more than 100cm away, POWER_OFF_DELAY has been met, and device is not powered off.
            power_off();
          }
        } else {
          // No presence detected
          if (state != 0 && state != 2 && millis() - lastMotion >= POWER_OFF_DELAY) {
            DEBUG_PRINTFLN("Power off millis(): %d", millis());
            power_off();
          }
        }
      }
    }
  }
}
