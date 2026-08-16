// Copy this file to src/wifi_config.h and add your network data. Git ignores
// src/wifi_config.h. WiFi is OPTIONAL. The RS485 translation to the Deye
// operates without WiFi. WiFi enables only the WebUI, over-the-air updates
// and MQTT.
#pragma once

#define WIFI_SSID  "YOUR_WIFI_SSID"
#define WIFI_PASS  "YOUR_WIFI_PASSWORD"

// ArduinoOTA (espota) network flashing needs this password.
#define OTA_PASS   "esmbank-ota"
