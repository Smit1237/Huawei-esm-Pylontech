// Copy this file to  src/wifi_config.h  (which is git-ignored) and fill in your
// network. WiFi is OPTIONAL: the RS485 translation to the Deye runs without it.
// WiFi only enables the WebUI, over-the-air updates and MQTT.
#pragma once

#define WIFI_SSID  "YOUR_WIFI_SSID"
#define WIFI_PASS  "YOUR_WIFI_PASSWORD"

// Password required for ArduinoOTA (espota) network flashing.
#define OTA_PASS   "esmbank-ota"
