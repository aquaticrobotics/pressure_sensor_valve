#pragma once

// Copy to secrets.h and enter local Wi-Fi credentials. Leave station values empty for AP-only mode.
#define WIFI_SSID ""
#define WIFI_PASSWORD ""

// The fallback AP is always started. Change this placeholder before deployment.
#define AP_SSID "SprinklerController"
#define AP_PASSWORD "ChangeMe123"

// Set the same unique value in the ignored platformio.local.ini upload_flags.
#define OTA_PASSWORD "change-this-unique-ota-password"
