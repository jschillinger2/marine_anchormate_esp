#pragma once

// Pin definitions
#define PIN_UP 14
#define PIN_DOWN 13
#define PIN_PULSE 17
#define PIN_PULSE_FREQUENCY 20

// Signal K timing configuration
#define SIGNALK_RELAY_CHECK_FREQUENCY 50
#define SIGNALK_HEARTBEAT_CHECK_FREQUENCY 300
#define SIGNALK_HEARTBEAT_CHECK_THRESHOLD 1500

// Signal K paths
#define SK_PATH_HEARTBEAT "vessels.self.anchor.control.heartbeat"
#define SK_PATH_CONTROL "vessels.self.anchor.control"
#define SK_PATH_CONTROL_UP "UP"
#define SK_PATH_CONTROL_DOWN "DOWN"
#define SK_PATH_ROTATIONS "sensors.windlass.rotations"
#define SK_PATH_ROTATIONS_LABEL "Windlass Rotations"

// Network configuration
#define WIFI_SSID "xx"
#define WIFI_PASSWORD "xx"
#define SIGNALK_HOSTNAME "AnchorMate"
