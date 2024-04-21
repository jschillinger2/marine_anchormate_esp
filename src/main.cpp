#include <iostream>

#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/linear.h"
#include "sensesp_app_builder.h"
#include "Arduino.h"

#include "sensesp_app.h"
#include "sensesp_app_builder.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/digital_output.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_listener.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/debounce.h"
#include "sensesp/transforms/integrator.h"
#include "sensesp/signalk/signalk_value_listener.h"
#include "sensesp/transforms/threshold.h"

#include "sensesp_minimal_app_builder.h"

using namespace sensesp;

ReactESP app;

#define PIN_UP 14
#define PIN_DOWN 13
#define PIN_PULSE 17
#define PIN_PULSE_FREQUENCY 20
#define SIGNALK_RELAY_CHECK_FREQUENCY 50
#define SIGNALK_HEARTBEAT_CHECK_FREQUENCY 300
#define SIGNALK_HEARTBEAT_CHECK_THRESHOLD 1500

const char *SK_PATH_HEARTBEAT = "vessels.self.anchor.control.heartbeat";
const char *SK_PATH_CONTROL = "vessels.self.anchor.control";
const char *SK_PATH_CONTROL_UP = "UP";
const char *SK_PATH_CONTROL_DOWN = "DOWN";
const char *SK_PATH_ROTATIONS = "sensors.windlass.rotations";
const char *SK_PATH_ROTATIONS_LABEL = "Windlass Rotations";

void setupRotationSensor();
void setupRelayOutputs();
void setupHeartbeatListener();
void HeartBeatTaskFunction(void *pvParameters);

const String WIFI_SSID = "xx";
const String WIFI_PASSWORD = "xx";
const String SIGNALK_HOSTNAME = "AnchorMate";

volatile unsigned long currentMillis;
volatile bool heartbeat = false;

void setup()
{

#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  currentMillis = 0;

  // reset flash and wifi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  esp_err_t ret = nvs_flash_init();
  ESP_ERROR_CHECK(nvs_flash_erase());
  ret = nvs_flash_init();
  ESP_ERROR_CHECK(ret);
  delay(1000);

  // init pins
  pinMode(PIN_UP, OUTPUT);
  pinMode(PIN_DOWN, OUTPUT);

  // init wifi
  //WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Attempting to connect to Wifi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(5000);
    Serial.print(".");
  }
  std::cout << "WIFI connected" << std::endl;

  // create app
  SensESPAppBuilder builder;
  auto sensesp_app = builder.set_hostname(SIGNALK_HOSTNAME)->get_app();

  // set up sensors
  sleep(2);
  setupRotationSensor();
  setupRelayOutputs();
  setupHeartbeatListener();
  sleep(2);

  // Configuration is done, lets start the readings of the sensors!
  sensesp_app->start();
  sleep(5);

  xTaskCreate(HeartBeatTaskFunction, "HeartbeatMonitorTask", 10000, NULL, 1, NULL);
}

class SKPathHandler : public ValueConsumer<String>
{
public:
  void set_input(String value, uint8_t input_channel = 0) override
  {

    if (value == SK_PATH_CONTROL_UP)
    {
      std::cout << "UP" << std::endl;
      if (heartbeat)
        digitalWrite(PIN_UP, HIGH);
      else
        Serial.println("Cant move anchor. No Heartbeat.");

      digitalWrite(PIN_DOWN, LOW);
    }
    else if (value == SK_PATH_CONTROL_DOWN)
    {
      std::cout << "DOWN" << std::endl;
      digitalWrite(PIN_UP, LOW);
      if (heartbeat)
        digitalWrite(PIN_DOWN, HIGH);
      else
        Serial.println("Cant move anchor. No Heartbeat.");
    }
    else
    {
      std::cout << "OFF" << std::endl;
      digitalWrite(PIN_UP, LOW);
      digitalWrite(PIN_DOWN, LOW);
    }
  }
};

class SKPathHandlerHeartbeat : public ValueConsumer<String>
{
public:
  void set_input(String value, uint8_t input_channel = 0) override
  {
    Serial.print(".");
    currentMillis = millis();
  }
};

void setupHeartbeatListener()
{
  auto *listener = new StringSKListener(SK_PATH_HEARTBEAT, SIGNALK_HEARTBEAT_CHECK_FREQUENCY);
  auto *pathHandler = new SKPathHandlerHeartbeat();
  listener->connect_to(pathHandler);
}

void setupRelayOutputs()
{
  auto *listener = new StringSKListener(SK_PATH_CONTROL, SIGNALK_RELAY_CHECK_FREQUENCY);
  auto *pathHandler = new SKPathHandler();
  listener->connect_to(pathHandler);
}

void setupRotationSensor()
{

  const uint8_t kDigitalInput2Pin = PIN_PULSE;
  const unsigned int kDigitalInput2Interval = PIN_PULSE_FREQUENCY;

  pinMode(kDigitalInput2Pin, INPUT_PULLUP);

  SKOutputInt *skRotationsOut = new SKOutputInt(
      SK_PATH_ROTATIONS, // Signal K path
      new SKMetadata("", // No units for boolean values
                     SK_PATH_ROTATIONS_LABEL));

  int *x = new int(0);
  (*skRotationsOut).set_input(*x);
  int *prevState = new int(0);

  auto *digital_input2 = new RepeatSensor<int>(
      kDigitalInput2Interval,
      [kDigitalInput2Pin, x, prevState, skRotationsOut]() mutable
      {
        int *newState = new int(0);
        (*newState) = digitalRead(kDigitalInput2Pin);
        if ((*prevState) != (*newState) && *newState == 1)
        {
          (*x)++;
          (*skRotationsOut).set_input(*x);
        }
        *prevState = *newState;
        delete newState;
        return *x;
      });

  // delete prevState;
  // delete x;
}

void HeartBeatTaskFunction(void *pvParameters)
{

  for (;;)
  {
    // std::cout << " HEARTBEAT TASK " << std::endl;
    // std::cout << millis() << "-" << currentMillis << std::endl;
    // std::cout << (millis() - currentMillis) << "-" << currentMillis << std::endl;
    // std::cout << (millis() - currentMillis > SIGNALK_HEARTBEAT_CHECK_THRESHOLD) << "-" << currentMillis << std::endl;

    if (millis() - currentMillis > SIGNALK_HEARTBEAT_CHECK_THRESHOLD)
    {
      Serial.print("x");
      digitalWrite(PIN_UP, LOW);
      digitalWrite(PIN_DOWN, LOW);
      heartbeat = false;
    }
    else
    {
      if (!heartbeat)
        std::cout << "HEARTBEAT -> ON" << std::endl;
      heartbeat = true;
    }
    delay(SIGNALK_HEARTBEAT_CHECK_FREQUENCY);
  }
}

// main program loop
void loop()
{
  app.tick();
}