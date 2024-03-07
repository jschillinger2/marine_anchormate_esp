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

using namespace sensesp;

ReactESP app;

#define PIN_UP 13
#define PIN_DOWN 14
#define PIN_PULSE 17
#define PIN_PULSE_FREQUENCY 20
#define SIGNALK_RELAY_CHECK_FREQUENCY 50


void setupRotationSensor();
void setupRelayOutputs();

const String WIFI_SSID = "rabbitAll";
const String WIFI_PASSWORD = "xx";
const String SIGNALK_HOSTNAME = "AnchorMate";

void setup()
{

#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // reset flash and wifi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  esp_err_t ret = nvs_flash_init();
  ESP_ERROR_CHECK(nvs_flash_erase());
  ret = nvs_flash_init();
  ESP_ERROR_CHECK(ret);

  // Create the global SensESPApp() object.
  SensESPAppBuilder builder;
  builder.set_hostname(SIGNALK_HOSTNAME);
  builder.set_wifi(WIFI_SSID, WIFI_PASSWORD);
  sensesp_app = builder.get_app();

  // set up sensors
  setupRotationSensor();
  setupRelayOutputs();

  // Configuration is done, lets start the readings of the sensors!
  sensesp_app->start();
}

class SKPathHandler : public ValueConsumer<String>
{
public:
  void set_input(String value, uint8_t input_channel = 0) override
  {
    pinMode(PIN_UP, OUTPUT);
    pinMode(PIN_DOWN, OUTPUT);

    if (value == "UP")
    {
      std::cout << "UP" << std::endl;
      digitalWrite(PIN_UP, HIGH);
      digitalWrite(PIN_DOWN, LOW);
    }
    else if (value == "DOWN")
    {
      std::cout << "DOWN" << std::endl;
      digitalWrite(PIN_UP, LOW);
      digitalWrite(PIN_DOWN, HIGH);
    }
    else
    {
      std::cout << "OFF" << std::endl;
      digitalWrite(PIN_UP, LOW);
      digitalWrite(PIN_DOWN, LOW);
    }
  }
};

void setupRelayOutputs()
{
  const char *sk_path = "vessels.self.anchor.control";
  auto *listener = new StringSKListener(sk_path, SIGNALK_RELAY_CHECK_FREQUENCY);
  auto *pathHandler = new SKPathHandler();
  listener->connect_to(pathHandler);
}

void setupRotationSensor()
{

  const uint8_t kDigitalInput2Pin = PIN_PULSE;
  const unsigned int kDigitalInput2Interval = PIN_PULSE_FREQUENCY;
  std::cout << "####1" << std::endl;
  pinMode(kDigitalInput2Pin, INPUT_PULLUP);
  std::cout << "####2" << std::endl;

  sleep(5);
  std::cout << "####3" << std::endl;

  SKOutputInt *skRotationsOut = new SKOutputInt(
      "sensors.windlass.rotations",  // Signal K path
      "/sensors/windlass/rotations", // configuration path
      new SKMetadata("",             // No units for boolean values
                     "Rotation Count"));
  std::cout << "####4" << std::endl;

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
}

// main program loop
void loop()
{
  app.tick();
}