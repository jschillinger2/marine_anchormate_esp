#include <iostream>

#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/transforms/linear.h"
#include "sensesp_app_builder.h"

#include "sensesp_app.h"
#include "sensesp_app_builder.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/debounce.h"
#include "sensesp/transforms/integrator.h"

using namespace sensesp;

ReactESP app;

void setupRotationSensor();
void setupRotationSensor2();

const String WIFI_SSID = "Huette";
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

  // Configuration is done, lets start the readings of the sensors!
  sensesp_app->start();
}

// setup seaock open digital sensor
void setupRotationSensor()
{

  const uint8_t kDigitalInput2Pin = 17;
  const unsigned int kDigitalInput2Interval = 20;

  pinMode(kDigitalInput2Pin, INPUT_PULLUP);

  SKOutputInt *skRotationsOut = new SKOutputInt(
      "sensors.windlass.rotations",  // Signal K path
      "/sensors/windlass/rotations", // configuration path
      new SKMetadata("",            // No units for boolean values
                     "Rotation Count"));

  // Define a new RepeatSensor that reads the pin every 20 ms. Whenever
  // pin changes from false to true count the rotations

  std::cout << "pre func" << std::endl;

  int *x = new int(0);
  (*skRotationsOut).set_input(*x);
  // bool *prevState = int(0);
  int *prevState = new int(0);

  std::cout << "pre func2" << std::endl;

  auto *digital_input2 = new RepeatSensor<int>(
      kDigitalInput2Interval,
      [kDigitalInput2Pin, x, prevState, skRotationsOut]() mutable
      {
        // std::cout << "in func 1" << std::endl;

        int *newState = new int(0);
        // std::cout << "in func 2" << std::endl;

        (*newState) = digitalRead(kDigitalInput2Pin);
        // std::cout << "in func 3" << std::endl;

        if ((*prevState) != (*newState) && *newState == 1)
        {
          (*x)++;
          (*skRotationsOut).set_input(*x);
        }
        // std::cout << "in func 4" << std::endl;

        

        *prevState = *newState;

        // std::cout << "in func 5" << std::endl;

        return *x;
      });

  std::cout << "post func" << std::endl;

  // Connect digital input to Signal K output.
  /*digital_input2->connect_to(new SKOutputInt(
      "sensors.windlass.rotation",     // Signal K path
      "/sensors/windlass/rotation",    // configuration path
      new SKMetadata("",               // No units for boolean values
                     "Rotation Count") // Value description
      )); */
}

// main program loop
void loop()
{
  app.tick();
}