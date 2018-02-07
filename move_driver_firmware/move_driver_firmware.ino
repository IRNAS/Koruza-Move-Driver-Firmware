#include "main.h"

//#include "SendOnlySoftwareSerial.h"

void setup()
{
  init_mcu();
  init_devices();
}

void loop()
{
  test_homing_and_calibration();
  
  run_motors();
}
