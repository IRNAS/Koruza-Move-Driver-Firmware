#include "main.h"

//#include "SendOnlySoftwareSerial.h"

void setup()
{
  char start_condition = 0;
  init_mcu();
  init_devices();

  // Start code execution by sending char a on Serial
  while (start_condition != 'a')
  {
    if (Serial.available())
    {
      start_condition = Serial.read();
    }
  }
}

void loop()
{
  test_homing_and_calibration();
  
  run_motors();
}
