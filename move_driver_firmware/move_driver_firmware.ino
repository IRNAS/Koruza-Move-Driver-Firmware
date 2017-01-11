#include "main.h"

void setup()
{
  init_mcu();
  init_devices();
}

void loop()
{
  run_motors();
  communicate();
  receive_bytes();
}


