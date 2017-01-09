#include "communication.h"

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  while (Serial.available())
  {
    receiveBytes((uint8_t)Serial.read());
  }
}
