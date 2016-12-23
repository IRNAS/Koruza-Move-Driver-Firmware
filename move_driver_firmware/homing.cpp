#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "homing.h"


extern Switch limit_switch1;
extern Switch limit_switch2;
extern AccelStepper stepper1;
extern AccelStepper stepper2;

void homing()
{
  Serial.println("Homing started.");

  stepper1.setCurrentPosition(0);
  stepper1.setMaxSpeed(1000);
  stepper1.setSpeed(100);
  stepper1.setAcceleration(50);
  stepper1.move(-1000000);
  stepper1.enableOutputs();

  stepper2.setCurrentPosition(0);
  stepper2.setMaxSpeed(1000);
  stepper2.setSpeed(100);
  stepper2.setAcceleration(50);
  stepper2.move(-1000000);
  stepper2.enableOutputs();

  while (true)
  {
    if ((limit_switch1.get_button_state() == true) && (stepper1.distanceToGo() < 0))
    {
      //Serial.println("Limit switch 1 pressed.");
      delay(1000);
      stepper1.setCurrentPosition(0);
      stepper1.setMaxSpeed(1000);
      stepper1.setSpeed(100);
      stepper1.setAcceleration(50);
      stepper1.move(10000);
      continue;
    }

    if ((limit_switch2.get_button_state() == true) && (stepper2.distanceToGo() < 0))
    {
      //Serial.println("Limit switch 2 pressed.");
      delay(1000);
      stepper2.setCurrentPosition(0);
      stepper2.setMaxSpeed(1000);
      stepper2.setSpeed(100);
      stepper2.setAcceleration(50);
      stepper2.move(10000);
      continue;
    }

    //if (stepper1.distanceToGo() != 0) Serial.println("Stepper 1 moving.");
    stepper1.run();

    //if (stepper2.distanceToGo() != 0) Serial.println("Stepper 2 moving.");
    stepper2.run();

    if (stepper1.distanceToGo() == 0)
    {
      Serial.println("Homing 1 finished.");
      stepper1.disableOutputs();
    }

    if (stepper2.distanceToGo() == 0)
    {
      Serial.println("Homing 2 finished.");
      stepper2.disableOutputs();
    }

    if ((stepper1.distanceToGo() == 0) && (stepper2.distanceToGo() == 0)) return;
  }
}
