#include "TLV493D.h" // https://github.com/IRNAS/TLV493D-3D-Magnetic-Sensor-Arduino-Library
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/

const int sensor1_pwr_pin = 2;

TLV493D sensor1(sensor1_pwr_pin);

const int stepper1_a_pin = 4;
const int stepper1_b_pin = 5;
const int stepper1_c_pin = 7;
const int stepper1_d_pin = 8;

AccelStepper stepper1(AccelStepper::FULL4WIRE, stepper1_a_pin, stepper1_c_pin, stepper1_b_pin, stepper1_d_pin, false);

void setup()
{
  Serial.begin(115200);

  sensor1.init(LOW);

  stepper1.setMaxSpeed(100);
  stepper1.setSpeed(100);
  stepper1.setAcceleration(100);
  stepper1.enableOutputs();

  delay(5000);
}

void loop()
{
  for (unsigned long i = 0; i < 4096; i++)
  {
    stepper1.moveTo(i);
    
    while (true)
    {
      stepper1.run();

      if (!stepper1.isRunning())
      {
        while(!sensor1.update());

        Serial.print(i);
        Serial.print(";");//\t");
        Serial.print(sensor1.m_dPhi_xy, 9);
        Serial.print(";");//\t");
        Serial.print(sensor1.m_dPhi_yz, 9);
        Serial.print(";");//\t");
        Serial.print(sensor1.m_dPhi_xz, 9);
        Serial.print(";");//\t");
        Serial.println(sensor1.m_dMag_2, 9);

        break;
      }
    }
  }

  stepper1.disableOutputs();
  while(true);
}
