#include "TLV493D.h" // https://github.com/IRNAS/TLV493D-3D-Magnetic-Sensor-Arduino-Library
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include <math.h>

const int sensor1_pwr_pin = 2;

TLV493D sensor1(sensor1_pwr_pin);

const int stepper1_a_pin = 4;
const int stepper1_b_pin = 5;
const int stepper1_c_pin = 7;
const int stepper1_d_pin = 8;

AccelStepper stepper1(AccelStepper::HALF4WIRE, stepper1_a_pin, stepper1_c_pin, stepper1_b_pin, stepper1_d_pin, false);

void setup()
{
  Serial.begin(115200);

  sensor1.init(LOW);

  stepper1.setMaxSpeed(1000);
  stepper1.setSpeed(150);
  stepper1.setAcceleration(100);
  stepper1.enableOutputs();

  delay(5000);
}

double phase_unwrap(const double angle_0_2pi)
{
  // these three lines execute only once, when calling the function for the first time
  static const double start_angle = angle_0_2pi;
  static int interval = 0;
  static double prev_value = angle_0_2pi;
  //////////////////////////////////////////////////////////////////////////////////

  double angle_continous = angle_0_2pi + (double)interval * M_PI * 2.0;

  if ((angle_continous - prev_value) < (-M_PI / 1.3))
  {
    interval++;

    if (interval != 0)
    {
      angle_continous += (double)interval * M_PI * 2.0 / abs((double)interval);
    }
  }
  else if ((angle_continous - prev_value) > (M_PI / 1.3))
  {
    interval--;

    if (interval != 0)
    {
      angle_continous += (double)interval * M_PI * 2.0 / abs((double)interval);
    }
  }

  prev_value = angle_continous;

  return angle_continous;
}

// calibration parameters
double start_point = 0.0;
double end_point = 0.0;
const int end_step = 4096;

void calibrate()
{
  double angle = 0.0;

  for (unsigned long i = 0; i < 9; i++)
  {
    stepper1.moveTo(i * 512);

    while (true)
    {
      stepper1.run();

      if (!stepper1.isRunning())
      {
        while (!sensor1.update());

        Serial.print(i * 512);
        Serial.print(";");//\t");

        angle = phase_unwrap(sensor1.m_dPhi_yz);
        Serial.println(angle, 9);

        break;
      }
    }

    if(i == 0) start_point = angle;
  }

  stepper1.disableOutputs();

  end_point = angle;
}

// wrap the value around the limits
int wrap (int value, const int min_value, const int max_value)
{
  if (min_value > max_value) Serial.println("min_value value of wrap function must be smaler than max_value value.");

  while ((value >= max_value) || (value < min_value))
  {
    if (value >= max_value)
    {
      if (min_value == max_value) return value;
      value -= max_value - min_value;
    }
    else if (value < min_value)
    {
      value += max_value - min_value;
    }
  }

  return value;
}

int calculate_step(double senzor_reading)
{
  int motor_step = (senzor_reading - start_point) * (double)end_step / (end_point - start_point);
  motor_step = wrap(motor_step, 0, end_step);
  return (motor_step);
}

void loop()
{
  calibrate();

  stepper1.moveTo(2000);

  double sezor_reading = 0.0;

  while (true)
  {
    stepper1.run();

    if (!stepper1.isRunning())
    {
      while (!sensor1.update());
      sezor_reading = sensor1.m_dPhi_yz;
      Serial.print(sezor_reading);
      Serial.print(";");
      Serial.println(calculate_step(sezor_reading));

      break;
    }
  }

  stepper1.disableOutputs();

  while (true);
}
