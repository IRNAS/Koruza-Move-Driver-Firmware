#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "TLV493D.h" // https://github.com/IRNAS/TLV493D-3D-Magnetic-Sensor-Arduino-Library
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "Wire.h"
#include "calibration.h"
#include <math.h>


const long Calibration::m_steps_per_revolution = 4096;


Calibration::Calibration(Switch& limit_switch, TLV493D& sensor, AccelStepper& stepper) :
  m_limit_switch(limit_switch),
  m_sensor(sensor),
  m_stepper(stepper),
  m_start_point(0.0),
  m_end_point(0.0),
  m_start_step(0)
{
}


double Calibration::phase_unwrap(const double angle_0_2pi, const double prev_value, long& interval)
{
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

  return angle_continous;
}


// wrap the value around the limits
uint8_t Calibration::wrap(long& value, const long min_value, const long max_value)
{
  //return status:
  //  0x00 = OK
  //  0x01 = invalid parameters

  if (min_value > max_value) return 0x01; // error

  while ((value >= max_value) || (value < min_value))
  {
    if (value >= max_value)
    {
      if (min_value == max_value) return 0x00; // success
      value -= max_value - min_value;
    }
    else if (value < min_value)
    {
      value += max_value - min_value;
    }
  }

  return 0x00; // success
}


uint8_t Calibration::calibrate(const long N_points)
{
  //return status:
  //  0x00 = OK
  //  0x01 = sensor error, invalid num. of bytes received
  //  0x02 = sensor error, sensor busy
  //  0x03 = invalid num. of points given
  //  0x04 = limit switch pressed

  Serial.println("Calibration started.");

  if ((N_points > m_steps_per_revolution) || (N_points < 2)) return 0x03; // error

  double angle_meas_unwrap = 0.0;
  long interval = 0;
  double prev_value = 0.0;

  int count_tries = 100;
  uint8_t status_sensor = 0x00;

  m_start_step = m_stepper.currentPosition();
  Serial.print("Starting step: ");
  Serial.println(m_start_step);

  for (long i = 0; i < N_points; i++)
  {
    m_stepper.setSpeed(100);
    m_stepper.setAcceleration(50);
    m_stepper.move(i * m_steps_per_revolution / (N_points - 1));

    while (true)
    {
      if (m_limit_switch.get_button_state() == true) return 0x04;
      m_stepper.run();

      if (!m_stepper.isRunning())
      {
        // try to update sensor value
        while (count_tries > 0)
        {
          status_sensor = m_sensor.update();
          if ((status_sensor != 0x01) && (status_sensor != 0x02)) break;
          count_tries --;
        }
        if (status_sensor != 0x00) return status_sensor; // error

        if (i == 0) prev_value = m_sensor.m_dPhi_yz;

        Serial.print(i * m_steps_per_revolution / (N_points - 1));
        Serial.print(";");

        angle_meas_unwrap = phase_unwrap(m_sensor.m_dPhi_yz, prev_value, interval);
        prev_value = angle_meas_unwrap;
        Serial.println(angle_meas_unwrap, 10);

        break;
      }
    }

    if (i == 0) m_start_point = angle_meas_unwrap;
  }

  m_end_point = angle_meas_unwrap;

  // move back to starting position
  m_stepper.move(-m_steps_per_revolution);
  double sezor_reading = 0.0;

  while (true)
  {
    if (m_limit_switch.get_button_state() == true) return 0x04;
    m_stepper.run();
    if (!m_stepper.isRunning()) break;
  }
  
  m_stepper.disableOutputs();

  return 0x00; // success
}


uint8_t Calibration::calculate_step(const double sensor_reading, long& motor_step)
{
  //return status:
  //  0x00 = OK
  //  0x01 = invalid wrap function parameters

  motor_step = -200 /*+ m_start_step*/ + (long)( (sensor_reading - m_start_point) * (double)m_steps_per_revolution / (m_end_point - m_start_point) );

  if (wrap(motor_step, 0, m_steps_per_revolution) != 0x00) return 0x01; // error

  return 0x00; // success
}
