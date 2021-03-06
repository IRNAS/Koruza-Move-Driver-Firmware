#include "calibration.h"
#include <math.h>


String convertToString(const CalibrationState& state)
{
  switch (state)
  {
    case (CalibrationState::STANDBY): return (String("STANDBY"));
    case (CalibrationState::NEXT_POINT): return (String("NEXT_POINT"));
    case (CalibrationState::RUN_MOTOR): return (String("RUN_MOTOR"));
    case (CalibrationState::READ_SENSOR): return (String("READ_SENSOR"));
    case (CalibrationState::ERROR): return (String("ERROR"));
  }
}


String convertToString(const CalibrationStatus& status)
{
  switch (status)
  {
    case (CalibrationStatus::OK): return (String("OK"));
    case (CalibrationStatus::SENSOR_COMM_ERROR): return (String("SENSOR_COMM_ERROR"));
    case (CalibrationStatus::SENSOR_BUSSY): return (String("SENSOR_BUSSY"));
    case (CalibrationStatus::LIMIT_SWITCH_PRESSED): return (String("LIMIT_SWITCH_PRESSED"));
  }
}


const long Calibration::m_steps_per_revolution = 4096;


Calibration::Calibration(Switch& limit_switch, TLV493D& sensor, AccelStepper& stepper) :
  m_limit_switch(limit_switch),
  m_sensor(sensor),
  m_stepper(stepper)
{
  m_state = CalibrationState::STANDBY;
  m_status = CalibrationStatus::OK;

  // calibration parameters
  m_start_step = 0;
  m_start_point = 0;
  m_end_point = 1.0;

  m_N_cal_points = m_steps_per_revolution;
  m_current_point = 0;

  m_angle_meas_unwrap = 0.0;
  m_interval = 0;
  m_prev_value = 0.0;
}



CalibrationState Calibration::currentState()
{
  return m_state;
}


CalibrationStatus Calibration::currentStatus()
{
  return m_status;
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


uint8_t Calibration::calculate_step(long& motor_step)
{
  //return status:
  //  0x00 = OK
  //  0x01 = sensor communciation error
  //  0x02 = sensor bussy
  //  0x03 = invalid wrap function parameters

  // try to update sensor value
  int count_tries = 1000;
  uint8_t status_sensor = 0x00;

  while (count_tries > 0)
  {
    status_sensor = m_sensor.update();
    if (status_sensor != 0x02) break; // if 0x02 the sensor is busy, need to try again
    count_tries --;
  }
  if (status_sensor != 0x00) return status_sensor; // error

  motor_step = -100 /*+ m_start_step*/ + (long)( (m_sensor.m_dPhi_xz - m_start_point) * (double)m_steps_per_revolution / (m_end_point - m_start_point) );

  if (wrap(motor_step, 0, m_steps_per_revolution) != 0x00) return 0x03; // error

  return 0x00; // success
}


void Calibration::reset()
{
  m_state = CalibrationState::STANDBY;
  m_status = CalibrationStatus::OK;
  m_stepper.disableOutputs();
}


bool Calibration::start(const long N_points)
{
  if ((N_points > m_steps_per_revolution) || (N_points < 2)) return true; // error

  switch (m_state)
  {
    case (CalibrationState::STANDBY):
      {
        m_N_cal_points = N_points;
        m_current_point = 0;

        m_stepper.setSpeed(100);
        m_stepper.setAcceleration(50);
        m_stepper.enableOutputs();

        m_start_step = m_stepper.currentPosition();

        m_state = CalibrationState::NEXT_POINT;
        break;
      }
    case (CalibrationState::NEXT_POINT):
      {
        break;
      }
    case (CalibrationState::RUN_MOTOR):
      {
        break;
      }
    case (CalibrationState::READ_SENSOR):
      {
        break;
      }
    case (CalibrationState::ERROR):
      {
        reset();
        return start(N_points);
      }
    default:
      {
        break;
      }
  }

  return false; // success
}


void Calibration::process()
{
  switch (m_state)
  {
    case (CalibrationState::STANDBY):
      {
        break;
      }
    case (CalibrationState::NEXT_POINT):
      {
        // movement relative to current position
        m_stepper.moveTo(m_start_step + m_current_point * m_steps_per_revolution / (m_N_cal_points - 1));

//        Serial.println("NEXT_POINT");
//        Serial.print("m_stepper.currentPosition(): "); Serial.println(m_stepper.currentPosition());
//        Serial.print("m_stepper.distanceToGo(): "); Serial.println(m_stepper.distanceToGo());
        
        m_state = CalibrationState::RUN_MOTOR;

        break;
      }
    case (CalibrationState::RUN_MOTOR):
      {
        if ((m_limit_switch.get_button_state() == true) && (m_stepper.distanceToGo() < 0))
        {
//          Serial.println("m_limit_switch pressed");
          
          m_status = CalibrationStatus::LIMIT_SWITCH_PRESSED;
          m_state = CalibrationState::ERROR;
          break;
        }

        m_stepper.run();

        if (!m_stepper.isRunning())
        {
//          Serial.println("RUN_MOTOR");
//          Serial.print("m_stepper.currentPosition(): "); Serial.println(m_stepper.currentPosition());
//          Serial.print("m_stepper.distanceToGo(): "); Serial.println(m_stepper.distanceToGo());
          m_state = CalibrationState::READ_SENSOR;
          break;
        }

        break;
      }
    case (CalibrationState::READ_SENSOR):
      {
        int count_tries = 1000;
        uint8_t status_sensor = 0x00;

        // try to update sensor value
        while (count_tries > 0)
        {
          status_sensor = m_sensor.update();
          if (status_sensor != 0x02) break;
          count_tries --;
        }

        switch (status_sensor)
        {
          case (0x00):
            {
              //Serial.println("READ_SENSOR");
              //Serial.print("m_stepper.currentPosition(): "); Serial.println(m_stepper.currentPosition());
              //Serial.print("m_stepper.distanceToGo(): "); Serial.println(m_stepper.distanceToGo());
              //Serial.print("m_current_point: "); Serial.println(m_current_point);
              
              if (m_current_point == 0)
              {
                m_interval = 0;
                m_prev_value = m_sensor.m_dPhi_xz;
              }

              //Serial.print("m_prev_value: "); Serial.println(m_prev_value);
              //Serial.print("m_interval: "); Serial.println(m_interval);

              m_angle_meas_unwrap = phase_unwrap(m_sensor.m_dPhi_xz, m_prev_value, m_interval);
              m_prev_value = m_angle_meas_unwrap;

              if (m_current_point == 0) m_start_point = m_angle_meas_unwrap;

              //Serial.print("m_sensor.m_dPhi_xz: "); Serial.println(m_sensor.m_dPhi_xz);
              //Serial.print("m_angle_meas_unwrap: "); Serial.println(m_angle_meas_unwrap);
              //Serial.print("m_interval: "); Serial.println(m_interval);

              if (m_current_point < m_N_cal_points - 1)
              {
                m_state = CalibrationState::NEXT_POINT;
                m_current_point ++;
              }
              else
              {
                m_end_point = m_angle_meas_unwrap;
                m_stepper.disableOutputs();
                m_state = CalibrationState::STANDBY;

                long motor_step;
                uint8_t status_cal = calculate_step(motor_step);
                //Serial.print("status_cal: "); Serial.println(status_cal);
                //Serial.print("motor_step: "); Serial.println(motor_step);
              }
              break;
            }
          case (0x01):
            {
              m_status = CalibrationStatus::SENSOR_COMM_ERROR;
              m_state = CalibrationState::ERROR;
              break;
            }
          case (0x02):
            {
              m_status = CalibrationStatus::SENSOR_BUSSY;
              m_state = CalibrationState::ERROR;
              break;
            }
          default:
            {
              break;
            }
        }

        break;
      }
    case (CalibrationState::ERROR):
      {
        break;
      }
    default:
      {
        break;
      }
  }
}
