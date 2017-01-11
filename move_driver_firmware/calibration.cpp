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



CalibrationState Homing::currentState()
{
	return m_state;
}


CalibrationStatus Homing::currentStatus()
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

	switch(m_state)
	{
		case(CalibrationState::STANDBY):
		{
			m_N_cal_points = N_points;
			m_current_point = 0;
			
			m_stepper.setSpeed(100);
			m_stepper.setAcceleration(50);
			m_stepper.enableOutputs();
			
			m_state = CalibrationState::NEXT_POINT;
			break;
		}
		case(!CalibrationState::STANDBY):
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
	switch(m_state)
	{
		case(CalibrationState::STANDBY):
		{
			break;
		}
		case(CalibrationState::NEXT_POINT):
		{
			// movement relative to current position
			m_stepper.moveTo(m_start_step + m_current_point * m_steps_per_revolution / (m_N_cal_points - 1));
			
			m_state = CalibrationState::RUN_MOTOR;
			
			break;
		}
		case(CalibrationState::RUN_MOTOR):
		{
			if (m_limit_switch.get_button_state() == true)
			{
				m_status = CalibrationStatus::LIMIT_SWITCH_PRESSED;
				m_state = CalibrationState::ERROR;
				break;
			}
			if (!m_stepper.isRunning())
			{
				m_state = CalibrationState::READ_SENSOR;
				break;
			}
			
			m_stepper.run();
	  
			break;
		}
		case(CalibrationState::READ_SENSOR):
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
			
			switch(status_sensor)
			{
				case(0x00):
				{
					if (m_current_point == 0)
					{
						m_interval = 0;
						m_prev_value = m_sensor.m_dPhi_xz;
						m_start_step = m_stepper.currentPosition();
					}
					
					m_angle_meas_unwrap = phase_unwrap(m_sensor.m_dPhi_xz, m_prev_value, m_interval);
					m_prev_value = m_angle_meas_unwrap;
					
					if (m_current_point == 0) m_start_point = m_angle_meas_unwrap;
					
					if (m_current_point < m_N_cal_points) m_current_point ++;
					else
					{
						m_end_point = m_angle_meas_unwrap;
						m_stepper.disableOutputs();
						m_status = CalibrationStatus::STANDBY;
					}
					break;
				}
				case(0x01):
				{
					m_status = CalibrationStatus::SENSOR_COMM_ERROR;
					m_state = CalibrationState::ERROR;
					break;
				}
				case(0x02):
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
		case(CalibrationState::ERROR):
		{
			break;
		}
		default:
		{
			break;
		}
	}
}