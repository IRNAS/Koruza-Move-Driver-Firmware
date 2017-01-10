#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "homing.h"

Homing::Homing(Switch& limit_switch1, Switch& limit_switch2, AccelStepper& stepper1, AccelStepper& stepper2) :
	m_limit_switch1(limit_switch1),
	m_limit_switch2(limit_switch2),
	m_stepper1(stepper1),
	m_stepper2(stepper2)
{
	m_state = HomingState::STANDBY;
	m_status = HomingStatus::OK;
}


HomingState Homing::currentState()
{
	return m_state;
}


HomingStatus Homing::currentStatus()
{
	return m_status;
}


void Homing::reset()
{
	m_stepper1.setCurrentPosition(0);
	m_stepper1.setMaxSpeed(1000);
	m_stepper1.setSpeed(100);
	m_stepper1.setAcceleration(50);
	m_stepper1.move(-1000000);
	m_stepper1.enableOutputs();

	m_stepper2.setCurrentPosition(0);
	m_stepper2.setMaxSpeed(1000);
	m_stepper2.setSpeed(100);
	m_stepper2.setAcceleration(50);
	m_stepper2.move(-1000000);
	m_stepper2.enableOutputs();
	
	m_state = HomingState::STANDBY;
	m_status = HomingStatus::OK;
}


void Homing::start()
{
	switch(m_state)
	{
		case (HomingState::STANDBY):
		{
			m_state = HomingState::INPROGRESS;
			break;
		}
		case (HomingState::INPROGRESS):
		{
			break;
		}
		case (HomingState::ERROR):
		{
			reset();
			start();
			break;
		}
	}
}


void Homing::process()
{
	switch(m_state)
	{
		case (HomingState::STANDBY):
		{
			break;
		}
		case (HomingState::INPROGRESS):
		{
			if ((limit_switch1.get_button_state() == true) && (m_stepper1.distanceToGo() < 0))
			{
			  m_stepper1.setCurrentPosition(0);
			  m_stepper1.setMaxSpeed(1000);
			  m_stepper1.setSpeed(100);
			  m_stepper1.setAcceleration(50);
			  m_stepper1.move(10000);
			  
			  break;
			}

			if ((limit_switch2.get_button_state() == true) && (m_stepper2.distanceToGo() < 0))
			{
			  m_stepper2.setCurrentPosition(0);
			  m_stepper2.setMaxSpeed(1000);
			  m_stepper2.setSpeed(100);
			  m_stepper2.setAcceleration(50);
			  m_stepper2.move(10000);
			  
			  break;
			}

			m_stepper1.run();
			m_stepper2.run();

			if ((m_stepper1.distanceToGo() == 0) && (m_stepper2.distanceToGo() == 0))
			{
				m_stepper1.disableOutputs();
				m_stepper2.disableOutputs();
				m_state = HomingState::STANDBY;
				return;
			}
			
			break;
		}
		case (HomingState::ERROR):
		{
			break;
		}
	}
}
