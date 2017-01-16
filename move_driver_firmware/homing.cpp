#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "homing.h"


String convertToString(const HomingState& state)
{
  switch (state)
  {
    case (HomingState::STANDBY): return (String("STANDBY"));
    case (HomingState::INPROGRESS): return (String("INPROGRESS"));
    case (HomingState::ERROR): return (String("ERROR"));
  }
}


String convertToString(const HomingStatus& status)
{
  switch (status)
  {
    case (HomingStatus::OK): return (String("OK"));
    case (HomingStatus::ERROR): return (String("ERROR"));
  }
}


Homing::Homing(Switch& limit_switch, AccelStepper& stepper) :
  m_limit_switch(limit_switch),
  m_stepper(stepper)
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
  m_stepper.setCurrentPosition(0);
  m_stepper.setMaxSpeed(1000);
  m_stepper.setSpeed(100);
  m_stepper.setAcceleration(50);
  m_stepper.move(-1000000);
  m_stepper.enableOutputs();

  m_state = HomingState::STANDBY;
  m_status = HomingStatus::OK;
}


void Homing::start()
{
  switch (m_state)
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
  switch (m_state)
  {
    case (HomingState::STANDBY):
      {
        break;
      }
    case (HomingState::INPROGRESS):
      {
        if ((m_limit_switch.get_button_state() == true) && (m_stepper.distanceToGo() < 0))
        {
          m_stepper.setCurrentPosition(-25000);
          m_stepper.setMaxSpeed(1000);
          m_stepper.setSpeed(100);
          m_stepper.setAcceleration(50);
          m_stepper.moveTo(0);

          break;
        }

        m_stepper.run();

        if (m_stepper.distanceToGo() == 0)
        {
          m_stepper.disableOutputs();
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
