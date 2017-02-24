#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "motor_move.h"


String convertToString(const MotorMoveState& state)
{
  switch (state)
  {
    case (MotorMoveState::STANDBY): return (String("STANDBY"));
    case (MotorMoveState::MOVING): return (String("MOVING"));
    case (MotorMoveState::ERROR): return (String("ERROR"));
  }
}


String convertToString(const MotorMoveStatus& status)
{
  switch (status)
  {
    case (MotorMoveStatus::OK): return (String("OK"));
    case (MotorMoveStatus::LIMIT_SWITCH_PRESSED): return (String("LIMIT_SWITCH_PRESSED"));
  }
}


MotorMove::MotorMove(Switch& limit_switch, AccelStepper& stepper) :
  m_limit_switch(limit_switch),
  m_stepper(stepper)
{
  m_state = MotorMoveState::STANDBY;
  m_status = MotorMoveStatus::OK;
}


MotorMoveState MotorMove::currentState()
{
  return m_state;
}


MotorMoveStatus MotorMove::currentStatus()
{
  return m_status;
}


void MotorMove::reset()
{
  //Serial.println("MotorMove reset");

  //m_stepper.stop();
  m_stepper.setMaxSpeed(10000);
  m_stepper.setSpeed(5000);
  m_stepper.setAcceleration(5000);
  m_stepper.disableOutputs();

  m_state = MotorMoveState::STANDBY;
  m_status = MotorMoveStatus::OK;
}


void MotorMove::move(long relative)
{
  //Serial.println("MotorMove move");

  switch (m_state)
  {
    case (MotorMoveState::STANDBY):
      {
        m_stepper.move(relative);
        m_stepper.enableOutputs();
        m_state = MotorMoveState::MOVING;
        break;
      }
    case (MotorMoveState::MOVING):
      {
        break;
      }
    case (MotorMoveState::ERROR):
      {
        reset();
        move(relative);
        break;
      }
  }
}


void MotorMove::moveTo(long absolute)
{
  //Serial.println("MotorMove moveTo");
  //Serial.println(m_stepper.currentPosition());

  switch (m_state)
  {
    case (MotorMoveState::STANDBY):
      {
        m_stepper.moveTo(absolute);
        m_stepper.enableOutputs();
        m_state = MotorMoveState::MOVING;
        break;
      }
    case (MotorMoveState::MOVING):
      {
        break;
      }
    case (MotorMoveState::ERROR):
      {
        reset();
        moveTo(absolute);
        break;
      }
  }
}


void MotorMove::process()
{
  //Serial.println("MotorMove process");
  switch (m_state)
  {
    case (MotorMoveState::STANDBY):
      {
        break;
      }
    case (MotorMoveState::MOVING):
      {
        if ((m_limit_switch.get_button_state() == true) && (m_stepper.distanceToGo() < 0))
        {
          m_stepper.disableOutputs();
          m_state = MotorMoveState::ERROR;
          m_status = MotorMoveStatus::LIMIT_SWITCH_PRESSED;
          break;
        }

        m_stepper.run();

        if (m_stepper.distanceToGo() == 0)
        {
          m_stepper.disableOutputs();
          m_state = MotorMoveState::STANDBY;
          return;
        }

        break;
      }
    case (MotorMoveState::ERROR):
      {
        break;
      }
  }
}
