#ifndef motor_move_h
#define motor_move_h

#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/

enum class MotorMoveState
{
  STANDBY,
  MOVING,
  ERROR
};

enum class MotorMoveStatus
{
  OK,
  LIMIT_SWITCH_PRESSED
};

String convertToString(const MotorMoveState& state);
String convertToString(const MotorMoveStatus& status);

class MotorMove
{
  private:
    Switch& m_limit_switch;
    AccelStepper& m_stepper;

    MotorMoveState m_state;
    MotorMoveStatus m_status;
  public:
    MotorMove(Switch& limit_switch, AccelStepper& stepper);
    MotorMoveState currentState();
    MotorMoveStatus currentStatus();
    void process();
    void reset();
    void move(long relative);
    void moveTo(long absolute);
};

#endif
