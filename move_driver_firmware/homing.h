#ifndef homing_h
#define homing_h

#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/

enum class HomingState
{
  STANDBY,
  INPROGRESS,
  ERROR
};

enum class HomingStatus
{
  OK,
  ERROR
};

String convertToString(const HomingState& state);
String convertToString(const HomingStatus& status);

class Homing
{
  private:
    Switch& m_limit_switch;
    AccelStepper& m_stepper;

    HomingState m_state;
    HomingStatus m_status;
  public:
    Homing(Switch& limit_switch, AccelStepper& stepper);
    HomingState currentState();
    HomingStatus currentStatus();
    void process();
    void reset();
    void start();
};

/*
  Function for homing
  Whating for both end sw, and then sending motors to new zero
*/
bool homing_check(Switch *end_sw, AccelStepper *stepper);

#endif
