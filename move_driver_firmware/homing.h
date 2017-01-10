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

class Homing
{
private:
	Switch& m_limit_switch1;
	Switch& m_limit_switch2;
	AccelStepper& m_stepper1;
	AccelStepper& m_stepper2;
	
	HomingState m_state;
	HomingStatus m_status;
public:
	Homing(Switch& limit_switch1, Switch& limit_switch2, AccelStepper& stepper1, AccelStepper& stepper2);
	HomingState currentState();
	HomingStatus currentStatus();
	void process();
	void reset();
	void start();
};

#endif
