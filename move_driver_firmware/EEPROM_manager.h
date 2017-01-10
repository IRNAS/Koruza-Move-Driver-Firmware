#ifndef EEPROM_manager_h
#define EEPROM_manager_h

#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "calibration.h"

class EEPROM_manager
{
private:
	AccelStepper& m_stepper1;
	AccelStepper& m_stepper2;
	Calibration& m_calibration1;
	Calibration& m_calibration2;
	
	unsigned long EEPROM_CRC();
public:
	EEPROM_manager(AccelStepper& stepper1, AccelStepper& stepper2, Calibration& calibration1, Calibration& calibration2);
	void EEPROMsave();
	bool EEPROMload();
}

#endif