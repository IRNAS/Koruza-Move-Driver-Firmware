#include "EEPROM_manager.h"
#include <Arduino.h>
#include <EEPROM.h>
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "calibration.h"


extern AccelStepper stepper1;
extern AccelStepper stepper2;

extern Calibration calibration1;
extern Calibration calibration2;


unsigned long EEPROM_CRC()
{
  const unsigned long crc_table[16] =
  {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0 ; index < 4 * (sizeof(long) + sizeof(double)); ++index)
  {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  
  return crc;
}


void EEPROMsave()
{
  unsigned long address = 0;
  
  EEPROM.put(address, stepper1.currentPosition());
  address += sizeof(long);
  
  EEPROM.put(address, stepper2.currentPosition());
  address += sizeof(long);

  EEPROM.put(address, calibration1.m_start_step);
  address += sizeof(long);

  EEPROM.put(address, calibration2.m_start_step);
  address += sizeof(long);

  EEPROM.put(address, calibration1.m_start_point);
  address += sizeof(double);

  EEPROM.put(address, calibration2.m_start_point);
  address += sizeof(double);

  EEPROM.put(address, calibration1.m_end_point);
  address += sizeof(double);

  EEPROM.put(address, calibration2.m_end_point);
  address += sizeof(double);

  unsigned long CRC = EEPROM_CRC();
  EEPROM.put(address, CRC);
}


bool EEPROMload()
{
  unsigned long address = 0;

  long stepper1_position;
  EEPROM.get(address, stepper1_position);
  address += sizeof(long);

  long stepper2_position;
  EEPROM.get(address, stepper2_position);
  address += sizeof(long);

  long cal1_m_start_step;
  EEPROM.get(address, cal1_m_start_step);
  address += sizeof(long);

  long cal2_m_start_step;
  EEPROM.get(address, cal2_m_start_step);
  address += sizeof(long);

  double cal1_m_start_point;
  EEPROM.get(address, cal1_m_start_point);
  address += sizeof(double);

  double cal2_m_start_point;
  EEPROM.get(address, cal2_m_start_point);
  address += sizeof(double);

  double cal1_m_end_point;
  EEPROM.get(address, cal1_m_end_point);
  address += sizeof(double);

  double cal2_m_end_point;
  EEPROM.get(address, cal2_m_end_point);
  address += sizeof(double);

  unsigned long CRC;
  EEPROM.get(address, CRC);

  unsigned long CRC_calc = EEPROM_CRC();

  if(CRC != CRC_calc) return true; // error

  stepper1.setCurrentPosition(stepper1_position);
  stepper2.setCurrentPosition(stepper2_position);

  calibration1.m_start_step = cal1_m_start_step;
  calibration1.m_start_point = cal1_m_start_point;
  calibration1.m_end_point = cal1_m_end_point;

  calibration2.m_start_step = cal2_m_start_step;
  calibration2.m_start_point = cal2_m_start_point;
  calibration2.m_end_point = cal2_m_end_point;
  
  return false; // success
}
