#ifndef main_h
#define main_h

#include "message.h"
#include "frame.h"
#include "EEPROM_manager.h"

#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "TLV493D.h" // https://github.com/IRNAS/TLV493D-3D-Magnetic-Sensor-Arduino-Library
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "Wire.h"

#include "calibration.h"
#include "communication.h"

typedef enum{
	COM_IDLE_STATE = 0,
	COM_TLV_ACTIVE_STATE = 1,
	COM_GET_STATUS_STATE = 2,
	COM_MOVE_MOTOR_STATE = 3,
	COM_HOMING_STATE = 4,
	COM_CALIBRATION_STATE = 5,
	COM_ERROR_STATE = 6,
  COM_END_STATE = 7
} com_state_t;

typedef enum{
	MOVE_IDLE_STATE = 0,
	MOVE_RUN_STATE = 1,
	MOVE_HOMING_STATE = 2,
	MOVE_CALIBRATION_STATE = 3,
	MOVE_ERROR_STATE = 4
} move_state_t;
	

typedef struct{
	AccelStepper *motor_x;
	AccelStepper *motor_y;
	TLV493D *sensor_x;
	TLV493D *sensor_y;
	com_state_t com_state;
	move_state_t move_state;
} koruza_move_t;

extern volatile koruza_move_t koruza_move; 

/**
   Initializes the MCU pheripherials.
*/
void init_mcu(void);

/**
   Initializes the devices connected to the MCU.
*/
void init_devices(void);

/**
   Handles the motor part of the firmware, movign, homing, calibration.
*/
void run_motors(void);

/**
   Handles the communication part of the formware, decoding messages,
   generating messages, sending messages.
*/
void communicate(void);

/**
   Handles receivign bytes from UART.
*/
void receive_bytes(void);

#endif
