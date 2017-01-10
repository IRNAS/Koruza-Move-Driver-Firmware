#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "TLV493D.h" // https://github.com/IRNAS/TLV493D-3D-Magnetic-Sensor-Arduino-Library
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "Wire.h"
#include "homing.h"
#include "calibration.h"
#include "EEPROM_manager.h"
#include "message.h"
#include "communication.h"

const int limit_switch1_pin = 4;
const int limit_switch2_pin = 3;
const unsigned long switch_debounce_delay = 50; //ms

Switch limit_switch1(limit_switch1_pin, LOW, false, switch_debounce_delay);
Switch limit_switch2(limit_switch2_pin, LOW, false, switch_debounce_delay);

const int unused_gpio_pin = 2;

TLV493D sensor1;
TLV493D sensor2;

const int sensor1_pwr_pin = A2;
const int sensor2_pwr_pin = A3;
const int i2c_sda = A4;

const int stepper1_a_pin = 5;
const int stepper1_b_pin = 6;
const int stepper1_c_pin = 7;
const int stepper1_d_pin = 8;

const int stepper2_a_pin = 10;
const int stepper2_b_pin = 9;
const int stepper2_c_pin = A0;
const int stepper2_d_pin = A1;

AccelStepper stepper1(AccelStepper::HALF4WIRE, stepper1_a_pin, stepper1_c_pin, stepper1_b_pin, stepper1_d_pin, false);
AccelStepper stepper2(AccelStepper::HALF4WIRE, stepper2_a_pin, stepper2_c_pin, stepper2_b_pin, stepper2_d_pin, false);


Calibration calibration1(limit_switch1, sensor1, stepper1);
Calibration calibration2(limit_switch2, sensor2, stepper2);


uint8_t status_sensor1;
uint8_t status_sensor2;
bool status_EEPROM;



void setup()
{
  // begin serial communication
  Serial.begin(115200);

  // initialize MCU pins
  pinMode(unused_gpio_pin, INPUT);
  pinMode(sensor1_pwr_pin, OUTPUT);
  pinMode(sensor2_pwr_pin, OUTPUT);
  pinMode(i2c_sda, OUTPUT);

  digitalWrite(sensor1_pwr_pin, LOW);
  digitalWrite(sensor2_pwr_pin, LOW);
  digitalWrite(i2c_sda, LOW);

  delay(100);


  // sensor1 power-up
  digitalWrite(sensor1_pwr_pin, HIGH);
  digitalWrite(i2c_sda, LOW); // address = 0x1F
  delay(500);

  // sensor2 power-up
  digitalWrite(sensor2_pwr_pin, HIGH);
  digitalWrite(i2c_sda, HIGH); // address = 0x5E
  delay(500);

  Wire.begin(); // begin I2C communication


  // initialize sensor 1
  status_sensor1 = sensor1.init(LOW); // address = 0x1F

  // initialize sensor 2
  status_sensor2 = sensor2.init(HIGH); // address = 0x5E


  // initialize stepper motor 1
  stepper1.setCurrentPosition(0);
  stepper1.setMaxSpeed(1000);
  stepper1.disableOutputs();

  // initialize stepper motor 2
  stepper2.setCurrentPosition(0);
  stepper2.setMaxSpeed(1000);
  stepper2.disableOutputs();

  delay(1000);

  status_EEPROM = EEPROMload();
}

void loop()
{
  while (Serial.available())
  {
    receiveBytes((uint8_t)Serial.read());
  }
}


void moveMotors(int32_t posMotor1, int32_t posMotor2)
{
  stepper1.move(posMotor1);
  stepper1.enableOutputs();

  stepper2.move(posMotor2);
  stepper2.enableOutputs();

  while (true)
  {
    if (limit_switch1.get_button_state() == true)
    {
      break;
    }

    if (limit_switch2.get_button_state() == true)
    {
      break;
    }

    stepper1.run();
    stepper2.run();

    if ((!stepper1.isRunning()) && (!stepper2.isRunning())) break;
  }

  stepper1.disableOutputs();
  stepper2.disableOutputs();
}


// decode command callback function
void decodeCommand(const message_t& msg)
{
  tlv_command_t parsed_command;
  if (message_tlv_get_command(&msg, &parsed_command) == MESSAGE_SUCCESS)
  {
    switch (parsed_command)
    {
      case (COMMAND_GET_STATUS):
        {
          Serial.println("get status!");
          break;
        }
      case (COMMAND_CALIBRATE_SENSORS):
        {
          uint8_t calibration_status = calibration1.calibrate(9); // 9 points used
          calibration_status = calibration2.calibrate(9); // 9 points used
          break;
        }
      case (COMMAND_GET_SENSOR_VALUE):
        {
          long motor1_step;
          uint8_t status_sensor1 = calibration1.calculate_step(motor1_step);
          long motor2_step;
          uint8_t status_sensor2 = calibration2.calculate_step(motor2_step);
          break;
        }
      case (COMMAND_MOVE_MOTOR):
        {
          tlv_motor_position_t position;
          if (message_tlv_get_motor_position(&msg, &position) == MESSAGE_SUCCESS)
          {
            moveMotors(position.x, position.y);
          }
          break;
        }
      case (COMMAND_REBOOT):
        {
          Serial.println("reboot!");
          break;
        }
      case (COMMAND_FIRMWARE_UPGRADE):
        {
          Serial.println("firmware upgrade!");
          break;
        }
      case (COMMAND_HOMING):
        {
          // homing sequence on both axes
          homing();
          break;
        }
      case (COMMAND_RESTORE_MOTOR):
        {
          Serial.println("restore motor!");
          break;
        }
      default:
        {
          break;
        }
    }
  }
}
