#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "TLV493D.h" // https://github.com/IRNAS/TLV493D-3D-Magnetic-Sensor-Arduino-Library
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "Wire.h"
#include "homing.h"
#include "calibration.h"
#include "EEPROM_manager.h"


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


Homing homing1(limit_switch1, stepper1);
Homing homing2(limit_switch2, stepper2);


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
  uint8_t status_sensor1 = sensor1.init(LOW); // address = 0x1F
  Serial.print("Sensor 1 status: 0x");
  Serial.println(status_sensor1, HEX);

  // initialize sensor 2
  uint8_t status_sensor2 = sensor2.init(HIGH); // address = 0x5E
  Serial.print("Sensor 2 status: 0x");
  Serial.println(status_sensor2, HEX);


  // initialize stepper motor 1
  stepper1.setCurrentPosition(0);
  stepper1.setMaxSpeed(1000);
  stepper1.disableOutputs();

  // initialize stepper motor 2
  stepper2.setCurrentPosition(0);
  stepper2.setMaxSpeed(1000);
  stepper2.disableOutputs();

  delay(1000);
  Serial.println("starting");

  homing1.reset();
  homing1.start();
  homing2.reset();
  homing2.start();

  calibration1.reset();
  calibration1.start(9);
  calibration2.reset();
  calibration2.start(9);
}

void loop()
{
  Serial.print("stepper1: "); Serial.println(stepper1.currentPosition());
  Serial.print("stepper2: "); Serial.println(stepper2.currentPosition());

  Serial.print("calibration1 state: "); Serial.println(convertToString(calibration1.currentState()));
  Serial.print("calibration2 state: "); Serial.println(convertToString(calibration2.currentState()));

  Serial.print("calibration1 status: "); Serial.println(convertToString(calibration1.currentStatus()));
  Serial.print("calibration2 status: "); Serial.println(convertToString(calibration2.currentStatus()));

  calibration1.process();
  calibration2.process();
}



