#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "TLV493D.h" // https://github.com/IRNAS/TLV493D-3D-Magnetic-Sensor-Arduino-Library
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "Wire.h"
#include "homing.h"
#include "calibration.h"

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


void error_handler(String error_message)
{
  while (true)
  {
    stepper1.disableOutputs();
    stepper2.disableOutputs();
    
    Serial.println("Error occured!");
    Serial.println(error_message);
  }
}


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
  digitalWrite(i2c_sda, LOW); // adress = 0x1F
  Serial.println("Sensor 1 powered up.");
  delay(500);

  // sensor2 power-up
  digitalWrite(sensor2_pwr_pin, HIGH);
  digitalWrite(i2c_sda, HIGH); // adress = 0x5E
  Serial.println("Sensor 2 powered up.");
  delay(500);

  Wire.begin(); // begin I2C communication


  // initialize sensor 1
  uint8_t status_sensor = sensor1.init(LOW); // adress = 0x1F
  if (status_sensor == 0x00) Serial.println("Sensor 1 initialized.");
  else error_handler("Sensor 1 failed to intialize: 0x" + String(status_sensor, HEX));


  // initialize sensor 2
  status_sensor = sensor2.init(HIGH); // adress = 0x5E
  if (status_sensor == 0x00) Serial.println("Sensor 2 initialized.");
  else error_handler("Sensor 2 failed to intialize: 0x" + String(status_sensor, HEX));


  // initialize stepper motor 1
  stepper1.setCurrentPosition(0);
  stepper1.setMaxSpeed(1000);
  stepper1.disableOutputs();

  // initialize stepper motor 2
  stepper2.setCurrentPosition(0);
  stepper2.setMaxSpeed(1000);
  stepper2.disableOutputs();

  delay(1000);
}

void loop()
{
  // homing sequence on both axes
  homing();


  // calibrate sensor 1
  Serial.println("Starting sensor 1 calibration.");
  uint8_t calibration_status = calibration1.calibrate(25); // 9 points used
  if (calibration_status == 0x00) Serial.println("Sensor 1 calibrated.");
  else error_handler("Sensor 1 calibration failed: 0x" + String(calibration_status, HEX));

  // calibrate sensor 2
  Serial.println("Starting sensor 2 calibration.");
  calibration_status = calibration2.calibrate(25); // 9 points used
  if (calibration_status == 0x00) Serial.println("Sensor 2 calibrated.");
  else error_handler("Sensor 2 calibration failed: 0x" + String(calibration_status, HEX));
  


  // try to update sensor value
  int count_tries = 1000;
  uint8_t status_sensor = 0x00;

  while (count_tries > 0)
  {
    status_sensor = sensor1.update();
    if (status_sensor != 0x02) break; // if 0x02 the sensor is busy, need to try again
    count_tries --;
  }
  if (status_sensor != 0x00) error_handler("Sensor 1 failed to update: 0x" + String(status_sensor, HEX)); // error

  Serial.print("Sensor 1 value: ");
  Serial.println(sensor1.m_dPhi_xz);

  long motor_step = 0;
  calibration1.calculate_step(sensor1.m_dPhi_xz, motor_step);
  Serial.print("Motor 1 step: ");
  Serial.println(motor_step);
  


  // try to update sensor value
  count_tries = 1000;
  status_sensor = 0x00;

  while (count_tries > 0)
  {
    status_sensor = sensor2.update();
    if (status_sensor != 0x02) break; // if 0x02 the sensor is busy, need to try again
    count_tries --;
  }
  if (status_sensor != 0x00) error_handler("Sensor 2 failed to update: 0x" + String(status_sensor, HEX)); // error

  Serial.print("Sensor 2 value: ");
  Serial.println(sensor2.m_dPhi_xz);

  motor_step = 0;
  calibration2.calculate_step(sensor2.m_dPhi_xz, motor_step);
  Serial.print("Motor 2 step: ");
  Serial.println(motor_step);
  

  while(true);
}
