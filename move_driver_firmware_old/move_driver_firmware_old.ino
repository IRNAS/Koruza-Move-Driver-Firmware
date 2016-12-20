#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "TLV493D.h" // https://github.com/IRNAS/TLV493D-3D-Magnetic-Sensor-Arduino-Library
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "Wire.h"

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

AccelStepper stepper1(AccelStepper::FULL4WIRE, stepper1_a_pin, stepper1_c_pin, stepper1_b_pin, stepper1_d_pin, false);
AccelStepper stepper2(AccelStepper::FULL4WIRE, stepper2_a_pin, stepper2_c_pin, stepper2_b_pin, stepper2_d_pin, false);

void setup()
{
  Serial.begin(115200);

  pinMode(unused_gpio_pin, INPUT);

  pinMode(sensor1_pwr_pin, OUTPUT);
  pinMode(sensor2_pwr_pin, OUTPUT);
  pinMode(i2c_sda, OUTPUT);

  digitalWrite(sensor1_pwr_pin, LOW);
  digitalWrite(sensor2_pwr_pin, LOW);
  digitalWrite(i2c_sda, LOW);

  delay(100);

  //init sensor1
  digitalWrite(sensor1_pwr_pin, HIGH);
  digitalWrite(i2c_sda, LOW); //0x1F
  Serial.println("Starting sensor 1");
  delay(500);

  //init sensor2
  digitalWrite(sensor2_pwr_pin, HIGH);
  digitalWrite(i2c_sda, HIGH); //0x5E
  Serial.println("Starting sensor 2");
  delay(500);

  Wire.begin(); // Begin I2C wire communication

  //initialize sensor 1
  Serial.print("Initializing sensor 1: 0x");
  Serial.println(sensor1.init(LOW), HEX);

  //initialize sensor 2
  Serial.print("Initializing sensor 2: 0x");
  Serial.println(sensor2.init(HIGH), HEX);

  stepper1.setCurrentPosition(0);
  stepper1.setMaxSpeed(1000);
  stepper1.setSpeed(100);
  stepper1.setAcceleration(50);
  stepper1.move(-1000000);
  stepper1.enableOutputs();

  stepper2.setCurrentPosition(0);
  stepper2.setMaxSpeed(1000);
  stepper2.setSpeed(100);
  stepper2.setAcceleration(50);
  stepper2.move(-1000000);
  stepper2.enableOutputs();
}

void loop()
{
  if((limit_switch1.get_button_state() == true) && (stepper1.distanceToGo() < 0))
  {
    Serial.println("Limit switch 1 pressed.");
    delay(5000);
    stepper1.setCurrentPosition(0);
    stepper1.setMaxSpeed(1000);
    stepper1.setSpeed(100);
    stepper1.setAcceleration(50);
    stepper1.move(10000);
    return;
  }
  
  if((limit_switch2.get_button_state() == true) && (stepper2.distanceToGo() < 0))
  {
    Serial.println("Limit switch 2 pressed.");
    delay(5000);
    stepper2.setCurrentPosition(0);
    stepper2.setMaxSpeed(1000);
    stepper2.setSpeed(100);
    stepper2.setAcceleration(50);
    stepper2.move(10000);
    return;
  }
  
//  sensor1.update();
//  Serial.println("sensor1:");

//  Serial.print(sensor1.m_dBx);
//  Serial.print(";");//\t");
//  Serial.print(sensor1.m_dBy);
//  Serial.print(";");//\t");
//  Serial.print(sensor1.m_dBz);
//  Serial.print(";");//\t");
//  Serial.println(sensor1.m_dTemp);

//  Serial.print(sensor1.m_dPhi_xy);
//  Serial.print(";");//\t");
//  Serial.print(sensor1.m_dPhi_yz);
//  Serial.print(";");//\t");
//  Serial.print(sensor1.m_dPhi_xz);
//  Serial.print(";");//\t");
//  Serial.println(sensor1.m_dMag_2);

//  sensor2.update();
//  Serial.println("sensor2:");

//  Serial.print(sensor2.m_dBx);
//  Serial.print(";");//\t");
//  Serial.print(sensor2.m_dBy);
//  Serial.print(";");//\t");
//  Serial.print(sensor2.m_dBz);
//  Serial.print(";");//\t");
//  Serial.println(sensor2.m_dTemp);

//  Serial.print(sensor2.m_dPhi_xy);
//  Serial.print(";");//\t");
//  Serial.print(sensor2.m_dPhi_yz);
//  Serial.print(";");//\t");
//  Serial.print(sensor2.m_dPhi_xz);
//  Serial.print(";");//\t");
//  Serial.println(sensor2.m_dMag_2);


  if(stepper1.distanceToGo() != 0) Serial.println("Stepper 1 moving.");
  stepper1.run();

  if(stepper2.distanceToGo() != 0) Serial.println("Stepper 2 moving.");
  stepper2.run();
}
