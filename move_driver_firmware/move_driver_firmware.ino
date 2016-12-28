#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "TLV493D.h" // https://github.com/IRNAS/TLV493D-3D-Magnetic-Sensor-Arduino-Library
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/

const int limit_switch1_pin = 3;
const int limit_switch2_pin = 4;
const unsigned long switch_debounce_delay = 50; //ms

Switch limit_switch1(limit_switch1_pin, LOW, false, switch_debounce_delay);
Switch limit_switch2(limit_switch2_pin, LOW, false, switch_debounce_delay);

const int unused_gpio_pin = 2;

const int sensor1_pwr_pin = A2;
const int sensor2_pwr_pin = A3;

TLV493D sensor1(sensor1_pwr_pin);
TLV493D sensor2(sensor2_pwr_pin);

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

  sensor1.init(LOW);
  sensor2.init(HIGH);

  stepper1.setMaxSpeed(100);
  stepper1.setSpeed(10);
  stepper1.setAcceleration(100);
  stepper1.moveTo(10000);
  stepper1.enableOutputs();

  stepper2.setMaxSpeed(100);
  stepper2.setSpeed(10);
  stepper2.setAcceleration(100);
  stepper2.moveTo(10000);
  stepper2.enableOutputs();
}

/* koruza move driver firmware main loop */
void loop()
{
  sensor1.update();
  Serial.println("sensor1:");

  Serial.print(sensor1.m_dBx);
  Serial.print(";");//\t");
  Serial.print(sensor1.m_dBy);
  Serial.print(";");//\t");
  Serial.print(sensor1.m_dBz);
  Serial.print(";");//\t");
  Serial.println(sensor1.m_dTemp);

  Serial.print(sensor1.m_dPhi_xy);
  Serial.print(";");//\t");
  Serial.print(sensor1.m_dPhi_yz);
  Serial.print(";");//\t");
  Serial.print(sensor1.m_dPhi_xz);
  Serial.print(";");//\t");
  Serial.println(sensor1.m_dMag_2);

  sensor2.update();
  Serial.println("sensor2:");

  Serial.print(sensor2.m_dBx);
  Serial.print(";");//\t");
  Serial.print(sensor2.m_dBy);
  Serial.print(";");//\t");
  Serial.print(sensor2.m_dBz);
  Serial.print(";");//\t");
  Serial.println(sensor2.m_dTemp);

  Serial.print(sensor2.m_dPhi_xy);
  Serial.print(";");//\t");
  Serial.print(sensor2.m_dPhi_yz);
  Serial.print(";");//\t");
  Serial.print(sensor2.m_dPhi_xz);
  Serial.print(";");//\t");
  Serial.println(sensor2.m_dMag_2);
  
  if(limit_switch1.get_button_state()) Serial.println("Limit switch 1 pressed.");
  if(limit_switch2.get_button_state()) Serial.println("Limit switch 2 pressed.");
  
  stepper1.run();
  stepper2.run();
}
