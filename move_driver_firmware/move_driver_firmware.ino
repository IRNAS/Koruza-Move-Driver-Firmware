#include "main.h"

//#include "SendOnlySoftwareSerial.h"
unsigned long t1 = 0;
unsigned long t2 = 0;
unsigned long t3 = 0;
unsigned long t4 = 0;
const int unused_gpio_pin = 2;
SendOnlySoftwareSerial debugSerial(unused_gpio_pin);

void setup()
{
  debugSerial.begin(115200);
  init_mcu();
  init_devices();
  //debugSerial.println("test begin");
}

void loop()
{
  //t1 = micros();
  run_motors();
  //t2 = micros();
  communicate();
  //t3 = micros();
  receive_bytes();
  //t4 = micros();
  
//  debugSerial.print("M: ");
//  debugSerial.print(t2-t1);
//  debugSerial.print("  C: ");
//  debugSerial.print(t3-t2);
//  debugSerial.print("  R: ");
//  debugSerial.print(t4-t3);
//  debugSerial.println();
}
