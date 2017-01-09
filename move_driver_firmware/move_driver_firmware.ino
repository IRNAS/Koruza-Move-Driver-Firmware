
#include "communication.h"
#include "switch.h" // https://github.com/IRNAS/Arduino-Switch-Debounce
#include "TLV493D.h" // https://github.com/IRNAS/TLV493D-3D-Magnetic-Sensor-Arduino-Library
#include "AccelStepper.h" // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include "Wire.h"
#include "homing.h"
#include "calibration.h"
#include "EEPROM_manager.h"
//TLV communication modules
#include "message.hpp"
#include "frame.hpp"
#include "inet.hpp"

/*** RECEIVING DATA VARIABLES ***/
/* True when receiving string is completed */
boolean stringComplete = false;
boolean send_frame_bool = true;
/* Receiving buffer for incomming serial data */
volatile uint8_t rx_buffer[100];
/* Temperary receiving buffer for incomming serial data*/
uint8_t rx_data[2];
/* Number of bytes received on serial */
int rx_indx;
/* Check buffer for receiving serual data */
uint8_t rx_last[2] = {0x00, 0x00};
/* Final length of serial received data */
int message_len = 0;

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
  digitalWrite(i2c_sda, LOW); // address = 0x1F
  //Serial.println("Sensor 1 powered up.");
  delay(500);

  // sensor2 power-up
  digitalWrite(sensor2_pwr_pin, HIGH);
  digitalWrite(i2c_sda, HIGH); // address = 0x5E
  //Serial.println("Sensor 2 powered up.");
  delay(500);

  Wire.begin(); // begin I2C communication


  // initialize sensor 1
//  uint8_t status_sensor = sensor1.init(LOW); // address = 0x1F
//  if (status_sensor != 0x00) error_handler("Sensor 1 failed to intialize: 0x" + String(status_sensor, HEX));
  //else Serial.println("Sensor 1 initialized.");


  // initialize sensor 2
//  status_sensor = sensor2.init(HIGH); // address = 0x5E
//  if (status_sensor != 0x00) error_handler("Sensor 2 failed to intialize: 0x" + String(status_sensor, HEX));
  //else Serial.println("Sensor 2 initialized.");


  // initialize stepper motor 1
  stepper1.setCurrentPosition(0);
  stepper1.setMaxSpeed(1000);
  stepper1.disableOutputs();

  // initialize stepper motor 2
  stepper2.setCurrentPosition(0);
  stepper2.setMaxSpeed(1000);
  stepper2.disableOutputs();

  delay(1000);

  ///TEST !!!
  EEPROMsave();
  if(EEPROMload()) error_handler("EEPROM error.");
}

/* koruza move driver firmware main loop */
void loop()
{
  //communication branch
  while (Serial.available())
  {
    receiveBytes((uint8_t)Serial.read());
////////
  /* Parsed message */
  message_t msg_parsed;
  
  /*** TRANSMITTING DATA VARIABLES ***/
  /* Sending frame buffer */
  uint8_t send_frame[100];
  /* Sending frame size */
  ssize_t send_frame_size;
  /* Sending message */
  message_t msg_send; 

  send_frame_bool = true;
  
  // homing sequence on both axes
  //homing();


  // calibrate sensor 1
//  Serial.println("Starting sensor 1 calibration.");
//  uint8_t calibration_status = calibration1.calibrate(9); // 9 points used
//  if (calibration_status != 0x00) error_handler("Sensor 1 calibration failed: 0x" + String(calibration_status, HEX));
//  else Serial.println("Sensor 1 calibrated.");

  // calibrate sensor 2
//  Serial.println("Starting sensor 2 calibration.");
//  calibration_status = calibration2.calibrate(9); // 9 points used
//  if (calibration_status != 0x00) error_handler("Sensor 2 calibration failed: 0x" + String(calibration_status, HEX));
//  else Serial.println("Sensor 2 calibrated.");



  // move motor to some position
//  stepper1.move(2000);
//  stepper1.enableOutputs();
  
//  while (true)
//  {
//    if (limit_switch1.get_button_state() == true) error_handler("Limit switch 1 pressed.");
//    stepper1.run();
//    if (!stepper1.isRunning()) break;
//  }
//  
//  stepper1.disableOutputs();
//
//
//  // move motor to some position
//  stepper2.move(1000);
//  stepper2.enableOutputs();
//  
//  while (true)
//  {
//    if (limit_switch2.get_button_state() == true) error_handler("Limit switch 2 pressed.");
//    stepper2.run();
//    if (!stepper2.isRunning()) break;
//  }
//  
//  stepper2.disableOutputs();
//  
//
//
//  // try to update sensor value
//  int count_tries = 1000;
//  uint8_t status_sensor = 0x00;
//
//  while (count_tries > 0)
//  {
//    status_sensor = sensor1.update();
//    if (status_sensor != 0x02) break; // if 0x02 the sensor is busy, need to try again
//    count_tries --;
//  }
//  if (status_sensor != 0x00) error_handler("Sensor 1 failed to update: 0x" + String(status_sensor, HEX)); // error
//
//  //Serial.print("Sensor 1 value: ");
//  //Serial.println(sensor1.m_dPhi_xz);
//
//  long motor_step = 0;
//  calibration1.calculate_step(sensor1.m_dPhi_xz, motor_step);
//  Serial.print("Motor 1 step: ");
//  Serial.println(motor_step);
//  
//
//
//  // try to update sensor value
//  count_tries = 1000;
//  status_sensor = 0x00;
//
//  while (count_tries > 0)
//  {
//    status_sensor = sensor2.update();
//    if (status_sensor != 0x02) break; // if 0x02 the sensor is busy, need to try again
//    count_tries --;
//  }
//  if (status_sensor != 0x00) error_handler("Sensor 2 failed to update: 0x" + String(status_sensor, HEX)); // error
//
//  //Serial.print("Sensor 2 value: ");
//  //Serial.println(sensor2.m_dPhi_xz);
//
//  motor_step = 0;
//  calibration2.calculate_step(sensor2.m_dPhi_xz, motor_step);
//  Serial.print("Motor 2 step: ");
//  Serial.println(motor_step);
  
  while(true){
    if(send_frame_bool){
      /* Init for sending message */
      message_init(&msg_send);
      /* Uncomment to add values to the message */
    // message_tlv_add_command(&msg_send, COMMAND_GET_STATUS);
       message_tlv_add_command(&msg_send, COMMAND_MOVE_MOTOR);
    // message_tlv_add_reply(&msg_send, REPLY_ERROR_REPORT);
       tlv_motor_position_t position = {1000, 0, 0};
       message_tlv_add_motor_position(&msg_send, &position);
    //  message_tlv_add_power_reading(&msg_send, 0x0444);
    //  tlv_error_report_t error_report;
    //  error_report.code = 35;
    //  message_tlv_add_error_report(&msg_send, &error_report);
      message_tlv_add_checksum(&msg_send);
      /* Print generated message */
      Serial.println("Generated protocol message: ");
      message_print(&msg_send);  
      Serial.println();
  
      send_frame_size = frame_message(send_frame, sizeof(send_frame), &msg_send);
      Serial.print("Serialized protocol message with frame:");
      Serial.println();
      for (size_t i = 0; i < send_frame_size; i++) {
        Serial.print(send_frame[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      /* Free generated message */
      message_free(&msg_send);

      send_frame_bool = false;
    }
    /* When new frame is received on serial, previously generated message will be sent to serial*/
    /* and received message will be decoded and value sent to serial.*/
    if (stringComplete){
      /* Received serail message */
      Serial.println("Received serialized protocol message:");
      for (size_t i = 0; i < message_len; i++) {
        Serial.print(rx_buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
  
      Serial.print("Received message_len: ");
      Serial.println(message_len);
      
  
      /* Parse received message */
      int num = frame_parser((uint8_t *)&rx_buffer, message_len, &msg_parsed);
      Serial.println("Parsed protocol message: ");
      message_print(&msg_parsed);
      Serial.println();
      /* Free received message */
      message_free(&msg_parsed); 
      stringComplete = false;
      //send generated string once more
      send_frame_bool = true;
    }//end if
    serialEvent();
  }//end while
}//end loop()

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    rx_data[0] = (uint8_t)Serial.read();
    /* Clear Rx_Buffer before receiving new data */
    if (rx_indx==0){
      for (int i=0; i<100; i++) rx_buffer[i]=0;
    }

    /* Start byte received */
    if(rx_data[0] == FRAME_MARKER_START){
      /* Start byte received in the frame */
//      Serial.println("rx_last, rx_buff[0]:");
//      Serial.println(rx_last[0], HEX);
//      Serial.println(rx_buffer[0], HEX);
      if((rx_last[0] == FRAME_MARKER_ESCAPE) && (rx_buffer[0] == FRAME_MARKER_START)){
        rx_buffer[rx_indx++]=rx_data[0];
      }
      /* Real start byte received */
      else if(rx_last[0] != FRAME_MARKER_ESCAPE){
        rx_indx = 0;
        rx_buffer[rx_indx++]=rx_data[0];

      }
    }
    /* End byte received */
    else if(rx_data[0] == FRAME_MARKER_END){
      /* End byte received in the frame */
      if(rx_last[0] == FRAME_MARKER_ESCAPE && rx_buffer[0] == FRAME_MARKER_START){
        rx_buffer[rx_indx++]=rx_data[0];
      }
      /* Real end byte received */
      else if(rx_last[0] != FRAME_MARKER_ESCAPE && rx_buffer[0] == FRAME_MARKER_START){
        rx_buffer[rx_indx++]=rx_data[0];
        message_len = rx_indx;
        rx_indx=0;
        /* Transfer complete, data is ready to read */
        stringComplete = true;
        /* Disable USART1 interrupt */
        //HAL_NVIC_DisableIRQ(USART1_IRQn);
      }
    }
    else{
      if(rx_buffer[0] == FRAME_MARKER_START){
        rx_buffer[rx_indx++]=rx_data[0];
      }
    }
    /* Store last received byte for ESC check */
    rx_last[0] = rx_data[0];
    

  }
}

/*
GET_STATUS:
F1 01 00 01 01 03 00 04 A5 05 DF 1B F2
    1  0  1  1  3  0  4  8  0  0  0 

*/

