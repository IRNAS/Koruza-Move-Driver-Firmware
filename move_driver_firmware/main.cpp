#include "main.h"
#include <Arduino.h>

/*** RECEIVING DATA VARIABLES ***/
/* True when receiving string is completed */
boolean command_received = false;
/* Receiving buffer for incoming serial data */
volatile uint8_t rx_buffer[100];
/* Temporary receiving buffer for incoming serial data*/
uint8_t rx_data[2];
/* Number of bytes received on serial */
int rx_indx = 0;
/* Final length of serial received data */
int message_len = 0;
/* True when escape received before character that should be escaped */
boolean is_frame_marker_escape_received = false;

/* Parsed message */
message_t msg_parsed;

/*** TRANSMITTING DATA VARIABLES ***/
/* Sending frame buffer */
//uint8_t send_frame[100];
/* Sending frame size */
//ssize_t send_frame_size;
/* Sending message */
message_t msg_send;
/* Command parsed form received message */
tlv_command_t parsed_command;

const int limit_switch1_pin = 3;
const int limit_switch2_pin = 4;
const unsigned long switch_debounce_delay = 50; //ms

Switch limit_switch1(limit_switch1_pin, LOW, false, switch_debounce_delay);
Switch limit_switch2(limit_switch2_pin, LOW, false, switch_debounce_delay);

const int unused_gpio_pin = 2;

TLV493D sensor1;
TLV493D sensor2;

const int sensor1_pwr_pin = A2;
const int sensor2_pwr_pin = A3;
const int i2c_sda = A4;

const int stepper1_a_pin = 10;
const int stepper1_b_pin = 9;
const int stepper1_c_pin = A0;
const int stepper1_d_pin = A1;

const int stepper2_a_pin = 5;
const int stepper2_b_pin = 6;
const int stepper2_c_pin = 7;
const int stepper2_d_pin = 8;

AccelStepper stepper1(AccelStepper::HALF4WIRE, stepper1_a_pin, stepper1_c_pin, stepper1_b_pin, stepper1_d_pin, false);
AccelStepper stepper2(AccelStepper::HALF4WIRE, stepper2_a_pin, stepper2_c_pin, stepper2_b_pin, stepper2_d_pin, false);


Calibration calibration1(limit_switch1, sensor1, stepper1);
Calibration calibration2(limit_switch2, sensor2, stepper2);

MotorMove motor_move1(limit_switch1, stepper1);
MotorMove motor_move2(limit_switch2, stepper2);

Homing motor_homing1(limit_switch1, stepper1);
Homing motor_homing2(limit_switch2, stepper2);

uint8_t status_sensor1;
uint8_t status_sensor2;
bool status_EEPROM;

bool restore_position = true;

//volatile koruza_move_t koruza_move;

com_state_t com_state = COM_IDLE_STATE;
move_state_t move_state = MOVE_IDLE_STATE;

bool do_homing = false;
bool do_calibration = false;
bool do_moving = false;

tlv_motor_position_t current_motor_position;
tlv_motor_position_t new_motor_position;

tlv_encoder_value_t current_encoder_value;

tlv_motor_position_t position_test;

//SendOnlySoftwareSerial debugSerial(unused_gpio_pin);

void init_mcu(void)
{
  // begin serial communication
  Serial.begin(115200);
  //Serial.println("Start");
  pinMode(unused_gpio_pin, OUTPUT);
  //debugSerial.begin(115200);

  // initialize MCU pins
  //pinMode(unused_gpio_pin, INPUT);
  pinMode(sensor1_pwr_pin, OUTPUT);
  pinMode(sensor2_pwr_pin, OUTPUT);
  pinMode(i2c_sda, OUTPUT);

  digitalWrite(sensor1_pwr_pin, LOW);
  digitalWrite(sensor2_pwr_pin, LOW);
  digitalWrite(i2c_sda, LOW);

  delay(100);
}

/**
   Initializes the devices connected to the MCU.
*/
void init_devices(void)
{
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
  stepper1.setSpeed(500);
  stepper1.setAcceleration(500);
  stepper1.disableOutputs();

  // initialize stepper motor 2
  stepper2.setCurrentPosition(0);
  stepper2.setMaxSpeed(1000);
  stepper2.setSpeed(500);
  stepper2.setAcceleration(500);
  stepper2.disableOutputs();

  delay(1000);

  //status_EEPROM = EEPROMload();
}

/**
   This function run motor
*/
static bool runm(AccelStepper *stepper, Switch *sw)
{
  bool sw_end = false;
  /* Check if end sw is reached */   
  if(sw->get_button_state() == true)
  {
    /* Negative direction end */
    if(stepper->targetPosition()<stepper->currentPosition())
    {
      stepper->stop();
      stepper->setCurrentPosition(stepper->currentPosition());//moveTo(stepper->currentPosition());
      sw_end = true;
    }
  }

  /* Motor move
     motor pins are enabled only while moving to conserve power
  */
  if(stepper->currentPosition()!=stepper->targetPosition()){
    stepper->enableOutputs();
    stepper->run();
  }
  else{
    stepper->stop();
    stepper->disableOutputs();
  }
  return sw_end;
}

static bool homem(AccelStepper *stepper, Switch *sw){

    /* TODO: set define for homing position */
    stepper->setCurrentPosition(-15000);
    stepper->setMaxSpeed(1000);
    stepper->setSpeed(500);
    stepper->setAcceleration(500);
    stepper->moveTo(0);
    return true;
}

/**
   Handles the motor part of the firmware, movign, homing, calibration.
*/
void run_motors(void)
{
  bool home_m1 = false;
  bool home_m2 = false;
  /* First block, always do this part
       move morors if nesseseru,
       check encoder error calculation
       update the end sw and encoder error status
  */

  home_m1 = runm(&stepper1, &limit_switch1); 
  home_m2 = runm(&stepper2, &limit_switch2);


  if(do_homing == true){
    if(home_m1 == true){
      homem(&stepper1, &limit_switch1);
    }
    if(home_m2 == true){
      homem(&stepper2, &limit_switch2);
    }
  }
}

/**
   Handles the communication part of the formware, decoding messages,
   generating messages, sending messages.
*/
void communicate(void)
{
  //debugSerial.print("do_moving: "); //debugSerial.println(do_moving);


  switch (com_state)
  {
    case COM_IDLE_STATE:
      if (command_received)
      {
        com_state = COM_TLV_ACTIVE_STATE;
      }
      else
      {
        com_state = COM_IDLE_STATE;
      }
      break;

    case COM_TLV_ACTIVE_STATE:
//      for(int i = 0; i < message_len; i++){
//        Serial.print(rx_buffer[i], HEX);
//        Serial.print(" ");  
//      }
//      Serial.println();
    
      frame_parser((uint8_t *)&rx_buffer, message_len, &msg_parsed);

      if (message_tlv_get_command(&msg_parsed, &parsed_command) != MESSAGE_SUCCESS)
      {
        message_free(&msg_parsed);
        com_state = COM_ERROR_STATE;
      }
      else
      {
        // first restore position coordinates, no other commands allowed except get status without motor position
        if(restore_position == true){
          if (parsed_command == COMMAND_RESTORE_MOTOR) {
            com_state = COM_RESTORE_MOTOR_STATE;
          }
          else if (parsed_command == COMMAND_GET_STATUS) {
            com_state = COM_GET_STATUS_STATE;
          }
          else {
            com_state = COM_END_STATE;
          }
        }
        // after restore, all other commands are allowed, except restore motor
        else{
          if (parsed_command == COMMAND_GET_STATUS) {
            com_state = COM_GET_STATUS_STATE;
          }
          else if (parsed_command == COMMAND_MOVE_MOTOR) {
            com_state = COM_MOVE_MOTOR_STATE;
          }
          else if (parsed_command == COMMAND_HOMING) {
            com_state = COM_HOMING_STATE;
          }
          else if (parsed_command == COMMAND_CALIBRATE_SENSORS) {
            com_state = COM_CALIBRATION_STATE;
          }
          else {
            com_state = COM_END_STATE;
          }
        }
      }
      break;

    case COM_GET_STATUS_STATE:

      sensor1.update();
      sensor2.update();
      //Serial.println(100*atan(sensor1.m_dBx/sensor1.m_dBy));
      current_encoder_value.y = (int32_t)(100*atan(sensor1.m_dBx/sensor1.m_dBy));
      current_encoder_value.x = (int32_t)(100*atan(sensor2.m_dBx/sensor2.m_dBy));
      /* Init for sending message */
      message_init(&msg_send);
      message_tlv_add_reply(&msg_send, REPLY_STATUS_REPORT);
      current_motor_position.x = stepper1.currentPosition();
      current_motor_position.y = stepper2.currentPosition();
      current_motor_position.z = 0;

      /* Debug for new received motor position */
//      Serial.print("motor position: (");
//      Serial.print(current_motor_position.x);
//      Serial.print(", ");
//      Serial.print(current_motor_position.y);
//      Serial.print(")");
//      Serial.println(do_homing);
      /*homing debug*/
//      Serial.print("hinf ");
//      Serial.print("X");
//      Serial.print(": ");
//      Serial.print(limit_switch1.get_button_state());
//      Serial.print(" ,");
//      Serial.print(stepper2.distanceToGo());
//      Serial.print(";  ");
//      Serial.print("Y");
//      Serial.print(": ");
//      Serial.print(limit_switch2.get_button_state());
//      Serial.print(" ,");
//      Serial.println(stepper1.distanceToGo());

      // motor position on the status reply is sent after restoring motor position
      if(restore_position != true){
        message_tlv_add_motor_position(&msg_send, &current_motor_position);
        message_tlv_add_encoder_value(&msg_send, &current_encoder_value);
        
      }
      message_tlv_add_checksum(&msg_send);
      send_bytes(&msg_send);
      message_free(&msg_send);

     

      com_state = COM_END_STATE;
      break;

    case COM_MOVE_MOTOR_STATE:
      /* Get new coordinates from received message */
      if (message_tlv_get_motor_position(&msg_parsed, &new_motor_position) != MESSAGE_SUCCESS)
      {
        message_free(&msg_parsed);
        com_state = COM_ERROR_STATE;
      }
      else
      {
        /* Debug for new received motor position */
//        Serial.print("new position: (");
//        Serial.print(new_motor_position.x);
//        Serial.print(", ");
//        Serial.print(new_motor_position.y);
//        Serial.println(")");

        /* Set motor state mashine new state
            MOVE_RUN_STATE = 1,
            MOVE_HOMING_STATE = 2,
            MOVE_CALIBRATION_STATE = 3,
        */
        
        do_homing = false;
        /* Set new coordinates for motors */
        // absolute move
    		if(new_motor_position.x != MOTOR_POSITION_UNDEFINED){
    			stepper1.moveTo((long)new_motor_position.x);
    		}
    		if(new_motor_position.y != MOTOR_POSITION_UNDEFINED){
    			stepper2.moveTo((long)new_motor_position.y);
    		}
        /* if command to move motors is send during homing,
           motors will go to new position and stop homing routine
        */


        com_state = COM_END_STATE;
      }
      break;

    case COM_HOMING_STATE:
      /* Debug the homming routine */
      //Serial.println("homing");
      stepper1.moveTo(MOTOR_X_HOMING_POSITION);
      stepper2.moveTo(MOTOR_Y_HOMING_POSITION);
      do_homing = true;


      com_state = COM_END_STATE;
      break;

    case COM_RESTORE_MOTOR_STATE:
      if (message_tlv_get_motor_position(&msg_parsed, &new_motor_position) != MESSAGE_SUCCESS)
      {
        message_free(&msg_parsed);
        com_state = COM_ERROR_STATE;
      }
      else{
        stepper1.setCurrentPosition((long)new_motor_position.x);
        stepper2.setCurrentPosition((long)new_motor_position.y);

        //motor position is restored, block restore command
        restore_position = false;
        com_state = COM_END_STATE;
      }
      break;

    case COM_CALIBRATION_STATE:
      
      break;

    case COM_ERROR_STATE:

      com_state = COM_END_STATE;
      break;
    case COM_END_STATE:
      /* Free the received mesage form the memory */
      message_free(&msg_parsed);

      command_received = false;
      com_state = COM_IDLE_STATE;
      break;
  }
}

/**
   Handles receivign bytes from UART.
*/
void receive_bytes(void)
{
  //debugSerial.println("receiving bytes");
  
  while (Serial.available()) {
    /* Receive new data */
    rx_data[0] = (uint8_t)Serial.read();

    /* Start byte received */
    if (rx_data[0] == FRAME_MARKER_START)
    {
      /* Start byte received in the frame */
      if ((is_frame_marker_escape_received == true) && (rx_buffer[0] == FRAME_MARKER_START))
      {
        rx_buffer[rx_indx++] = rx_data[0];
      }
      else
      {
        /* Real start byte received */
        rx_indx = 0;
        /* Clear buffer since start of the new frame is detected */
        for (int i = 0; i < 100; i++)
        {
          rx_buffer[i] = 0;
        }
        /* Store start marker to buffer */ 
        rx_buffer[rx_indx++] = rx_data[0];
      }
    }
    else if (rx_data[0] == FRAME_MARKER_END)
    {
      /* End byte received */
      if ((is_frame_marker_escape_received == true) && (rx_buffer[0] == FRAME_MARKER_START))
      {
        rx_buffer[rx_indx++] = rx_data[0];
      }
      else if (rx_buffer[0] == FRAME_MARKER_START)
      {
        /* Real end byte received */
        rx_buffer[rx_indx++] = rx_data[0];
        message_len = rx_indx;
        rx_indx = 0;
        /* Transfer complete, data is ready to read */
        command_received = true;
        /* Disable USART1 interrupt */
      }
    }
    else
    {
      /* If FRAME is valid */ 
      if (rx_buffer[0] == FRAME_MARKER_START)
      {
        /* put received character in the buffer */
        rx_buffer[rx_indx++] = rx_data[0];
      }
    }

    /* Handle ESC frame check */
    if ((rx_data[0] == FRAME_MARKER_ESCAPE) && (is_frame_marker_escape_received == false))
    {
      is_frame_marker_escape_received = true;
    }
    else
    {
      is_frame_marker_escape_received = false;
    }
  }
}
