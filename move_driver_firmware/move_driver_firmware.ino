#include "message.hpp"
#include "serial_receive_transmitt.h"

message_t msg_send;

void setup()
{
  // begin serial communication
  Serial.begin(115200);
}

/* koruza move driver firmware main loop */
void loop()
{
  while (true)
  {
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

    sendBytes(msg_send);

    /* Free generated message */
    message_free(&msg_send);
  }//end while
}//end loop()

/*
  SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent()
{
  while (Serial.available())
  {
    receiveBytes((uint8_t)Serial.read());
  }
}

/*
  GET_STATUS:
  F1 01 00 01 01 03 00 04 A5 05 DF 1B F2
    1  0  1  1  3  0  4  8  0  0  0

*/


