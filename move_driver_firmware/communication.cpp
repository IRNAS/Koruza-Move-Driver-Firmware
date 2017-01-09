#include "Arduino.h"
#include "communication.h"

//TLV communication modules
#include "message.h"
#include "frame.h"
#include "inet.h"


/*** RECEIVING DATA VARIABLES ***/
/* Receiving buffer for incomming serial data */
volatile uint8_t rx_buffer[100];
/* Temporary receiving buffer for incomming serial data*/
uint8_t rx_data;
/* Number of bytes received on serial */
int rx_indx;
/* Check buffer for receiving serial data */
uint8_t rx_last = 0x00;
/* Final length of serial received data */
int message_len = 0;
/* Parsed message */
message_t msg_parsed;

// decode command callback function (implemented in move_driver_fimware.ino)
extern void decodeCommand(const message_t& msg);


void receiveBytes(uint8_t rx_data)
{
  /* Clear Rx_Buffer before receiving new data */
  if (rx_indx == 0)
  {
    for (int i = 0; i < 100; i++) rx_buffer[i] = 0;
  }

  /* Start byte received */
  if (rx_data == FRAME_MARKER_START)
  {
    /* Start byte received in the frame */
    //      Serial.println("rx_last, rx_buff[0]:");
    //      Serial.println(rx_last, HEX);
    //      Serial.println(rx_buffer, HEX);
    if ((rx_last == FRAME_MARKER_ESCAPE) && (rx_buffer[0] == FRAME_MARKER_START))
    {
      rx_buffer[rx_indx++] = rx_data;
    }
    /* Real start byte received */
    else if (rx_last != FRAME_MARKER_ESCAPE)
    {
      rx_indx = 0;
      rx_buffer[rx_indx++] = rx_data;
    }
  }
  /* End byte received */
  else if (rx_data == FRAME_MARKER_END)
  {
    /* End byte received in the frame */
    if (rx_last == FRAME_MARKER_ESCAPE && rx_buffer[0] == FRAME_MARKER_START)
    {
      rx_buffer[rx_indx++] = rx_data;
    }
    /* Real end byte received */
    else if (rx_last != FRAME_MARKER_ESCAPE && rx_buffer[0] == FRAME_MARKER_START)
    {
      rx_buffer[rx_indx++] = rx_data;
      message_len = rx_indx;
      rx_indx = 0;

      /* Transfer complete, data is ready to read */
      /* Received serial message */
      //		  Serial.println("Received serialized protocol message:");
      //	    for (size_t i = 0; i < message_len; i++)
      //	    {
      //			  Serial.print(rx_buffer[i], HEX);
      //			  Serial.print(" ");
      //	    }
      //		  Serial.println();
      //		  Serial.print("Received message_len: ");
      //		  Serial.println(message_len);


      /* Parse received message */
      frame_parser((uint8_t *)&rx_buffer, message_len, &msg_parsed);
      //		  Serial.println("Parsed protocol message: ");
      //		  message_print(&msg_parsed);
      //		  Serial.println();

      // callback function
      decodeCommand(msg_parsed); // implemented in move_driver_fimware.ino

      /* Free received message */
      message_free(&msg_parsed);
    }
  }
  else
  {
    if (rx_buffer[0] == FRAME_MARKER_START)
    {
      rx_buffer[rx_indx++] = rx_data;
    }
  }
  /* Store last received byte for ESC check */
  rx_last = rx_data;
}


bool sendBytes(const message_t& msg_send)
{
  /*** TRANSMITTING DATA VARIABLES ***/
  /* Sending frame buffer */
  uint8_t send_frame[100];
  /* Sending frame size */
  ssize_t send_frame_size;

  send_frame_size = frame_message(send_frame, sizeof(send_frame), &msg_send);

  if (send_frame_size <= 0) return false;

  //	Serial.print("Serialized protocol message with frame:");
  //	Serial.println();
  Serial.write(send_frame, send_frame_size);
  //	Serial.println();

  return true;
}
