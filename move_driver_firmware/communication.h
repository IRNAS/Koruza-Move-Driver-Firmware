#ifndef communication_h
#define communication_h

#include "Arduino.h"
#include "message.h"

extern void receiveBytes(uint8_t rx_data);
extern bool sendBytes(const message_t& msg_send);
extern void decodeCommand(const message_t& msg);

#endif
