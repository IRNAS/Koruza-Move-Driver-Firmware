#ifndef communication_h
#define communication_h

#include "message.h"
//
//extern void receiveBytes(uint8_t rx_data);
extern bool send_bytes(const message_t *msg_send);
//extern void decodeCommand(const message_t& msg);
//
#endif
