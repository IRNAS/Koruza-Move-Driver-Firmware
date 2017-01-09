#include "Arduino.h"
#include "message.hpp"

extern void receiveBytes(uint8_t rx_data);
extern void sendBytes(message_t& msg_send);
