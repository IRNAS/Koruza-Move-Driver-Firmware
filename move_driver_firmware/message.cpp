/*
 * koruza-driver - KORUZA driver
 *
 * Copyright (C) 2016 Jernej Kos <jernej@kos.mx>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Affero General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Affero General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "message.h"
#include <Arduino.h> 
#include "CRC32.h" // lib for CRC32 https://github.com/bakercp/CRC32
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Ethernet.h"
#include "inet.h"

// Forward declarations.
uint32_t message_checksum(const message_t *message);

message_result_t message_init(message_t *message)
{
  memset(message, 0, sizeof(message_t));
  return MESSAGE_SUCCESS;
}

void message_free(message_t *message)
{
  for (size_t i = 0; i < message->length; i++)
  {
    free(message->tlv[i].value);
  }

  message_init(message);
}

message_result_t message_parse(message_t *message, const uint8_t *data, size_t length)
{
  message_init(message);

  size_t offset = 0;
  while (offset < length)
  {
    if (message->length >= MAX_TLV_COUNT)
    {
      message_free(message);
      return MESSAGE_ERROR_TOO_MANY_TLVS;
    }

    size_t i = message->length;

    // Parse type.
    message->tlv[i].type = data[offset];
    offset += sizeof(uint8_t);

    if (offset + sizeof(uint16_t) > length) 
    {
      message_free(message);
      return MESSAGE_ERROR_PARSE_ERROR;
    }

    // Parse length.
    memcpy(&message->tlv[i].length, &data[offset], sizeof(uint16_t));
    message->tlv[i].length = ntohs(message->tlv[i].length);
    offset += sizeof(uint16_t);

    if (offset + message->tlv[i].length > length) 
    {
      message_free(message);
      return MESSAGE_ERROR_PARSE_ERROR;
    }

    // Parse value.
    message->tlv[i].value = (uint8_t*) malloc(message->tlv[i].length);
    if (!message->tlv[i].value) 
    {
      message_free(message);
      return MESSAGE_ERROR_OUT_OF_MEMORY;
    }

    memcpy(message->tlv[i].value, &data[offset], message->tlv[i].length);
    offset += message->tlv[i].length;

    // If this is a checksum TLV, do checksum verification immediately.
    if (message->tlv[i].type == TLV_CHECKSUM) 
    {
      uint32_t checksum = message_checksum(message);
      if (memcmp(&checksum, message->tlv[i].value, sizeof(uint32_t) != 0))
      {
        message_free(message);
        return MESSAGE_ERROR_CHECKSUM_MISMATCH;
      }
    }

    message->length++;
  }

  return MESSAGE_SUCCESS;
}

message_result_t message_tlv_add(message_t *message, uint8_t type, uint16_t length, const uint8_t *value)
{
  //Serial.print("message.length enter add function ");
  //Serial.println(message->length);
  if (message->length >= MAX_TLV_COUNT)
  {
    return MESSAGE_ERROR_TOO_MANY_TLVS;
  }

  size_t i = message->length;
  message->tlv[i].value = (uint8_t*) malloc(length);
  if (!message->tlv[i].value)
  {
    return MESSAGE_ERROR_OUT_OF_MEMORY;
  }

  message->tlv[i].type = type;
  message->tlv[i].length = length;
  memcpy(message->tlv[i].value, value, length);
  message->length++;
  //Serial.print("message.length exit add function ");
  //Serial.println(message->length);

  return MESSAGE_SUCCESS;
}

message_result_t message_tlv_add_command(message_t *message, tlv_command_t command)
{
  uint8_t _command = command;
  return message_tlv_add(message, TLV_COMMAND, sizeof(uint8_t), (uint8_t*) &_command);
}

message_result_t message_tlv_add_reply(message_t *message, tlv_reply_t reply)
{
  uint8_t _reply = reply;
  return message_tlv_add(message, TLV_REPLY, sizeof(uint8_t), (uint8_t*) &_reply);
}

message_result_t message_tlv_add_motor_position(message_t *message, const tlv_motor_position_t *position)
{
  tlv_motor_position_t tmp;
  tmp.x = htonl(position->x);
  tmp.y = htonl(position->y);
  return message_tlv_add(message, TLV_MOTOR_POSITION, sizeof(tlv_motor_position_t), (uint8_t*) &tmp);
}

message_result_t message_tlv_add_encoder_value(message_t *message, const tlv_encoder_value_t *value)
{
  tlv_encoder_value_t tmp;
  tmp.x = htonl(value->x);
  tmp.y = htonl(value->y);
  return message_tlv_add(message, TLV_ENCODER_VALUE, sizeof(tlv_encoder_value_t), (uint8_t*) &tmp);
}

message_result_t message_tlv_add_checksum(message_t *message)
{
  uint32_t checksum = message_checksum(message);
//  Serial.print("checksum ");
//  Serial.println(checksum);
//  Serial.println(checksum, HEX);
  return message_tlv_add(message, TLV_CHECKSUM, sizeof(uint32_t), (uint8_t*) &checksum);
}

message_result_t message_tlv_get(const message_t *message, uint8_t type, uint8_t *destination, size_t length)
{
  for (size_t i = 0; i < message->length; i++)
  {
    if (message->tlv[i].type == type)
    {
      //assert(message->tlv[i].length <= length);
      memcpy(destination, message->tlv[i].value, message->tlv[i].length);
      return MESSAGE_SUCCESS;
    }
  }

  return MESSAGE_ERROR_TLV_NOT_FOUND;
}

message_result_t message_tlv_get_command(const message_t *message, tlv_command_t *command)
{
  uint8_t _command;
  message_result_t result = message_tlv_get(message, TLV_COMMAND, &_command, sizeof(uint8_t));
  if (result != MESSAGE_SUCCESS)
  {
    return result;
  }

  *command = (tlv_command_t) _command;

  return MESSAGE_SUCCESS;
}

message_result_t message_tlv_get_reply(const message_t *message, tlv_reply_t *reply)
{
  uint8_t _reply;
  message_result_t result = message_tlv_get(message, TLV_REPLY, &_reply, sizeof(uint8_t));
  if (result != MESSAGE_SUCCESS)
  {
    return result;
  }

  *reply = (tlv_reply_t)_reply;

  return MESSAGE_SUCCESS;
}

message_result_t message_tlv_get_motor_position(const message_t *message, tlv_motor_position_t *position)
{
  message_result_t result = message_tlv_get(message, TLV_MOTOR_POSITION, (uint8_t*) position, sizeof(tlv_motor_position_t));
  if (result != MESSAGE_SUCCESS)
  {
    return result;
  }

  position->x = ntohl(position->x);
  position->y = ntohl(position->y);

  return MESSAGE_SUCCESS;
}

size_t message_serialized_size(const message_t *message)
{
  size_t size = 0;
  for (size_t i = 0; i < message->length; i++)
  {
    size += sizeof(uint8_t) + sizeof(uint16_t) + message->tlv[i].length;
  }

  return size;
}

ssize_t message_serialize(uint8_t *buffer, size_t length, const message_t *message)
{
  size_t offset = 0;
  size_t remaining_buffer = length;
  for (size_t i = 0; i < message->length; i++)
  {
    size_t tlv_length = sizeof(uint8_t) + sizeof(uint16_t) + message->tlv[i].length;
    if (remaining_buffer < tlv_length) 
    {
      return MESSAGE_ERROR_BUFFER_TOO_SMALL;
    }

    uint16_t hlength = htons(message->tlv[i].length);

    memcpy(buffer + offset, &message->tlv[i].type, sizeof(uint8_t));
    memcpy(buffer + offset + sizeof(uint8_t), &hlength, sizeof(uint16_t));
    memcpy(buffer + offset + sizeof(uint8_t) + sizeof(uint16_t), message->tlv[i].value, message->tlv[i].length);

    offset += tlv_length;
    remaining_buffer -= tlv_length;
  }

  return offset;
}

void message_print(const message_t *message)
{

  Serial.print("<Message tlvs(");
  Serial.print((unsigned int) message->length);
  Serial.print(")=[");
  for (size_t i = 0; i < message->length; i++) 
  {
    uint8_t *data = message->tlv[i].value;
    size_t data_length = message->tlv[i].length;

    Serial.print("{");
    Serial.print(message->tlv[i].type);
    Serial.print(", \"");
    for (size_t j = 0; j < data_length; j++) 
    {
      //Serial.print("%02X%s", data[j], (j < data_length - 1) ? " " : "");
      Serial.print(data[j], HEX);
    }
    //printf("\"}%s", (i < message->length - 1) ? "," : "");
    Serial.print("\"}");
  }
  //printf("]>");
  Serial.print("]>");
}

uint32_t message_checksum(const message_t *message)
{
  
  uint32_t checksum = 0;
  // Create a CRC32 checksum calculator.
  CRC32 crc;
  for (size_t i = 0; i < message->length; i++) 
  {
    crc.update(message->tlv[i].value, message->tlv[i].length);
    //checksum = crc32(checksum, message->tlv[i].value, message->tlv[i].length);
    //Serial.print("Voja");
    //Serial.println(checksum);
  }
  checksum = crc.finalize();
  //return checksum;
  return htonl(checksum);
}

message_result_t message_tlv_add_error_report(message_t *message, const tlv_error_report_t *report)
{
	tlv_error_report_t tmp;
	tmp.code = htonl(report->code);
	return message_tlv_add(message, TLV_ERROR_REPORT, sizeof(tlv_error_report_t), (uint8_t*) &tmp);
}

message_result_t message_tlv_get_error_report(const message_t *message, tlv_error_report_t *report)
{
  message_result_t result = message_tlv_get(message, TLV_ERROR_REPORT, (uint8_t*) report, sizeof(tlv_error_report_t));
  if (result != MESSAGE_SUCCESS)
  {
    return result;
  }

  report->code = ntohl(report->code);

  return MESSAGE_SUCCESS;
}

message_result_t message_tlv_add_debug_string(message_t *message, String debug_string)
{
  // Length (with one extra character for the null terminator).
  int str_len = debug_string.length() + 1; 
  
  // Prepare the character array (the buffer).
  char char_array[str_len];
  
  // Copy it over.
  debug_string.toCharArray(char_array, str_len);

  // Add debug TLV to a protocol message.
  return message_tlv_add(message, TLV_DEBUG, str_len, (uint8_t*) char_array);
}

message_result_t message_tlv_get_debug_string(const message_t *message, String *debug_string)
{
  for (size_t i = 0; i < message->length; i++)
  {
    if (message->tlv[i].type == TLV_DEBUG)
    {
      *debug_string = String((char*) message->tlv[i].value);
      return MESSAGE_SUCCESS;
    }
  }

  debug_string = nullptr;
  return MESSAGE_ERROR_TLV_NOT_FOUND;
}

