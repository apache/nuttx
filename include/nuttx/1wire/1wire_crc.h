/****************************************************************************
 * include/nuttx/1wire/1wire_crc.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_1WIRE_1WIRE_CRC_H
#define __INCLUDE_NUTTX_1WIRE_1WIRE_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: onewire_crc8
 *
 * Description:
 *   Compute a Dallas Semiconductor 8 bit CRC.
 *
 * Input Parameters:
 *   input - Input buffer for crc calculation
 *   len   - Buffer len
 *
 * Returned Value:
 *   Calculated crc8
 *
 ****************************************************************************/

uint8_t onewire_crc8(FAR const uint8_t *input, uint8_t len);

/****************************************************************************
 * Name: onewire_crc16
 *
 * Description:
 *   Compute a Dallas Semiconductor 16 bit CRC. This is used to check
 *   the integrity of received data from many 1-wire devices.
 *
 *   Note: the CRC-16 computed here is not what you'll get from the 1-wire
 *   network, because:
 * - The CRC-16 is transmitted bitwise inverted.
 * - The binary representation of the return value may have a different
 *   byte order than the two bytes you get from 1-wire due to endian-ness.
 *
 * Input Parameters:
 *   input       - Input buffer for crc calculation
 *   len         - Buffer len
 *   initial_crc - Initial crc for calculation
 *
 * Returned Value:
 *   Calculated crc16
 *
 ****************************************************************************/

uint16_t onewire_crc16(FAR const uint8_t *input, uint16_t len,
                       uint16_t initial_crc);

/****************************************************************************
 * Name: onewire_check_crc16
 *
 * Description:
 *   Check CRC-16 of received input
 *
 * Input Parameters:
 *   input         - Array of bytes to checksum
 *   len           - Length of input
 *   inverted_crc  - The two CRC16 bytes in the received data
 *
 * Returned Value:
 *   true if CRC-16 matches
 *
 ****************************************************************************/

bool onewire_check_crc16(FAR const uint8_t *input, uint16_t len,
                         FAR const uint8_t *inverted_crc);

#endif /* __INCLUDE_NUTTX_1WIRE_1WIRE_CRC_H */
