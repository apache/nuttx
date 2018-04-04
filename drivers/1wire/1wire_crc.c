/****************************************************************************
 * drivers/1wire/1wire_crc.c
 *
 *   Copyright (C) 2018 Haltian Ltd. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "1wire_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Compute a Dallas Semiconductor 8 bit CRC. */

uint8_t onewire_crc8(FAR const uint8_t *input, uint8_t len)
{
  uint8_t crc = 0;

  while (len-- > 0)
    {
      int i;
      uint8_t inbyte = *input++;

      for (i = 0; i < 8; i++)
        {
          uint8_t mix = (crc ^ inbyte) & 0x01;
          crc >>= 1;
          if (mix)
            crc ^= 0x8c;

          inbyte >>= 1;
        }
    }

  return crc;
}

/* Compute a Dallas Semiconductor 16 bit CRC. This is used to check
 * the integrity of received data from many 1-wire devices.
 *
 * Note: the CRC-16 computed here is not what you'll get from the 1-wire
 * network, because:
 * - The CRC-16 is transmitted bitwise inverted.
 * - The binary representation of the return value may have a different
 *   byte order than the two bytes you get from 1-wire due to endian-ness.
 */

uint16_t onewire_crc16(FAR const uint8_t *input, uint16_t len,
                       uint16_t initial_crc)
{
  uint16_t crc = initial_crc;

  while (len-- > 0)
    {
      int i;
      uint8_t inbyte = *input++;

      for (i = 0; i < 8; i++)
        {
          uint8_t mix = ((crc & 0xff) ^ inbyte) & 0x01;

          crc >>= 1;
          if (mix)
            {
              crc ^= 0xa001;
            }

          inbyte >>= 1;
        }
    }

  return crc;
}

/****************************************************************************
 * Name: onewire_valid_rom
 *
 * Description:
 *   Check CRC-8 of received input
 *
 * Input Parameters:
 *   rom   - 64-bit rom code
 *
 * Returned Value:
 *   true if CRC-8 of rom matches
 *
 ****************************************************************************/

bool onewire_valid_rom(uint64_t rom)
{
  uint8_t crc;
  uint8_t buf[8];

  memcpy(buf, &rom, sizeof(buf));
  crc = onewire_crc8(buf, 7);
  return crc == buf[7];
}

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
                         FAR const uint8_t *inverted_crc)
{
  uint16_t crc;

  crc = ~onewire_crc16(input, len, 0);
  return (crc & 0xff) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}
