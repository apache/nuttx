/****************************************************************************
 * include/nuttx/sensors/msxxxx_crc4.h
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_SENSORS_MSXXXX_CRC4_H
#define __INCLUDE_NUTTX_SENSORS_MSXXXX_CRC4_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ms58xx_crc
 *
 * Description:
 *   Calculate the CRC.
 *
 ****************************************************************************/

static uint8_t msxxxx_crc4(FAR uint16_t *src,
                           uint8_t crcndx,
                           uint16_t crcmask)
{
  uint16_t cnt;
  uint16_t n_rem;
  uint16_t crc_read;
  uint8_t n_bit;

  n_rem = 0x00;
  crc_read = src[crcndx];
  src[crcndx] &= ~crcmask;

  for (cnt = 0; cnt < 16; cnt++)
    {
      if (cnt % 2 == 1)
        {
          n_rem ^= src[cnt >> 1] & 0x00ff;
        }
      else
        {
          n_rem ^= src[cnt >> 1] >> 8;
        }

      for (n_bit = 8; n_bit > 0; n_bit--)
        {
          if (n_rem & (0x8000) != 0)
            {
              n_rem = (n_rem << 1) ^ 0x3000;
            }
          else
            {
              n_rem <<= 1;
            }
        }
    }

  n_rem = (n_rem >> 12) & 0x000f;
  src[crcndx] = crc_read;
  return n_rem ^ 0x00;
}

#endif /* __INCLUDE_NUTTX_SENSORS_MSXXXX_CRC4_H */
