/****************************************************************************
 * libs/libc/misc/lib_crc16h8005.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/* [AUTOSAR_SWS_Crc_00067] The CRC module shall implement the CRC16
 * based on the CRC-16/ARC Standard.
 *
 * CRC16 parameters:
 * CRC result width: 16 bits
 * Polynomial: 8005h
 * Initial value: 0000h
 * Input data reflected: Yes
 * Result data reflected: Yes
 * XOR value: 0000h
 *
 * [AUTOSAR_SWS_Crc_00068] The Crc_CalculateCRC16ARC function of the CRC
 * module shall provide the following CRC results:
 *
 * Data bytes (hexadecimal)    |   CRC
 * 00 00 00 00                 |   0000
 * F2 01 83                    |   C2E1
 * 0F AA 00 55                 |   0BE3
 * 00 FF 55 11                 |   6CCF
 * 33 22 55 AA BB CC DD EE FF  |   AE98
 * 92 6B 55                    |   E24E
 * FF FF FF FF                 |   9401
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/crc16.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * CRC16 table generated with:
 *
 * #include <stdint.h>
 *
 * void generate_crc16_table(uint16_t poly, uint16_t *table)
 * {
 *   for (int i = 0; i < 256; i++)
 *     {
 *       uint16_t crc = (uint16_t)(i << 8);
 *       for (uint8_t bit = 0; bit < 8; bit++)
 *         {
 *           if (crc & 0x8000)
 *             {
 *               crc = (crc << 1) ^ polynomial;
 *             }
 *           else
 *             {
 *               crc = crc << 1;
 *             }
 *         }
 *       crcTable[i] = crc;
 *     }
 * }
 *
 * int main()
 * {
 *   uint16_t crc16_table[256];
 *   const uint8_t poly = 0x2F;
 *
 *   generate_crc16_table(poly, crc16_table);
 *
 *   printf("CRC16 Static Table (Poly 0x%02X):\n", poly);
 *   for (int i = 0; i < 256; i++)
 *     {
 *       if (i % 16 == 0) printf("\n");
 *       printf("0x%02X, ", crc16_table[i]);
 *     }
 *
 *   return 0;
 * }
 *
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: crc16h8005_part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer
 *   using the 0x8005 polynomial.
 *
 ****************************************************************************/

uint16_t crc16h8005_part(FAR const uint8_t *src,
                         size_t len, uint16_t crc16val)
{
  size_t i;
  uint16_t v = crc16val;
  static const uint16_t g_crc16_tab[256] =
  {
    0x0000u, 0xc0c1u, 0xc181u, 0x0140u, 0xc301u, 0x03c0u, 0x0280u, 0xc241u,
    0xc601u, 0x06c0u, 0x0780u, 0xc741u, 0x0500u, 0xc5c1u, 0xc481u, 0x0440u,
    0xcc01u, 0x0cc0u, 0x0d80u, 0xcd41u, 0x0f00u, 0xcfc1u, 0xce81u, 0x0e40u,
    0x0a00u, 0xcac1u, 0xcb81u, 0x0b40u, 0xc901u, 0x09c0u, 0x0880u, 0xc841u,
    0xd801u, 0x18c0u, 0x1980u, 0xd941u, 0x1b00u, 0xdbc1u, 0xda81u, 0x1a40u,
    0x1e00u, 0xdec1u, 0xdf81u, 0x1f40u, 0xdd01u, 0x1dc0u, 0x1c80u, 0xdc41u,
    0x1400u, 0xd4c1u, 0xd581u, 0x1540u, 0xd701u, 0x17c0u, 0x1680u, 0xd641u,
    0xd201u, 0x12c0u, 0x1380u, 0xd341u, 0x1100u, 0xd1c1u, 0xd081u, 0x1040u,
    0xf001u, 0x30c0u, 0x3180u, 0xf141u, 0x3300u, 0xf3c1u, 0xf281u, 0x3240u,
    0x3600u, 0xf6c1u, 0xf781u, 0x3740u, 0xf501u, 0x35c0u, 0x3480u, 0xf441u,
    0x3c00u, 0xfcc1u, 0xfd81u, 0x3d40u, 0xff01u, 0x3fc0u, 0x3e80u, 0xfe41u,
    0xfa01u, 0x3ac0u, 0x3b80u, 0xfb41u, 0x3900u, 0xf9c1u, 0xf881u, 0x3840u,
    0x2800u, 0xe8c1u, 0xe981u, 0x2940u, 0xeb01u, 0x2bc0u, 0x2a80u, 0xea41u,
    0xee01u, 0x2ec0u, 0x2f80u, 0xef41u, 0x2d00u, 0xedc1u, 0xec81u, 0x2c40u,
    0xe401u, 0x24c0u, 0x2580u, 0xe541u, 0x2700u, 0xe7c1u, 0xe681u, 0x2640u,
    0x2200u, 0xe2c1u, 0xe381u, 0x2340u, 0xe101u, 0x21c0u, 0x2080u, 0xe041u,
    0xa001u, 0x60c0u, 0x6180u, 0xa141u, 0x6300u, 0xa3c1u, 0xa281u, 0x6240u,
    0x6600u, 0xa6c1u, 0xa781u, 0x6740u, 0xa501u, 0x65c0u, 0x6480u, 0xa441u,
    0x6c00u, 0xacc1u, 0xad81u, 0x6d40u, 0xaf01u, 0x6fc0u, 0x6e80u, 0xae41u,
    0xaa01u, 0x6ac0u, 0x6b80u, 0xab41u, 0x6900u, 0xa9c1u, 0xa881u, 0x6840u,
    0x7800u, 0xb8c1u, 0xb981u, 0x7940u, 0xbb01u, 0x7bc0u, 0x7a80u, 0xba41u,
    0xbe01u, 0x7ec0u, 0x7f80u, 0xbf41u, 0x7d00u, 0xbdc1u, 0xbc81u, 0x7c40u,
    0xb401u, 0x74c0u, 0x7580u, 0xb541u, 0x7700u, 0xb7c1u, 0xb681u, 0x7640u,
    0x7200u, 0xb2c1u, 0xb381u, 0x7340u, 0xb101u, 0x71c0u, 0x7080u, 0xb041u,
    0x5000u, 0x90c1u, 0x9181u, 0x5140u, 0x9301u, 0x53c0u, 0x5280u, 0x9241u,
    0x9601u, 0x56c0u, 0x5780u, 0x9741u, 0x5500u, 0x95c1u, 0x9481u, 0x5440u,
    0x9c01u, 0x5cc0u, 0x5d80u, 0x9d41u, 0x5f00u, 0x9fc1u, 0x9e81u, 0x5e40u,
    0x5a00u, 0x9ac1u, 0x9b81u, 0x5b40u, 0x9901u, 0x59c0u, 0x5880u, 0x9841u,
    0x8801u, 0x48c0u, 0x4980u, 0x8941u, 0x4b00u, 0x8bc1u, 0x8a81u, 0x4a40u,
    0x4e00u, 0x8ec1u, 0x8f81u, 0x4f40u, 0x8d01u, 0x4dc0u, 0x4c80u, 0x8c41u,
    0x4400u, 0x84c1u, 0x8581u, 0x4540u, 0x8701u, 0x47c0u, 0x4680u, 0x8641u,
    0x8201u, 0x42c0u, 0x4380u, 0x8341u, 0x4100u, 0x81c1u, 0x8081u, 0x4040u
  };

  for (i = 0u; i < len; i++)
    {
      v = (v >> 8) ^ g_crc16_tab[(v ^ src[i]) & 0xffu];
    }

  return v;
}

/****************************************************************************
 * Name: crc16h8005
 *
 * Description:
 *   Return a 16-bit CRC of the contents of the 'src' buffer, length 'len'
 *   using the 0x8005 polynomial.
 *
 ****************************************************************************/

uint16_t crc16h8005(FAR const uint8_t *src, size_t len)
{
  return crc16h8005_part(src, len, 0x0000u);
}
