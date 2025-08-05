/****************************************************************************
 * libs/libc/misc/lib_crc8h1d.c
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

/* [AUTOSAR_SWS_Crc_00030] The Crc_CalculateCRC8 function of the CRC module
 * shall implement the CRC8 routine based on the SAE-J1850 CRC8 Standard.
 *
 * CRC8 parameters:
 * CRC result width: 8 bits
 * Polynomial: 1Dh
 * Initial value: FFh
 * Input data reflected: No
 * Result data reflected: No
 * XOR value: FFh
 *
 * [AUTOSAR_SWS_Crc_00052] The Crc_CalculateCRC8 function of the CRC module
 * shall provide the following CRC results:
 *
 * Data bytes (hexadecimal)    |   CRC
 * 00 00 00 00                 |   59
 * F2 01 83                    |   37
 * 0F AA 00 55                 |   79
 * 00 FF 55 11                 |   B8
 * 33 22 55 AA BB CC DD EE FF  |   CB
 * 92 6B 55                    |   8C
 * FF FF FF FF                 |   74
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/crc8.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * CRC8 table generated with:
 *
 * #include <stdint.h>
 *
 * void generate_crc8_table(uint8_t poly, uint8_t *table)
 * {
 *   for (int i = 0; i < 256; i++)
 *     {
 *       uint8_t crc = i;
 *       for (uint8_t bit = 0; bit < 8; bit++)
 *         {
 *           if (crc & 0x80)
 *             {
 *               crc = (crc << 1) ^ poly;
 *             }
 *           else
 *             {
 *               crc = crc << 1;
 *             }
 *         }
 *       table[i] = crc;
 *     }
 * }
 *
 * int main()
 * {
 *   uint8_t crc8_table[256];
 *   const uint8_t poly = 0x2F;
 *
 *   generate_crc8_table(poly, crc8_table);
 *
 *   printf("CRC8 Static Table (Poly 0x%02X):\n", poly);
 *   for (int i = 0; i < 256; i++)
 *     {
 *       if (i % 16 == 0) printf("\n");
 *       printf("0x%02X, ", crc8_table[i]);
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
 * Name: crc8h1d_part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer
 *   using the 0x1D polynomial.
 *
 ****************************************************************************/

uint8_t crc8h1d_part(FAR const uint8_t *src, size_t len, uint8_t crc8val)
{
  static const uint8_t g_crc8_tab[256] =
  {
    0x00u, 0x1du, 0x3au, 0x27u, 0x74u, 0x69u, 0x4eu, 0x53u,
    0xe8u, 0xf5u, 0xd2u, 0xcfu, 0x9cu, 0x81u, 0xa6u, 0xbbu,
    0xcdu, 0xd0u, 0xf7u, 0xeau, 0xb9u, 0xa4u, 0x83u, 0x9eu,
    0x25u, 0x38u, 0x1fu, 0x02u, 0x51u, 0x4cu, 0x6bu, 0x76u,
    0x87u, 0x9au, 0xbdu, 0xa0u, 0xf3u, 0xeeu, 0xc9u, 0xd4u,
    0x6fu, 0x72u, 0x55u, 0x48u, 0x1bu, 0x06u, 0x21u, 0x3cu,
    0x4au, 0x57u, 0x70u, 0x6du, 0x3eu, 0x23u, 0x04u, 0x19u,
    0xa2u, 0xbfu, 0x98u, 0x85u, 0xd6u, 0xcbu, 0xecu, 0xf1u,
    0x13u, 0x0eu, 0x29u, 0x34u, 0x67u, 0x7au, 0x5du, 0x40u,
    0xfbu, 0xe6u, 0xc1u, 0xdcu, 0x8fu, 0x92u, 0xb5u, 0xa8u,
    0xdeu, 0xc3u, 0xe4u, 0xf9u, 0xaau, 0xb7u, 0x90u, 0x8du,
    0x36u, 0x2bu, 0x0cu, 0x11u, 0x42u, 0x5fu, 0x78u, 0x65u,
    0x94u, 0x89u, 0xaeu, 0xb3u, 0xe0u, 0xfdu, 0xdau, 0xc7u,
    0x7cu, 0x61u, 0x46u, 0x5bu, 0x08u, 0x15u, 0x32u, 0x2fu,
    0x59u, 0x44u, 0x63u, 0x7eu, 0x2du, 0x30u, 0x17u, 0x0au,
    0xb1u, 0xacu, 0x8bu, 0x96u, 0xc5u, 0xd8u, 0xffu, 0xe2u,
    0x26u, 0x3bu, 0x1cu, 0x01u, 0x52u, 0x4fu, 0x68u, 0x75u,
    0xceu, 0xd3u, 0xf4u, 0xe9u, 0xbau, 0xa7u, 0x80u, 0x9du,
    0xebu, 0xf6u, 0xd1u, 0xccu, 0x9fu, 0x82u, 0xa5u, 0xb8u,
    0x03u, 0x1eu, 0x39u, 0x24u, 0x77u, 0x6au, 0x4du, 0x50u,
    0xa1u, 0xbcu, 0x9bu, 0x86u, 0xd5u, 0xc8u, 0xefu, 0xf2u,
    0x49u, 0x54u, 0x73u, 0x6eu, 0x3du, 0x20u, 0x07u, 0x1au,
    0x6cu, 0x71u, 0x56u, 0x4bu, 0x18u, 0x05u, 0x22u, 0x3fu,
    0x84u, 0x99u, 0xbeu, 0xa3u, 0xf0u, 0xedu, 0xcau, 0xd7u,
    0x35u, 0x28u, 0x0fu, 0x12u, 0x41u, 0x5cu, 0x7bu, 0x66u,
    0xddu, 0xc0u, 0xe7u, 0xfau, 0xa9u, 0xb4u, 0x93u, 0x8eu,
    0xf8u, 0xe5u, 0xc2u, 0xdfu, 0x8cu, 0x91u, 0xb6u, 0xabu,
    0x10u, 0x0du, 0x2au, 0x37u, 0x64u, 0x79u, 0x5eu, 0x43u,
    0xb2u, 0xafu, 0x88u, 0x95u, 0xc6u, 0xdbu, 0xfcu, 0xe1u,
    0x5au, 0x47u, 0x60u, 0x7du, 0x2eu, 0x33u, 0x14u, 0x09u,
    0x7fu, 0x62u, 0x45u, 0x58u, 0x0bu, 0x16u, 0x31u, 0x2cu,
    0x97u, 0x8au, 0xadu, 0xb0u, 0xe3u, 0xfeu, 0xd9u, 0xc4u
  };

  return crc8table(g_crc8_tab, src, len, crc8val);
}

/****************************************************************************
 * Name: crc8h1d
 *
 * Description:
 *   Return a 8-bit CRC of the contents of the 'src' buffer, length 'len'
 *   using the 0x1D polynomial.
 *
 ****************************************************************************/

uint8_t crc8h1d(FAR const uint8_t *src, size_t len)
{
  return crc8h1d_part(src, len, 0x00u);
}
