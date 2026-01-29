/****************************************************************************
 * libs/libc/misc/lib_crc8h2f.c
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

/* [AUTOSAR_SWS_Crc_00042] The Crc_CalculateCRC8H2F function of the CRC
 * module shall implement the CRC8 routine based on the generator
 * polynomial 0x2F.
 *
 * CRC8 parameters:
 * CRC result width: 8 bits
 * Polynomial: 2Fh
 * Initial value: FFh
 * Input data reflected: No
 * Result data reflected: No
 * XOR value: FFh
 *
 * [AUTOSAR_SWS_Crc_00053] The Crc_CalculateCRC8H2F function of the CRC
 * module shall provide the following CRC results:
 *
 * Data bytes (hexadecimal)    |   CRC
 * 00 00 00 00                 |   12
 * F2 01 83                    |   C2
 * 0F AA 00 55                 |   C6
 * 00 FF 55 11                 |   77
 * 33 22 55 AA BB CC DD EE FF  |   11
 * 92 6B 55                    |   33
 * FF FF FF FF                 |   6C
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
 * Name: crc8h2f_part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer
 *   using the 0x2F polynomial.
 *
 ****************************************************************************/

uint8_t crc8h2f_part(FAR const uint8_t *src, size_t len, uint8_t crc8val)
{
  static const uint8_t g_crc8_tab[256] =
  {
    0x00u, 0x2fu, 0x5eu, 0x71u, 0xbcu, 0x93u, 0xe2u, 0xcdu,
    0x57u, 0x78u, 0x09u, 0x26u, 0xebu, 0xc4u, 0xb5u, 0x9au,
    0xaeu, 0x81u, 0xf0u, 0xdfu, 0x12u, 0x3du, 0x4cu, 0x63u,
    0xf9u, 0xd6u, 0xa7u, 0x88u, 0x45u, 0x6au, 0x1bu, 0x34u,
    0x73u, 0x5cu, 0x2du, 0x02u, 0xcfu, 0xe0u, 0x91u, 0xbeu,
    0x24u, 0x0bu, 0x7au, 0x55u, 0x98u, 0xb7u, 0xc6u, 0xe9u,
    0xddu, 0xf2u, 0x83u, 0xacu, 0x61u, 0x4eu, 0x3fu, 0x10u,
    0x8au, 0xa5u, 0xd4u, 0xfbu, 0x36u, 0x19u, 0x68u, 0x47u,
    0xe6u, 0xc9u, 0xb8u, 0x97u, 0x5au, 0x75u, 0x04u, 0x2bu,
    0xb1u, 0x9eu, 0xefu, 0xc0u, 0x0du, 0x22u, 0x53u, 0x7cu,
    0x48u, 0x67u, 0x16u, 0x39u, 0xf4u, 0xdbu, 0xaau, 0x85u,
    0x1fu, 0x30u, 0x41u, 0x6eu, 0xa3u, 0x8cu, 0xfdu, 0xd2u,
    0x95u, 0xbau, 0xcbu, 0xe4u, 0x29u, 0x06u, 0x77u, 0x58u,
    0xc2u, 0xedu, 0x9cu, 0xb3u, 0x7eu, 0x51u, 0x20u, 0x0fu,
    0x3bu, 0x14u, 0x65u, 0x4au, 0x87u, 0xa8u, 0xd9u, 0xf6u,
    0x6cu, 0x43u, 0x32u, 0x1du, 0xd0u, 0xffu, 0x8eu, 0xa1u,
    0xe3u, 0xccu, 0xbdu, 0x92u, 0x5fu, 0x70u, 0x01u, 0x2eu,
    0xb4u, 0x9bu, 0xeau, 0xc5u, 0x08u, 0x27u, 0x56u, 0x79u,
    0x4du, 0x62u, 0x13u, 0x3cu, 0xf1u, 0xdeu, 0xafu, 0x80u,
    0x1au, 0x35u, 0x44u, 0x6bu, 0xa6u, 0x89u, 0xf8u, 0xd7u,
    0x90u, 0xbfu, 0xceu, 0xe1u, 0x2cu, 0x03u, 0x72u, 0x5du,
    0xc7u, 0xe8u, 0x99u, 0xb6u, 0x7bu, 0x54u, 0x25u, 0x0au,
    0x3eu, 0x11u, 0x60u, 0x4fu, 0x82u, 0xadu, 0xdcu, 0xf3u,
    0x69u, 0x46u, 0x37u, 0x18u, 0xd5u, 0xfau, 0x8bu, 0xa4u,
    0x05u, 0x2au, 0x5bu, 0x74u, 0xb9u, 0x96u, 0xe7u, 0xc8u,
    0x52u, 0x7du, 0x0cu, 0x23u, 0xeeu, 0xc1u, 0xb0u, 0x9fu,
    0xabu, 0x84u, 0xf5u, 0xdau, 0x17u, 0x38u, 0x49u, 0x66u,
    0xfcu, 0xd3u, 0xa2u, 0x8du, 0x40u, 0x6fu, 0x1eu, 0x31u,
    0x76u, 0x59u, 0x28u, 0x07u, 0xcau, 0xe5u, 0x94u, 0xbbu,
    0x21u, 0x0eu, 0x7fu, 0x50u, 0x9du, 0xb2u, 0xc3u, 0xecu,
    0xd8u, 0xf7u, 0x86u, 0xa9u, 0x64u, 0x4bu, 0x3au, 0x15u,
    0x8fu, 0xa0u, 0xd1u, 0xfeu, 0x33u, 0x1cu, 0x6du, 0x42u
  };

  return crc8table(g_crc8_tab, src, len, crc8val);
}

/****************************************************************************
 * Name: crc8h2f
 *
 * Description:
 *   Return a 8-bit CRC of the contents of the 'src' buffer, length 'len'
 *   using the 0x2F polynomial.
 *
 ****************************************************************************/

uint8_t crc8h2f(FAR const uint8_t *src, size_t len)
{
  return crc8h2f_part(src, len, 0x00u);
}
