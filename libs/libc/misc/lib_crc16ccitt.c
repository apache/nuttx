/****************************************************************************
 * libs/libc/misc/lib_crc16ccitt.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: crc16ccittpart
 *
 * Description:
 *   Continue 16-bit CRC-CCITT calculation on a part of the buffer using the
 *   polynomial x^16+x^12+x^5+1.
 *
 *   This function is able to calculate any CRC that uses 0x1021 as it
 *   polynomial and requires reflecting both the input and the output.
 *   The following checksums can, among others, be calculated by this
 *   function, depending on the value provided for the initial seed and the
 *   value the final calculated CRC is XORed with:
 *
 *   - CRC-16/CCITT, CRC-16/CCITT-TRUE, CRC-16/KERMIT
 *   https://reveng.sourceforge.io/crc-catalogue/16.htm#crc.cat.crc-16-kermit
 *   initial seed: 0x0000, xor output: 0x0000
 *
 ****************************************************************************/

uint16_t crc16ccittpart(FAR const uint8_t *src, size_t len,
                        uint16_t crc16val)
{
  size_t i;
  uint16_t v = crc16val;
  static const uint16_t g_crc16_tab[256] =
  {
    0x0000u, 0x1189u, 0x2312u, 0x329bu, 0x4624u, 0x57adu, 0x6536u, 0x74bfu,
    0x8c48u, 0x9dc1u, 0xaf5au, 0xbed3u, 0xca6cu, 0xdbe5u, 0xe97eu, 0xf8f7u,
    0x1081u, 0x0108u, 0x3393u, 0x221au, 0x56a5u, 0x472cu, 0x75b7u, 0x643eu,
    0x9cc9u, 0x8d40u, 0xbfdbu, 0xae52u, 0xdaedu, 0xcb64u, 0xf9ffu, 0xe876u,
    0x2102u, 0x308bu, 0x0210u, 0x1399u, 0x6726u, 0x76afu, 0x4434u, 0x55bdu,
    0xad4au, 0xbcc3u, 0x8e58u, 0x9fd1u, 0xeb6eu, 0xfae7u, 0xc87cu, 0xd9f5u,
    0x3183u, 0x200au, 0x1291u, 0x0318u, 0x77a7u, 0x662eu, 0x54b5u, 0x453cu,
    0xbdcbu, 0xac42u, 0x9ed9u, 0x8f50u, 0xfbefu, 0xea66u, 0xd8fdu, 0xc974u,
    0x4204u, 0x538du, 0x6116u, 0x709fu, 0x0420u, 0x15a9u, 0x2732u, 0x36bbu,
    0xce4cu, 0xdfc5u, 0xed5eu, 0xfcd7u, 0x8868u, 0x99e1u, 0xab7au, 0xbaf3u,
    0x5285u, 0x430cu, 0x7197u, 0x601eu, 0x14a1u, 0x0528u, 0x37b3u, 0x263au,
    0xdecdu, 0xcf44u, 0xfddfu, 0xec56u, 0x98e9u, 0x8960u, 0xbbfbu, 0xaa72u,
    0x6306u, 0x728fu, 0x4014u, 0x519du, 0x2522u, 0x34abu, 0x0630u, 0x17b9u,
    0xef4eu, 0xfec7u, 0xcc5cu, 0xddd5u, 0xa96au, 0xb8e3u, 0x8a78u, 0x9bf1u,
    0x7387u, 0x620eu, 0x5095u, 0x411cu, 0x35a3u, 0x242au, 0x16b1u, 0x0738u,
    0xffcfu, 0xee46u, 0xdcddu, 0xcd54u, 0xb9ebu, 0xa862u, 0x9af9u, 0x8b70u,
    0x8408u, 0x9581u, 0xa71au, 0xb693u, 0xc22cu, 0xd3a5u, 0xe13eu, 0xf0b7u,
    0x0840u, 0x19c9u, 0x2b52u, 0x3adbu, 0x4e64u, 0x5fedu, 0x6d76u, 0x7cffu,
    0x9489u, 0x8500u, 0xb79bu, 0xa612u, 0xd2adu, 0xc324u, 0xf1bfu, 0xe036u,
    0x18c1u, 0x0948u, 0x3bd3u, 0x2a5au, 0x5ee5u, 0x4f6cu, 0x7df7u, 0x6c7eu,
    0xa50au, 0xb483u, 0x8618u, 0x9791u, 0xe32eu, 0xf2a7u, 0xc03cu, 0xd1b5u,
    0x2942u, 0x38cbu, 0x0a50u, 0x1bd9u, 0x6f66u, 0x7eefu, 0x4c74u, 0x5dfdu,
    0xb58bu, 0xa402u, 0x9699u, 0x8710u, 0xf3afu, 0xe226u, 0xd0bdu, 0xc134u,
    0x39c3u, 0x284au, 0x1ad1u, 0x0b58u, 0x7fe7u, 0x6e6eu, 0x5cf5u, 0x4d7cu,
    0xc60cu, 0xd785u, 0xe51eu, 0xf497u, 0x8028u, 0x91a1u, 0xa33au, 0xb2b3u,
    0x4a44u, 0x5bcdu, 0x6956u, 0x78dfu, 0x0c60u, 0x1de9u, 0x2f72u, 0x3efbu,
    0xd68du, 0xc704u, 0xf59fu, 0xe416u, 0x90a9u, 0x8120u, 0xb3bbu, 0xa232u,
    0x5ac5u, 0x4b4cu, 0x79d7u, 0x685eu, 0x1ce1u, 0x0d68u, 0x3ff3u, 0x2e7au,
    0xe70eu, 0xf687u, 0xc41cu, 0xd595u, 0xa12au, 0xb0a3u, 0x8238u, 0x93b1u,
    0x6b46u, 0x7acfu, 0x4854u, 0x59ddu, 0x2d62u, 0x3cebu, 0x0e70u, 0x1ff9u,
    0xf78fu, 0xe606u, 0xd49du, 0xc514u, 0xb1abu, 0xa022u, 0x92b9u, 0x8330u,
    0x7bc7u, 0x6a4eu, 0x58d5u, 0x495cu, 0x3de3u, 0x2c6au, 0x1ef1u, 0x0f78u,
  };

  for (i = 0u; i < len; i++)
    {
      v = (v >> 8) ^ g_crc16_tab[(v ^ src[i]) & 0xffu];
    }

  return v;
}

/****************************************************************************
 * Name: crc16ccitt
 *
 * Description:
 *   Return a 16-bit CRC-CCITT of the contents of the 'src' buffer, length
 *   'len' using the polynomial x^16+x^12+x^5+1.
 *
 *   This function is able to calculate any CRC that uses 0x1021 as it
 *   polynomial and requires reflecting both the input and the output.
 *   The following checksums can, among others, be calculated by this
 *   function, depending on the value provided for the initial seed and the
 *   value the final calculated CRC is XORed with:
 *
 *   - CRC-16/CCITT, CRC-16/CCITT-TRUE, CRC-16/KERMIT
 *   https://reveng.sourceforge.io/crc-catalogue/16.htm#crc.cat.crc-16-kermit
 *   initial seed: 0x0000, xor output: 0x0000
 *
 ****************************************************************************/

uint16_t crc16ccitt(FAR const uint8_t *src, size_t len)
{
  return crc16ccittpart(src, len, 0u);
}
