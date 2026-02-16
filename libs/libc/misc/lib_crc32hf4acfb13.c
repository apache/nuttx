/****************************************************************************
 * libs/libc/misc/lib_crc32hf4acfb13.c
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

/* [AUTOSAR_SWS_Crc_00056] The CRC module shall implement the CRC32 routine
 * using the 0xF4’AC’FB’13 polynomial.
 *
 * CRC32 parameters:
 * CRC result width: 32 bits
 * Polynomial: F4ACFB13h
 * Initial value: FFFFFFFFh
 * Input data reflected: Yes
 * Result data reflected: Yes
 * XOR value: FFFFFFFFh
 *
 * [AUTOSAR_SWS_Crc_00057] The Crc_CalculateCRC32P4 function of the CRC
 * module shall provide the following CRC results:
 *
 * Data bytes (hexadecimal)    |   CRC
 * 00 00 00 00                 |   6FB32240
 * F2 01 83                    |   4F721A25
 * 0F AA 00 55                 |   20662DF8
 * 00 FF 55 11                 |   9BD7996E
 * 33 22 55 AA BB CC DD EE FF  |   A65A343D
 * 92 6B 55                    |   EE688A78
 * FF FF FF FF                 |   FFFFFFFF
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/crc32.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * CRC32 table generated with:
 *
 * #include <stdint.h>
 *
 * void generate_crc32_table(uint32_t poly, uint32_t *table)
 * {
 *   for (int i = 0; i < 256; i++)
 *     {
 *       uint32_t crc = i;
 *       for (uint8_t bit = 0; bit < 8; bit++)
 *         {
 *           if (crc & 1)
 *             {
 *               crc = (crc >> 1) ^ poly;
 *             }
 *           else
 *             {
 *               crc = crc >> 1;
 *             }
 *         }
 *       table[i] = crc;
 *     }
 * }
 *
 * int main()
 * {
 *   uint32_t crc32_table[256];
 *   const uint32_t poly = 0xF4ACFB13;
 *
 *   generate_crc32_table(poly, crc32_table);
 *
 *   printf("CRC32 Static Table (Poly 0x%08X):\n", poly);
 *   for (int i = 0; i < 256; i++)
 *     {
 *       if (i % 4 == 0) printf("\n");
 *       printf("0x%08X, ", crc32_table[i]);
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
 * Name: crc32hf4acfb13_part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer
 *   using the 0xF4ACFB13 polynomial.
 *
 ****************************************************************************/

uint32_t crc32hf4acfb13_part(FAR const uint8_t *src,
                             size_t len, uint32_t crc32val)
{
  size_t i;
  static const uint32_t g_crc32_tab[256] =
  {
    0x00000000u, 0x30850ff5u,  0x610a1feau, 0x518f101fu,
    0xc2143fd4u, 0xf2913021u,  0xa31e203eu, 0x939b2fcbu,
    0x159615f7u, 0x25131a02u,  0x749c0a1du, 0x441905e8u,
    0xd7822a23u, 0xe70725d6u,  0xb68835c9u, 0x860d3a3cu,
    0x2b2c2beeu, 0x1ba9241bu,  0x4a263404u, 0x7aa33bf1u,
    0xe938143au, 0xd9bd1bcfu,  0x88320bd0u, 0xb8b70425u,
    0x3eba3e19u, 0x0e3f31ecu,  0x5fb021f3u, 0x6f352e06u,
    0xfcae01cdu, 0xcc2b0e38u,  0x9da41e27u, 0xad2111d2u,
    0x565857dcu, 0x66dd5829u,  0x37524836u, 0x07d747c3u,
    0x944c6808u, 0xa4c967fdu,  0xf54677e2u, 0xc5c37817u,
    0x43ce422bu, 0x734b4ddeu,  0x22c45dc1u, 0x12415234u,
    0x81da7dffu, 0xb15f720au,  0xe0d06215u, 0xd0556de0u,
    0x7d747c32u, 0x4df173c7u,  0x1c7e63d8u, 0x2cfb6c2du,
    0xbf6043e6u, 0x8fe54c13u,  0xde6a5c0cu, 0xeeef53f9u,
    0x68e269c5u, 0x58676630u,  0x09e8762fu, 0x396d79dau,
    0xaaf65611u, 0x9a7359e4u,  0xcbfc49fbu, 0xfb79460eu,
    0xacb0afb8u, 0x9c35a04du,  0xcdbab052u, 0xfd3fbfa7u,
    0x6ea4906cu, 0x5e219f99u,  0x0fae8f86u, 0x3f2b8073u,
    0xb926ba4fu, 0x89a3b5bau,  0xd82ca5a5u, 0xe8a9aa50u,
    0x7b32859bu, 0x4bb78a6eu,  0x1a389a71u, 0x2abd9584u,
    0x879c8456u, 0xb7198ba3u,  0xe6969bbcu, 0xd6139449u,
    0x4588bb82u, 0x750db477u,  0x2482a468u, 0x1407ab9du,
    0x920a91a1u, 0xa28f9e54u,  0xf3008e4bu, 0xc38581beu,
    0x501eae75u, 0x609ba180u,  0x3114b19fu, 0x0191be6au,
    0xfae8f864u, 0xca6df791u,  0x9be2e78eu, 0xab67e87bu,
    0x38fcc7b0u, 0x0879c845u,  0x59f6d85au, 0x6973d7afu,
    0xef7eed93u, 0xdffbe266u,  0x8e74f279u, 0xbef1fd8cu,
    0x2d6ad247u, 0x1defddb2u,  0x4c60cdadu, 0x7ce5c258u,
    0xd1c4d38au, 0xe141dc7fu,  0xb0cecc60u, 0x804bc395u,
    0x13d0ec5eu, 0x2355e3abu,  0x72daf3b4u, 0x425ffc41u,
    0xc452c67du, 0xf4d7c988u,  0xa558d997u, 0x95ddd662u,
    0x0646f9a9u, 0x36c3f65cu,  0x674ce643u, 0x57c9e9b6u,
    0xc8df352fu, 0xf85a3adau,  0xa9d52ac5u, 0x99502530u,
    0x0acb0afbu, 0x3a4e050eu,  0x6bc11511u, 0x5b441ae4u,
    0xdd4920d8u, 0xedcc2f2du,  0xbc433f32u, 0x8cc630c7u,
    0x1f5d1f0cu, 0x2fd810f9u,  0x7e5700e6u, 0x4ed20f13u,
    0xe3f31ec1u, 0xd3761134u,  0x82f9012bu, 0xb27c0edeu,
    0x21e72115u, 0x11622ee0u,  0x40ed3effu, 0x7068310au,
    0xf6650b36u, 0xc6e004c3u,  0x976f14dcu, 0xa7ea1b29u,
    0x347134e2u, 0x04f43b17u,  0x557b2b08u, 0x65fe24fdu,
    0x9e8762f3u, 0xae026d06u,  0xff8d7d19u, 0xcf0872ecu,
    0x5c935d27u, 0x6c1652d2u,  0x3d9942cdu, 0x0d1c4d38u,
    0x8b117704u, 0xbb9478f1u,  0xea1b68eeu, 0xda9e671bu,
    0x490548d0u, 0x79804725u,  0x280f573au, 0x188a58cfu,
    0xb5ab491du, 0x852e46e8u,  0xd4a156f7u, 0xe4245902u,
    0x77bf76c9u, 0x473a793cu,  0x16b56923u, 0x263066d6u,
    0xa03d5ceau, 0x90b8531fu,  0xc1374300u, 0xf1b24cf5u,
    0x6229633eu, 0x52ac6ccbu,  0x03237cd4u, 0x33a67321u,
    0x646f9a97u, 0x54ea9562u,  0x0565857du, 0x35e08a88u,
    0xa67ba543u, 0x96feaab6u,  0xc771baa9u, 0xf7f4b55cu,
    0x71f98f60u, 0x417c8095u,  0x10f3908au, 0x20769f7fu,
    0xb3edb0b4u, 0x8368bf41u,  0xd2e7af5eu, 0xe262a0abu,
    0x4f43b179u, 0x7fc6be8cu,  0x2e49ae93u, 0x1ecca166u,
    0x8d578eadu, 0xbdd28158u,  0xec5d9147u, 0xdcd89eb2u,
    0x5ad5a48eu, 0x6a50ab7bu,  0x3bdfbb64u, 0x0b5ab491u,
    0x98c19b5au, 0xa84494afu,  0xf9cb84b0u, 0xc94e8b45u,
    0x3237cd4bu, 0x02b2c2beu,  0x533dd2a1u, 0x63b8dd54u,
    0xf023f29fu, 0xc0a6fd6au,  0x9129ed75u, 0xa1ace280u,
    0x27a1d8bcu, 0x1724d749u,  0x46abc756u, 0x762ec8a3u,
    0xe5b5e768u, 0xd530e89du,  0x84bff882u, 0xb43af777u,
    0x191be6a5u, 0x299ee950u,  0x7811f94fu, 0x4894f6bau,
    0xdb0fd971u, 0xeb8ad684u,  0xba05c69bu, 0x8a80c96eu,
    0x0c8df352u, 0x3c08fca7u,  0x6d87ecb8u, 0x5d02e34du,
    0xce99cc86u, 0xfe1cc373u,  0xaf93d36cu, 0x9f16dc99u
  };

  crc32val ^= 0xffffffffu;
  for (i = 0u; i < len; i++)
    {
      crc32val = g_crc32_tab[(uint8_t)(crc32val & 0xffu) ^ src[i]] ^
                 (crc32val >> 8);
    }

  return crc32val ^ 0xffffffffu;
}

/****************************************************************************
 * Name: crc32hf4acfb13
 *
 * Description:
 *   Return a 32-bit CRC of the contents of the 'src' buffer, length 'len'
 *   using the 0xF4ACFB13 polynomial.
 *
 ****************************************************************************/

uint32_t crc32hf4acfb13(FAR const uint8_t *src, size_t len)
{
  return crc32hf4acfb13_part(src, len, 0x00000000u);
}
