/****************************************************************************
 * libs/libc/misc/lib_crc64emac.c
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

/* [AUTOSAR_SWS_Crc_00062] The CRC module shall implement the CRC64 routine
 * using the 0x42F0E1EBA9EA3693 polynomial.
 *
 * CRC64 parameters:
 * CRC result width: 64 bits
 * Polynomial: 42F0E1EBA9EA3693h
 * Initial value: FFFFFFFFFFFFFFFFh
 * Input data reflected: Yes
 * Result data reflected: Yes
 * XOR value: FFFFFFFFFFFFFFFFh
 *
 * [AUTOSAR_SWS_Crc_00063] The Crc_CalculateCRC64 function of the CRC module
 * shall provide the following CRC results:
 *
 * Data bytes (hexadecimal)    |   CRC
 * 00 00 00 00                 |   F4A586351E1B9F4B
 * F2 01 83                    |   319C27668164F1C6
 * 0F AA 00 55                 |   54C5D0F7667C1575
 * 00 FF 55 11                 |   A63822BE7E0704E6
 * 33 22 55 AA BB CC DD EE FF  |   701ECEB219A8E5D5
 * 92 6B 55                    |   5FAA96A9B59F3E4E
 * FF FF FF FF                 |   FFFFFFFF00000000
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/crc64.h>

#ifdef CONFIG_HAVE_LONG_LONG

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * CRC64 table generated with:
 *
 *   #include <inttypes.h>
 *   #include <stdint.h>
 *   #include <stdio.h>
 *
 *   #define CRC64_POLY   ((uint64_t)0x42f0e1eba9ea3693)
 *   #define CRC64_TABLEN ((size_t)256)
 *
 *   int main(void)
 *   {
 *     uint64_t crc64val;
 *     size_t i;
 *     size_t j;
 *
 *     printf("static const uint64_t crc64_tab[%zu] =\n", CRC64_TABLEN);
 *     printf("{\n  ");
 *
 *     for (i = 0; i < CRC64_TABLEN; i++)
 *       {
 *         crc64val = (uint64_t)i << 56;
 *         for (j = 0; j < 8; j++)
 *           {
 *             if ((crc64val & ((uint64_t)1 << 63)) != 0)
 *               {
 *                 crc64val = (crc64val << 1) ^ CRC64_POLY;
 *               }
 *             else
 *               {
 *                 crc64val = crc64val << 1;
 *               }
 *           }
 *
 *         printf("0x%016"PRIx64, crc64val);
 *         if (i + 1 >= CRC64_TABLEN)
 *           {
 *             printf("\n");
 *           }
 *         else if (i % 2 == 1)
 *           {
 *             printf(",\n  ");
 *           }
 *         else
 *           {
 *             printf(", ");
 *           }
 *       }
 *
 *     printf("};\n");
 *     return 0;
 *   }
 *
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: crc64emac_part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer
 *   using the 0x42F0E1EBA9EA3693 polynomial.
 *
 ****************************************************************************/

uint64_t crc64emac_part(FAR const uint8_t *src,
                        size_t len, uint64_t crc64val)
{
  size_t i;
  static const uint64_t g_crc64_tab[256] =
  {
    0x0000000000000000ull,   0xb32e4cbe03a75f6full,
    0xf4843657a840a05bull,   0x47aa7ae9abe7ff34ull,
    0x7bd0c384ff8f5e33ull,   0xc8fe8f3afc28015cull,
    0x8f54f5d357cffe68ull,   0x3c7ab96d5468a107ull,
    0xf7a18709ff1ebc66ull,   0x448fcbb7fcb9e309ull,
    0x0325b15e575e1c3dull,   0xb00bfde054f94352ull,
    0x8c71448d0091e255ull,   0x3f5f08330336bd3aull,
    0x78f572daa8d1420eull,   0xcbdb3e64ab761d61ull,
    0x7d9ba13851336649ull,   0xceb5ed8652943926ull,
    0x891f976ff973c612ull,   0x3a31dbd1fad4997dull,
    0x064b62bcaebc387aull,   0xb5652e02ad1b6715ull,
    0xf2cf54eb06fc9821ull,   0x41e11855055bc74eull,
    0x8a3a2631ae2dda2full,   0x39146a8fad8a8540ull,
    0x7ebe1066066d7a74ull,   0xcd905cd805ca251bull,
    0xf1eae5b551a2841cull,   0x42c4a90b5205db73ull,
    0x056ed3e2f9e22447ull,   0xb6409f5cfa457b28ull,
    0xfb374270a266cc92ull,   0x48190ecea1c193fdull,
    0x0fb374270a266cc9ull,   0xbc9d3899098133a6ull,
    0x80e781f45de992a1ull,   0x33c9cd4a5e4ecdceull,
    0x7463b7a3f5a932faull,   0xc74dfb1df60e6d95ull,
    0x0c96c5795d7870f4ull,   0xbfb889c75edf2f9bull,
    0xf812f32ef538d0afull,   0x4b3cbf90f69f8fc0ull,
    0x774606fda2f72ec7ull,   0xc4684a43a15071a8ull,
    0x83c230aa0ab78e9cull,   0x30ec7c140910d1f3ull,
    0x86ace348f355aadbull,   0x3582aff6f0f2f5b4ull,
    0x7228d51f5b150a80ull,   0xc10699a158b255efull,
    0xfd7c20cc0cdaf4e8ull,   0x4e526c720f7dab87ull,
    0x09f8169ba49a54b3ull,   0xbad65a25a73d0bdcull,
    0x710d64410c4b16bdull,   0xc22328ff0fec49d2ull,
    0x85895216a40bb6e6ull,   0x36a71ea8a7ace989ull,
    0x0adda7c5f3c4488eull,   0xb9f3eb7bf06317e1ull,
    0xfe5991925b84e8d5ull,   0x4d77dd2c5823b7baull,
    0x64b62bcaebc387a1ull,   0xd7986774e864d8ceull,
    0x90321d9d438327faull,   0x231c512340247895ull,
    0x1f66e84e144cd992ull,   0xac48a4f017eb86fdull,
    0xebe2de19bc0c79c9ull,   0x58cc92a7bfab26a6ull,
    0x9317acc314dd3bc7ull,   0x2039e07d177a64a8ull,
    0x67939a94bc9d9b9cull,   0xd4bdd62abf3ac4f3ull,
    0xe8c76f47eb5265f4ull,   0x5be923f9e8f53a9bull,
    0x1c4359104312c5afull,   0xaf6d15ae40b59ac0ull,
    0x192d8af2baf0e1e8ull,   0xaa03c64cb957be87ull,
    0xeda9bca512b041b3ull,   0x5e87f01b11171edcull,
    0x62fd4976457fbfdbull,   0xd1d305c846d8e0b4ull,
    0x96797f21ed3f1f80ull,   0x2557339fee9840efull,
    0xee8c0dfb45ee5d8eull,   0x5da24145464902e1ull,
    0x1a083bacedaefdd5ull,   0xa9267712ee09a2baull,
    0x955cce7fba6103bdull,   0x267282c1b9c65cd2ull,
    0x61d8f8281221a3e6ull,   0xd2f6b4961186fc89ull,
    0x9f8169ba49a54b33ull,   0x2caf25044a02145cull,
    0x6b055fede1e5eb68ull,   0xd82b1353e242b407ull,
    0xe451aa3eb62a1500ull,   0x577fe680b58d4a6full,
    0x10d59c691e6ab55bull,   0xa3fbd0d71dcdea34ull,
    0x6820eeb3b6bbf755ull,   0xdb0ea20db51ca83aull,
    0x9ca4d8e41efb570eull,   0x2f8a945a1d5c0861ull,
    0x13f02d374934a966ull,   0xa0de61894a93f609ull,
    0xe7741b60e174093dull,   0x545a57dee2d35652ull,
    0xe21ac88218962d7aull,   0x5134843c1b317215ull,
    0x169efed5b0d68d21ull,   0xa5b0b26bb371d24eull,
    0x99ca0b06e7197349ull,   0x2ae447b8e4be2c26ull,
    0x6d4e3d514f59d312ull,   0xde6071ef4cfe8c7dull,
    0x15bb4f8be788911cull,   0xa6950335e42fce73ull,
    0xe13f79dc4fc83147ull,   0x521135624c6f6e28ull,
    0x6e6b8c0f1807cf2full,   0xdd45c0b11ba09040ull,
    0x9aefba58b0476f74ull,   0x29c1f6e6b3e0301bull,
    0xc96c5795d7870f42ull,   0x7a421b2bd420502dull,
    0x3de861c27fc7af19ull,   0x8ec62d7c7c60f076ull,
    0xb2bc941128085171ull,   0x0192d8af2baf0e1eull,
    0x4638a2468048f12aull,   0xf516eef883efae45ull,
    0x3ecdd09c2899b324ull,   0x8de39c222b3eec4bull,
    0xca49e6cb80d9137full,   0x7967aa75837e4c10ull,
    0x451d1318d716ed17ull,   0xf6335fa6d4b1b278ull,
    0xb199254f7f564d4cull,   0x02b769f17cf11223ull,
    0xb4f7f6ad86b4690bull,   0x07d9ba1385133664ull,
    0x4073c0fa2ef4c950ull,   0xf35d8c442d53963full,
    0xcf273529793b3738ull,   0x7c0979977a9c6857ull,
    0x3ba3037ed17b9763ull,   0x888d4fc0d2dcc80cull,
    0x435671a479aad56dull,   0xf0783d1a7a0d8a02ull,
    0xb7d247f3d1ea7536ull,   0x04fc0b4dd24d2a59ull,
    0x3886b22086258b5eull,   0x8ba8fe9e8582d431ull,
    0xcc0284772e652b05ull,   0x7f2cc8c92dc2746aull,
    0x325b15e575e1c3d0ull,   0x8175595b76469cbfull,
    0xc6df23b2dda1638bull,   0x75f16f0cde063ce4ull,
    0x498bd6618a6e9de3ull,   0xfaa59adf89c9c28cull,
    0xbd0fe036222e3db8ull,   0x0e21ac88218962d7ull,
    0xc5fa92ec8aff7fb6ull,   0x76d4de52895820d9ull,
    0x317ea4bb22bfdfedull,   0x8250e80521188082ull,
    0xbe2a516875702185ull,   0x0d041dd676d77eeaull,
    0x4aae673fdd3081deull,   0xf9802b81de97deb1ull,
    0x4fc0b4dd24d2a599ull,   0xfceef8632775faf6ull,
    0xbb44828a8c9205c2ull,   0x086ace348f355aadull,
    0x34107759db5dfbaaull,   0x873e3be7d8faa4c5ull,
    0xc094410e731d5bf1ull,   0x73ba0db070ba049eull,
    0xb86133d4dbcc19ffull,   0x0b4f7f6ad86b4690ull,
    0x4ce50583738cb9a4ull,   0xffcb493d702be6cbull,
    0xc3b1f050244347ccull,   0x709fbcee27e418a3ull,
    0x3735c6078c03e797ull,   0x841b8ab98fa4b8f8ull,
    0xadda7c5f3c4488e3ull,   0x1ef430e13fe3d78cull,
    0x595e4a08940428b8ull,   0xea7006b697a377d7ull,
    0xd60abfdbc3cbd6d0ull,   0x6524f365c06c89bfull,
    0x228e898c6b8b768bull,   0x91a0c532682c29e4ull,
    0x5a7bfb56c35a3485ull,   0xe955b7e8c0fd6beaull,
    0xaeffcd016b1a94deull,   0x1dd181bf68bdcbb1ull,
    0x21ab38d23cd56ab6ull,   0x9285746c3f7235d9ull,
    0xd52f0e859495caedull,   0x6601423b97329582ull,
    0xd041dd676d77eeaaull,   0x636f91d96ed0b1c5ull,
    0x24c5eb30c5374ef1ull,   0x97eba78ec690119eull,
    0xab911ee392f8b099ull,   0x18bf525d915feff6ull,
    0x5f1528b43ab810c2ull,   0xec3b640a391f4fadull,
    0x27e05a6e926952ccull,   0x94ce16d091ce0da3ull,
    0xd3646c393a29f297ull,   0x604a2087398eadf8ull,
    0x5c3099ea6de60cffull,   0xef1ed5546e415390ull,
    0xa8b4afbdc5a6aca4ull,   0x1b9ae303c601f3cbull,
    0x56ed3e2f9e224471ull,   0xe5c372919d851b1eull,
    0xa26908783662e42aull,   0x114744c635c5bb45ull,
    0x2d3dfdab61ad1a42ull,   0x9e13b115620a452dull,
    0xd9b9cbfcc9edba19ull,   0x6a978742ca4ae576ull,
    0xa14cb926613cf817ull,   0x1262f598629ba778ull,
    0x55c88f71c97c584cull,   0xe6e6c3cfcadb0723ull,
    0xda9c7aa29eb3a624ull,   0x69b2361c9d14f94bull,
    0x2e184cf536f3067full,   0x9d36004b35545910ull,
    0x2b769f17cf112238ull,   0x9858d3a9ccb67d57ull,
    0xdff2a94067518263ull,   0x6cdce5fe64f6dd0cull,
    0x50a65c93309e7c0bull,   0xe388102d33392364ull,
    0xa4226ac498dedc50ull,   0x170c267a9b79833full,
    0xdcd7181e300f9e5eull,   0x6ff954a033a8c131ull,
    0x28532e49984f3e05ull,   0x9b7d62f79be8616aull,
    0xa707db9acf80c06dull,   0x14299724cc279f02ull,
    0x5383edcd67c06036ull,   0xe0ada17364673f59ull
  };

  crc64val ^= 0xffffffffffffffffull;
  for (i = 0u; i < len; i++)
    {
      crc64val = g_crc64_tab[(uint8_t)(crc64val & 0xffull) ^ src[i]] ^
                 (crc64val >> 8);
    }

  return crc64val ^ 0xffffffffffffffffull;
}

/****************************************************************************
 * Name: crc64emac
 *
 * Description:
 *   Return a 64-bit CRC of the contents of the 'src' buffer, length 'len'
 *   using the 0x42F0E1EBA9EA3693 polynomial.
 *
 ****************************************************************************/

uint64_t crc64emac(FAR const uint8_t *src, size_t len)
{
  return crc64emac_part(src, len, 0x0000000000000000ull);
}

#endif /* CONFIG_HAVE_LONG_LONG */
