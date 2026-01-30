/****************************************************************************
 * libs/libc/misc/lib_crc64.c
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
 *     printf("static const uint64_t g_crc64_tab[%zu] =\n", CRC64_TABLEN);
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
 * Name: crc64part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer.
 *
 ****************************************************************************/

#ifdef CONFIG_LIBC_CRC64_FAST
uint64_t crc64part(FAR const uint8_t *src, size_t len, uint64_t crc64val)
{
  size_t i;
  static const uint64_t g_crc64_tab[256] =
  {
    0x0000000000000000u, 0x42f0e1eba9ea3693u,
    0x85e1c3d753d46d26u, 0xc711223cfa3e5bb5u,
    0x493366450e42ecdfu, 0x0bc387aea7a8da4cu,
    0xccd2a5925d9681f9u, 0x8e224479f47cb76au,
    0x9266cc8a1c85d9beu, 0xd0962d61b56fef2du,
    0x17870f5d4f51b498u, 0x5577eeb6e6bb820bu,
    0xdb55aacf12c73561u, 0x99a54b24bb2d03f2u,
    0x5eb4691841135847u, 0x1c4488f3e8f96ed4u,
    0x663d78ff90e185efu, 0x24cd9914390bb37cu,
    0xe3dcbb28c335e8c9u, 0xa12c5ac36adfde5au,
    0x2f0e1eba9ea36930u, 0x6dfeff5137495fa3u,
    0xaaefdd6dcd770416u, 0xe81f3c86649d3285u,
    0xf45bb4758c645c51u, 0xb6ab559e258e6ac2u,
    0x71ba77a2dfb03177u, 0x334a9649765a07e4u,
    0xbd68d2308226b08eu, 0xff9833db2bcc861du,
    0x388911e7d1f2dda8u, 0x7a79f00c7818eb3bu,
    0xcc7af1ff21c30bdeu, 0x8e8a101488293d4du,
    0x499b3228721766f8u, 0x0b6bd3c3dbfd506bu,
    0x854997ba2f81e701u, 0xc7b97651866bd192u,
    0x00a8546d7c558a27u, 0x4258b586d5bfbcb4u,
    0x5e1c3d753d46d260u, 0x1cecdc9e94ace4f3u,
    0xdbfdfea26e92bf46u, 0x990d1f49c77889d5u,
    0x172f5b3033043ebfu, 0x55dfbadb9aee082cu,
    0x92ce98e760d05399u, 0xd03e790cc93a650au,
    0xaa478900b1228e31u, 0xe8b768eb18c8b8a2u,
    0x2fa64ad7e2f6e317u, 0x6d56ab3c4b1cd584u,
    0xe374ef45bf6062eeu, 0xa1840eae168a547du,
    0x66952c92ecb40fc8u, 0x2465cd79455e395bu,
    0x3821458aada7578fu, 0x7ad1a461044d611cu,
    0xbdc0865dfe733aa9u, 0xff3067b657990c3au,
    0x711223cfa3e5bb50u, 0x33e2c2240a0f8dc3u,
    0xf4f3e018f031d676u, 0xb60301f359dbe0e5u,
    0xda050215ea6c212fu, 0x98f5e3fe438617bcu,
    0x5fe4c1c2b9b84c09u, 0x1d14202910527a9au,
    0x93366450e42ecdf0u, 0xd1c685bb4dc4fb63u,
    0x16d7a787b7faa0d6u, 0x5427466c1e109645u,
    0x4863ce9ff6e9f891u, 0x0a932f745f03ce02u,
    0xcd820d48a53d95b7u, 0x8f72eca30cd7a324u,
    0x0150a8daf8ab144eu, 0x43a04931514122ddu,
    0x84b16b0dab7f7968u, 0xc6418ae602954ffbu,
    0xbc387aea7a8da4c0u, 0xfec89b01d3679253u,
    0x39d9b93d2959c9e6u, 0x7b2958d680b3ff75u,
    0xf50b1caf74cf481fu, 0xb7fbfd44dd257e8cu,
    0x70eadf78271b2539u, 0x321a3e938ef113aau,
    0x2e5eb66066087d7eu, 0x6cae578bcfe24bedu,
    0xabbf75b735dc1058u, 0xe94f945c9c3626cbu,
    0x676dd025684a91a1u, 0x259d31cec1a0a732u,
    0xe28c13f23b9efc87u, 0xa07cf2199274ca14u,
    0x167ff3eacbaf2af1u, 0x548f120162451c62u,
    0x939e303d987b47d7u, 0xd16ed1d631917144u,
    0x5f4c95afc5edc62eu, 0x1dbc74446c07f0bdu,
    0xdaad56789639ab08u, 0x985db7933fd39d9bu,
    0x84193f60d72af34fu, 0xc6e9de8b7ec0c5dcu,
    0x01f8fcb784fe9e69u, 0x43081d5c2d14a8fau,
    0xcd2a5925d9681f90u, 0x8fdab8ce70822903u,
    0x48cb9af28abc72b6u, 0x0a3b7b1923564425u,
    0x70428b155b4eaf1eu, 0x32b26afef2a4998du,
    0xf5a348c2089ac238u, 0xb753a929a170f4abu,
    0x3971ed50550c43c1u, 0x7b810cbbfce67552u,
    0xbc902e8706d82ee7u, 0xfe60cf6caf321874u,
    0xe224479f47cb76a0u, 0xa0d4a674ee214033u,
    0x67c58448141f1b86u, 0x253565a3bdf52d15u,
    0xab1721da49899a7fu, 0xe9e7c031e063acecu,
    0x2ef6e20d1a5df759u, 0x6c0603e6b3b7c1cau,
    0xf6fae5c07d3274cdu, 0xb40a042bd4d8425eu,
    0x731b26172ee619ebu, 0x31ebc7fc870c2f78u,
    0xbfc9838573709812u, 0xfd39626eda9aae81u,
    0x3a28405220a4f534u, 0x78d8a1b9894ec3a7u,
    0x649c294a61b7ad73u, 0x266cc8a1c85d9be0u,
    0xe17dea9d3263c055u, 0xa38d0b769b89f6c6u,
    0x2daf4f0f6ff541acu, 0x6f5faee4c61f773fu,
    0xa84e8cd83c212c8au, 0xeabe6d3395cb1a19u,
    0x90c79d3fedd3f122u, 0xd2377cd44439c7b1u,
    0x15265ee8be079c04u, 0x57d6bf0317edaa97u,
    0xd9f4fb7ae3911dfdu, 0x9b041a914a7b2b6eu,
    0x5c1538adb04570dbu, 0x1ee5d94619af4648u,
    0x02a151b5f156289cu, 0x4051b05e58bc1e0fu,
    0x87409262a28245bau, 0xc5b073890b687329u,
    0x4b9237f0ff14c443u, 0x0962d61b56fef2d0u,
    0xce73f427acc0a965u, 0x8c8315cc052a9ff6u,
    0x3a80143f5cf17f13u, 0x7870f5d4f51b4980u,
    0xbf61d7e80f251235u, 0xfd913603a6cf24a6u,
    0x73b3727a52b393ccu, 0x31439391fb59a55fu,
    0xf652b1ad0167feeau, 0xb4a25046a88dc879u,
    0xa8e6d8b54074a6adu, 0xea16395ee99e903eu,
    0x2d071b6213a0cb8bu, 0x6ff7fa89ba4afd18u,
    0xe1d5bef04e364a72u, 0xa3255f1be7dc7ce1u,
    0x64347d271de22754u, 0x26c49cccb40811c7u,
    0x5cbd6cc0cc10fafcu, 0x1e4d8d2b65facc6fu,
    0xd95caf179fc497dau, 0x9bac4efc362ea149u,
    0x158e0a85c2521623u, 0x577eeb6e6bb820b0u,
    0x906fc95291867b05u, 0xd29f28b9386c4d96u,
    0xcedba04ad0952342u, 0x8c2b41a1797f15d1u,
    0x4b3a639d83414e64u, 0x09ca82762aab78f7u,
    0x87e8c60fded7cf9du, 0xc51827e4773df90eu,
    0x020905d88d03a2bbu, 0x40f9e43324e99428u,
    0x2cffe7d5975e55e2u, 0x6e0f063e3eb46371u,
    0xa91e2402c48a38c4u, 0xebeec5e96d600e57u,
    0x65cc8190991cb93du, 0x273c607b30f68faeu,
    0xe02d4247cac8d41bu, 0xa2dda3ac6322e288u,
    0xbe992b5f8bdb8c5cu, 0xfc69cab42231bacfu,
    0x3b78e888d80fe17au, 0x7988096371e5d7e9u,
    0xf7aa4d1a85996083u, 0xb55aacf12c735610u,
    0x724b8ecdd64d0da5u, 0x30bb6f267fa73b36u,
    0x4ac29f2a07bfd00du, 0x08327ec1ae55e69eu,
    0xcf235cfd546bbd2bu, 0x8dd3bd16fd818bb8u,
    0x03f1f96f09fd3cd2u, 0x41011884a0170a41u,
    0x86103ab85a2951f4u, 0xc4e0db53f3c36767u,
    0xd8a453a01b3a09b3u, 0x9a54b24bb2d03f20u,
    0x5d45907748ee6495u, 0x1fb5719ce1045206u,
    0x919735e51578e56cu, 0xd367d40ebc92d3ffu,
    0x1476f63246ac884au, 0x568617d9ef46bed9u,
    0xe085162ab69d5e3cu, 0xa275f7c11f7768afu,
    0x6564d5fde549331au, 0x279434164ca30589u,
    0xa9b6706fb8dfb2e3u, 0xeb46918411358470u,
    0x2c57b3b8eb0bdfc5u, 0x6ea7525342e1e956u,
    0x72e3daa0aa188782u, 0x30133b4b03f2b111u,
    0xf7021977f9cceaa4u, 0xb5f2f89c5026dc37u,
    0x3bd0bce5a45a6b5du, 0x79205d0e0db05dceu,
    0xbe317f32f78e067bu, 0xfcc19ed95e6430e8u,
    0x86b86ed5267cdbd3u, 0xc4488f3e8f96ed40u,
    0x0359ad0275a8b6f5u, 0x41a94ce9dc428066u,
    0xcf8b0890283e370cu, 0x8d7be97b81d4019fu,
    0x4a6acb477bea5a2au, 0x089a2aacd2006cb9u,
    0x14dea25f3af9026du, 0x562e43b4931334feu,
    0x913f6188692d6f4bu, 0xd3cf8063c0c759d8u,
    0x5dedc41a34bbeeb2u, 0x1f1d25f19d51d821u,
    0xd80c07cd676f8394u, 0x9afce626ce85b507u
  };

  for (i = 0u; i < len; i++)
    {
      crc64val = g_crc64_tab[((crc64val >> 56) & 0xff) ^ src[i]] ^
                 (crc64val << 8);
    }

  return crc64val;
}
#else
uint64_t crc64part(FAR const uint8_t *src, size_t len, uint64_t crc64val)
{
  size_t i;
  size_t j;

  for (i = 0u; i < len; i++)
    {
      crc64val ^= (uint64_t)src[i] << 56;
      for (j = 0u; j < 8u; j++)
        {
          if ((crc64val & ((uint64_t)1 << 63)) != 0u)
            {
              crc64val = (crc64val << 1) ^ CRC64_POLY;
            }
          else
            {
              crc64val = crc64val << 1;
            }
        }
    }

  return crc64val;
}
#endif

/****************************************************************************
 * Name: crc64
 *
 * Description:
 *   Return a 64-bit CRC of the contents of the 'src' buffer, length 'len'.
 *
 ****************************************************************************/

uint64_t crc64(FAR const uint8_t *src, size_t len)
{
  return crc64part(src, len, CRC64_INIT) ^ CRC64_XOROUT;
}

#endif /* CONFIG_HAVE_LONG_LONG */
