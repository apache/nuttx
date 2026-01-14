/****************************************************************************
 * net/netdev/netdev_notify_recvcpu.c
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

#include <assert.h>
#include <debug.h>

#include "netdev/netdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PACKET_BYTE_SIZE        36
#define RANDOM_KEY_SIZE         40

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum hashcal_algo_e
{
    HASHCAL_ALGO_TOEPLITZ,
    HASHCAL_ALGO_XOR,
    HASHCAL_ALGO_CRC32
} hashcal_algo_e;

typedef enum hashcal_type_e
{
  HASHCAL_TYPE_4TUPLE,
  HASHCAL_TYPE_2TUPLE,
} hashcal_type_e;

typedef union toeplitz_key_u
{
  uint8_t u8[RANDOM_KEY_SIZE];
  uint32_t u32[10];
} toeplitz_key_u;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const toeplitz_key_u g_random_key =
{
  .u8 =
  {
    0x6d, 0x5a, 0x56, 0xda, 0x25, 0x5b, 0x0e, 0xc2, 0x41, 0x67,
    0x25, 0x3d, 0x43, 0xa3, 0x8f, 0xb0, 0xd0, 0xca, 0x2b, 0xcb,
    0xae, 0x7b, 0x30, 0xb4, 0x77, 0xcb, 0x2d, 0xa3, 0x80, 0x30,
    0xf2, 0x0c, 0x6a, 0x42, 0xb7, 0x3b, 0xbe, 0xac, 0x01, 0xfa,
  }
};

static const uint32_t g_crc32c_table[256] =
{
  0x00000000, 0xf26b8303, 0xe13b70f7, 0x1350f3f4, 0xc79a971f, 0x35f1141c,
  0x26a1e7e8, 0xd4ca64eb, 0x8ad958cf, 0x78b2dbcc, 0x6be22838, 0x9989ab3b,
  0x4d43cfd0, 0xbf284cd3, 0xac78bf27, 0x5e133c24, 0x105ec76f, 0xe235446c,
  0xf165b798, 0x030e349b, 0xd7c45070, 0x25afd373, 0x36ff2087, 0xc494a384,
  0x9a879fa0, 0x68ec1ca3, 0x7bbcef57, 0x89d76c54, 0x5d1d08bf, 0xaf768bbc,
  0xbc267848, 0x4e4dfb4b, 0x20bd8ede, 0xd2d60ddd, 0xc186fe29, 0x33ed7d2a,
  0xe72719c1, 0x154c9ac2, 0x061c6936, 0xf477ea35, 0xaa64d611, 0x580f5512,
  0x4b5fa6e6, 0xb93425e5, 0x6dfe410e, 0x9f95c20d, 0x8cc531f9, 0x7eaeb2fa,
  0x30e349b1, 0xc288cab2, 0xd1d83946, 0x23b3ba45, 0xf779deae, 0x05125dad,
  0x1642ae59, 0xe4292d5a, 0xba3a117e, 0x4851927d, 0x5b016189, 0xa96ae28a,
  0x7da08661, 0x8fcb0562, 0x9c9bf696, 0x6ef07595, 0x417b1dbc, 0xb3109ebf,
  0xa0406d4b, 0x522bee48, 0x86e18aa3, 0x748a09a0, 0x67dafa54, 0x95b17957,
  0xcba24573, 0x39c9c670, 0x2a993584, 0xd8f2b687, 0x0c38d26c, 0xfe53516f,
  0xed03a29b, 0x1f682198, 0x5125dad3, 0xa34e59d0, 0xb01eaa24, 0x42752927,
  0x96bf4dcc, 0x64d4cecf, 0x77843d3b, 0x85efbe38, 0xdbfc821c, 0x2997011f,
  0x3ac7f2eb, 0xc8ac71e8, 0x1c661503, 0xee0d9600, 0xfd5d65f4, 0x0f36e6f7,
  0x61c69362, 0x93ad1061, 0x80fde395, 0x72966096, 0xa65c047d, 0x5437877e,
  0x4767748a, 0xb50cf789, 0xeb1fcbad, 0x197448ae, 0x0a24bb5a, 0xf84f3859,
  0x2c855cb2, 0xdeeedfb1, 0xcdb52c45, 0x3fd5af46, 0x7198540d, 0x83f3d70e,
  0x90a324fa, 0x62c8a7f9, 0xb602c312, 0x44694011, 0x5739b3e5, 0xa55230e6,
  0xfb410cc2, 0x092a8fc1, 0x1a7a7c35, 0xe811ff36, 0x3cdb9bdd, 0xceb018de,
  0xdde0eb2a, 0x2f8b6829, 0x82f63b78, 0x709db87b, 0x63cd4b8f, 0x91a6c88c,
  0x456cac67, 0xb7072f64, 0xa457dc90, 0x563c5f93, 0x082f63b7, 0xfa44e0b4,
  0xe9141340, 0x1b7f9043, 0xcfb5f4a8, 0x3dde77ab, 0x2e8e845f, 0xdce5075c,
  0x92a8fc17, 0x60c37f14, 0x73938ce0, 0x81f80fe3, 0x55326b08, 0xa759e80b,
  0xb4091bff, 0x466298fc, 0x1871a4d8, 0xea1a27db, 0xf94ad42f, 0x0b21572c,
  0xdfeb33c7, 0x2d80b0c4, 0x3ed04330, 0xccbbc033, 0xa24bb5a6, 0x502036a5,
  0x4370c551, 0xb11b4652, 0x65d122b9, 0x97baa1ba, 0x84ea524e, 0x7681d14d,
  0x2892ed69, 0xdaf96e6a, 0xc9a99d9e, 0x3bc21e9d, 0xef087a76, 0x1d63f975,
  0x0e330a81, 0xfc588982, 0xb21572c9, 0x407ef1ca, 0x532e023e, 0xa145813d,
  0x758fe5d6, 0x87e466d5, 0x94b49521, 0x66df1622, 0x38cc2a06, 0xcaa7a905,
  0xd9f75af1, 0x2b9cd9f2, 0xff56bd19, 0x0d3d3e1a, 0x1e6dcdee, 0xec064eed,
  0xc38d26c4, 0x31e6a5c7, 0x22b65633, 0xd0ddd530, 0x0417b1db, 0xf67c32d8,
  0xe52cc12c, 0x1747422f, 0x49547e0b, 0xbb3ffd08, 0xa86f0efc, 0x5a048dff,
  0x8ecee914, 0x7ca56a17, 0x6ff599e3, 0x9d9e1ae0, 0xd3d3e1ab, 0x21b862a8,
  0x32e8915c, 0xc083125f, 0x144976b4, 0xe622f5b7, 0xf5720643, 0x07198540,
  0x590ab964, 0xab613a67, 0xb831c993, 0x4a5a4a90, 0x9e902e7b, 0x6cfbad78,
  0x7fab5e8c, 0x8dc0dd8f, 0xe330a81a, 0x115b2b19, 0x020bd8ed, 0xf0605bee,
  0x24aa3f05, 0xd6c1bc06, 0xc5914ff2, 0x37faccf1, 0x69e9f0d5, 0x9b8273d6,
  0x88d28022, 0x7ab90321, 0xae7367ca, 0x5c18e4c9, 0x4f48173d, 0xbd23943e,
  0xf36e6f75, 0x0105ec76, 0x12551f82, 0xe03e9c81, 0x34f4f86a, 0xc69f7b69,
  0xd5cf889d, 0x27a40b9e, 0x79b737ba, 0x8bdcb4b9, 0x988c474d, 0x6ae7c44e,
  0xbe2da0a5, 0x4c4623a6, 0x5f16d052, 0xad7d5351
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: compute_toeplitz_hash
 *
 * Description:
 *   HASHCAL_ALGO_TOEPLITZ is a hash algorithm that uses Toeplitz matrix to
 *   calculate hash values. Toeplitz matrix is a special matrix where each
 *   diagonal has the same elements.
 *
 * Input Parameters:
 *   packet - The packet data
 *   len    - The packet length in bytes
 *
 * Returned Value:
 *   The hash value with toeplitz matrix calculation
 *
 ****************************************************************************/

static uint32_t compute_toeplitz_hash(FAR const uint8_t *packet,
                                      uint32_t len)
{
  uint32_t key = (g_random_key.u8[0] << 24) | (g_random_key.u8[1] << 16) |
                 (g_random_key.u8[2] << 8) | g_random_key.u8[3];
  uint32_t ret = 0;
  int i;
  int j;

  for (i = 0; i < len; i++)
    {
      for (j = 0; j < 8; j++)
        {
          if (packet[i] & (1 << (7 - j)))
            {
              ret ^= key;
            }

          key <<= 1;
          if ((i + 4) < RANDOM_KEY_SIZE &&
              (g_random_key.u8[i + 4] & (1 << (7 - j))))
            {
              key |= 1;
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: compute_xor_hash
 *
 * Description:
 *   Calculate hash value using HASHCAL_ALGO_XOR type.
 *
 * Input Parameters:
 *   packet - The packet data
 *   len    - The packet length in bytes
 *
 * Returned Value:
 *   The hash value with xor calculation
 *
 ****************************************************************************/

static uint32_t compute_xor_hash(FAR const uint8_t *packet, uint32_t len)
{
  uint32_t val  = 0;
  uint32_t iter = 0;

  len = len >> 2;

  while (iter < len)
    {
      val ^= packet[iter];
      iter++;
    }

  return val;
}

/****************************************************************************
 * Name: compute_crc32c_hash
 *
 * Description:
 *   Calculate hash value using HASHCAL_ALGO_CRC32. crc32c polynomial
 *   coefficients: x^32 + x^28 + x^27 + x^26 + x^25 + x^23 + x^22 +
 *   x^20 + x^19 + x^18 + x^14 + x^13 + x^11 + x^10 + x^9 + x^8 + x^6 + 1
 *   crc32c verilog code is generated from:
 *   https://bues.ch/cms/hacking/crcgen.html
 *   we can verify the result online:
 *   https://crccalc.com/?crc=24002&method=crc32&datatype=ascii&outtype=dec
 *
 * Input Parameters:
 *   packet    - The packet data
 *   len       - The packet length in bytes
 *
 * Returned Value:
 *  The hash value with crc32c calculation
 *
 ****************************************************************************/

static uint32_t compute_crc32c_hash(FAR uint8_t *packet, uint32_t len)
{
  uint32_t crc32 = 0xffffffff;
  uint32_t i = 0;

  for (i = 0; i != len; ++i)
    {
      uint8_t b = packet[i];
      uint8_t index = (crc32 ^ b) & 0xff;
      crc32 = ((crc32 >> 8) & 0xffffff) ^ g_crc32c_table[index];
    }

  return crc32 ^ 0xffffffff;
}

/****************************************************************************
 * Name: create_binary
 *
 * Description:
 *   Create a binary representation of the specified data structure and
 *   return a length of binary data
 *
 * Input Parameters:
 *   hash_type - The hash type
 *   domain    - The layer 3 protocol, PF_INET/PF_INET6
 *   packet    - The packet data
 *   src_addr  - The source address
 *   src_port  - The source port
 *   dst_addr  - The destination address
 *   dst_port  - The destination port
 *
 * Returned Value:
 *  The length of the packet need to be calculated
 *
 ****************************************************************************/

static uint32_t create_binary(hashcal_type_e hash_type, uint8_t domain,
                              FAR const uint32_t *src_addr,
                              uint16_t src_port,
                              FAR const uint32_t *dst_addr,
                              uint16_t dst_port,
                              FAR uint8_t *packet)
{
  uint32_t iter = 0;

  if (domain == PF_INET)
    {
      packet[iter++] = (*src_addr & 0xff000000) >> 24;
      packet[iter++] = (*src_addr & 0x00ff0000) >> 16;
      packet[iter++] = (*src_addr & 0x0000ff00) >> 8;
      packet[iter++] = (*src_addr & 0x000000ff);

      packet[iter++] = (*dst_addr & 0xff000000) >> 24;
      packet[iter++] = (*dst_addr & 0x00ff0000) >> 16;
      packet[iter++] = (*dst_addr & 0x0000ff00) >> 8;
      packet[iter++] = (*dst_addr & 0x000000ff);

      if (hash_type == HASHCAL_TYPE_4TUPLE)
        {
          packet[iter++] = (src_port & 0xff00) >> 8;
          packet[iter++] = (src_port & 0x00ff);
          packet[iter++] = (dst_port & 0xff00) >> 8;
          packet[iter++] = (dst_port & 0x00ff);
        }
    }
  else
    {
      uint32_t tmp;
      int i;
      int j;

      for (i = 0, j = 0; i < 16; i++)
        {
          if (0 < i && i % 4 == 0)
            {
              j++;
            }

          tmp = src_addr[j];
          packet[i] = tmp >> (24 - (8 * (i % 4)));
        }

      for (i = 16, j = 0; i < 32; i++)
        {
          if (16 < i && i % 4 == 0)
            {
              j++;
            }

          tmp = dst_addr[j];
          packet[i] = tmp >> (24 - (8 * (i % 4)));
        }

      iter = 32;
      if (hash_type == HASHCAL_TYPE_4TUPLE)
        {
          packet[iter++] = (src_port & 0xff00) >> 8;
          packet[iter++] = (src_port & 0x00ff);
          packet[iter++] = (dst_port & 0xff00) >> 8;
          packet[iter++] = (dst_port & 0x00ff);
        }
    }

  return iter;
}

/****************************************************************************
 * Name: compute_hash
 *
 * Description:
 *   Calculate hash value using the specified hash algorithm.
 *
 * Input Parameters:
 *   hash_algo - The hash algorithm
 *   hash_type - The hash type
 *   domain    - The layer 3 protocol, PF_INET/PF_INET6
 *   src_addr  - The source address
 *   src_port  - The source port
 *   dst_addr  - The destination address
 *   dst_port  - The destination port
 *
 * Returned Value:
 *  The hash value
 *
 ****************************************************************************/

static uint32_t compute_hash(hashcal_algo_e hash_algo,
                             hashcal_type_e hash_type, uint8_t domain,
                             FAR const uint32_t *src_addr, uint16_t src_port,
                             FAR const uint32_t *dst_addr, uint16_t dst_port)
{
  uint8_t packet[PACKET_BYTE_SIZE];
  uint32_t hash_val;
  uint32_t cal_len;

  memset(packet, 0, PACKET_BYTE_SIZE);

  cal_len = create_binary(hash_type, domain, src_addr,
                          src_port, dst_addr, dst_port, packet);
  DEBUGASSERT(cal_len % 4 == 0);

  switch (hash_algo)
    {
      case HASHCAL_ALGO_TOEPLITZ:
        hash_val = compute_toeplitz_hash(packet, cal_len);
        break;

      case HASHCAL_ALGO_XOR:
        hash_val = compute_xor_hash(packet, cal_len);
        break;

      case HASHCAL_ALGO_CRC32:
        hash_val = compute_crc32c_hash(packet, cal_len);
        break;

      default:
        DEBUGASSERT(0);
    }

  return hash_val;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_notify_recvcpu
 *
 * Description:
 *   Notify the cpu id for the network device driver.
 *
 * Input Parameters:
 *   dev      - The network device driver state structure
 *   cpu      - The current cpu id
 *   domain   - The layer 3 protocol, PF_INET/PF_INET6
 *   src_addr - The source address
 *   src_port - The source port
 *   dst_addr - The destination address
 *   dst_port - The destination port
 *
 * Returned Value:
 *  None
 *
 ****************************************************************************/

void netdev_notify_recvcpu(FAR struct net_driver_s *dev,
                           int cpu, uint8_t domain,
                           FAR const void *src_addr, uint16_t src_port,
                           FAR const void *dst_addr, uint16_t dst_port)
{
  if (dev != NULL && dev->d_ioctl != NULL)
    {
      uint32_t hash = compute_hash(HASHCAL_ALGO_CRC32,
                                   HASHCAL_TYPE_4TUPLE,
                                   domain,
                                   src_addr, src_port,
                                   dst_addr, dst_port);
      struct netdev_rss_s arg;
      int ret;

      arg.cpu = cpu;
      arg.hash = hash;

      ret = dev->d_ioctl(dev, SIOCNOTIFYRECVCPU,
                         (unsigned long)(uintptr_t)&arg);
      if (ret < 0)
        {
          nerr("ERROR: SIOCNOTIFYRECVCPU failed: %d\n", ret);
        }
    }
}
