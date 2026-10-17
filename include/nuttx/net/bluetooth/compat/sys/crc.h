/****************************************************************************
 * include/nuttx/net/bluetooth/compat/sys/crc.h
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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

/* SPDX-License-Identifier: Apache-2.0 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SYS_CRC_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SYS_CRC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <nuttx/crc8.h>
#include <nuttx/crc16.h>
#include <nuttx/crc32.h>

#ifdef crc8
#undef crc8
#endif

#ifdef crc16
#undef crc16
#endif

#ifdef crc32
#undef crc32
#endif

#define CRC8_POLY 0x07
#define CRC8_REFLECT_POLY 0xe0
#define CRC16_POLY 0x8005
#define CRC16_REFLECT_POLY 0xa001
#define CRC16_CCITT_POLY 0x1021

static inline uint8_t crc8(
  const uint8_t *src, size_t len, uint8_t polynomial, uint8_t initial_value,
  bool reversed)
{
  uint8_t crc = initial_value;

  while (len-- > 0)
    {
      crc ^= *src++;

      for (int i = 0; i < 8; i++)
        {
          if (reversed)
            {
              crc = (crc & 0x01) ? (crc >> 1) ^ polynomial : crc >> 1;
            }
          else
            {
              crc = (crc & 0x80) ? (crc << 1) ^ polynomial : crc << 1;
            }
        }
    }

  return crc;
}

static inline uint16_t crc16(uint16_t poly,
                             uint16_t seed, const uint8_t *src, size_t len)
{
  if (poly == CRC16_POLY)
    {
      return crc16h8005_part(src, len, seed);
    }

  if (poly == CRC16_CCITT_POLY)
    {
      return crc16h1021_part(src, len, seed);
    }

  uint16_t crc = seed;

  while (len-- > 0)
    {
      crc ^= (uint16_t)*src++ << 8;

      for (int i = 0; i < 8; i++)
        {
          crc = (crc & 0x8000) ? (crc << 1) ^ poly : crc << 1;
        }
    }

  return crc;
}

static inline uint16_t crc16_reflect(uint16_t poly,
                                     uint16_t seed, const uint8_t *src,
                                     size_t len)
{
  if (poly == CRC16_REFLECT_POLY)
    {
      return crc16ibmpart(src, len, seed);
    }

  uint16_t crc = seed;

  while (len-- > 0)
    {
      crc ^= *src++;

      for (int i = 0; i < 8; i++)
        {
          if ((crc & 0x0001) != 0)
            {
              crc = (crc >> 1) ^ poly;
            }
          else
            {
              crc >>= 1;
            }
        }
    }

  return crc;
}

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_COMPAT_SYS_CRC_H */
