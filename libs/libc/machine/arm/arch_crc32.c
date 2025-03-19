/****************************************************************************
 * libs/libc/machine/arm/arch_crc32.c
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

#include <stdint.h>
#include <sys/types.h>
#include <arm_acle.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: crc32part
 *
 * Description:
 *   crc32 polynomial 0x04C11DB7 (bitreflected 0xEDB88320)
 *
 ****************************************************************************/

uint32_t crc32part(FAR const uint8_t *src, size_t len, uint32_t crc32val)
{
  size_t i = 0;

  for (; (i + 8) <= len; i += 8)
    {
      uint64_t data = *(uint64_t *)(src + i);
      crc32val = __crc32d(crc32val, data);
    }

  if (i + 4 <= len)
    {
      uint32_t data = *(uint32_t *)(src + i);
      crc32val = __crc32w(crc32val, data);
      i += 4;
    }

  if (i + 2 <= len)
    {
      uint16_t data = *(uint16_t *)(src + i);
      crc32val = __crc32h(crc32val, data);
      i += 2;
    }

  if (i < len)
    {
      uint8_t data = *(uint8_t *)(src + i);
      crc32val = __crc32b(crc32val, data);
    }

  return crc32val;
}

/****************************************************************************
 * Name: crc32part_c
 *
 * Description:
 *   crc32 Castagnoli polynomial 0x1EDC6F41
 *
 ****************************************************************************/

uint32_t crc32part_c(FAR const uint8_t *src, size_t len, uint32_t crc32val)
{
  size_t i = 0;

  for (; (i + 8) <= len; i += 8)
    {
      uint64_t data = *(uint64_t *)(src + i);
      crc32val = __crc32cd(crc32val, data);
    }

  if (i + 4 <= len)
    {
      uint32_t data = *(uint32_t *)(src + i);
      crc32val = __crc32cw(crc32val, data);
      i += 4;
    }

  if (i + 2 <= len)
    {
      uint16_t data = *(uint16_t *)(src + i);
      crc32val = __crc32ch(crc32val, data);
      i += 2;
    }

  if (i < len)
    {
      uint8_t data = *(uint8_t *)(src + i);
      crc32val = __crc32cb(crc32val, data);
    }

  return crc32val;
}
