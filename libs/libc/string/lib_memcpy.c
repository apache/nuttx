/****************************************************************************
 * libs/libc/string/lib_memcpy.c
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
#include <string.h>

#include "libc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: memcpy
 ****************************************************************************/

#if !defined(CONFIG_LIBC_ARCH_MEMCPY) && defined(LIBC_BUILD_MEMCPY)
#undef memcpy
no_builtin("memcpy")
FAR void *memcpy(FAR void *dest, FAR const void *src, size_t n)
{
  FAR uint8_t *pout = (FAR uint8_t *)dest;
  FAR const uint8_t *pin  = (FAR const uint8_t *)src;

  if (!TOO_SMALL4(n) && !(((uintptr_t)pout ^ (uintptr_t)pin) & 3))
    {
      FAR uint32_t *paligned_out;
      FAR const uint32_t *paligned_in;

      while (UNALIGNED4_X(pout))
        {
          *pout++ = *pin++;
          n--;
        }

      paligned_out = (FAR uint32_t *)pout;
      paligned_in = (FAR const uint32_t *)pin;

      while (n >= BIGBLOCKSIZE4)
        {
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          *paligned_out++ = *paligned_in++;
          n -= BIGBLOCKSIZE4;
        }

      while (n >= LITTLEBLOCKSIZE4)
        {
          *paligned_out++ = *paligned_in++;
          n -= LITTLEBLOCKSIZE4;
        }

      pout = (FAR uint8_t *)paligned_out;
      pin = (FAR const uint8_t *)paligned_in;
    }

  while (n-- > 0)
    {
      *pout++ = *pin++;
    }

  return dest;
}
#endif
