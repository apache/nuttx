/****************************************************************************
 * libs/libc/misc/lib_crc8table.c
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
#include <crc8.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: crc8table
 *
 * Description:
 *   Return a 8-bit CRC of the contents of the 'src' buffer, length 'len'
 *   using a CRC with crc table used for calculation.
 *
 ****************************************************************************/

uint8_t crc8table(const uint8_t table[256], const uint8_t *src,
                  size_t len, uint8_t crc8val)
{
  size_t i;

  crc8val ^= 0xff;
  for (i = 0; i < len; i++)
    {
      crc8val = table[crc8val ^ src[i]];
    }

  return crc8val ^ 0xff;
}
