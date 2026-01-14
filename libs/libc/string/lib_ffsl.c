/****************************************************************************
 * libs/libc/string/lib_ffsl.c
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
 * Pre-processor Definitions
 ****************************************************************************/

#define NBITS (8 * sizeof(unsigned long))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ffsl
 *
 * Description:
 *   The ffsl() function will find the first bit set (beginning with the
 *   least significant bit) in j, and return the index of that bit. Bits are
 *   numbered starting at one (the least significant bit).
 *
 * Returned Value:
 *   The ffsl() function will return the index of the first bit set. If j is
 *   0, then ffsl() will return 0.
 *
 ****************************************************************************/

int ffsl(long j)
{
  int ret = 0;

  if (j != 0)
    {
      unsigned long value = (unsigned long)j;
      int bitno;

      for (bitno = 1; bitno <= NBITS; bitno++, value >>= 1)
        {
          if ((value & 1) != 0)
            {
              ret = bitno;
              break;
            }
        }
    }

  return ret;
}
