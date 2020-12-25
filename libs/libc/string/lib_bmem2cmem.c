/****************************************************************************
 * libc/stdlib/lib_bmem2cmem.c
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

#include <stdlib.h>

#if !defined(CONFIG_ENDIAN_BIG) && CHAR_BIT != 8

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void bmem2cmem(FAR void *dst_, FAR const void *src_, size_t rem, size_t len)
{
  char *dst = dst_;
  const char *src = src_;

  while (1)
    {
      int i;

      for (i = 8 * rem; i < CHAR_BIT; i += 8)
        {
          if (len-- == 0)
            {
              return;
            }

          *dst++ = (*src >> i) & 0xff;
        }

      rem = 0;
      src++;
    }
}

#endif
