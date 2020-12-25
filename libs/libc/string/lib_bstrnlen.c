/****************************************************************************
 * libc/stdlib/lib_bstrnlen.c
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

size_t bstrnlen(FAR const char *src, size_t maxlen)
{
  size_t len = 0;

  while (1)
    {
      int i;

      for (i = 0; i < CHAR_BIT; i += 8, len++)
        {
          if (maxlen-- == 0 || ((*src >> i) & 0xff) == 0)
            {
              return len;
            }
        }

      src++;
    }
}

#endif
