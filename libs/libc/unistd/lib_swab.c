/****************************************************************************
 * libs/libc/unistd/lib_swab.c
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
#include <unistd.h>
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strncpy
 *
 * Description:
 *   The swab() function will copy nbytes bytes, which are pointed to by
 *   'src', to the object pointed to by 'dest', exchanging adjacent bytes.
 *   The 'nbytes' argument should be even. If 'nbytes' is odd, swab() copies
 *   and exchanges 'nbytes'-1 bytes and the disposition of the last byte is
 *   unspecified. If copying takes place between objects that overlap, the
 *   behavior is undefined. If nbytes is negative, swab() does nothing.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void swab(FAR const void *src, FAR void *dest, ssize_t nbytes)
{
  FAR const uint8_t *src8 = (FAR const uint8_t *)src;
  FAR uint8_t *dest8 = (FAR uint8_t *)dest;
  FAR uint8_t *end8;

  DEBUGASSERT(src != NULL && dest != NULL);

  /* Do nother if nbytes is negative or it there are few then 2 bytes to be
   * transferred.
   */

  if (nbytes > 1)
    {
      /* The end of dest buffer + 1 byte (skipping any odd numbered byte at
       * the end of the buffer.
       */

      end8 = dest8 + (nbytes & ~1);

      /* Loop until the destination is equal to the end + 1 address */

      while (dest8 != end8)
        {
          register uint8_t tmp;

          /* Transfer the bytes, swapping the order */

          tmp      = *src8++;
          *dest8++ = *src8++;
          *dest8++ = tmp;
        }
    }
}
