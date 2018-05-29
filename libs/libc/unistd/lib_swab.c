/****************************************************************************
 * libs/libc/unistd/lib_swab.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
