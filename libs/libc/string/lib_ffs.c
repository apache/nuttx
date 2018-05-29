/****************************************************************************
 * libs/libc/string/lib_ffs.c
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

#include <nuttx/compiler.h>
#include <strings.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NBITS (8 * sizeof(unsigned int))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ffs
 *
 * Description:
 *   The ffs() function will find the first bit set (beginning with the least
 *   significant bit) in j, and return the index of that bit. Bits are
 *   numbered starting at one (the least significant bit).
 *
 * Returned Value:
 *   The ffs() function will return the index of the first bit set. If j is
 *   0, then ffs() will return 0.
 *
 ****************************************************************************/

int ffs(int j)
{
  int ret = 0;

  if (j != 0)
    {
#ifdef CONFIG_HAVE_BUILTIN_CTZ
      /* Count trailing zeros function can be used to implement ffs. */

      ret = __builtin_ctz(j) + 1;
#else
      unsigned int value = (unsigned int)j;
      int bitno;

      for (bitno = 1; bitno <= NBITS; bitno++, value >>= 1)
        {
          if ((value & 1) != 0)
            {
              ret = bitno;
              break;
            }
        }
#endif
    }

  return ret;
}
