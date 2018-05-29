/****************************************************************************
 * libs/libc/string/lib_fls.c
 *
 *   Copyright (C) 2017 Haltian Ltd. All rights reserved.
 *   Author: Jussi Kivilinna <jussi.kivilinna@haltian.com>
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

#define NBITS (8 * sizeof(unsigned long long))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_LONG

/****************************************************************************
 * Name: flsll
 *
 * Description:
 *   The flsll() function will find the last bit set in value and return
 *   the index of that bit. Bits are numbered starting at one (the least
 *   significant bit).
 *
 * Returned Value:
 *   The flsll() function will return the index of the last bit set. If j is
 *   0, then flsll() will return 0.
 *
 ****************************************************************************/

int flsll(long long j)
{
  int ret = 0;

  if (j != 0)
    {
#ifdef CONFIG_HAVE_BUILTIN_CLZ
      /* Count leading zeros function can be used to implement fls. */

      ret = NBITS - __builtin_clzll(j);
#else
      unsigned long long value = (unsigned long long)j;
      int bitno;

      for (bitno = 1; bitno <= NBITS; bitno++, value >>= 1)
        {
          if (value == 1)
            {
              ret = bitno;
              break;
            }
        }
#endif
    }

  return ret;
}

#endif

