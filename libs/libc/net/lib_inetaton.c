/****************************************************************************
 * libs/libc/net/lib_inetaton
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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

#include <stdint.h>
#include <arpa/inet.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_aton
 *
 * Description:
 *   The inet_aton() function converts the string pointed to by cp, in the
 *   standard IPv4 dotted decimal notation, to an integer value suitable for
 *   use as an Internet address.
 *
 * Note:
 *   inet_aton() returns nonzero if the address is valid, zero if not.
 *   Therefore macros OK and ERROR are not used here.
 *
 ****************************************************************************/

int inet_aton(FAR const char *cp, FAR struct in_addr *inp)
{
  int dots = 0;
  uint32_t num = 0;
  uint32_t addr = 0;
  int c;

  do
    {
      c = *cp;

      switch (c)
        {
          case '0':
          case '1':
          case '2':
          case '3':
          case '4':
          case '5':
          case '6':
          case '7':
          case '8':
          case '9':
            num = 10 * num + (c - '0');
            break;

          case '.':
            dots++;
            if (dots > 3)
              {
                return 0;
              }

            /* no break */

          case '\0':
            if (num > 255)
              {
                return 0;
              }

            addr = addr << 8 | num;
            num = 0;
            break;

          default:
            return 0;
        }
    }
  while (*cp++);

  /* Normalize it */

  if (dots < 3)
    {
      addr <<= 8 * (3 - dots);
    }

  if (inp)
    {
      inp->s_addr = HTONL(addr);
    }

  return 1;
}
