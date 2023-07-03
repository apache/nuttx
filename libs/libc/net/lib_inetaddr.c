/****************************************************************************
 * libs/libc/net/lib_inetaddr.c
 *
 *   Copyright (C) 2011 Yu Qiang. All rights reserved.
 *   Author: Yu Qiang <yuq825@gmail.com>
 *
 * This file is a part of NuttX:
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <arpa/inet.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * name inet_addr
 *
 * Description:
 *   The inet_addr() function converts the string pointed to by cp, in the
 *   standard IPv4 dotted decimal notation, to an integer value suitable for
 *   use as an Internet address.
 *
 *   inet_aton() returns nonzero if the address is valid, zero if not.
 *   The address supplied in cp can have one of the following forms:
 *
 *   a.b.c.d Each of the four numeric parts specifies a byte of the address;
 *           the bytes are assigned in left-to-right order to produce the
 *           binary address.
 *
 *   a.b.c   Parts a and b specify the first two bytes of the binary address.
 *           Part c is interpreted as a 16-bit value that defines the
 *           rightmost two bytes of the binary address. This notation is
 *           suitable for specifying (outmoded) Class B network addresses.
 *
 *   a.b     Part a specifies the first byte of the binary address. Part b is
 *           interpreted as a 24-bit value that defines the rightmost three
 *           bytes of the binary address. This notation is suitable for
 *           specifying (outmoded) Class A network addresses.
 *
 *   a       The value a is interpreted as a 32-bit value that is stored
 *           directly into the binary address without any byte rearrangement.
 *
 * Returned Value:
 *   If input string cannot be recognized as a valid IPv4 number, function
 *   returns zero.
 *
 ****************************************************************************/

in_addr_t inet_addr(FAR const char *cp)
{
  unsigned int a;
  unsigned int b;
  unsigned int c;
  unsigned int d;
  uint32_t result = INADDR_NONE;

  switch (sscanf(cp, "%u.%u.%u.%u", &a, &b, &c, &d))
    {
      case 1:
        {
          result = a;
          break;
        }

      case 2:
        {
          if ((a < 0x100) && (b < 0x1000000))
            {
              result = (((uint32_t) a) << 24) | b;
            }
          break;
        }

      case 3:
        {
          if ((a < 0x100) && (b < 0x100) && (c < 0x10000))
            {
              result = (((uint32_t) a) << 24) | (((uint32_t) b) << 16) | c;
            }
          break;
        }

      case 4:
        {
          if ((a < 0x100) && (b < 0x100) && (c < 0x100) && (d < 0x100))
            {
              result = (((uint32_t) a) << 24) | (((uint32_t) b) << 16) |
                       (((uint32_t) c) << 8) | d;
            }
          break;
        }
    }

  return HTONL(result);
}
