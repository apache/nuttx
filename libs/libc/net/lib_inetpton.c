/****************************************************************************
 * libs/libc/net/lib_inetpton.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes some logic extracted from hwport_ftpd, written by Jaehyuk Cho
 * <minzkn@minzkn.com> which was released under the BSD license.
 *
 *   Copyright (C) HWPORT.COM. All rights reserved.
 *   Author: JAEHYUK CHO <mailto:minzkn@minzkn.com>
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

#include <sys/socket.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <arpa/inet.h>
#include <netinet/in.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If netdb support is enabled, then we need to be able to manage IPv4 and
 * IPv6 addresses, regardless of networking support.
 */

#ifdef CONFIG_NETDB_HOSTFILE
#  undef  CONFIG_LIBC_IPv4_ADDRCONV
#  undef  CONFIG_LIBC_IPv6_ADDRCONV
#  define CONFIG_LIBC_IPv4_ADDRCONV 1
#  define CONFIG_LIBC_IPv6_ADDRCONV 1
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_ipv4_pton
 *
 * Description:
 *  The inet_ipv4_pton() function converts an IPv4 address in its standard
 *  text presentation form into its numeric binary form.
 *
 * Input Parameters:
 *   src  - The src argument points to the string being passed in.
 *   dest - The dest argument points to a numstr into which the function stores
 *          the numeric address; this must be large enough to hold the numeric
 *          address (32 bits for AF_INET, 128 bits for AF_INET6).
 *
 * Returned Value:
 *   inet_ipv4_pton() will returns 1 if the conversion succeeds. It will
 *   return 0 if the input is not a valid IPv4 dotted-decimal string string.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) || defined(CONFIG_LIBC_IPv4_ADDRCONV)
static int inet_ipv4_pton(FAR const char *src, FAR void *dest)
{
  size_t srcoffset;
  size_t numoffset;
  int value;
  int ndots;
  uint8_t ch;
  char numstr[4];
  uint8_t *ip;

  memset(dest, 0, sizeof(struct in_addr));

  ip        = (uint8_t *)dest;
  srcoffset = 0;
  numoffset = 0;
  ndots     = 0;

  for (; ; )
    {
      ch = (uint8_t)src[srcoffset++];

      if (ch == '.' || ch == '\0')
        {
          if (ch == '.' && ndots >= 4)
            {
              /* Too many dots */

              break;
            }

          if (numoffset < 1)
            {
              /* Empty numeric string */

              break;
            }

          numstr[numoffset] = '\0';
          numoffset = 0;

          value = atoi(numstr);
          if (value < 0 || value > 255)
            {
              /* Out of range value */

              break;
            }

          ip[ndots] = (uint8_t)value;

          if (ch == '\0')
            {
              if (ndots != 3)
                {
                  /* Not enough dots */

                  break;
                }

              /* Return 1 if the conversion succeeds */

              return 1;
            }

          ndots++;
        }
      else if (ch >= '0' && ch <= '9')
        {
          numstr[numoffset++] = ch;
          if (numoffset >= 4)
            {
              /* Number is too long */

              break;
            }
        }
      else
        {
          /* Illegal character */

          break;
        }
    }

  /* Return zero if there is any problem parsing the input */

  return 0;
}
#endif

/****************************************************************************
 * Name: inet_ipv6_pton
 *
 * Description:
 *  The inet_ipv6_pton() function converts an IPv6 address in its standard
 *  text presentation form into its numeric binary form.
 *
 * Input Parameters:
 *   src  - The src argument points to the string being passed in.
 *   dest - The dest argument points to a numstr into which the function stores
 *          the numeric address; this must be large enough to hold the numeric
 *          address (32 bits for AF_INET, 128 bits for AF_INET6).
 *
 * Returned Value:
 *   inet_ipv6_pton() will returns 1 if the conversion succeeds. It will
 *   return 0 if the input is not a valid IPv6 address string.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) || defined(CONFIG_LIBC_IPv6_ADDRCONV)
static int inet_ipv6_pton(FAR const char *src, FAR void *dest)
{
  size_t srcoffset;
  size_t numoffset;
  long value;
  int nsep;
  int nrsep;
  uint8_t ch;
  char numstr[5];
  uint8_t ip[sizeof(struct in6_addr)];
  uint8_t rip[sizeof(struct in6_addr)];
  bool rtime;

  memset(dest, 0, sizeof(struct in6_addr));

  srcoffset = 0;
  numoffset = 0;
  nsep      = 0;
  nrsep     = 0;
  rtime     = false;

  for (; ; )
    {
      ch = (uint8_t)src[srcoffset++];

      if (ch == ':' || ch == '\0')
        {
          if (ch == ':' && (nsep + nrsep) >= 8)
            {
              /* Too many separators */

              break;
            }

          if (ch != '\0' && numoffset < 1)
            {
              /* Empty numeric string */

              if (rtime && nrsep > 1)
                {
                  /* dup simple */

                  break;
                }

              numoffset = 0;
              rtime = true;
              continue;
            }

          numstr[numoffset] = '\0';
          numoffset = 0;

          value = strtol(numstr, NULL, 16);
          if (value < 0 || value > 0xffff)
            {
              /* Out of range value */

              break;
            }

          if (!rtime)
            {
              ip[(nsep << 1) + 0] = (uint8_t)((value >> 8) & 0xff);
              ip[(nsep << 1) + 1] = (uint8_t)((value >> 0) & 0xff);
              nsep++;
            }
          else
            {
              rip[(nrsep << 1) + 0] = (uint8_t)((value >> 8) & 0xff);
              rip[(nrsep << 1) + 1] = (uint8_t)((value >> 0) & 0xff);
              nrsep++;
            }

          if (ch == '\0' /* || ch == '/' */)
            {
              if ((nsep <= 1 && nrsep <= 0) ||
                  (nsep + nrsep) < 1 ||
                  (nsep + nrsep) > 8)
                {
                  /* Separator count problem */

                  break;
                }

              if (nsep > 0)
                {
                  memcpy(dest, &ip[0], nsep << 1);
                }

              if (nrsep > 0)
                {
                  memcpy(dest + (16 - (nrsep << 1)), &rip[0], nrsep << 1);
                }

              /* Return 1 if the conversion succeeds */

              return 1;
            }
        }
      else if ((ch >= '0' && ch <= '9') ||
               (ch >= 'a' && ch <= 'f') ||
               (ch >= 'A' && ch <= 'F'))
        {
          numstr[numoffset++] = ch;
          if (numoffset >= 5)
            {
              /* Numeric string is too long */

              break;
            }
        }
      else
        {
          /* Illegal character */

          break;
        }
    }


  /* Return zero if there is any problem parsing the input */

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_pton
 *
 * Description:
 *  The inet_pton() function converts an address in its standard text
 *  presentation form into its numeric binary form.
 *
 *  If the af argument of inet_pton() is AF_INET, the src string will be
 *  in the standard IPv4 dotted-decimal form:
 *
 *    ddd.ddd.ddd.ddd
 *
 *  where "ddd" is a one to three digit decimal number between 0 and 255.
 *
 *  If the af argument of inet_pton() is AF_INET6, the src string will be in
 *  one of the following standard IPv6 text forms:
 *
 *  1. The preferred form is "x:x:x:x:x:x:x:x", where the 'x' s are the
 *     hexadecimal values of the eight 16-bit pieces of the address. Leading
 *     zeros in individual fields can be omitted, but there must be at least
 *     one numeral in every field.
 *
 *  2. A string of contiguous zero fields in the preferred form can be shown
 *     as "::". The "::" can only appear once in an address. Unspecified
 *     addresses ( "0:0:0:0:0:0:0:0" ) may be represented simply as "::".
 *
 *  3. A third form that is sometimes more convenient when dealing with a
 *     mixed environment of IPv4 and IPv6 nodes is "x:x:x:x:x:x:d.d.d.d",
 *     where the 'x' s are the hexadecimal values of the six high-order
 *     16-bit pieces of the address, and the 'd' s are the decimal values
 *     of the four low-order 8-bit pieces of the address (standard IPv4
 *     representation).
 *
 * Input Parameters:
 *   af   - The af argument specifies the family of the address. This can be
 *          AF_INET or AF_INET6.
 *   src  - The src argument points to the string being passed in.
 *   dest - The dest argument points to memory into which the function stores
 *          the numeric address; this must be large enough to hold the numeric
 *          address (32 bits for AF_INET, 128 bits for AF_INET6).
 *
 * Returned Value:
 *   The inet_pton() function returns 1 if the conversion succeeds, with the
 *   address pointed to by dest in network byte order. It will return 0 if the
 *   input is not a valid IPv4 dotted-decimal string or a valid IPv6 address
 *   string, or -1 with errno set to EAFNOSUPPORT if the af argument is
 *   unknown.
 *
 ****************************************************************************/

int inet_pton(int af, FAR const char *src, FAR void *dest)
{
  DEBUGASSERT(src && dest);

  /* Do the conversion according to the IP version */

  switch (af)
    {
#if defined(CONFIG_NET_IPv4) || defined(CONFIG_LIBC_IPv4_ADDRCONV)
    case AF_INET:
      return inet_ipv4_pton(src, dest);
#endif

#if defined(CONFIG_NET_IPv6) || defined(CONFIG_LIBC_IPv6_ADDRCONV)
    case AF_INET6:
      return inet_ipv6_pton(src, dest);
#endif

    default:
      set_errno(EAFNOSUPPORT);
      return ERROR;
    }
}
