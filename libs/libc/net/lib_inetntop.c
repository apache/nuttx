/****************************************************************************
 * libs/libc/net/lib_inetntop.c
 *
 *   Copyright (C) 2012, 2015 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <arpa/inet.h>

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
 * Name: inet_ipv4_ntop
 *
 * Description:
 *  The inet_ipv4_ntop() function converts a numeric IPv4 address into a
 *  text string suitable for presentation.
 *
 * Input Parameters:
 *   src  - The src argument points to a buffer holding an address of the
 *          specified type.  The address must be in network byte order.
 *   dest - The dest argument points to a buffer where the function stores
 *          the resulting text string; it shall not be NULL.
 *   size - The size argument specifies the size of this buffer, which must
 *          be large enough to hold the text string (INET_ADDRSTRLEN
 *          characters for IPv4, INET6_ADDRSTRLEN characters for IPv6).
 *
 * Returned Value:
 *   inet_ntop() returns a pointer to the buffer containing the text string
 *   if the conversion succeeds.  Otherwise, NULL is returned and the errno
 *   is set to indicate the error.  There follow errno values may be set:
 *
 *   ENOSPC - The size of the inet_ntop() result buffer is inadequate
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv4) || defined(CONFIG_LIBC_IPv4_ADDRCONV)
static int inet_ipv4_ntop(FAR const void *src, FAR char *dest,
                          socklen_t size)
{
  FAR uint8_t *ptr;

  if (size < INET_ADDRSTRLEN)
    {
      return -ENOSPC;
    }

  ptr = (FAR uint8_t *)src;

  /* Data is in network order.  However, indexed access is the same in both
   * big and little endian cases:
   *
   * Big Endian:
   *                  +---+---+---+---+
   *   Network Order: | 0 | 1 | 2 | 3 |  n=Network byte order
   *                  |192|168| 1 | 2 |  Example
   *                  +---+---+---+---+
   *   Host Index:    | 0 | 1 | 2 | 3 |  n=Host Index
   *                  |192|168| 1 | 2 |  Example
   *                  +---+---+---+---+
   *
   * Little Endian:
   *
   *                +---+---+---+---+
   * Network Order: | 0 | 1 | 2 | 3 |  n=Network byte order
   *                |192|168| 1 | 2 |  Example
   *                +---+---+---+---+
   * Host Index:    | 3 | 2 | 1 | 0 |  n=Host Index
   *                | 2 | 1 |168|192|  Example
   *                +---+---+---+---+
   */

  snprintf(dest, INET_ADDRSTRLEN, "%u.%u.%u.%u",
           ptr[0], ptr[1], ptr[2], ptr[3]);

  return OK;
}
#endif

/****************************************************************************
 * Name: inet_ipv6_ntop
 *
 * Description:
 *  The inet_ipv6_ntop() function converts a numeric IPv6 address into a
 *  text string suitable for presentation.
 *
 * Input Parameters:
 *   af   - The af argument specifies the family of the address. This can be
 *          AF_INET or AF_INET6.
 *   src  - The src argument points to a buffer holding an address of the
 *          specified type.  The address must be in network byte order.
 *   dest - The dest argument points to a buffer where the function stores
 *          the resulting text string; it shall not be NULL.
 *   size - The size argument specifies the size of this buffer, which must
 *          be large enough to hold the text string (INET_ADDRSTRLEN
 *          characters for IPv4, INET6_ADDRSTRLEN characters for IPv6).
 *
 * Returned Value:
 *   inet_ntop() returns a pointer to the buffer containing the text string
 *   if the conversion succeeds.  Otherwise, NULL is returned and the errno
 *   is set to indicate the error.  There follow errno values may be set:
 *
 *   EAFNOSUPPORT - The af argument is invalid.
 *   ENOSPC - The size of the inet_ntop() result buffer is inadequate
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) || defined(CONFIG_LIBC_IPv6_ADDRCONV)
static int inet_ipv6_ntop(FAR const void *src, FAR char *dest,
                          socklen_t size)
{
  FAR const struct in6_addr *in6_addr;
  uint16_t warray[8];
  int offset;
  int entry;
  int count;
  int maxentry;
  int maxcount;

  if (size < INET6_ADDRSTRLEN)
    {
      return -ENOSPC;
    }

  in6_addr = (FAR const struct in6_addr *)src;
  entry    = -1;
  maxentry = -1;
  maxcount = 0;
  offset   = 0;

  while (offset < 8)
    {
      warray[offset] = NTOHS(in6_addr->s6_addr16[offset]);
      if (warray[offset] == 0)
        {
          entry = offset;
          count = 1;
          offset++;

          while (offset < 8)
            {
              warray[offset] = NTOHS(in6_addr->s6_addr16[offset]);
              if (warray[offset] != 0)
                {
                  break;
                }

              offset++;
              count++;
            }

          if (count > maxcount)
            {
              maxentry = entry;
              maxcount = count;
            }
        }

      offset++;
    }

  offset = 0;
  dest[0] = '\0';

  while (offset < 8)
    {
      if (offset == maxentry)
        {
          size   -= snprintf(&dest[strlen(dest)], size, ":");
          offset += maxcount;
          if (offset >= 8)
            {
              size -= snprintf(&dest[strlen(dest)], size, ":");
            }
        }
      else
        {
          if (offset > 0)
            {
              size -= snprintf(&dest[strlen(dest)], size, ":");
            }

          size -= snprintf(&dest[strlen(dest)], size, "%x", warray[offset]);
          offset++;
        }
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_ntop
 *
 * Description:
 *  The inet_ntop() function converts a numeric address into a text string
 *  suitable for presentation.
 *
 * Input Parameters:
 *   af   - The af argument specifies the family of the address. This can be
 *          AF_INET or AF_INET6.
 *   src  - The src argument points to a buffer holding an address of the
 *          specified type.  The address must be in network byte order.
 *   dest - The dest argument points to a buffer where the function stores
 *          the resulting text string; it shall not be NULL.
 *   size - The size argument specifies the size of this buffer, which must
 *          be large enough to hold the text string (INET_ADDRSTRLEN
 *          characters for IPv4, INET6_ADDRSTRLEN characters for IPv6).
 *
 * Returned Value:
 *   inet_ntop() returns a pointer to the buffer containing the text string
 *   if the conversion succeeds.  Otherwise, NULL is returned and the errno
 *   is set to indicate the error.  There follow errno values may be set:
 *
 *   EAFNOSUPPORT - The af argument is invalid.
 *   ENOSPC - The size of the inet_ntop() result buffer is inadequate
 *
 ****************************************************************************/

FAR const char *inet_ntop(int af, FAR const void *src, FAR char *dest,
                          socklen_t size)
{
  int ret;

  DEBUGASSERT(src != NULL && dest != NULL);

  /* Do the conversion according to the IP version */

  switch (af)
    {
#if defined(CONFIG_NET_IPv4) || defined(CONFIG_LIBC_IPv4_ADDRCONV)
    case AF_INET:
      ret = inet_ipv4_ntop(src, dest, size);
      break;
#endif

#if defined(CONFIG_NET_IPv6) || defined(CONFIG_LIBC_IPv6_ADDRCONV)
    case AF_INET6:
      ret = inet_ipv6_ntop(src, dest, size);
      break;
#endif

    default:
      ret = -EAFNOSUPPORT;
      break;
    }

  /* Handle errors in the conversion */

  if (ret < 0)
    {
      set_errno(-ret);
      memset(dest, 0, size);
      return NULL;
    }

  /* Return success */

  return dest;
}
