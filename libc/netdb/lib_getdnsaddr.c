/****************************************************************************
 * libc/netdb/lib_dnsgetaddr.c
 *
 *   Copyright (C) 2007-2009, 2011, 2015 Gregory Nutt. All rights reserved.
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

#include <sys/socket.h>

#include <string.h>
#include <errno.h>
#include <assert.h>

#include <netinet/in.h>

#include <nuttx/net/dns.h>

#include <apps/netutils/netlib.h>

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NETDB_DNSCLIENT)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_getaddr
 *
 * Description:
 *   Get the DNS server IPv4 address
 *
 * Parameters:
 *   ipaddr   The location to return the IPv4 address
 *
 * Return:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int dns_getaddr(FAR struct in_addr *inaddr)
{
  struct sockaddr_in addr;
  socklen_t addrlen;
  int ret = -EINVAL;

  if (inaddr)
    {
      addrlen = sizeof(struct sockaddr_in);
      ret = dns_getserver((FAR struct sockaddr *)&addr, &addrlen);
      if (ret >= 0)
        {
          /* Sanity check */

          DEBUGASSERT(addr.sin_family == AF_INET &&
                      addrlen == sizeof(struct sockaddr_in));
          memcpy(inaddr, &addr.sin_addr, sizeof(struct in_addr));
        }
    }

  return ret;
}

#endif /* CONFIG_NET_IPv4 && CONFIG_NETDB_DNSCLIENT */
