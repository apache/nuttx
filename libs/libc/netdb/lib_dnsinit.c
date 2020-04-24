/****************************************************************************
 * libs/libc/netdb/lib_dnsinit.c
 *
 *   Copyright (C) 2007, 2009, 2012, 2014-2017 Gregory Nutt.
 *   All rights reserved.
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

#include <string.h>
#include <errno.h>
#include <assert.h>

#include <arpa/inet.h>

#include <nuttx/semaphore.h>

#include "netdb/lib_dns.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Protects DNS cache, nameserver list and notify list. */

static sem_t g_dns_sem = SEM_INITIALIZER(1);

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(CONFIG_NETDB_DNSSERVER_IPv6) && !defined(CONFIG_NETDB_RESOLVCONF)

/* This is the default IPv6 DNS server address */

static const uint16_t g_ipv6_hostaddr[8] =
{
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_1),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_2),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_3),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_4),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_5),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_6),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_7),
  HTONS(CONFIG_NETDB_DNSSERVER_IPv6ADDR_8)
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_initialize
 *
 * Description:
 *   Make sure that the DNS client has been properly initialized for use.
 *
 ****************************************************************************/

bool dns_initialize(void)
{
#ifndef CONFIG_NETDB_RESOLVCONF
  int nservers;

  dns_semtake();
  nservers = g_dns_nservers;
  dns_semgive();

  /* Has at least one DNS server IP address been assigned? */

  if (nservers == 0)
    {
#if defined(CONFIG_NETDB_DNSSERVER_IPv4)
      struct sockaddr_in addr4;
      int ret;

      /* No, configure the default IPv4 DNS server address */

      addr4.sin_family      = AF_INET;
      addr4.sin_port        = HTONS(DNS_DEFAULT_PORT);
      addr4.sin_addr.s_addr = HTONL(CONFIG_NETDB_DNSSERVER_IPv4ADDR);

      ret = dns_add_nameserver((FAR struct sockaddr *)&addr4,
                               sizeof(struct sockaddr_in));
      if (ret < 0)
        {
          return false;
        }

#elif defined(CONFIG_NETDB_DNSSERVER_IPv6)
      struct sockaddr_in6 addr6;
      int ret;

      /* No, configure the default IPv6 DNS server address */

      addr6.sin6_family = AF_INET6;
      addr6.sin6_port   = HTONS(DNS_DEFAULT_PORT);
      memcpy(addr6.sin6_addr.s6_addr, g_ipv6_hostaddr, 16);

      ret = dns_add_nameserver((FAR struct sockaddr *)&addr6,
                               sizeof(struct sockaddr_in6));
      if (ret < 0)
        {
          return false;
        }

#else
      /* Then we are not ready to perform DNS queries */

      return false;
#endif
    }
#endif /* !CONFIG_NETDB_RESOLVCONF */

  return true;
}

/****************************************************************************
 * Name: dns_semtake
 *
 * Description:
 *   Take the DNS semaphore, ignoring errors due to the receipt of signals.
 *
 ****************************************************************************/

void dns_semtake(void)
{
  int errcode = 0;
  int ret;

  do
    {
      ret = _SEM_WAIT(&g_dns_sem);
      if (ret < 0)
        {
          errcode = _SEM_ERRNO(ret);
          DEBUGASSERT(errcode == EINTR || errcode == ECANCELED);
        }
    }
  while (ret < 0 && errcode == EINTR);
}

/****************************************************************************
 * Name: dns_semgive
 *
 * Description:
 *   Release the DNS semaphore
 *
 ****************************************************************************/

void dns_semgive(void)
{
  DEBUGVERIFY(_SEM_POST(&g_dns_sem));
}
