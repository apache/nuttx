/****************************************************************************
 * libs/libc/netdb/lib_dnsinit.c
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

#include <string.h>
#include <errno.h>
#include <assert.h>

#include <arpa/inet.h>

#include <nuttx/sched.h>
#include <nuttx/mutex.h>
#include "netdb/lib_dns.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Protects DNS cache, nameserver list and notify list. */

static rmutex_t g_dns_lock = NXRMUTEX_INITIALIZER;

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
 *   Take the DNS lock, ignoring errors due to the receipt of signals.
 *
 ****************************************************************************/

void dns_semtake(void)
{
  nxrmutex_lock(&g_dns_lock);
}

/****************************************************************************
 * Name: dns_semgive
 *
 * Description:
 *   Release the DNS lock
 *
 ****************************************************************************/

void dns_semgive(void)
{
  nxrmutex_unlock(&g_dns_lock);
}
