/****************************************************************************
 * libs/libc/netdb/lib_dnsqueryfamily.c
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

#include <sys/types.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include "netdb/lib_dns.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int g_query_family = AF_UNSPEC;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_is_queryfamily
 *
 * Description:
 *   Determine if the specified address family is available for DNS query.
 *
 ****************************************************************************/

bool dns_is_queryfamily(sa_family_t family)
{
  DEBUGASSERT(family == AF_INET || family == AF_INET6);

  return g_query_family == AF_UNSPEC || g_query_family == family;
}

/****************************************************************************
 * Name: dns_set_queryfamily
 *
 * Description:
 *   Configure the address family to be used for queries.
 *
 ****************************************************************************/

int dns_set_queryfamily(sa_family_t family)
{
  int ret = OK;

  switch (family)
    {
#ifdef CONFIG_NET_IPv4
      case AF_INET:
#endif
#ifdef CONFIG_NET_IPv6
      case AF_INET6:
#endif
      case AF_UNSPEC:
        if (g_query_family != family)
          {
            dns_lock();
            g_query_family = family;
            dns_unlock();
#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
            dns_clear_answer();
#endif
            ninfo("Configure address family for dns query: %d\n", family);
          }

        break;
      default:
        nerr("ERROR: Unsupported family: %d\n", family);
        ret = -ENOSYS;
        break;
    }

  return ret;
}

