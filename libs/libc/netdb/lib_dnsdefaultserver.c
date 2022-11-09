/****************************************************************************
 * libs/libc/netdb/lib_dnsdefaultserver.c
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

#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/dns.h>

#include "netdb/lib_dns.h"

#ifdef CONFIG_NETDB_DNSCLIENT

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_default_nameserver
 *
 * Description:
 *   Reset the resolver to use only the default DNS server, if any.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_RESOLVCONF
int dns_default_nameserver(void)
{
  /* REVISIT: not implemented */

  return -ENOSYS;
}
#else /* CONFIG_NETDB_RESOLVCONF */
int dns_default_nameserver(void)
{
  dns_lock();
  g_dns_nservers = 0;
  dns_unlock();
  return OK;
}
#endif /* CONFIG_NETDB_RESOLVCONF */
#endif /* CONFIG_NETDB_DNSCLIENT */
