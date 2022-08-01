/****************************************************************************
 * libs/libc/netdb/lib_dnsbind.c
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

#include <sys/time.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/dns.h>

#include "netdb/lib_dns.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NET_SOCKOPTS
#  error CONFIG_NET_SOCKOPTS required by this logic
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_bind
 *
 * Description:
 *   Initialize the DNS resolver and return a socket bound to the DNS name
 *   server.  The name server was previously selected via dns_server().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, the bound, non-negative socket descriptor is returned.  A
 *   negated errno value is returned on any failure.
 *
 ****************************************************************************/

int dns_bind(sa_family_t family)
{
  struct timeval tv;
  int sd;
  int ret;

  /* Has the DNS client been properly initialized? */

  if (!dns_initialize())
    {
      nerr("ERROR: DNS client has not been initialized\n");
      return -EDESTADDRREQ;
    }

  /* Create a new socket */

  sd = socket(family, SOCK_DGRAM, 0);
  if (sd < 0)
    {
      ret = -get_errno();
      nerr("ERROR: socket() failed: %d\n", ret);
      return ret;
    }

  /* Set up a receive timeout */

  tv.tv_sec  = CONFIG_NETDB_DNSCLIENT_RECV_TIMEOUT;
  tv.tv_usec = 0;

  ret = setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval));
  if (ret < 0)
    {
      ret = -get_errno();
      nerr("ERROR: setsockopt() failed: %d\n", ret);
      close(sd);
      return ret;
    }

  return sd;
}
