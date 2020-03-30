/****************************************************************************
 * libs/libc/netdb/lib_dnsbind.c
 *
 *   Copyright (C) 2007, 2009, 2012, 2014-2016 Gregory Nutt.
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

int dns_bind(void)
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

  sd = socket(PF_INET, SOCK_DGRAM, 0);
  if (sd < 0)
    {
      ret = -errno;
      nerr("ERROR: socket() failed: %d\n", ret);
      return ret;
    }

  /* Set up a receive timeout */

  tv.tv_sec  = CONFIG_NETDB_DNSCLIENT_RECV_TIMEOUT;
  tv.tv_usec = 0;

  ret = setsockopt(sd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval));
  if (ret < 0)
    {
      ret = -errno;
      nerr("ERROR: setsockopt() failed: %d\n", ret);
      close(sd);
      return ret;
    }

  return sd;
}
