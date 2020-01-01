/****************************************************************************
 * net/route/net_foreach_fileroute.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "route/fileroute.h"
#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_FILEROUTE) || defined(CONFIG_ROUTE_IPv6_FILEROUTE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_foreachroute_ipv4 and net_foreachroute_ipv6
 *
 * Description:
 *   Traverse the routing table
 *
 * Input Parameters:
 *   handler - Will be called for each route in the routing table.
 *   arg     - An arbitrary value that will be passed tot he handler.
 *
 * Returned Value:
 *   Zero (OK) returned if the entire table was search.  A negated errno
 *   value will be returned in the event of a failure.  Handlers may also
 *   terminate the search early with any non-zero, non-negative value.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_foreachroute_ipv4(route_handler_ipv4_t handler, FAR void *arg)
{
  struct net_route_ipv4_s route;
  struct file fshandle;
  ssize_t nread;
  int ret = 0;

  /* Open the IPv4 routing table for read-only access */

  ret = net_openroute_ipv4(O_RDONLY, &fshandle);
  if (ret < 0)
    {
      /* Special case:  the routing table has not yet been created.  This is
       * not an error.  We will just want to return successful completion of
       * the traversal.
       */

      if (ret == -ENOENT)
        {
          /* The routing table does not exit.. return successful completion */

          ninfo("The IPv4 routing table file does not exist\n");
          return OK;
        }

      /* Some other error occurred. */

      nerr("ERROR: Could not open IPv4 routing table: %d\n", ret);
      return ret;
    }

  /* Read each entry from the routing table */

  for (; ; )
    {
      nread = net_readroute_ipv4(&fshandle, &route);
      if (nread < 0)
        {
          /* File read error */

          nerr("ERROR: net_readroute_ipv4() failed: %ld\n", (long)nread);
          ret = (int)nread;
          break;
        }
      else if (nread == 0)
        {
          /* End of file */

          ret = OK;
          break;
        }

      /* Call the handler. */

      ret = handler(&route, arg);
      if (ret != OK)
        {
          /* Terminate early if the handler returns any non-zero value. */

          break;
        }
    }

  net_closeroute_ipv4(&fshandle);
  return ret;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_foreachroute_ipv6(route_handler_ipv6_t handler, FAR void *arg)
{
  struct net_route_ipv6_s route;
  struct file fshandle;
  ssize_t nread;
  int ret = 0;

  /* Open the IPv6 routing table for read-only access */

  ret = net_openroute_ipv6(O_RDONLY, &fshandle);
  if (ret < 0)
    {
      /* Special case:  the routing table has not yet been created.  This is
       * not an error.  We will just want to return successful completion of
       * the traversal.
       */

      if (ret == -ENOENT)
        {
          /* The routing table does not exit.. return successful completion */

          ninfo("The IPv6 routing table file does not exist\n");
          return OK;
        }

      /* Some other error occurred. */

      nerr("ERROR: Could not open IPv6 routing table: %d\n", ret);
      return ret;
    }

  /* Read each entry from the routing table */

  for (; ; )
    {
      nread = net_readroute_ipv6(&fshandle, &route);
      if (nread < 0)
        {
          /* File read error */

          nerr("ERROR: net_readroute_ipv6() failed: %ld\n", (long)nread);
          ret = (int)nread;
          break;
        }
      else if (nread == 0)
        {
          /* End of file */

          ret = OK;
          break;
        }

      /* Call the handler. */

      ret = handler(&route, arg);
      if (ret != OK)
        {
          /* Terminate early if the handler returns any non-zero value. */

          break;
        }
    }

  net_closeroute_ipv6(&fshandle);
  return ret;
}
#endif

#endif /* CONFIG_ROUTE_IPv4_FILEROUTE || CONFIG_ROUTE_IPv6_FILEROUTE */
