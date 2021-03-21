/****************************************************************************
 * net/route/net_foreach_fileroute.c
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
 *   arg     - An arbitrary value that will be passed to the handler.
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
