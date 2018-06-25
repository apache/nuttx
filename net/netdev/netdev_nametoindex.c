/****************************************************************************
 * net/netdev/netdev_nametoindex.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <assert.h>
#include <errno.h>

#include <net/if.h>

#include "nuttx/net/net.h"
#include "nuttx/net/netdev.h"

#include "netdev/netdev.h"

#ifdef CONFIG_NETDEV_IFINDEX

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_nametoindex
 *
 * Description:
 *   The if_nametoindex() function returns the interface index corresponding
 *   to name ifname.
 *
 * Input Parameters:
 *   ifname - The interface name
 *
 * Returned Value:
 *   The corresponding index if ifname is the name of an interface;
 *   otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

unsigned int netdev_nametoindex(FAR const char *ifname)
{
  FAR struct net_driver_s *dev;
  unsigned int ifindex = -ENODEV;

  /* Find the driver with this name */

  net_lock();
  dev = netdev_findbyname(ifname);
  if (dev != NULL)
    {
      ifindex = dev->d_ifindex;
    }

  net_unlock();
  return ifindex;
}

/****************************************************************************
 * Name: if_nametoindex
 *
 * Description:
 *   The if_nametoindex() function returns the interface index corresponding
 *   to name ifname.
 *
 * Input Parameters:
 *   ifname - The interface name
 *
 * Returned Value:
 *   The corresponding index if ifname is the name of an interface;
 *   otherwise, zero.  Although not specified, the errno value will be set.
 *
 ****************************************************************************/

unsigned int if_nametoindex(FAR const char *ifname)
{
  int ret;

  /* Let netdev_nametoindex to the work */

  ret = netdev_nametoindex(ifname);
  if (ret < 0)
    {
      set_errno(-ret);
      return 0;
    }

  return ret;
}

#endif /* CONFIG_NETDEV_IFINDEX */
