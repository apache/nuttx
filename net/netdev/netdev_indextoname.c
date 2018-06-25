/****************************************************************************
 * net/netdev/netdev_indextoname.c
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

#include <string.h>
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
 * Name: netdev_indextoname
 *
 * Description:
 *   The if_indextoname() function maps an interface index to its
 *   corresponding name.
 *
 * Input Parameters:
 *   ifname  - Points to a buffer of at least IF_NAMESIZE bytes.
 *             if_indextoname() will place in this buffer the name of the
 *             interface with index ifindex.
 *
 * Returned Value:
 *   If ifindex is an interface index, then the function will return zero
 *   (OK). Otherwise, the function returns a negated errno value;
 *
 ****************************************************************************/

int netdev_indextoname(unsigned int ifindex, FAR char *ifname)
{
  FAR struct net_driver_s *dev;
  int ret = -ENODEV;

  DEBUGASSERT(ifindex > 0 && ifindex <= MAX_IFINDEX);
  DEBUGASSERT(ifname != NULL);

  /* Find the driver with this name */

  net_lock();
  dev = netdev_findbyindex(ifindex);
  if (dev != NULL)
    {
      memcpy(ifname, dev->d_ifname, IF_NAMESIZE);
      ret = OK;
    }

  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: if_indextoname
 *
 * Description:
 *   The if_indextoname() function maps an interface index to its
 *   corresponding name.
 *
 * Input Parameters:
 *   ifname  - Points to a buffer of at least IF_NAMESIZE bytes.
 *             if_indextoname() will place in this buffer the name of the
 *             interface with index ifindex.
 *
 * Returned Value:
 *   If ifindex is an interface index, then the function will return the
 *   value supplied by ifname. Otherwise, the function returns a NULL pointer
 *   and sets errno to indicate the error.
 *
 ****************************************************************************/

FAR char *if_indextoname(unsigned int ifindex, FAR char *ifname)
{
  int ret;

  /* Let netdev_indextoname to the work */

  ret = netdev_indextoname(ifindex, ifname);
  if (ret < 0)
    {
      set_errno(-ret);
      return NULL;
    }

  return ifname;
}

#endif /* CONFIG_NETDEV_IFINDEX */
