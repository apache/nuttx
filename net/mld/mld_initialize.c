/****************************************************************************
 * net/mld/mld_init.c
 * MLD initialization logic
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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of CITEL Technologies Ltd nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CITEL TECHNOLOGIES AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL CITEL TECHNOLOGIES OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * SUBSTITUTE GOODS HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/wdog.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "inet/inet.h"
#include "mld/mld.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_initialize
 *
 * Description:
 *   Perform one-time MLD initialization.
 *
 ****************************************************************************/

void mld_initialize(void)
{
}

/****************************************************************************
 * Name:  mld_devinit
 *
 * Description:
 *   Called when a new network device is registered to configure that device
 *   for MLD support.
 *
 ****************************************************************************/

void mld_devinit(struct net_driver_s *dev)
{
  mldinfo("MLD initializing dev %p\n", dev);

  /* Initialize the MLD state in the device structure */

  memset(&dev->d_mld, 0, sizeof(struct mld_netdev_s));

  /* All routers start up as a Querier on each of their attached links. */

  SET_MLD_QUERIER(dev->d_mld.flags);

  /* Add the all nodes address to the group
   * REVISIT: Do we need this?  What is it for?  It is clone from IGMP and
   * probably is not relevant here.
   */

  mld_grpalloc(dev, g_ipv6_allnodes);

  /* Allow the MLD messages at the MAC level */

  mld_addmcastmac(dev, g_ipv6_allnodes);
  mld_addmcastmac(dev, g_ipv6_allrouters);
  mld_addmcastmac(dev, g_ipv6_allmldv2routers);

#ifdef CONFIG_NET_MLD_ROUTER
  /* Start the general query timer. */

  mld_start_gentimer(dev, MSEC2TICK(MLD_QUERY_MSEC));
#endif
}
