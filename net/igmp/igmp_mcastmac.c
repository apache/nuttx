/****************************************************************************
 * net/igmp/igmp_mcastmac.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The NuttX implementation of IGMP was inspired by the IGMP add-on for the
 * lwIP TCP/IP stack by Steve Reynolds:
 *
 *   Copyright (c) 2002 CITEL Technologies Ltd.
 *   All rights reserved.
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
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <inttypes.h>
#include <stdint.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "igmp/igmp.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_mcastmac
 *
 * Description:
 *   Given an IP address (in network order), create a IGMP multicast MAC
 *   address.
 *
 ****************************************************************************/

static void igmp_mcastmac(FAR const in_addr_t *ip, FAR uint8_t *mac)
{
  /* This mapping is from the IETF IN RFC 1700 */

  mac[0] = 0x01;
  mac[1] = 0x00;
  mac[2] = 0x5e;
  mac[3] = ip4_addr2(*ip) & 0x7f;
  mac[4] = ip4_addr3(*ip);
  mac[5] = ip4_addr4(*ip);

  ninfo("IP: %08" PRIx32 " -> MAC: %02x%02x%02x%02x%02x%02x\n",
        (uint32_t)*ip, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_addmcastmac
 *
 * Description:
 *   Add an IGMP MAC address to the device's MAC filter table.
 *
 ****************************************************************************/

void igmp_addmcastmac(FAR struct net_driver_s *dev, FAR const in_addr_t *ip)
{
  uint8_t mcastmac[6];

  ninfo("Adding: IP %08" PRIx32 "\n", (uint32_t)*ip);
  if (dev->d_addmac)
    {
      igmp_mcastmac(ip, mcastmac);
      dev->d_addmac(dev, mcastmac);
    }
}

/****************************************************************************
 * Name:  igmp_removemcastmac
 *
 * Description:
 *   Remove an IGMP MAC address from the device's MAC filter table.
 *
 ****************************************************************************/

void igmp_removemcastmac(FAR struct net_driver_s *dev,
                         FAR const in_addr_t *ip)
{
  uint8_t mcastmac[6];

  ninfo("Removing: IP %08" PRIx32 "\n", (uint32_t)*ip);
  if (dev->d_rmmac)
    {
      igmp_mcastmac(ip, mcastmac);
      dev->d_rmmac(dev, mcastmac);
    }
}

#endif /* CONFIG_NET_IGMP */
