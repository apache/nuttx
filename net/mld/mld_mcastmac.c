/****************************************************************************
 * net/mld/mld_mcastmac.c
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
 * 3. Neither the name of CITEL Technologies Ltd nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CITEL TECHNOLOGIES AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL CITEL TECHNOLOGIES OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "mld/mld.h"

#ifdef CONFIG_NET_MLD

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_mcastmac
 *
 * Description:
 *   Given an IP address (in network order), create a MLD multicast MAC
 *   address.
 *
 ****************************************************************************/

static void mld_mcastmac(FAR const net_ipv6addr_t ipaddr, FAR uint8_t *mac)
{
  FAR uint8_t *ipaddr8 = (FAR uint8_t *)ipaddr;

  mldinfo("Mcast IP: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
          ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3],
          ipaddr[4], ipaddr[5], ipaddr[6], ipaddr[7]);

  /* For the MAC address by insert the low 32 Bits of the multicast IPv6
   * Address into the Ethernet Address .  This mapping is from the IETF
   * RFC 7042.
   */

  mac[0] = 0x33;
  mac[1] = 0x33;
  mac[2] = ipaddr8[12];  /* Bits: 96-103 */
  mac[3] = ipaddr8[13];  /* Bits: 104-111 */
  mac[4] = ipaddr8[14];  /* Bits: 112-119 */
  mac[5] = ipaddr8[15];  /* Bits: 120-127 */

  mldinfo("Mcast MAC: %02x:%02x:%02x:%02x:%02x:%02x\n",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_addmcastmac
 *
 * Description:
 *   Add an MLD MAC address to the device's MAC filter table.
 *
 ****************************************************************************/

void mld_addmcastmac(FAR struct net_driver_s *dev,
                     FAR const net_ipv6addr_t ipaddr)
{
  uint8_t mcastmac[6];

  mldinfo("Adding MAC address filter\n");

  if (dev->d_addmac != NULL)
    {
      mld_mcastmac(ipaddr, mcastmac);
      dev->d_addmac(dev, mcastmac);
    }
}

/****************************************************************************
 * Name:  mld_removemcastmac
 *
 * Description:
 *   Remove an MLD MAC address from the device's MAC filter table.
 *
 ****************************************************************************/

void mld_removemcastmac(FAR struct net_driver_s *dev,
                        FAR const net_ipv6addr_t ipaddr)
{
  uint8_t mcastmac[6];

  mldinfo("Removing MAC address filter\n");

  if (dev->d_rmmac != NULL)
    {
      mld_mcastmac(ipaddr, mcastmac);
      dev->d_rmmac(dev, mcastmac);
    }
}

#endif /* CONFIG_NET_MLD */
