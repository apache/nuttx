/****************************************************************************
 * net/netdev/netdev_register.c
 *
 *   Copyright (C) 2007-2012, 2014-2015 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && CONFIG_NSOCKET_DESCRIPTORS > 0

#include <sys/socket.h>
#include <stdio.h>
#include <semaphore.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <net/if.h>
#include <net/ethernet.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>

#include "utils/utils.h"
#include "igmp/igmp.h"
#include "netdev/netdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NETDEV_SLIP_FORMAT      "sl%d"
#define NETDEV_ETH_FORMAT       "eth%d"
#define NETDEV_TUN_FORMAT       "tun%d"

#if defined(CONFIG_NET_SLIP)
#  define NETDEV_DEFAULT_FORMAT NETDEV_SLIP_FORMAT
#else /* if defined(CONFIG_NET_ETHERNET) */
#  define NETDEV_DEFAULT_FORMAT NETDEV_ETH_FORMAT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Then next available device number */

#ifndef CONFIG_NET_MULTILINK
static int g_next_devnum = 0;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* List of registered Ethernet device drivers */

struct net_driver_s *g_netdevices = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: find_devnum
 *
 * Description:
 *   Given a device name format string, find the next device number for the
 *   class of device represented by that format string.
 *
 * Parameters:
 *   devfmt - The device format string
 *
 * Returned Value:
 *   The next device number for that device class
 *
 ****************************************************************************/

#ifdef CONFIG_NET_MULTILINK
static int find_devnum(FAR const char *devfmt)
{
  FAR struct net_driver_s *curr;
  size_t fmt_size;
  int result = 0;

  fmt_size = strlen(devfmt);

  /* Assumed that devfmt is xxx%d */

  DEBUGASSERT(fmt_size > 2);
  fmt_size -= 2;

  /* Search the list of currently registered network devices */

  for (curr = g_netdevices; curr; curr = curr->flink )
    {
      /* Does this device name match the format we were given? */

      if (strncmp(curr->d_ifname, devfmt, fmt_size) == 0)
        {
          /* Yes.. increment the candidate device number */

          result++;
        }
    }

  /* Return this next device number for this format */

  return result;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: netdev_register
 *
 * Description:
 *   Register a network device driver and assign a name to it so that it can
 *   be found in subsequent network ioctl operations on the device.
 *
 * Parameters:
 *   dev    - The device driver structure to be registered.
 *   lltype - Link level protocol used by the driver (Ethernet, SLIP, PPP, ...
 *              ...
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 * Assumptions:
 *   Called during system initialization from normal user mode
 *
 ****************************************************************************/

int netdev_register(FAR struct net_driver_s *dev, enum net_lltype_e lltype)
{
  FAR const char *devfmt;
#ifdef CONFIG_NET_USER_DEVFMT
  FAR const char devfmt_str[IFNAMSIZ];
#endif
  net_lock_t save;
  int devnum;

  if (dev)
    {
#ifdef CONFIG_NET_MULTILINK
      /* We are supporting multiple network devices and using different link
       * level protocols.  Set the protocol usd by the device and the size
       * level protocols.  Set the protocol used by the device and the size
       * of the link header used by this protocol.
       */

      switch (lltype)
        {
#ifdef CONFIG_NET_ETHERNET
          case NET_LL_ETHERNET:  /* Ethernet */
            dev->d_llhdrlen = ETH_HDRLEN;
            dev->d_mtu      = CONFIG_NET_ETH_MTU;
#ifdef CONFIG_NET_TCP
            dev->d_recvwndo = CONFIG_NET_ETH_TCP_RECVWNDO;
#endif
            devfmt          = NETDEV_ETH_FORMAT;
            break;
#endif

#ifdef CONFIG_NET_SLIP
          case NET_LL_SLIP:      /* Serial Line Internet Protocol (SLIP) */
            dev->d_llhdrlen = 0;
            dev->d_mtu      = CONFIG_NET_SLIP_MTU;
#ifdef CONFIG_NET_TCP
            dev->d_recvwndo = CONFIG_NET_SLIP_TCP_RECVWNDO;
#endif
            devfmt          = NETDEV_SLIP_FORMAT;
            break;
#endif

#ifdef CONFIG_NET_TUN
          case NET_LL_TUN:       /* Virtual Network Device (TUN) */
            dev->d_llhdrlen = 0;
            dev->d_mtu      = CONFIG_NET_TUN_MTU;
#ifdef CONFIG_NET_TCP
            dev->d_recvwndo = CONFIG_NET_TUN_TCP_RECVWNDO;
#endif
            devfmt          = NETDEV_TUN_FORMAT;
            break;
#endif

#if 0                            /* REVISIT: Not yet supported */
          case NET_LL_PPP:       /* Point-to-Point Protocol (PPP) */
            dev->d_llhdrlen = 0;
            dev->d_mtu      = CONFIG_NET_PPP_MTU;
#ifdef CONFIG_NET_TCP
            dev->d_recvwndo = CONFIG_NET_PPP_TCP_RECVWNDO;
#endif
            devfmt          = NETDEV_PPP_FORMAT;
            break;
#endif

          default:
            nlldbg("ERROR: Unrecognized link type: %d\n", lltype);
            return -EINVAL;
        }

      /* Remember the verified link type */

      dev->d_lltype = (uint8_t)lltype;

#else
     /* Use the default device name */

     devfmt = NETDEV_DEFAULT_FORMAT;
#endif

      /* There are no clients of the device yet */

      dev->d_conncb = NULL;
      dev->d_devcb = NULL;

      /* Get the next available device number and sssign a device name to
       * the interface
       */

      save = net_lock();
#ifdef CONFIG_NET_MULTILINK
      devnum = find_devnum(devfmt);
#else
      devnum = g_next_devnum++;
#endif
#ifdef CONFIG_NET_USER_DEVFMT
      if (*dev->d_ifname)
        {
          strncpy(devfmt_str, dev->d_ifname, IFNAMSIZ);
          devfmt = devfmt_str;
        }
#endif

      snprintf(dev->d_ifname, IFNAMSIZ, devfmt, devnum );

      /* Add the device to the list of known network devices */

      dev->flink  = g_netdevices;
      g_netdevices = dev;

      /* Configure the device for IGMP support */

#ifdef CONFIG_NET_IGMP
      igmp_devinit(dev);
#endif
      net_unlock(save);

#ifdef CONFIG_NET_ETHERNET
      nlldbg("Registered MAC: %02x:%02x:%02x:%02x:%02x:%02x as dev: %s\n",
             dev->d_mac.ether_addr_octet[0], dev->d_mac.ether_addr_octet[1],
             dev->d_mac.ether_addr_octet[2], dev->d_mac.ether_addr_octet[3],
             dev->d_mac.ether_addr_octet[4], dev->d_mac.ether_addr_octet[5],
             dev->d_ifname);
#else
      nlldbg("Registered dev: %s\n", dev->d_ifname);
#endif
      return OK;
    }

  return -EINVAL;
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
