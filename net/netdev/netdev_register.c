/****************************************************************************
 * net/netdev/netdev_register.c
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

#include <sys/socket.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <net/if.h>
#include <net/ethernet.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/net/can.h>

#include "utils/utils.h"
#include "igmp/igmp.h"
#include "mld/mld.h"
#include "netdev/netdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NETDEV_ETH_FORMAT   "eth%d"
#define NETDEV_LO_FORMAT    "lo"
#define NETDEV_SLIP_FORMAT  "sl%d"
#define NETDEV_TUN_FORMAT   "tun%d"
#define NETDEV_BNEP_FORMAT  "bnep%d"
#define NETDEV_PAN_FORMAT   "pan%d"
#define NETDEV_WLAN_FORMAT  "wlan%d"
#define NETDEV_WPAN_FORMAT  "wpan%d"
#define NETDEV_WWAN_FORMAT  "wwan%d"
#define NETDEV_CAN_FORMAT   "can%d"

#if defined(CONFIG_DRIVERS_IEEE80211) /* Usually also has CONFIG_NET_ETHERNET */
#  define NETDEV_DEFAULT_FORMAT NETDEV_WLAN_FORMAT
#elif defined(CONFIG_NET_ETHERNET)
#  define NETDEV_DEFAULT_FORMAT NETDEV_ETH_FORMAT
#elif defined(CONFIG_NET_6LOWPAN)
#  define NETDEV_DEFAULT_FORMAT NETDEV_WPAN_FORMAT
#elif defined(CONFIG_NET_SLIP)
#  define NETDEV_DEFAULT_FORMAT NETDEV_SLIP_FORMAT
#elif defined(CONFIG_NET_TUN)
#  define NETDEV_DEFAULT_FORMAT NETDEV_TUN_FORMAT
#elif defined(CONFIG_NET_CAN)
#  define NETDEV_DEFAULT_FORMAT NETDEV_CAN_FORMAT
#else /* if defined(CONFIG_NET_LOOPBACK) */
#  define NETDEV_DEFAULT_FORMAT NETDEV_LO_FORMAT
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* List of registered Ethernet device drivers */

struct net_driver_s *g_netdevices = NULL;

#ifdef CONFIG_NETDEV_IFINDEX
/* The set of network devices that have been registered.  This is used to
 * assign a unique device index to the newly registered device.
 *
 * REVISIT:  The width of g_nassigned limits the number of registered
 * devices to 32 (MAX_IFINDEX).
 */

uint32_t g_devset;

/* The set of network devices that have been freed.  The purpose of this
 * set is to postpone reuse of a interface index for as long as possible,
 * i.e., don't reuse an interface index until all of the possible indices
 * have been used.
 */

uint32_t g_devfreed;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: find_devnum
 *
 * Description:
 *   Given a device name format string, find the next device number for the
 *   class of device represented by that format string.
 *
 * Input Parameters:
 *   devfmt - The device format string
 *
 * Returned Value:
 *   The next device number for that device class
 *
 ****************************************************************************/

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

/****************************************************************************
 * Name: get_ifindex
 *
 * Description:
 *   Assign a unique interface index to the device.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The interface index assigned to the device.  -ENOSPC is returned if
 *   more the MAX_IFINDEX names have been assigned.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IFINDEX
static int get_ifindex(void)
{
  uint32_t devset;
  int ndx;

  /* Try to postpone re-using interface indices as long as possible */

  devset = g_devset | g_devfreed;
  if (devset == 0xffffffff)
    {
      /* Time start re-using interface indices */

      devset     = g_devset;
      g_devfreed = 0;
    }

  /* Search for an unused index */

  for (ndx = 0; ndx < MAX_IFINDEX; ndx++)
    {
      uint32_t bit = 1UL << ndx;
      if ((devset & bit) == 0)
        {
          /* Indicate that this index is in use */

          g_devset |= bit;

          /* NOTE that the index + 1 is returned.  Zero is reserved to
           * mean no-index in the POSIX standards.
           */

          return ndx + 1;
        }
    }

  return -ENOSPC;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_register
 *
 * Description:
 *   Register a network device driver and assign a name to it so that it can
 *   be found in subsequent network ioctl operations on the device.
 *
 *   A custom, device-specific interface name format string may be selected
 *   by putting that format string into the device structure's d_ifname[]
 *   array before calling netdev_register().  Otherwise, the d_ifname[] must
 *   be zeroed on entry.
 *
 * Input Parameters:
 *   dev    - The device driver structure to be registered.
 *   lltype - Link level protocol used by the driver (Ethernet, SLIP, TUN,
 *            ...)
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 * Assumptions:
 *   Called during system bring-up, but also when a removable network
 *   device is installed.
 *
 ****************************************************************************/

int netdev_register(FAR struct net_driver_s *dev, enum net_lltype_e lltype)
{
  FAR struct net_driver_s **last;
  FAR char devfmt_str[IFNAMSIZ];
  FAR const char *devfmt;
  uint16_t pktsize = 0;
  uint8_t llhdrlen = 0;
  int devnum;
#ifdef CONFIG_NETDEV_IFINDEX
  int ifindex;
#endif

  if (dev != NULL)
    {
      /* Set the protocol used by the device and the size of the link
       * header used by this protocol.
       */

      switch (lltype)
        {
#ifdef CONFIG_NET_LOOPBACK
          case NET_LL_LOOPBACK:   /* Local loopback */
            llhdrlen = 0;
            pktsize  = NET_LO_PKTSIZE;
            devfmt   = NETDEV_LO_FORMAT;
            break;
#endif

#ifdef CONFIG_NET_ETHERNET
          case NET_LL_ETHERNET:   /* Ethernet */
            llhdrlen = ETH_HDRLEN;
            pktsize  = CONFIG_NET_ETH_PKTSIZE;
            devfmt   = NETDEV_ETH_FORMAT;
            break;
#endif

#ifdef CONFIG_DRIVERS_IEEE80211
          case NET_LL_IEEE80211:  /* IEEE 802.11 */
            llhdrlen = ETH_HDRLEN;
            pktsize  = CONFIG_NET_ETH_PKTSIZE;
            devfmt   = NETDEV_WLAN_FORMAT;
            break;
#endif

#ifdef CONFIG_NET_CAN
          case NET_LL_CAN:  /* CAN bus */
            dev->d_llhdrlen = 0;
            dev->d_pktsize  = NET_CAN_PKTSIZE;
            devfmt          = NETDEV_CAN_FORMAT;
            break;
#endif

#ifdef CONFIG_NET_BLUETOOTH
          case NET_LL_BLUETOOTH:              /* Bluetooth */
            llhdrlen = BLUETOOTH_MAX_HDRLEN;  /* Determined at runtime */
#ifdef CONFIG_NET_6LOWPAN
            pktsize  = CONFIG_NET_6LOWPAN_PKTSIZE;
#endif
            devfmt   = NETDEV_BNEP_FORMAT;
            break;
#endif

#if defined(CONFIG_NET_6LOWPAN) || defined(CONFIG_NET_IEEE802154)
          case NET_LL_IEEE802154: /* IEEE 802.15.4 MAC */
          case NET_LL_PKTRADIO:   /* Non-IEEE 802.15.4 packet radio */
            llhdrlen = 0;         /* Determined at runtime */
#ifdef CONFIG_NET_6LOWPAN
            pktsize  = CONFIG_NET_6LOWPAN_PKTSIZE;
#endif
            devfmt   = NETDEV_WPAN_FORMAT;
            break;
#endif

#ifdef CONFIG_NET_SLIP
          case NET_LL_SLIP:       /* Serial Line Internet Protocol (SLIP) */
            llhdrlen = 0;
            pktsize  = CONFIG_NET_SLIP_PKTSIZE;
            devfmt   = NETDEV_SLIP_FORMAT;
            break;
#endif

#ifdef CONFIG_NET_TUN
          case NET_LL_TUN:        /* Virtual Network Device (TUN) */
            llhdrlen = 0;         /* This will be overwritten by tun_ioctl
                                   * if used as a TAP (layer 2) device */
            pktsize  = CONFIG_NET_TUN_PKTSIZE;
            devfmt   = NETDEV_TUN_FORMAT;
            break;
#endif

#ifdef CONFIG_NET_MBIM
          case NET_LL_MBIM:
            llhdrlen = 0;
            pktsize  = 1200;
            devfmt   = NETDEV_WWAN_FORMAT;
            break;
#endif

          default:
            nerr("ERROR: Unrecognized link type: %d\n", lltype);
            return -EINVAL;
        }

      /* Update the package length.  A network driver may provide custom
       * values for MAC header length and for the maximum packet size.  Some
       * driver implementations (for example, the simulator) will require
       * dynamic MTU calculations to support tunneling (as an example).
       */

      if (dev->d_llhdrlen == 0)
        {
          dev->d_llhdrlen = llhdrlen;
        }

      if (dev->d_pktsize == 0)
        {
          dev->d_pktsize = pktsize;
        }

      /* Remember the verified link type */

      dev->d_lltype = (uint8_t)lltype;

      /* There are no clients of the device yet */

      dev->d_conncb = NULL;
      dev->d_conncb_tail = NULL;
      dev->d_devcb = NULL;

      /* We need exclusive access for the following operations */

      net_lock();

#ifdef CONFIG_NETDEV_IFINDEX
      ifindex = get_ifindex();
      if (ifindex < 0)
        {
          net_unlock();
          return ifindex;
        }

      dev->d_ifindex = (uint8_t)ifindex;
#endif

      /* Get the next available device number and assign a device name to
       * the interface
       */

      /* Check if the caller has provided a user device-specific interface
       * name format string (in d_ifname).
       */

      if (dev->d_ifname[0] != '\0')
        {
          /* Copy the string in a temporary buffer.  How do we know that the
           * string is valid and not just uninitialized memory?  We don't.
           * Let's at least make certain that the format string is NUL
           * terminated.
           */

          dev->d_ifname[IFNAMSIZ - 1] = '\0';
          strlcpy(devfmt_str, dev->d_ifname, IFNAMSIZ);

          /* Then use the content of the temporary buffer as the format
           * string.
           */

          devfmt = (FAR const char *)devfmt_str;
        }

#ifdef CONFIG_NET_LOOPBACK
      /* The local loopback device is a special case:  There can be only one
       * local loopback device so it is unnumbered.
       */

      if (lltype == NET_LL_LOOPBACK)
        {
          devnum = 0;
        }
      else
#endif
        {
          devnum = find_devnum(devfmt);
        }

      /* Complete the device name by including the device number (if
       * included in the format).
       */

      snprintf(dev->d_ifname, IFNAMSIZ, devfmt, devnum);

      /* Add the device to the list of known network devices */

      last = &g_netdevices;
      while (*last)
        {
          last = &((*last)->flink);
        }

      *last = dev;

      dev->flink = NULL;

#ifdef CONFIG_NET_IGMP
      /* Configure the device for IGMP support */

      igmp_devinit(dev);
#endif

#ifdef CONFIG_NET_MLD
      /* Configure the device for MLD support */

      mld_devinit(dev);
#endif

      net_unlock();

#if defined(CONFIG_NET_ETHERNET) || defined(CONFIG_DRIVERS_IEEE80211)
      ninfo("Registered MAC: %02x:%02x:%02x:%02x:%02x:%02x as dev: %s\n",
            dev->d_mac.ether.ether_addr_octet[0],
            dev->d_mac.ether.ether_addr_octet[1],
            dev->d_mac.ether.ether_addr_octet[2],
            dev->d_mac.ether.ether_addr_octet[3],
            dev->d_mac.ether.ether_addr_octet[4],
            dev->d_mac.ether.ether_addr_octet[5],
            dev->d_ifname);
#else
      ninfo("Registered dev: %s\n", dev->d_ifname);
#endif
      return OK;
    }

  return -EINVAL;
}
