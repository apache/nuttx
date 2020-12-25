/****************************************************************************
 * net/netdev/netdev_ioctl.c
 *
 *   Copyright (C) 2007-2012, 2015-2019 Gregory Nutt. All rights reserved.
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

#include <sys/socket.h>
#include <sys/ioctl.h>

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>

#include <net/if.h>
#include <net/route.h>
#include <net/ethernet.h>
#include <netinet/in.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/arp.h>

#ifdef CONFIG_NET_6LOWPAN
#  include <nuttx/net/sixlowpan.h>
#endif

#include <sys/sockio.h>
#include <nuttx/net/igmp.h>

#ifdef CONFIG_NETDEV_WIRELESS_IOCTL
#  include <nuttx/wireless/wireless.h>
#endif

#ifdef CONFIG_WIRELESS_BLUETOOTH
#  include <nuttx/wireless/bluetooth/bt_ioctl.h>
#endif

#ifdef CONFIG_WIRELESS_IEEE802154
#  include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#endif

#ifdef CONFIG_WIRELESS_PKTRADIO
#  include <nuttx/wireless/pktradio.h>
#endif

#include "arp/arp.h"
#include "socket/socket.h"
#include "netdev/netdev.h"
#include "devif/devif.h"
#include "igmp/igmp.h"
#include "icmpv6/icmpv6.h"
#include "route/route.h"
#include "netlink/netlink.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#undef HAVE_WRITABLE_IPv4ROUTE
#undef HAVE_WRITABLE_IPv6ROUTE

#ifdef CONFIG_NET_ROUTE
#  if defined(CONFIG_NET_IPv4) && !defined(CONFIG_ROUTE_IPv4_ROMROUTE)
#    define HAVE_WRITABLE_IPv4ROUTE 1
#  endif

#  if defined(CONFIG_NET_IPv6) && !defined(CONFIG_ROUTE_IPv6_ROMROUTE)
#    define HAVE_WRITABLE_IPv6ROUTE 1
#  endif
#endif

#undef HAVE_IEEE802154_IOCTL
#undef HAVE_PKTRADIO_IOCTL
#undef HAVE_BLUETOOTH_IOCTL

#ifdef CONFIG_NETDEV_IOCTL
/* IEEE 802.15.4 6LoWPAN or raw packet support */

#if defined(CONFIG_NET_IEEE802154) || (defined(CONFIG_NET_6LOWPAN) && \
    defined(CONFIG_WIRELESS_IEEE802154))
#  define HAVE_IEEE802154_IOCTL 1
#endif

/* pktradio raw packet support not implemented */

#if defined(CONFIG_NET_6LOWPAN) && defined(CONFIG_WIRELESS_PKTRADIO)
#  define HAVE_PKTRADIO_IOCTL 1
#endif

/* Bluetooth 6LoWPAN support not implemented */

#if defined(CONFIG_NET_BLUETOOTH)
#  define HAVE_BLUETOOTH_IOCTL 1
#endif
#endif /* CONFIG_NETDEV_IOCTL */

/* This is really kind of bogus.. When asked for an IP address, this is
 * family that is returned in the ifr structure.  Probably could just skip
 * this since the address family has nothing to do with the Ethernet address.
 */

#ifdef CONFIG_NET_IPv6
#  define AF_INETX AF_INET6
#else
#  define AF_INETX AF_INET
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ioctl_add_ipv4route
 *
 * Description:
 *   Add an IPv4 route to the routing table.
 *
 * Input Parameters:
 *   rentry - Describes the route to be added
 *
 ****************************************************************************/

#ifdef HAVE_WRITABLE_IPv4ROUTE
static int ioctl_add_ipv4route(FAR struct rtentry *rtentry)
{
  FAR struct sockaddr_in *addr;
  in_addr_t target;
  in_addr_t netmask;
  in_addr_t router;

  addr    = (FAR struct sockaddr_in *)&rtentry->rt_dst;
  target  = (in_addr_t)addr->sin_addr.s_addr;

  addr    = (FAR struct sockaddr_in *)&rtentry->rt_genmask;
  netmask = (in_addr_t)addr->sin_addr.s_addr;

  addr    = (FAR struct sockaddr_in *)&rtentry->rt_gateway;
  router  = (in_addr_t)addr->sin_addr.s_addr;

  return net_addroute_ipv4(target, netmask, router);
}
#endif /* HAVE_WRITABLE_IPv4ROUTE */

/****************************************************************************
 * Name: ioctl_add_ipv6route
 *
 * Description:
 *   Add an IPv6 route to the routing table.
 *
 * Input Parameters:
 *   rentry - Describes the route to be added
 *
 ****************************************************************************/

#ifdef HAVE_WRITABLE_IPv6ROUTE
static int ioctl_add_ipv6route(FAR struct rtentry *rtentry)
{
  FAR struct sockaddr_in6 *target;
  FAR struct sockaddr_in6 *netmask;
  FAR struct sockaddr_in6 *gateway;
  net_ipv6addr_t router;

  target  = (FAR struct sockaddr_in6 *)&rtentry->rt_dst;
  netmask = (FAR struct sockaddr_in6 *)&rtentry->rt_genmask;

  /* The router is an optional argument */

  gateway = (FAR struct sockaddr_in6 *)&rtentry->rt_gateway;
  net_ipv6addr_copy(router, gateway->sin6_addr.s6_addr16);

  return net_addroute_ipv6(target->sin6_addr.s6_addr16,
                           netmask->sin6_addr.s6_addr16, router);
}
#endif /* HAVE_WRITABLE_IPv6ROUTE */

/****************************************************************************
 * Name: ioctl_del_ipv4route
 *
 * Description:
 *   Delete an IPv4 route to the routing table.
 *
 * Input Parameters:
 *   rentry - Describes the route to be deleted
 *
 ****************************************************************************/

#ifdef HAVE_WRITABLE_IPv4ROUTE
static int ioctl_del_ipv4route(FAR struct rtentry *rtentry)
{
  FAR struct sockaddr_in *addr;
  in_addr_t target;
  in_addr_t netmask;

  addr    = (FAR struct sockaddr_in *)&rtentry->rt_dst;
  target  = (in_addr_t)addr->sin_addr.s_addr;

  addr    = (FAR struct sockaddr_in *)&rtentry->rt_genmask;
  netmask = (in_addr_t)addr->sin_addr.s_addr;

  return net_delroute_ipv4(target, netmask);
}
#endif /* HAVE_WRITABLE_IPv4ROUTE */

/****************************************************************************
 * Name: ioctl_del_ipv6route
 *
 * Description:
 *   Delete an IPv6 route to the routing table.
 *
 * Input Parameters:
 *   rentry - Describes the route to be deleted
 *
 ****************************************************************************/

#ifdef HAVE_WRITABLE_IPv6ROUTE
static int ioctl_del_ipv6route(FAR struct rtentry *rtentry)
{
  FAR struct sockaddr_in6 *target;
  FAR struct sockaddr_in6 *netmask;

  target  = (FAR struct sockaddr_in6 *)&rtentry->rt_dst;
  netmask = (FAR struct sockaddr_in6 *)&rtentry->rt_genmask;

  return net_delroute_ipv6(target->sin6_addr.s6_addr16,
                           netmask->sin6_addr.s6_addr16);
}
#endif /* HAVE_WRITABLE_IPv6ROUTE */

/****************************************************************************
 * Name: ioctl_get_ipv4addr
 *
 * Description:
 *   Copy IP addresses from device structure to user memory.
 *
 * Input Parameters:
 *   outaddr - Pointer to the user-provided memory to receive the address.
 *   inaddr - The source IP address in the device structure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static void ioctl_get_ipv4addr(FAR struct sockaddr *outaddr,
                               in_addr_t inaddr)
{
  FAR struct sockaddr_in *dest  = (FAR struct sockaddr_in *)outaddr;
  dest->sin_family              = AF_INET;
  dest->sin_port                = 0;
  dest->sin_addr.s_addr         = inaddr;
  memset(dest->sin_zero, 0, sizeof(dest->sin_zero));
}
#endif

/****************************************************************************
 * Name: ioctl_get_ipv4broadcast
 *
 * Description:
 *   Return the sub-net broadcast address to user memory.
 *
 * Input Parameters:
 *   outaddr - Pointer to the user-provided memory to receive the address.
 *   inaddr  - The source IP address in the device structure.
 *   netmask  - The netmask address mask in the device structure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static void ioctl_get_ipv4broadcast(FAR struct sockaddr *outaddr,
                                    in_addr_t inaddr, in_addr_t netmask)
{
  FAR struct sockaddr_in *dest  = (FAR struct sockaddr_in *)outaddr;
  dest->sin_family              = AF_INET;
  dest->sin_port                = 0;
  dest->sin_addr.s_addr         = net_ipv4addr_broadcast(inaddr, netmask);
  memset(dest->sin_zero, 0, sizeof(dest->sin_zero));
}
#endif

/****************************************************************************
 * Name: ioctl_get_ipv6addr
 *
 * Description:
 *   Copy IP addresses from device structure to user memory.
 *
 * Input Parameters:
 *   outaddr - Pointer to the user-provided memory to receive the address.
 *   inaddr - The source IP address in the device structure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static void ioctl_get_ipv6addr(FAR struct sockaddr_storage *outaddr,
                               FAR const net_ipv6addr_t inaddr)
{
  FAR struct sockaddr_in6 *dest = (FAR struct sockaddr_in6 *)outaddr;
  dest->sin6_family             = AF_INET6;
  dest->sin6_port               = 0;
  memcpy(dest->sin6_addr.in6_u.u6_addr8, inaddr, 16);
}
#endif

/****************************************************************************
 * Name: ioctl_set_ipv4addr
 *
 * Description:
 *   Copy IP addresses from user memory into the device structure
 *
 * Input Parameters:
 *   outaddr - Pointer to the source IP address in the device structure.
 *   inaddr - Pointer to the user-provided memory to containing the new IP
 *     address.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
static void ioctl_set_ipv4addr(FAR in_addr_t *outaddr,
                               FAR const struct sockaddr *inaddr)
{
  FAR const struct sockaddr_in *src = (FAR const struct sockaddr_in *)inaddr;
  *outaddr = src->sin_addr.s_addr;
}
#endif

/****************************************************************************
 * Name: ioctl_set_ipv6addr
 *
 * Description:
 *   Copy IP addresses from user memory into the device structure
 *
 * Input Parameters:
 *   outaddr - Pointer to the source IP address in the device structure.
 *   inaddr  - Pointer to the user-provided memory to containing the new IP
 *     address.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static void ioctl_set_ipv6addr(FAR net_ipv6addr_t outaddr,
                               FAR const struct sockaddr_storage *inaddr)
{
  FAR const struct sockaddr_in6 *src =
    (FAR const struct sockaddr_in6 *)inaddr;
  memcpy(outaddr, src->sin6_addr.in6_u.u6_addr8, 16);
}
#endif

/****************************************************************************
 * Name: netdev_bluetooth_ioctl
 *
 * Description:
 *   Perform Bluetooth network device specific operations.
 *
 * Input Parameters:
 *   psock  - Socket structure
 *   dev    - Ethernet driver device structure
 *   cmd    - The ioctl command
 *   req    - The argument of the ioctl cmd
 *
 * Returned Value:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef HAVE_BLUETOOTH_IOCTL
static int netdev_bluetooth_ioctl(FAR struct socket *psock, int cmd,
                                  unsigned long arg)
{
  FAR struct net_driver_s *dev;
  FAR char *ifname;
  int ret = -EINVAL;

  if (arg != 0ul)
    {
      if (WL_IBLUETOOTHCMD(cmd))
        {
          /* Get the name of the Bluetooth device to receive the IOCTL
           * command
           */

          FAR struct btreq_s *btreq =
            (FAR struct btreq_s *)((uintptr_t)arg);

          ifname = btreq->btr_name;
        }
      else
        {
          /* Not a Bluetooth IOCTL command */

          return -ENOTTY;
        }

      /* Find the device with this name */

      dev = netdev_findbyname(ifname);
      ret = -ENODEV;

      if (dev != NULL && dev->d_lltype == NET_LL_BLUETOOTH)
        {
          /* Perform the device IOCTL */

          ret = dev->d_ioctl(dev, cmd, arg);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: netdev_iee802154_ioctl
 *
 * Description:
 *   Perform IEEE802.15.4 network device specific operations.
 *
 * Input Parameters:
 *   psock  - Socket structure
 *   dev    - Ethernet driver device structure
 *   cmd    - The ioctl command
 *   req    - The argument of the ioctl cmd
 *
 * Returned Value:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef HAVE_IEEE802154_IOCTL
static int netdev_iee802154_ioctl(FAR struct socket *psock, int cmd,
                                  unsigned long arg)
{
  FAR struct net_driver_s *dev;
  FAR char *ifname;
  int ret = -ENOTTY;

  if (arg != 0ul)
    {
      if (_MAC802154IOCVALID(cmd))
        {
          /* Get the IEEE802.15.4 MAC device to receive the radio IOCTL
           * command
           */

          FAR struct ieee802154_netmac_s *netmac =
            (FAR struct ieee802154_netmac_s *)((uintptr_t)arg);

          ifname = netmac->ifr_name;
        }
      else
        {
          /* Not an EEE802.15.4 MAC IOCTL command */

          return -ENOTTY;
        }

      /* Find the device with this name */

      dev = netdev_findbyname(ifname);
      if (dev != NULL && dev->d_lltype == NET_LL_IEEE802154)
        {
          /* Perform the device IOCTL */

          ret = dev->d_ioctl(dev, cmd, arg);
        }
    }

  return ret;
}
#endif /* HAVE_IEEE802154_IOCTL */

/****************************************************************************
 * Name: netdev_pktradio_ioctl
 *
 * Description:
 *   Perform non-IEEE802.15.4 packet radio network device specific operation.
 *
 * Input Parameters:
 *   psock  - Socket structure
 *   dev    - Ethernet driver device structure
 *   cmd    - The ioctl command
 *   req    - The argument of the ioctl cmd
 *
 * Returned Value:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef HAVE_PKTRADIO_IOCTL
static int netdev_pktradio_ioctl(FAR struct socket *psock, int cmd,
                                 unsigned long arg)
{
  FAR struct net_driver_s *dev;
  FAR char *ifname;
  int ret = -ENOTTY;

  if (arg != 0ul)
    {
      if (WL_ISPKTRADIOCMD(cmd))
        {
          /* Get the packet radio device to receive the radio IOCTL
           * command
           */

          FAR struct pktradio_ifreq_s *cmddata =
            (FAR struct pktradio_ifreq_s *)((uintptr_t)arg);

          ifname = cmddata->pifr_name;
        }
      else
        {
          /* Not a packet radio IOCTL command */

          nwarn("WARNING: Not a packet radio IOCTL command: %d\n", cmd);
          return -ENOTTY;
        }

      /* Find the device with this name */

      dev = netdev_findbyname(ifname);
      if (dev != NULL && dev->d_lltype == NET_LL_PKTRADIO)
        {
          /* Perform the device IOCTL */

          ret = dev->d_ioctl(dev, cmd, arg);
        }
    }

  return ret;
}
#endif /* HAVE_PKTRADIO_IOCTL */

/****************************************************************************
 * Name: netdev_wifr_ioctl
 *
 * Description:
 *   Perform wireless network device specific operations.
 *
 * Input Parameters:
 *   psock    Socket structure
 *   dev      Ethernet driver device structure
 *   cmd      The ioctl command
 *   req      The argument of the ioctl cmd
 *
 * Returned Value:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#if defined(CONFIG_NETDEV_IOCTL) && defined(CONFIG_NETDEV_WIRELESS_IOCTL)
static int netdev_wifr_ioctl(FAR struct socket *psock, int cmd,
                             FAR struct iwreq *req)
{
  FAR struct net_driver_s *dev;
  int ret = -ENOTTY;

  /* Verify that this is a valid wireless network IOCTL command */

  if (_WLIOCVALID(cmd) && (unsigned)_IOC_NR(cmd) <= WL_NNETCMDS)
    {
      /* Get the wireless device associated with the IOCTL command */

      dev = netdev_findbyname(req->ifr_name);
      if (dev != NULL)
        {
          /* Just forward the IOCTL to the wireless driver */

          ret = dev->d_ioctl(dev, cmd, (unsigned long)(uintptr_t)req);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: netdev_ifr_dev
 *
 * Description:
 *   Verify the struct ifreq and get the Ethernet device.
 *
 * Input Parameters:
 *   req - The argument of the ioctl cmd
 *
 * Returned Value:
 *  A pointer to the driver structure on success; NULL on failure.
 *
 ****************************************************************************/

static FAR struct net_driver_s *netdev_ifr_dev(FAR struct ifreq *req)
{
  if (req != NULL)
    {
      /* Find the network device associated with the device name
       * in the request data.
       */

      return netdev_findbyname(req->ifr_name);
    }

  return NULL;
}

/****************************************************************************
 * Name: netdev_ifr_ioctl
 *
 * Description:
 *   Perform network device specific operations.
 *
 * Input Parameters:
 *   psock    Socket structure
 *   cmd      The ioctl command
 *   req      The argument of the ioctl cmd
 *
 * Returned Value:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

static int netdev_ifr_ioctl(FAR struct socket *psock, int cmd,
                            FAR struct ifreq *req)
{
  FAR struct net_driver_s *dev;
  int ret = -EINVAL;

  ninfo("cmd: %d\n", cmd);

  /* Execute the command */

  switch (cmd)
    {
#ifdef CONFIG_NET_IPv4
      case SIOCGIFADDR:  /* Get IP address */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              ioctl_get_ipv4addr(&req->ifr_addr, dev->d_ipaddr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCSIFADDR:  /* Set IP address */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              ioctl_set_ipv4addr(&dev->d_ipaddr, &req->ifr_addr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCGIFDSTADDR:  /* Get P-to-P address */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              ioctl_get_ipv4addr(&req->ifr_dstaddr, dev->d_draddr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCSIFDSTADDR:  /* Set P-to-P address */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              ioctl_set_ipv4addr(&dev->d_draddr, &req->ifr_dstaddr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCGIFBRDADDR:  /* Get broadcast IP address */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              ioctl_get_ipv4broadcast(&req->ifr_broadaddr, dev->d_ipaddr,
                                      dev->d_netmask);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCSIFBRDADDR:  /* Set broadcast IP address */
        {
          ret = -ENOSYS;
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCGIFNETMASK:  /* Get network mask */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              ioctl_get_ipv4addr(&req->ifr_addr, dev->d_netmask);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCSIFNETMASK:  /* Set network mask */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              ioctl_set_ipv4addr(&dev->d_netmask, &req->ifr_addr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCGLIFADDR:  /* Get IP address */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;

              ioctl_get_ipv6addr(&lreq->lifr_addr, dev->d_ipv6addr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCSLIFADDR:  /* Set IP address */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;

              ioctl_set_ipv6addr(dev->d_ipv6addr, &lreq->lifr_addr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCGLIFDSTADDR:  /* Get P-to-P address */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;

              ioctl_get_ipv6addr(&lreq->lifr_dstaddr, dev->d_ipv6draddr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCSLIFDSTADDR:  /* Set P-to-P address */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;

              ioctl_set_ipv6addr(dev->d_ipv6draddr, &lreq->lifr_dstaddr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCGLIFBRDADDR:  /* Get broadcast IP address */
      case SIOCSLIFBRDADDR:  /* Set broadcast IP address */
        {
          ret = -ENOSYS;
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCGLIFNETMASK:  /* Get network mask */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;

              ioctl_get_ipv6addr(&lreq->lifr_addr, dev->d_ipv6netmask);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCSLIFNETMASK:  /* Set network mask */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;
              ioctl_set_ipv6addr(dev->d_ipv6netmask, &lreq->lifr_addr);
              ret = OK;
            }
        }
        break;
#endif

      case SIOCGLIFMTU:  /* Get MTU size */
      case SIOCGIFMTU:   /* Get MTU size */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              req->ifr_mtu = NETDEV_PKTSIZE(dev);
              ret = OK;
            }
        }
        break;

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
      case SIOCIFAUTOCONF:  /* Perform ICMPv6 auto-configuration */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              ret = icmpv6_autoconfig(dev);
            }
        }
        break;
#endif

      case SIOCSIFFLAGS:  /* Sets the interface flags */
        {
          /* Is this a request to bring the interface up? */

          dev = netdev_ifr_dev(req);
          if (dev)
            {
              if ((req->ifr_flags & IFF_UP) != 0)
                {
                  /* Yes.. bring the interface up */

                  netdev_ifup(dev);
                }

              /* Is this a request to take the interface down? */

              else if ((req->ifr_flags & IFF_DOWN) != 0)
                {
                  /* Yes.. take the interface down */

                  netdev_ifdown(dev);
                }
            }

          ret = OK;
        }
        break;

      case SIOCGIFFLAGS:  /* Gets the interface flags */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
              req->ifr_flags = dev->d_flags;
            }

          ret = OK;
        }
        break;

      /* MAC address operations only make sense if Ethernet or 6LoWPAN are
       * supported.
       */

#if defined(CONFIG_NET_ETHERNET) || defined(CONFIG_NET_6LOWPAN)
      case SIOCGIFHWADDR:  /* Get hardware address */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
#ifdef CONFIG_NET_ETHERNET
              if (dev->d_lltype == NET_LL_ETHERNET ||
                  dev->d_lltype == NET_LL_IEEE80211)
                {
                  req->ifr_hwaddr.sa_family = AF_INETX;
                  memcpy(req->ifr_hwaddr.sa_data,
                         dev->d_mac.ether.ether_addr_octet, IFHWADDRLEN);
                  ret = OK;
                }
              else
#endif

#ifdef CONFIG_NET_6LOWPAN
              if (dev->d_lltype == NET_LL_IEEE802154 ||
                  dev->d_lltype == NET_LL_PKTRADIO)
                {
                  req->ifr_hwaddr.sa_family = AF_INETX;
                  memcpy(req->ifr_hwaddr.sa_data,
                         dev->d_mac.radio.nv_addr,
                         dev->d_mac.radio.nv_addrlen);
                  ret = OK;
                }
               else
#endif
                {
                  nerr("Unsupported link layer\n");
                }
            }
        }
        break;

      case SIOCSIFHWADDR:  /* Set hardware address -- will not take effect until ifup */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
#ifdef CONFIG_NET_ETHERNET
              if (dev->d_lltype == NET_LL_ETHERNET ||
                  dev->d_lltype == NET_LL_IEEE80211)
                {
                  memcpy(dev->d_mac.ether.ether_addr_octet,
                         req->ifr_hwaddr.sa_data, IFHWADDRLEN);
                  ret = OK;
                }
              else
#endif

#ifdef CONFIG_NET_6LOWPAN
              if (dev->d_lltype == NET_LL_IEEE802154 ||
                  dev->d_lltype == NET_LL_PKTRADIO)
                {
                  FAR struct radio_driver_s *radio;
                  struct radiodev_properties_s properties;

                  /* Get the radio properties */

                  radio = (FAR struct radio_driver_s *)dev;
                  DEBUGASSERT(radio->r_properties != NULL);

                  ret = radio->r_properties(radio, &properties);
                  if (ret >= 0)
                    {
                      dev->d_mac.radio.nv_addrlen = properties.sp_addrlen;
                      memcpy(dev->d_mac.radio.nv_addr,
                             req->ifr_hwaddr.sa_data, NET_6LOWPAN_ADDRSIZE);
                    }
                }
              else
#endif
                {
                  nerr("Unsupported link layer\n");
                }
            }
        }
        break;
#endif

      case SIOCDIFADDR:  /* Delete IP address */
        {
          dev = netdev_ifr_dev(req);
          if (dev)
            {
#ifdef CONFIG_NET_IPv4
              dev->d_ipaddr = 0;
#endif
#ifdef CONFIG_NET_IPv6
              memset(&dev->d_ipv6addr, 0, sizeof(net_ipv6addr_t));
#endif
              ret = OK;
            }
        }
        break;

      case SIOCGIFCOUNT:  /* Get number of devices */
        {
          req->ifr_count = netdev_count();
          ret = -ENOSYS;
        }
        break;

#ifdef CONFIG_NET_IPv4
      case SIOCGIFCONF:  /* Return an interface list (IPv4) */
        {
          ret = netdev_ipv4_ifconf((FAR struct ifconf *)req);
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCGLIFCONF:  /* Return an interface list (IPv6) */
        {
          ret = netdev_ipv6_ifconf((FAR struct lifconf *)req);
        }
        break;
#endif

#if defined(CONFIG_NETDEV_IOCTL) && defined(CONFIG_NETDEV_PHY_IOCTL)
#ifdef CONFIG_ARCH_PHY_INTERRUPT
      case SIOCMIINOTIFY: /* Set up for PHY event notifications */
        {
          dev = netdev_ifr_dev(req);
          if (dev && dev->d_ioctl)
            {
              struct mii_ioctl_notify_s *notify =
                &req->ifr_ifru.ifru_mii_notify;
              ret = dev->d_ioctl(dev, cmd, (unsigned long)(uintptr_t)notify);
            }
        }
        break;
#endif

      case SIOCGMIIPHY: /* Get address of MII PHY in use */
      case SIOCGMIIREG: /* Get MII register via MDIO */
      case SIOCSMIIREG: /* Set MII register via MDIO */
        {
          dev = netdev_ifr_dev(req);
          if (dev && dev->d_ioctl)
            {
              struct mii_ioctl_data_s *mii_data =
                &req->ifr_ifru.ifru_mii_data;
              ret = dev->d_ioctl(dev, cmd,
                                 (unsigned long)(uintptr_t)mii_data);
            }
        }
        break;
#endif

#if defined(CONFIG_NETDEV_IOCTL) && defined(CONFIG_NETDEV_CAN_BITRATE_IOCTL)
      case SIOCGCANBITRATE:  /* Get bitrate from a CAN controller */
      case SIOCSCANBITRATE:  /* Set bitrate of a CAN controller */
        {
          dev = netdev_ifr_dev(req);
          if (dev && dev->d_ioctl)
            {
              struct can_ioctl_data_s *can_bitrate_data =
                            &req->ifr_ifru.ifru_can_data;
              ret = dev->d_ioctl(dev, cmd,
                            (unsigned long)(uintptr_t)can_bitrate_data);
            }
        }
        break;
#endif

#ifdef CONFIG_NETDEV_IFINDEX
      case SIOCGIFNAME:  /* Get interface name */
        {
          dev = netdev_findbyindex(req->ifr_ifindex);
          if (dev != NULL)
            {
              strncpy(req->ifr_name, dev->d_ifname, IFNAMSIZ);
              ret = OK;
            }
          else
            {
              ret = -ENODEV;
            }
        }
        break;

      case SIOCGIFINDEX:  /* Index to name mapping */
        {
          dev = netdev_findbyname(req->ifr_name);
          if (dev != NULL)
            {
              req->ifr_ifindex = dev->d_ifindex;
              ret = OK;
            }
          else
            {
              ret = -ENODEV;
            }
        }
        break;
#endif

      default:
        {
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: netdev_imsfdev
 *
 * Description:
 *   Verify the struct ip_msfilter and get the Ethernet device.
 *
 * Input Parameters:
 *   req - The argument of the ioctl cmd
 *
 * Returned Value:
 *  A pointer to the driver structure on success; NULL on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static FAR struct net_driver_s *netdev_imsfdev(FAR struct ip_msfilter *imsf)
{
  if (imsf == NULL)
    {
      return NULL;
    }

  /* Find the network device associated with the address of the IP address
   * of the local device.
   */

  return netdev_findby_lipv4addr(imsf->imsf_interface.s_addr);
}
#endif

/****************************************************************************
 * Name: netdev_imsf_ioctl
 *
 * Description:
 *   Perform network device specific operations.
 *
 * Input Parameters:
 *   psock    Socket structure
 *   dev      Ethernet driver device structure
 *   cmd      The ioctl command
 *   imsf     The argument of the ioctl cmd
 *
 * Returned Value:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int netdev_imsf_ioctl(FAR struct socket *psock, int cmd,
                             FAR struct ip_msfilter *imsf)
{
  FAR struct net_driver_s *dev;
  int ret = -EINVAL;

  ninfo("cmd: %d\n", cmd);

  /* Execute the command */

  switch (cmd)
    {
      case SIOCSIPMSFILTER:  /* Set source filter content */
        {
          dev = netdev_imsfdev(imsf);
          if (dev)
            {
              if (imsf->imsf_fmode == MCAST_INCLUDE)
                {
                  ret = igmp_joingroup(dev, &imsf->imsf_multiaddr);
                }
              else
                {
                  DEBUGASSERT(imsf->imsf_fmode == MCAST_EXCLUDE);
                  ret = igmp_leavegroup(dev, &imsf->imsf_multiaddr);
                }
            }
        }
        break;

      case SIOCGIPMSFILTER:  /* Retrieve source filter addresses */
      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: netdev_arp_ioctl
 *
 * Description:
 *   Perform ARP table specific operations.
 *
 * Input Parameters:
 *   psock  Socket structure
 *   dev    Ethernet driver device structure
 *   cmd    The ioctl command
 *   req    The argument of the ioctl cmd
 *
 * Returned Value:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP
static int netdev_arp_ioctl(FAR struct socket *psock, int cmd,
                            FAR struct arpreq *req)
{
  int ret;

  /* Execute the command */

  switch (cmd)
    {
      case SIOCSARP:  /* Set an ARP mapping */
        {
          if (req != NULL &&
              req->arp_pa.sa_family == AF_INET &&
              req->arp_ha.sa_family == ARPHRD_ETHER)
            {
              FAR struct sockaddr_in *addr =
                (FAR struct sockaddr_in *)&req->arp_pa;

              /* Update any existing ARP table entry for this protocol
               * address -OR- add a new ARP table entry if there is not.
               */

              ret = arp_update(addr->sin_addr.s_addr,
                               (FAR uint8_t *)req->arp_ha.sa_data);
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      case SIOCDARP:  /* Delete an ARP mapping */
        {
          if (req != NULL && req->arp_pa.sa_family == AF_INET)
            {
              FAR struct sockaddr_in *addr =
                (FAR struct sockaddr_in *)&req->arp_pa;

              /* Find the existing ARP entry for this protocol address. */

              FAR struct arp_entry_s *entry =
                arp_lookup(addr->sin_addr.s_addr);
              if (entry != NULL)
                {
                  /* The ARP table is fixed size; an entry is deleted
                   * by nullifying its protocol address.
                   */

                  entry->at_ipaddr = 0;
                  ret = OK;
                }
              else
                {
                  ret = -ENOENT;
                }
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      case SIOCGARP:  /* Get an ARP mapping */
        {
          if (req != NULL && req->arp_pa.sa_family == AF_INET)
            {
              FAR struct sockaddr_in *addr =
                (FAR struct sockaddr_in *)&req->arp_pa;

              /* Get the hardware address from an existing ARP table entry
               * matching this protocol address.
               */

              ret = arp_find(addr->sin_addr.s_addr,
                            (FAR struct ether_addr *)req->arp_ha.sa_data);
              if (ret >= 0)
                {
                  /* Return the mapped hardware address. */

                  req->arp_ha.sa_family = ARPHRD_ETHER;
                  ret = OK;
                }
            }
          else
            {
              ret = -EINVAL;
            }
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: netdev_rt_ioctl
 *
 * Description:
 *   Perform routing table specific operations.
 *
 * Input Parameters:
 *   psock    Socket structure
 *   dev      Ethernet driver device structure
 *   cmd      The ioctl command
 *   rtentry  The argument of the ioctl cmd
 *
 * Returned Value:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ROUTE
static int netdev_rt_ioctl(FAR struct socket *psock, int cmd,
                           FAR struct rtentry *rtentry)
{
  int ret = -EAFNOSUPPORT;

  /* Execute the command */

  switch (cmd)
    {
      case SIOCADDRT:  /* Add an entry to the routing table */
        {
          /* The target address and the netmask are required values */

          if (rtentry == NULL)
            {
              return -EINVAL;
            }

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
          if (rtentry->rt_dst.ss_family == AF_INET)
#endif
            {
#ifdef HAVE_WRITABLE_IPv4ROUTE
              ret = ioctl_add_ipv4route(rtentry);
#else
              ret = -EACCES;
#endif
            }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
          else
#endif
            {
#ifdef HAVE_WRITABLE_IPv6ROUTE
              ret = ioctl_add_ipv6route(rtentry);
#else
              ret = -EACCES;
#endif
            }
#endif /* CONFIG_NET_IPv6 */
        }
        break;

      case SIOCDELRT:  /* Delete an entry from the routing table */
        {
          /* The target address and the netmask are required values */

          if (rtentry == 0)
            {
              return -EINVAL;
            }

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
          if (rtentry->rt_dst.ss_family == AF_INET)
#endif
            {
#ifdef HAVE_WRITABLE_IPv4ROUTE
              ret = ioctl_del_ipv4route(rtentry);
#else
              ret = -EACCES;
#endif
            }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
          else
#endif
            {
#ifdef HAVE_WRITABLE_IPv6ROUTE
              ret = ioctl_del_ipv6route(rtentry);
#else
              ret = -EACCES;
#endif
            }
#endif /* CONFIG_NET_IPv6 */
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: netdev_usrsock_ioctl
 *
 * Description:
 *   Perform user private ioctl operations.
 *
 * Parameters:
 *   psock    Socket structure
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_USRSOCK
static int netdev_usrsock_ioctl(FAR struct socket *psock, int cmd,
                                unsigned long arg)
{
  if (psock->s_sockif && psock->s_sockif->si_ioctl)
    {
      ssize_t arglen;

      arglen = net_ioctl_arglen(cmd);
      if (arglen < 0)
        {
          return arglen;
        }

      return psock->s_sockif->si_ioctl(psock, cmd, (FAR void *)arg, arglen);
    }
  else
    {
      return -ENOTTY;
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_ioctl_arglen
 *
 * Description:
 *   Calculate the ioctl argument buffer length.
 *
 * Input Parameters:
 *
 *   cmd      The ioctl command
 *
 * Returned Value:
 *   The argument buffer length, or error code.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_USRSOCK
ssize_t net_ioctl_arglen(int cmd)
{
  switch (cmd)
    {
      case SIOCGIFADDR:
      case SIOCSIFADDR:
      case SIOCGIFDSTADDR:
      case SIOCSIFDSTADDR:
      case SIOCGIFBRDADDR:
      case SIOCSIFBRDADDR:
      case SIOCGIFNETMASK:
      case SIOCSIFNETMASK:
      case SIOCGIFMTU:
      case SIOCGIFHWADDR:
      case SIOCSIFHWADDR:
      case SIOCDIFADDR:
      case SIOCGIFCOUNT:
      case SIOCSIFFLAGS:
      case SIOCGIFFLAGS:
        return sizeof(struct ifreq);

      case SIOCGLIFADDR:
      case SIOCSLIFADDR:
      case SIOCGLIFDSTADDR:
      case SIOCSLIFDSTADDR:
      case SIOCGLIFBRDADDR:
      case SIOCSLIFBRDADDR:
      case SIOCGLIFNETMASK:
      case SIOCSLIFNETMASK:
      case SIOCGLIFMTU:
      case SIOCIFAUTOCONF:
        return sizeof(struct lifreq);

      case SIOCGIFCONF:
        return sizeof(struct ifconf);

      case SIOCGLIFCONF:
        return sizeof(struct lifconf);

      case SIOCGIPMSFILTER:
      case SIOCSIPMSFILTER:
        return sizeof(struct ip_msfilter);

      case SIOCSARP:
      case SIOCDARP:
      case SIOCGARP:
        return sizeof(struct arpreq);

      case SIOCADDRT:
      case SIOCDELRT:
        return sizeof(struct rtentry);

      case SIOCMIINOTIFY:
        return sizeof(struct mii_ioctl_notify_s);

      case SIOCGMIIPHY:
      case SIOCGMIIREG:
      case SIOCSMIIREG:
        return sizeof(struct mii_ioctl_data_s);

      default:
#ifdef CONFIG_NETDEV_IOCTL
#  ifdef CONFIG_NETDEV_WIRELESS_IOCTL
        if (_WLIOCVALID(cmd) && _IOC_NR(cmd) <= WL_NNETCMDS)
          {
            return sizeof(struct iwreq);
          }
#  endif

#  ifdef CONFIG_WIRELESS_IEEE802154
        if (_MAC802154IOCVALID(cmd))
          {
            return sizeof(struct ieee802154_netmac_s);
          }
#  endif

#  ifdef CONFIG_WIRELESS_PKTRADIO
        if (WL_ISPKTRADIOCMD(cmd))
          {
            return sizeof(struct pktradio_ifreq_s);
          }
#  endif

#  ifdef CONFIG_WIRELESS_BLUETOOTH
        if (WL_IBLUETOOTHCMD(cmd))
          {
            return sizeof(struct btreq_s);
          }
#  endif
#endif

        return -ENOTTY;
    }
}
#endif

/****************************************************************************
 * Name: psock_ioctl and psock_vioctl
 *
 * Description:
 *   Perform network device specific operations.
 *
 * Input Parameters:
 *   psock    A pointer to a NuttX-specific, internal socket structure
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 * Returned Value:
 *   A non-negative value is returned on success; a negated errno value is
 *   returned on any failure to indicate the nature of the failure:
 *
 *   EBADF
 *     'psock' is not a valid, connected socket structure.
 *   EFAULT
 *     'arg' references an inaccessible memory area.
 *   ENOTTY
 *     'cmd' not valid.
 *   EINVAL
 *     'arg' is not valid.
 *   ENOTTY
 *     'sockfd' is not associated with a network device.
 *   ENOTTY
 *      The specified request does not apply to the kind of object that the
 *      descriptor 'sockfd' references.
 *
 ****************************************************************************/

int psock_vioctl(FAR struct socket *psock, int cmd, va_list ap)
{
  unsigned long arg;
  int ret;

  /* Verify that the psock corresponds to valid, allocated socket */

  if (psock == NULL || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  arg = va_arg(ap, unsigned long);

#ifdef CONFIG_NET_USRSOCK
  /* Check for a USRSOCK ioctl command */

  ret = netdev_usrsock_ioctl(psock, cmd, arg);
  if (ret == -ENOTTY)
#endif
    {
      /* Check for a standard network IOCTL command. */

      ret = netdev_ifr_ioctl(psock, cmd, (FAR struct ifreq *)(uintptr_t)arg);
    }

#if defined(CONFIG_NETDEV_IOCTL) && defined(CONFIG_NETDEV_WIRELESS_IOCTL)
  /* Check for a wireless network command */

  if (ret == -ENOTTY)
    {
      FAR struct iwreq *wifrreq;

      wifrreq = (FAR struct iwreq *)((uintptr_t)arg);
      ret     = netdev_wifr_ioctl(psock, cmd, wifrreq);
    }
#endif

#ifdef HAVE_IEEE802154_IOCTL
  /* Check for a IEEE802.15.4 network device IOCTL command */

  if (ret == -ENOTTY)
    {
      ret = netdev_iee802154_ioctl(psock, cmd, arg);
    }
#endif

#ifdef HAVE_PKTRADIO_IOCTL
  /* Check for a non-IEEE802.15.4 packet radio network device IOCTL command */

  if (ret == -ENOTTY)
    {
      ret = netdev_pktradio_ioctl(psock, cmd, arg);
    }
#endif

#ifdef HAVE_BLUETOOTH_IOCTL
  /* Check for Bluetooth network device IOCTL command */

  if (ret == -ENOTTY)
    {
      ret = netdev_bluetooth_ioctl(psock, cmd, arg);
    }
#endif

#ifdef CONFIG_NET_IGMP
  /* Check for address filtering commands */

  if (ret == -ENOTTY)
    {
      ret = netdev_imsf_ioctl(psock, cmd,
                              (FAR struct ip_msfilter *)(uintptr_t)arg);
    }
#endif

#ifdef CONFIG_NET_ARP
  /* Check for ARP table IOCTL commands */

  if (ret == -ENOTTY)
    {
      ret = netdev_arp_ioctl(psock, cmd,
                             (FAR struct arpreq *)(uintptr_t)arg);
    }
#endif

#ifdef CONFIG_NET_ROUTE
  /* Check for Routing table IOCTL commands */

  if (ret == -ENOTTY)
    {
      ret = netdev_rt_ioctl(psock, cmd,
                            (FAR struct rtentry *)(uintptr_t)arg);
    }
#endif

  return ret;
}

int psock_ioctl(FAR struct socket *psock, int cmd, ...)
{
  va_list ap;
  int ret;

  /* Setup to access the variable argument list */

  va_start(ap, cmd);

  /* Let psock_vfcntl() do the real work.  The errno is not set on
   * failures.
   */

  ret = psock_vioctl(psock, cmd, ap);

  va_end(ap);
  return ret;
}

/****************************************************************************
 * Name: netdev_vioctl
 *
 * Description:
 *   Perform network device specific operations.
 *
 * Input Parameters:
 *   sockfd   Socket descriptor of device
 *   cmd      The ioctl command
 *   ap       The argument of the ioctl cmd
 *
 * Returned Value:
 *   A non-negative value is returned on success; a negated errno value is
 *   returned on any failure to indicate the nature of the failure:
 *
 *   EBADF
 *     'sockfd' is not a valid socket descriptor.
 *   EFAULT
 *     'arg' references an inaccessible memory area.
 *   ENOTTY
 *     'cmd' not valid.
 *   EINVAL
 *     'arg' is not valid.
 *   ENOTTY
 *     'sockfd' is not associated with a network device.
 *   ENOTTY
 *      The specified request does not apply to the kind of object that the
 *      descriptor 'sockfd' references.
 *
 ****************************************************************************/

int netdev_vioctl(int sockfd, int cmd, va_list ap)
{
  FAR struct socket *psock = sockfd_socket(sockfd);

  return psock_vioctl(psock, cmd, ap);
}

/****************************************************************************
 * Name: netdev_ifup / netdev_ifdown
 *
 * Description:
 *   Bring the interface up/down
 *
 ****************************************************************************/

void netdev_ifup(FAR struct net_driver_s *dev)
{
  /* Make sure that the device supports the d_ifup() method */

  if (dev->d_ifup != NULL)
    {
      /* Is the interface already up? */

      if ((dev->d_flags & IFF_UP) == 0)
        {
          /* No, bring the interface up now */

          if (dev->d_ifup(dev) == OK)
            {
              /* Mark the interface as up */

              dev->d_flags |= IFF_UP;

              /* Update the driver status */

              netlink_device_notify(dev);
            }
        }
    }
}

void netdev_ifdown(FAR struct net_driver_s *dev)
{
  /* Check sure that the device supports the d_ifdown() method */

  if (dev->d_ifdown != NULL)
    {
      /* Is the interface already down? */

      if ((dev->d_flags & IFF_UP) != 0)
        {
          /* No, take the interface down now */

          if (dev->d_ifdown(dev) == OK)
            {
              /* Mark the interface as down */

              dev->d_flags &= ~IFF_UP;

              /* Update the driver status */

              netlink_device_notify(dev);
            }
        }

      /* Notify clients that the network has been taken down */

      devif_dev_event(dev, NULL, NETDEV_DOWN);

#ifdef CONFIG_NETDOWN_NOTIFIER
      /* Provide signal notifications to threads that want to be
       * notified of the network down state via signal.
       */

      netdown_notifier_signal(dev);
#endif
    }
}
