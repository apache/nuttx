/****************************************************************************
 * net/netdev/netdev_ioctl.c
 *
 *   Copyright (C) 2007-2012, 2015 Gregory Nutt. All rights reserved.
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

#ifdef CONFIG_NET_IGMP
#  include "sys/sockio.h"
#  include "nuttx/net/igmp.h"
#endif

#include "socket/socket.h"
#include "netdev/netdev.h"
#include "devif/devif.h"
#include "igmp/igmp.h"
#include "icmpv6/icmpv6.h"
#include "route/route.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: ioctl_addipv4route
 *
 * Description:
 *   Add an IPv4 route to the routing table.
 *
 * Input Parameters:
 *   rentry - Describes the route to be added
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ROUTE) && defined(CONFIG_NET_IPv4)
static int ioctl_addipv4route(FAR struct rtentry *rtentry)
{
  FAR struct sockaddr_in *addr;
  in_addr_t target;
  in_addr_t netmask;
  in_addr_t router;

  addr    = (FAR struct sockaddr_in *)rtentry->rt_target;
  target  = (in_addr_t)addr->sin_addr.s_addr;

  addr    = (FAR struct sockaddr_in *)rtentry->rt_netmask;
  netmask = (in_addr_t)addr->sin_addr.s_addr;

  /* The router is an optional argument */

  if (rtentry->rt_router)
    {
      addr   = (FAR struct sockaddr_in *)rtentry->rt_router;
      router = (in_addr_t)addr->sin_addr.s_addr;
    }
  else
    {
      router = 0;
    }

  return net_addroute(target, netmask, router);
}
#endif /* CONFIG_NET_ROUTE && CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: ioctl_addipv6route
 *
 * Description:
 *   Add an IPv6 route to the routing table.
 *
 * Input Parameters:
 *   rentry - Describes the route to be added
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ROUTE) && defined(CONFIG_NET_IPv6)
static int ioctl_addipv6route(FAR struct rtentry *rtentry)
{
  FAR struct sockaddr_in6 *target;
  FAR struct sockaddr_in6 *netmask;
  net_ipv6addr_t router;

  target    = (FAR struct sockaddr_in6 *)rtentry->rt_target;
  netmask    = (FAR struct sockaddr_in6 *)rtentry->rt_netmask;

  /* The router is an optional argument */

  if (rtentry->rt_router)
    {
      FAR struct sockaddr_in6 *addr;
      addr = (FAR struct sockaddr_in6 *)rtentry->rt_router;
      net_ipv6addr_copy(router, addr->sin6_addr.s6_addr16);
    }
  else
    {
      net_ipv6addr_copy(router, in6addr_any.s6_addr16);
    }

  return net_addroute_ipv6(target->sin6_addr.s6_addr16, netmask->sin6_addr.s6_addr16, router);
}
#endif /* CONFIG_NET_ROUTE && CONFIG_NET_IPv6 */

/****************************************************************************
 * Name: ioctl_delipv4route
 *
 * Description:
 *   Delete an IPv4 route to the routing table.
 *
 * Input Parameters:
 *   rentry - Describes the route to be deleted
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ROUTE) && defined(CONFIG_NET_IPv4)
static int ioctl_delipv4route(FAR struct rtentry *rtentry)
{
  FAR struct sockaddr_in *addr;
  in_addr_t target;
  in_addr_t netmask;

  addr    = (FAR struct sockaddr_in *)rtentry->rt_target;
  target  = (in_addr_t)addr->sin_addr.s_addr;

  addr    = (FAR struct sockaddr_in *)rtentry->rt_netmask;
  netmask = (in_addr_t)addr->sin_addr.s_addr;

  return net_delroute(target, netmask);
}
#endif /* CONFIG_NET_ROUTE && CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: ioctl_delipv6route
 *
 * Description:
 *   Delete an IPv6 route to the routing table.
 *
 * Input Parameters:
 *   rentry - Describes the route to be deleted
 *
 ****************************************************************************/

#if defined(CONFIG_NET_ROUTE) && defined(CONFIG_NET_IPv6)
static int ioctl_delipv6route(FAR struct rtentry *rtentry)
{
  FAR struct sockaddr_in6 *target;
  FAR struct sockaddr_in6 *netmask;

  target    = (FAR struct sockaddr_in6 *)rtentry->rt_target;
  netmask    = (FAR struct sockaddr_in6 *)rtentry->rt_netmask;

  return net_delroute_ipv6(target->sin6_addr.s6_addr16, netmask->sin6_addr.s6_addr16);
}
#endif /* CONFIG_NET_ROUTE && CONFIG_NET_IPv6 */

/****************************************************************************
 * Name: ioctl_getipv4addr
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
static void ioctl_getipv4addr(FAR struct sockaddr *outaddr, in_addr_t inaddr)
{
  FAR struct sockaddr_in *dest  = (FAR struct sockaddr_in *)outaddr;
  dest->sin_family              = AF_INET;
  dest->sin_port                = 0;
  dest->sin_addr.s_addr         = inaddr;
}
#endif

/****************************************************************************
 * Name: ioctl_getipv6addr
 *
 * Description:
 *   Copy IP addresses from device structure to user memory.
 *
 * Input Parameters:
 *   outaddr - Pointer to the user-provided memory to receive the address.
 *   inaddr - The source IP adress in the device structure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
static void ioctl_getipv6addr(FAR struct sockaddr_storage *outaddr,
                              FAR const net_ipv6addr_t inaddr)
{
  FAR struct sockaddr_in6 *dest = (FAR struct sockaddr_in6 *)outaddr;
  dest->sin6_family             = AF_INET6;
  dest->sin6_port               = 0;
  memcpy(dest->sin6_addr.in6_u.u6_addr8, inaddr, 16);
}
#endif

/****************************************************************************
 * Name: ioctl_setipv4addr
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
static void ioctl_setipv4addr(FAR in_addr_t *outaddr,
                              FAR const struct sockaddr *inaddr)
{
  FAR const struct sockaddr_in *src = (FAR const struct sockaddr_in *)inaddr;
  *outaddr = src->sin_addr.s_addr;
}
#endif

/****************************************************************************
 * Name: ioctl_setipv6addr
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

#ifdef CONFIG_NET_IPv6
static void ioctl_setipv6addr(FAR net_ipv6addr_t outaddr,
                              FAR const struct sockaddr_storage *inaddr)
{
  FAR const struct sockaddr_in6 *src = (FAR const struct sockaddr_in6 *)inaddr;
  memcpy(outaddr, src->sin6_addr.in6_u.u6_addr8, 16);
}
#endif

/****************************************************************************
 * Name: netdev_ifrdev
 *
 * Description:
 *   Verify the struct ifreq and get the Ethernet device.
 *
 * Parameters:
 *   req - The argument of the ioctl cmd
 *
 * Return:
 *  A pointer to the driver structure on success; NULL on failure.
 *
 ****************************************************************************/

static FAR struct net_driver_s *netdev_ifrdev(FAR struct ifreq *req)
{
  if (!req)
    {
      return NULL;
    }

  /* Find the network device associated with the device name
   * in the request data.
   */

  return netdev_findbyname(req->ifr_name);
}

/****************************************************************************
 * Name: netdev_ifrioctl
 *
 * Description:
 *   Perform network device specific operations.
 *
 * Parameters:
 *   psock    Socket structure
 *   dev      Ethernet driver device structure
 *   cmd      The ioctl command
 *   req      The argument of the ioctl cmd
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

static int netdev_ifrioctl(FAR struct socket *psock, int cmd,
                           FAR struct ifreq *req)
{
  FAR struct net_driver_s *dev;
  int ret = -EINVAL;

  nvdbg("cmd: %d\n", cmd);

  /* Execute the command */

  switch (cmd)
    {
#ifdef CONFIG_NET_IPv4
      case SIOCGIFADDR:  /* Get IP address */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              ioctl_getipv4addr(&req->ifr_addr, dev->d_ipaddr);
              ret = OK;
            }
        }
        break;
#endif


#ifdef CONFIG_NET_IPv4
      case SIOCSIFADDR:  /* Set IP address */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              netdev_ifdown(dev);
              ioctl_setipv4addr(&dev->d_ipaddr, &req->ifr_addr);
              netdev_ifup(dev);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCGIFDSTADDR:  /* Get P-to-P address */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              ioctl_getipv4addr(&req->ifr_dstaddr, dev->d_draddr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCSIFDSTADDR:  /* Set P-to-P address */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              ioctl_setipv4addr(&dev->d_draddr, &req->ifr_dstaddr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCGIFBRDADDR:  /* Get broadcast IP address */
      case SIOCSIFBRDADDR:  /* Set broadcast IP address */
        {
          ret = -ENOSYS;
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCGIFNETMASK:  /* Get network mask */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              ioctl_getipv4addr(&req->ifr_addr, dev->d_netmask);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv4
      case SIOCSIFNETMASK:  /* Set network mask */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              ioctl_setipv4addr(&dev->d_netmask, &req->ifr_addr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCGLIFADDR:  /* Get IP address */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;

              ioctl_getipv6addr(&lreq->lifr_addr, dev->d_ipv6addr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCSLIFADDR:  /* Set IP address */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;

              netdev_ifdown(dev);
              ioctl_setipv6addr(dev->d_ipv6addr, &lreq->lifr_addr);
              netdev_ifup(dev);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCGLIFDSTADDR:  /* Get P-to-P address */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;

              ioctl_getipv6addr(&lreq->lifr_dstaddr, dev->d_ipv6draddr);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCSLIFDSTADDR:  /* Set P-to-P address */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;

              ioctl_setipv6addr(dev->d_ipv6draddr, &lreq->lifr_dstaddr);
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
          dev = netdev_ifrdev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;

              ioctl_getipv6addr(&lreq->lifr_addr, dev->d_ipv6netmask);
              ret = OK;
            }
        }
        break;
#endif

#ifdef CONFIG_NET_IPv6
      case SIOCSLIFNETMASK:  /* Set network mask */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              FAR struct lifreq *lreq = (FAR struct lifreq *)req;
              ioctl_setipv6addr(dev->d_ipv6netmask, &lreq->lifr_addr);
              ret = OK;
            }
        }
        break;
#endif

      case SIOCGLIFMTU:  /* Get MTU size */
      case SIOCGIFMTU:   /* Get MTU size */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              req->ifr_mtu = NET_DEV_MTU(dev);
              ret = OK;
            }
        }
        break;

#ifdef CONFIG_NET_ICMPv6_AUTOCONF
      case SIOCIFAUTOCONF:  /* Perform ICMPv6 auto-configuration */
        {
          dev = netdev_ifrdev(req);
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

          dev = netdev_ifrdev(req);
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
          dev = netdev_ifrdev(req);
          if (dev)
            {
              req->ifr_flags = dev->d_flags;
            }

          ret = OK;
        }
        break;

      /* MAC address operations only make sense if Ethernet is supported */

#ifdef CONFIG_NET_ETHERNET
      case SIOCGIFHWADDR:  /* Get hardware address */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              req->ifr_hwaddr.sa_family = AF_INETX;
              memcpy(req->ifr_hwaddr.sa_data,
                     dev->d_mac.ether_addr_octet, IFHWADDRLEN);
              ret = OK;
            }
        }
        break;

      case SIOCSIFHWADDR:  /* Set hardware address -- will not take effect until ifup */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              memcpy(dev->d_mac.ether_addr_octet,
                     req->ifr_hwaddr.sa_data, IFHWADDRLEN);
              ret = OK;
            }
        }
        break;
#endif

      case SIOCDIFADDR:  /* Delete IP address */
        {
          dev = netdev_ifrdev(req);
          if (dev)
            {
              netdev_ifdown(dev);
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

#ifdef CONFIG_NET_ARPIOCTLS
      case SIOCSARP:  /* Set a ARP mapping */
      case SIOCDARP:  /* Delete an ARP mapping */
      case SIOCGARP:  /* Get an ARP mapping */
# error "IOCTL Commands not implemented"
#endif

#ifdef CONFIG_NETDEV_PHY_IOCTL
#ifdef CONFIG_ARCH_PHY_INTERRUPT
      case SIOCMIINOTIFY: /* Set up for PHY event notifications */
        {
          dev = netdev_ifrdev(req);
          if (dev && dev->d_ioctl)
            {
              struct mii_iotcl_notify_s *notify = &req->ifr_ifru.ifru_mii_notify;
              ret = dev->d_ioctl(dev, cmd, ((long)(uintptr_t)notify));
            }
        }
        break;
#endif

      case SIOCGMIIPHY: /* Get address of MII PHY in use */
      case SIOCGMIIREG: /* Get MII register via MDIO */
      case SIOCSMIIREG: /* Set MII register via MDIO */
        {
          dev = netdev_ifrdev(req);
          if (dev && dev->d_ioctl)
            {
              struct mii_ioctl_data_s *mii_data = &req->ifr_ifru.ifru_mii_data;
              ret = dev->d_ioctl(dev, cmd, ((long)(uintptr_t)mii_data));
            }
        }
        break;
#endif

      default:
        {
          ret = -ENOTTY;
        }
        break;;
    }

  return ret;
}

/****************************************************************************
 * Name: netdev_imsfdev
 *
 * Description:
 *   Verify the struct ip_msfilter and get the Ethernet device.
 *
 * Parameters:
 *   req - The argument of the ioctl cmd
 *
 * Return:
 *  A pointer to the driver structure on success; NULL on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static FAR struct net_driver_s *netdev_imsfdev(FAR struct ip_msfilter *imsf)
{
  if (!imsf)
    {
      return NULL;
    }

  /* Find the network device associated with the device name
   * in the request data.
   */

  return netdev_findbyname(imsf->imsf_name);
}
#endif

/****************************************************************************
 * Name: netdev_imsfioctl
 *
 * Description:
 *   Perform network device specific operations.
 *
 * Parameters:
 *   psock    Socket structure
 *   dev      Ethernet driver device structure
 *   cmd      The ioctl command
 *   imsf     The argument of the ioctl cmd
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IGMP
static int netdev_imsfioctl(FAR struct socket *psock, int cmd,
                            FAR struct ip_msfilter *imsf)
{
  FAR struct net_driver_s *dev;
  int ret = -EINVAL;

  nvdbg("cmd: %d\n", cmd);

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
 * Name: netdev_rtioctl
 *
 * Description:
 *   Perform routing table specific operations.
 *
 * Parameters:
 *   psock    Socket structure
 *   dev      Ethernet driver device structure
 *   cmd      The ioctl command
 *   rtentry  The argument of the ioctl cmd
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   Negated errno returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ROUTE
static int netdev_rtioctl(FAR struct socket *psock, int cmd,
                          FAR struct rtentry *rtentry)
{
  int ret = -EAFNOSUPPORT;

  /* Execute the command */

  switch (cmd)
    {
      case SIOCADDRT:  /* Add an entry to the routing table */
        {
          /* The target address and the netmask are required values */

          if (!rtentry || !rtentry->rt_target || !rtentry->rt_netmask)
            {
              return -EINVAL;
            }

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
          if (rtentry->rt_target->ss_family == AF_INET)
#endif
            {
              ret = ioctl_addipv4route(rtentry);
            }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
          else
#endif
            {
              ret = ioctl_addipv6route(rtentry);
            }
#endif /* CONFIG_NET_IPv6 */
        }
        break;

      case SIOCDELRT:  /* Delete an entry from the routing table */
        {
          /* The target address and the netmask are required values */

          if (!rtentry || !rtentry->rt_target || !rtentry->rt_netmask)
            {
              return -EINVAL;
            }

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
          if (rtentry->rt_target->ss_family == AF_INET)
#endif
            {
              ret = ioctl_delipv4route(rtentry);
            }
#endif /* CONFIG_NET_IPv4 */

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
          else
#endif
            {
              ret = ioctl_delipv6route(rtentry);
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_ioctl
 *
 * Description:
 *   Perform network device specific operations.
 *
 * Parameters:
 *   sockfd   Socket descriptor of device
 *   cmd      The ioctl command
 *   arg      The argument of the ioctl cmd
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   On a failure, -1 is returned with errno set appropriately
 *
 *   EBADF
 *     'sockfd' is not a valid descriptor.
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

int netdev_ioctl(int sockfd, int cmd, unsigned long arg)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
  int ret;

  /* Check if this is a valid command.  In all cases, arg is a pointer that has
   * been cast to unsigned long.  Verify that the value of the to-be-pointer is
   * non-NULL.
   */

  if (!_SIOCVALID(cmd))
    {
      ret = -ENOTTY;
      goto errout;
    }

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      ret = -EBADF;
      goto errout;
    }

  /* Execute the command */

  ret = netdev_ifrioctl(psock, cmd, (FAR struct ifreq*)((uintptr_t)arg));
#ifdef CONFIG_NET_IGMP
  if (ret == -ENOTTY)
    {

      ret = netdev_imsfioctl(psock, cmd, (FAR struct ip_msfilter*)((uintptr_t)arg));
    }
#endif
#ifdef CONFIG_NET_ROUTE
  if (ret == -ENOTTY)
    {
      ret = netdev_rtioctl(psock, cmd, (FAR struct rtentry*)((uintptr_t)arg));
    }
#endif

  /* Check for success or failure */

  if (ret >= 0)
    {
      return ret;
    }

/* On failure, set the errno and return -1 */

errout:
  errno = -ret;
  return ERROR;
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

  if (dev->d_ifup)
    {
      /* Is the interface already up? */

      if ((dev->d_flags & IFF_UP) == 0)
        {
          /* No, bring the interface up now */

          if (dev->d_ifup(dev) == OK)
            {
              /* Mark the interface as up */

              dev->d_flags |= IFF_UP;
            }
        }
    }
}

void netdev_ifdown(FAR struct net_driver_s *dev)
{
  /* Check sure that the device supports the d_ifdown() method */

  if (dev->d_ifdown)
    {
      /* Is the interface already down? */

      if ((dev->d_flags & IFF_UP) != 0)
        {
          /* No, take the interface down now */

          if (dev->d_ifdown(dev) == OK)
            {
              /* Mark the interface as down */

              dev->d_flags &= ~IFF_UP;
            }
        }

      /* Notify clients that the network has been taken down */

      (void)devif_dev_event(dev, NULL, NETDEV_DOWN);
    }
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
