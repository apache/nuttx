/****************************************************************************
 * net/netdev-ioctl.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net.h>

#include <net/uip/uip-arch.h>
#include <net/uip/uip.h>

#include "net-internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
# define AF_INETX AF_INET6
#else
# define AF_INETX AF_INET
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ioctl_getipaddr / ioctl_setipaddr
 *
 * Description:
 *   Copy IP addresses into and out of device structure
 *
 ****************************************************************************/

static void ioctl_getipaddr(struct sockaddr *outaddr, uip_ipaddr_t *inaddr)
{
#ifdef CONFIG_NET_IPv6
#error " big enough for IPv6 address"
  struct sockaddr_in6 *dest = (struct sockaddr_in6 *)outaddr;
  dest->sin_family = AF_INET6;
  dest->sin_port   = 0;
  memcpy(dest->sin6_addr.in6_u.u6_addr8, inaddr, 16);
#else
  struct sockaddr_in *dest = (struct sockaddr_in *)outaddr;
  dest->sin_family = AF_INET;
  dest->sin_port   = 0;
  dest->sin_addr.s_addr = *inaddr;
#endif
}

static void ioctl_setipaddr(uip_ipaddr_t *outaddr, struct sockaddr *inaddr)
{
#ifdef CONFIG_NET_IPv6
  struct sockaddr_in6 *src = (struct sockaddr_in6 *)inaddr;
  memcpy(outaddr, src->sin6_addr.in6_u.u6_addr8, 16);
#else
  struct sockaddr_in *src = (struct sockaddr_in *)inaddr;
  *outaddr = src->sin_addr.s_addr;
#endif
}

static void ioctl_ifup(FAR struct uip_driver_s *dev)
{
  if (dev->d_ifup)
    {
      dev->d_ifup(dev);
    }
}

static void ioctl_ifdown(FAR struct uip_driver_s *dev)
{
  if (dev->d_ifdown)
    {
      dev->d_ifdown(dev);
    }
}

/****************************************************************************
 * Global Functions
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
 *   req      The argument of the ioctl cmd
 *
 * Return:
 *   >=0 on success (positive non-zero values are cmd-specific)
 *   -1 on failure withi errno set properly:
 *
 *   EBADF
 *     'sockfd' is not a valid descriptor.
 *   EFAULT
 *     'req' references an inaccessible memory area.
 *   EINVAL
 *     'cmd' or 'req' is not valid.
 *   ENOTTY
 *     'sockfd' is not associated with a network device.
 *   ENOTTY
 *      The specified request does not apply to the kind of object that the
 *      descriptor 'sockfd' references.
 *
 ****************************************************************************/

int netdev_ioctl(int sockfd, int cmd, struct ifreq *req)
{
  FAR struct socket *psock = sockfd_socket(sockfd);
  FAR struct uip_driver_s *dev;
  int err;

  if (!_SIOCVALID(cmd) || !req)
    {
      err = EINVAL;
      goto errout;
    }

  /* Verify that the sockfd corresponds to valid, allocated socket */

  if (!psock || psock->s_crefs <= 0)
    {
      err = EBADF;
      goto errout;
    }

  /* Find the network device associated with the device name
   * in the request data.
   */

  dev = netdev_find(req->ifr_name);
  if (!dev)
    {
      err = EINVAL;
      goto errout;
    }

  /* Execute the command */

  switch (cmd)
    {
      case SIOCGIFADDR:     /* Get IP address */
        ioctl_getipaddr(&req->ifr_addr, &dev->d_ipaddr);
        break;

      case SIOCSIFADDR:     /* Set IP address */
        ioctl_ifdown(dev);
        ioctl_setipaddr(&dev->d_ipaddr, &req->ifr_addr);
        ioctl_ifup(dev);
        break;

      case SIOCGIFDSTADDR:  /* Get P-to-P address */
        ioctl_getipaddr(&req->ifr_dstaddr, &dev->d_draddr);
        break;

      case SIOCSIFDSTADDR:  /* Set P-to-P address */
        ioctl_setipaddr(&dev->d_draddr, &req->ifr_dstaddr);
        break;

      case SIOCGIFNETMASK:  /* Get network mask */
        ioctl_getipaddr(&req->ifr_addr, &dev->d_netmask);
        break;

      case SIOCSIFNETMASK:  /* Set network mask */
        ioctl_setipaddr(&dev->d_netmask, &req->ifr_addr);
        break;

      case SIOCGIFMTU:  /* Get MTU size */
        req->ifr_mtu = CONFIG_NET_BUFSIZE;
        break;

      case SIOCGIFHWADDR:  /* Get hardware address */
        req->ifr_hwaddr.sa_family = AF_INETX;
        memcpy(req->ifr_hwaddr.sa_data, dev->d_mac.addr, IFHWADDRLEN);
        break;

      case SIOCSIFHWADDR:  /* Set hardware address */
        req->ifr_hwaddr.sa_family = AF_INETX;
        memcpy(dev->d_mac.addr, req->ifr_hwaddr.sa_data, IFHWADDRLEN);
        break;

      case SIOCDIFADDR:  /* Delete IP address */
        ioctl_ifdown(dev);
        memset(&dev->d_ipaddr, 0, sizeof(uip_ipaddr_t));
        break;

      case SIOCGIFCOUNT:  /* Get number of devices */
        req->ifr_count = netdev_count();
        err = ENOSYS;
       break;

      case SIOCGIFBRDADDR:  /* Get broadcast IP address	*/
      case SIOCSIFBRDADDR:  /* Set broadcast IP address	*/
        err = ENOSYS;
        goto errout;

      default:
        err = EINVAL;
        goto errout;
    }

  return OK;

errout:
  *get_errno_ptr() = err;
  return ERROR;
}

#endif /* CONFIG_NET && CONFIG_NSOCKET_DESCRIPTORS */
