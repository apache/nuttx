/****************************************************************************
 * net/inet/ipv4_setsockopt.c
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

#include <sys/types.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>

#include <netinet/in.h>

#include "netdev/netdev.h"
#include "igmp/igmp.h"
#include "inet/inet.h"

#ifdef CONFIG_NET_IPv4

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_setsockopt
 *
 * Description:
 *   ipv4_setsockopt() sets the IPv4-protocol socket option specified by the
 *   'option' argument to the value pointed to by the 'value' argument for
 *   the socket specified by the 'psock' argument.
 *
 *   See <netinet/in.h> for the a complete list of values of IPv4 protocol
 *   socket options.
 *
 * Input Parameters:
 *   psock     Socket structure of socket to operate on
 *   option    identifies the option to set
 *   value     Points to the argument value
 *   value_len The length of the argument value
 *
 * Returned Value:
 *   Returns zero (OK) on success.  On failure, it returns a negated errno
 *   value to indicate the nature of the error.  See psock_setcockopt() for
 *   the list of possible error values.
 *
 ****************************************************************************/

int ipv4_setsockopt(FAR struct socket *psock, int option,
                    FAR const void *value, socklen_t value_len)
{
#ifdef CONFIG_NET_IGMP
  int ret;

  ninfo("option: %d\n", option);

  /* With IPv4, the multicast-related socket options are simply an alternative
   * way to access IGMP.  That IGMP functionality can also be accessed via
   * IOCTL commands (see netdev/netdev_ioctl.c)
   *
   * REVISIT:  Clone the logic from netdev_ioctl.c here.
   */

  net_lock();
  switch (option)
    {
      case IP_MSFILTER:    /* Access advanced, full-state filtering API */
        {
#if 0 /* REVISIT:  This is not a proper implementation of IP_MSGFILTER */
          FAR const struct ip_msfilter *imsf;
          FAR struct net_driver_s *dev;

          imsf = (FAR const struct ip_msfilter *)value;
          if (imsf == NULL || value_len < sizeof(struct ip_msfilter))
            {
              nerr("ERROR: Bad value or value_len\n");
              ret = -EINVAL;
            }
          else
            {
              /* Get the device associated with the local interface address */

              dev = netdev_findby_lipv4addr(imsf->imsf_interface.s_addr);
              if (dev == NULL)
                {
                  nwarn("WARNING: Could not find device for imsf_interface\n");
                  ret = -ENODEV;
                }
              else if (imsf->imsf_fmode == MCAST_INCLUDE)
                {
                  ret = igmp_joingroup(dev, &imsf->imsf_multiaddr);
                }
              else
                {
                  DEBUGASSERT(imsf->imsf_fmode == MCAST_EXCLUDE);
                  ret = igmp_leavegroup(dev, &imsf->imsf_multiaddr);
                }
            }
#else
          ret = -ENOSYS;
#endif
        }
        break;

      case IP_ADD_MEMBERSHIP:         /* Join a multicast group */
      case IP_DROP_MEMBERSHIP:        /* Leave a multicast group */
        {
          FAR const struct ip_mreq *mrec;
          FAR struct net_driver_s *dev;

          /* REVISIT:  This is not a proper implementation of IP_MSGFILTER */

          mrec = (FAR const struct ip_mreq *)value;
          if (mrec == NULL || value_len < sizeof(struct ip_mreq))
            {
              nerr("ERROR: Bad value or value_len\n");
              ret = -EINVAL;
            }
          else
            {
              /* Use the default network device is imr_interface is INADDRY_ANY. */

              if (mrec->imr_interface.s_addr == INADDR_ANY)
                {
                  dev = netdev_default();
                }
              else
                {
                  /* Get the device associated with the local interface address */

                  dev = netdev_findby_lipv4addr(mrec->imr_interface.s_addr);
                }

              if (dev == NULL)
                {
                  nwarn("WARNING: Could not find device for imr_interface\n");
                  ret = -ENODEV;
                }
              else if (option == IP_ADD_MEMBERSHIP)
                {
                  ret = igmp_joingroup(dev, &mrec->imr_multiaddr);
                }
              else
                {
                  ret = igmp_leavegroup(dev, &mrec->imr_multiaddr);
                }
            }
        }
        break;

      /* The following IPv4 socket options are defined, but not implemented */

      case IP_MULTICAST_IF:           /* Set local device for a multicast
                                       * socket */
      case IP_MULTICAST_TTL:          /* Set/read the time-to-live value of
                                       * outgoing multicast packets */
      case IP_MULTICAST_LOOP:         /* Set/read boolean that determines
                                       * whether sent multicast packets
                                       * should be looped back to local
                                       * sockets. */
      case IP_UNBLOCK_SOURCE:         /* Unblock previously blocked multicast
                                       * source */
      case IP_BLOCK_SOURCE:           /* Stop receiving multicast data from
                                       * source */
      case IP_ADD_SOURCE_MEMBERSHIP:  /* Join a multicast group; allow receive
                                       * only from source */
      case IP_DROP_SOURCE_MEMBERSHIP: /* Leave a source-specific group.  Stop
                                       * receiving data from a given multicast
                                       * group that come from a given source */
      case IP_MULTICAST_ALL:          /* Modify the delivery policy of
                                       * multicast messages bound to
                                       * INADDR_ANY */
#warning Missing logic
        nwarn("WARNING: Unimplemented IPv4 option: %d\n", option);
        ret = -ENOSYS;
        break;

      default:
        nerr("ERROR: Unrecognized IPv4 option: %d\n", option);
        ret = -ENOPROTOOPT;
        break;
    }

  net_unlock();
  return ret;
#else
  return -ENOPROTOOPT;
#endif
}

#endif /* CONFIG_NET_IPv4 */
