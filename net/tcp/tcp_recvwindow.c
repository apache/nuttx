/****************************************************************************
 * net/tcp/tcp_recvwindow.c
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

#include <stdint.h>
#include <debug.h>

#include <net/if.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/tcp.h>

#include "tcp/tcp.h"

#ifdef CONFIG_NET_TCP_RWND_CONTROL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_update_recvwindws
 *
 * Description:
 *   Update the TCP receive window for the specified device.
 *
 * Input Parameters:
 *   dev - The device whose TCP receive window will be updated.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void tcp_update_recvwindws(FAR struct net_driver_s *dev)
{
  uint16_t iplen;
  uint16_t overhead;
  int  navail;

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
  if (IFF_IS_IPv6(dev->d_flags))
#endif
    {
      iplen = IPv6_HDRLEN;
    }
#endif /* CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
  else
#endif
    {
      iplen = IPv4_HDRLEN;
    }
#endif /* CONFIG_NET_IPv4 */

  /* Update the TCP received window based on I/O buffer availability
   *
   * REVISIT:  The actual TCP header length is varialble.  TCP_HDRLEN
   * is the minimum size.
   */

  overhead = NET_LL_HDRLEN(dev) + iplen + TCP_HDRLEN;
  navail   = iob_navail();
  if (navail > 0)
    {
      uint32_t rwnd;

      /* The optimal TCP window size is the amount of TCP data that we can
       * currently buffer via TCP read-ahead buffering minus overhead for the
       * link-layer, IP, and TCP headers.  This logic here assumes that
       * all IOBs are available for TCP buffering.
       *
       * Assume that all of the available IOBs are can be used for buffering
       * on this connection.  Also assume that at least one chain is available
       * concatenate the IOBs.
       *
       * REVISIT:  In an environment with multiple, active read-ahead TCP
       * sockets (and perhaps multiple network devices) or if there are
       * other consumers of IOBs (such as for TCP write buffering) then the
       * total number of IOBs will all not be available for read-ahead
       * buffering for this connection.
       */

      rwnd = (navail * CONFIG_IOB_BUFSIZE) - overhead;
      if (rwnd > UINT16_MAX)
        {
          rwnd = UINT16_MAX;
        }

      /* Save the new receive window size */

      NET_DEV_RCVWNDO(dev) = (uint16_t)rwnd;
    }
  else /* if (navail == 0) */
    {
      /* No IOBs are available... fall back to the configured default
       * which assumes no write buffering.  The only buffering available
       * is within the packet buffer itself.
       *
       * NOTE:  If no IOBs are available, then the next packet will be
       * lost if there is no listener on the connection.
       */

      NET_DEV_RCVWNDO(dev) = dev->d_mtu - overhead;
    }
}

#endif /* CONFIG_NET_TCP_RWND_CONTROL */
