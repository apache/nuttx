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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_get_recvwindow
 *
 * Description:
 *   Calculate the TCP receive window for the specified device.
 *
 * Input Parameters:
 *   dev - The device whose TCP receive window will be updated.
 *
 * Returned Value:
 *   The value of the TCP receive window to use.
 *
 ****************************************************************************/

uint16_t tcp_get_recvwindow(FAR struct net_driver_s *dev)
{
  uint16_t iplen;
  uint16_t mss;
  uint16_t recvwndo;
#ifdef CONFIG_NET_TCP_READAHEAD
  int  navail;
#endif

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

  /* Calculate the packet MSS.
   *
   * REVISIT:  The actual TCP header length is variable.  TCP_HDRLEN
   * is the minimum size.
   */

  mss = dev->d_pktsize - (NET_LL_HDRLEN(dev) + iplen + TCP_HDRLEN);

#ifdef CONFIG_NET_TCP_READAHEAD
  /* Update the TCP received window based on read-ahead I/O buffer
   * availability.
   */

  navail = iob_navail();

  /* Are the read-ahead allocations throttled?  If so, then not all of these
   * IOBs are available for read-ahead buffering.
   *
   * REVISIT: Should also check that there is at least one available IOB
   * chain.
   */

#if CONFIG_IOB_THROTTLE > 0
  if (navail > CONFIG_IOB_THROTTLE)
    {
      navail -= CONFIG_IOB_THROTTLE;
    }
  else
    {
      navail = 0;
    }
#endif

  /* Are there any IOBs available for read-ahead buffering? */

  if (navail > 0)
    {
      uint32_t rwnd;

      /* The optimal TCP window size is the amount of TCP data that we can
       * currently buffer via TCP read-ahead buffering plus MSS for the
       * device packet buffer.  This logic here assumes that all IOBs are
       * available for TCP buffering.
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

      rwnd = (navail * CONFIG_IOB_BUFSIZE) + mss;
      if (rwnd > UINT16_MAX)
        {
          rwnd = UINT16_MAX;
        }

      /* Save the new receive window size */

      recvwndo = (uint16_t)rwnd;
    }
  else /* if (navail == 0) */
#endif
    {
      /* No IOBs are available.  The only buffering available is within the
       * packet buffer itself.  We can buffer no more than the MSS (unless
       * we are very fast).
       *
       * NOTE:  If no IOBs are available, then the next packet will be
       * lost if there is no listener on the connection.
       */

      recvwndo = mss;
    }

  return recvwndo;
}
