/****************************************************************************
 * net/tcp/tcp_recvwindow.c
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

#include <stdint.h>
#include <stdbool.h>
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

uint16_t tcp_get_recvwindow(FAR struct net_driver_s *dev,
                            FAR struct tcp_conn_s *conn)
{
  uint16_t iplen;
  uint16_t mss;
  uint16_t recvwndo;
  int niob_avail;
  int nqentry_avail;

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

  /* Update the TCP received window based on read-ahead I/O buffer
   * and IOB chain availability.  At least one queue entry is required.
   * If one queue entry is available, then the amount of read-ahead
   * data that can be buffered is given by the number of IOBs available
   * (ignoring competition with other IOB consumers).
   */

  niob_avail    = iob_navail(true);
  nqentry_avail = iob_qentry_navail();

  /* Is there a a queue entry and IOBs available for read-ahead buffering? */

  if (nqentry_avail > 0 && niob_avail > 0)
    {
      uint32_t rwnd;

      /* The optimal TCP window size is the amount of TCP data that we can
       * currently buffer via TCP read-ahead buffering for the device packet
       * buffer.  This logic here assumes that all IOBs are available for
       * TCP buffering.
       *
       * Assume that all of the available IOBs are can be used for buffering
       * on this connection.  Also assume that at least one chain is
       * available concatenate the IOBs.
       *
       * REVISIT:  In an environment with multiple, active read-ahead TCP
       * sockets (and perhaps multiple network devices) or if there are
       * other consumers of IOBs (such as for TCP write buffering) then the
       * total number of IOBs will all not be available for read-ahead
       * buffering for this connection.
       */

      rwnd = (niob_avail * CONFIG_IOB_BUFSIZE);
      if (rwnd > UINT16_MAX)
        {
          rwnd = UINT16_MAX;
        }

      /* Save the new receive window size */

      recvwndo = (uint16_t)rwnd;
    }
  else if (IOB_QEMPTY(&conn->readahead))
    {
      /* Advertise maximum segment size for window edge if here is no
       * available iobs on current "free" connection.
       */

      recvwndo = mss;
    }
  else /* nqentry_avail == 0 || niob_avail == 0 */
    {
      /* No IOB chains or noIOBs are available.
       * Advertise the edge of window to zero.
       *
       * NOTE:  If no IOBs are available, then the next packet will be
       * lost if there is no listener on the connection.
       */

      recvwndo = 0;
    }

  return recvwndo;
}
