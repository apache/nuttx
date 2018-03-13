/****************************************************************************
 * net/tcp/tcp_devpoll.c
 * Driver poll for the availability of TCP TX data
 *
 *   Copyright (C) 2007-2009, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Adapted for NuttX from logic in uIP which also has a BSD-like license:
 *
 *   Original author Adam Dunkels <adam@dunkels.com>
 *   Copyright () 2001-2003, Adam Dunkels.
 *   All rights reserved.
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_NET) && defined(CONFIG_NET_TCP)

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/tcp.h>

#include "devif/devif.h"
#include "tcp/tcp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_poll
 *
 * Description:
 *   Poll a TCP connection structure for availability of TX data
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *   conn - The TCP "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called with the network locked.
 *
 ****************************************************************************/

void tcp_poll(FAR struct net_driver_s *dev, FAR struct tcp_conn_s *conn)
{
  uint16_t result;

  /* Discard any currently buffered data */

  dev->d_len     = 0;
  dev->d_sndlen  = 0;

  /* Verify that the connection is established. */

  if ((conn->tcpstateflags & TCP_STATE_MASK) == TCP_ESTABLISHED)
    {
      /* The TCP connection is established and, hence, should be bound
       * to a device. Make sure that the polling device is the one that
       * we are bound to.
       */

      DEBUGASSERT(conn->dev != NULL);
      if (dev == conn->dev)
        {
          /* Set up for the callback.  We can't know in advance if the
           * application is going to send a IPv4 or an IPv6 packet, so this
           * setup may not actually be used.
           */

#if defined(CONFIG_NET_IPv6) && defined(CONFIG_NET_IPv4)
          if (conn->domain == PF_INET)
            {
              tcp_ipv4_select(dev);
            }
          else
            {
              tcp_ipv6_select(dev);
            }

#elif defined(CONFIG_NET_IPv4)
          tcp_ipv4_select(dev);

#else /* if defined(CONFIG_NET_IPv6) */
          tcp_ipv6_select(dev);
#endif
          /* Perform the callback */

          result = tcp_callback(dev, conn, TCP_POLL);

          /* Handle the callback response */

          tcp_appsend(dev, conn, result);
        }
    }
}

#endif /* CONFIG_NET && CONFIG_NET_TCP */
