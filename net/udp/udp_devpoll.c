/****************************************************************************
 * net/udp/udp_devpoll.c
 * Network device poll for the availability of UDP TX data
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_UDP)

#include <assert.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/udp.h>

#include "devif/devif.h"
#include "udp/udp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_poll
 *
 * Description:
 *   Poll a UDP "connection" structure for availability of TX data
 *
 * Input Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - The UDP "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *   dev is not NULL.
 *   conn is not NULL.
 *   The connection (conn) is bound to the polling device (dev) in case of
 *   enabled CONFIG_NET_UDP_WRITE_BUFFERS option.
 *
 ****************************************************************************/

void udp_poll(FAR struct net_driver_s *dev, FAR struct udp_conn_s *conn)
{
  DEBUGASSERT(dev != NULL && conn != NULL);

#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
  DEBUGASSERT(dev == conn->dev);
#endif

  /* Verify that the UDP connection is valid */

  if (conn->lport != 0)
    {
      dev->d_len    = 0;
      dev->d_sndlen = 0;

      /* Perform the application callback */

      udp_callback(dev, conn, UDP_POLL);

      /* If the application has data to send, setup the UDP/IP header */

      if (dev->d_sndlen > 0)
        {
          udp_send(dev, conn);
          return;
        }
    }

  /* Make sure that d_len is zero meaning that there is nothing to be sent */

  dev->d_len   = 0;
}

#endif /* CONFIG_NET && CONFIG_NET_UDP */
