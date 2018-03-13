/****************************************************************************
 * net/pkt/pkt_input.c
 * Handling incoming packet input
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_PKT)

#include <debug.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/pkt.h>

#include "devif/devif.h"
#include "pkt/pkt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PKTBUF ((struct eth_hdr_s *)&dev->d_buf)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_input
 *
 * Description:
 *   Handle incoming packet input
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received packet
 *
 * Returned Value:
 *   OK    The packet has been processed  and can be deleted
 *   ERROR There is a matching connection, but could not dispatch the packet
 *         yet.  Useful when a packet arrives before a recv call is in
 *         place.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

int pkt_input(struct net_driver_s *dev)
{
  FAR struct pkt_conn_s *conn;
  FAR struct eth_hdr_s  *pbuf = (struct eth_hdr_s *)dev->d_buf;
  int ret = OK;

  conn = pkt_active(pbuf);
  if (conn)
    {
      uint16_t flags;

      /* Setup for the application callback */

      dev->d_appdata = dev->d_buf;
      dev->d_sndlen  = 0;

      /* Perform the application callback */

      flags = pkt_callback(dev, conn, PKT_NEWDATA);

      /* If the operation was successful, the PKT_NEWDATA flag is removed
       * and thus the packet can be deleted (OK will be returned).
       */

      if ((flags & PKT_NEWDATA) != 0)
        {
          /* No.. the packet was not processed now.  Return ERROR so
           * that the driver may retry again later.  We still need to
           * set d_len to zero so that the driver is aware that there
           * is nothing to be sent.
           */

           nwarn("WARNING: Packet not processed\n");
           //dev->d_len = 0; REVISIT
           ret = ERROR;
        }
    }
  else
    {
      nerr("ERROR: No listener\n");
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_PKT */
