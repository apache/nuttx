/****************************************************************************
 * net/icmp/icmp_input.c
 * Handling incoming ICMP input
 *
 *   Copyright (C) 2007-2009, 2012, 2014-2015, 2017, 2019 Gregory Nutt. All
 *     rights reserved.
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
#ifdef CONFIG_NET

#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <net/if.h>
#include <arpa/inet.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>

#include "devif/devif.h"
#include "icmp/icmp.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_ICMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ICMPSIZE(hl) ((dev)->d_len - (hl))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_datahandler
 *
 * Description:
 *   Handle ICMP echo replies that are not accepted by the application.
 *
 * Input Parameters:
 *   dev    - Device instance only the input packet in d_buf, length = d_len;
 *   conn   - A pointer to the ICMP connection structure
 *   buffer - A pointer to the buffer to be copied to the read-ahead
 *     buffers
 *   buflen - The number of bytes to copy to the read-ahead buffer.
 *
 * Returned Value:
 *   The number of bytes actually buffered is returned.  This will be either
 *   zero or equal to buflen; partial packets are not buffered.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP_SOCKET
static uint16_t icmp_datahandler(FAR struct net_driver_s *dev,
                                 FAR struct icmp_conn_s *conn)
{
  FAR struct ipv4_hdr_s *ipv4;
  struct sockaddr_in inaddr;
  FAR struct iob_s *iob;
  uint16_t iphdrlen;
  uint16_t buflen;
  int ret;

  /* Put the IPv4 address at the beginning of the read-ahead buffer */

  iob               = dev->d_iob;
  ipv4              = IPv4BUF;
  inaddr.sin_family = AF_INET;
  inaddr.sin_port   = 0;

  net_ipv4addr_copy(inaddr.sin_addr.s_addr,
                    net_ip4addr_conv32(ipv4->srcipaddr));
  memset(inaddr.sin_zero, 0, sizeof(inaddr.sin_zero));

  /* Get the IP header length (accounting for possible options). */

  iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;

  /* Copy the src address info into the front of I/O buffer chain which
   * overwrites the contents of the packet header field.
   */

  memcpy(iob->io_data, &inaddr, sizeof(struct sockaddr_in));

  /* Copy the new ICMP reply into the I/O buffer chain (without waiting) */

  buflen = ICMPSIZE(iphdrlen);

  /* Trim l3 header */

  iob = iob_trimhead(iob, iphdrlen);

  /* Add the new I/O buffer chain to the tail of the read-ahead queue (again
   * without waiting).
   */

  ret = iob_tryadd_queue(iob, &conn->readahead);
  if (ret < 0)
    {
      nerr("ERROR: Failed to queue the I/O buffer chain: %d\n", ret);
      iob_free_chain(iob);
    }
  else
    {
      ninfo("Buffered %d bytes\n", buflen);
    }

  /* Device buffer must be enqueue or freed, clear the handle */

  netdev_iob_clear(dev);

  return buflen;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_input
 *
 * Description:
 *   Handle incoming ICMP input
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received ICMP
 *         packet
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void icmp_input(FAR struct net_driver_s *dev)
{
  FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
  FAR struct icmp_hdr_s *icmp;
  uint16_t iphdrlen;

#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmp.recv++;
#endif

  /* Get the IP header length (accounting for possible options). */

  iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;

  /* The ICMP header immediately follows the IP header */

  icmp = IPBUF(iphdrlen);

  /* ICMP echo (i.e., ping) processing. This is simple, we only change the
   * ICMP type from ECHO to ECHO_REPLY and adjust the ICMP checksum before
   * we return the packet.
   */

  if (icmp->type == ICMP_ECHO_REQUEST)
    {
      /* Change the ICMP type */

      icmp->type = ICMP_ECHO_REPLY;

      /* Swap IP addresses. */

      net_ipv4addr_hdrcopy(ipv4->destipaddr, ipv4->srcipaddr);
      net_ipv4addr_hdrcopy(ipv4->srcipaddr, &dev->d_ipaddr);

      /* Recalculate the ICMP checksum */

#if 0
      /* Get the IP header length (accounting for possible options). */

      iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;

      /* The slow way... sum over the ICMP message */

      icmp->icmpchksum = 0;
      icmp->icmpchksum = ~icmp_chksum(dev,
                                     (((uint16_t)ipv4->len[0] << 8) |
                                       (uint16_t)ipv4->len[1]) - iphdrlen);
      if (icmp->icmpchksum == 0)
        {
          icmp->icmpchksum = 0xffff;
        }
#else
      /* The quick way -- Since only the type has changed, just adjust the
       * checksum for the change of type
       */

      if (icmp->icmpchksum >= HTONS(0xffff - (ICMP_ECHO_REQUEST << 8)))
        {
          icmp->icmpchksum += HTONS(ICMP_ECHO_REQUEST << 8) + 1;
        }
      else
        {
          icmp->icmpchksum += HTONS(ICMP_ECHO_REQUEST << 8);
        }
#endif

      ninfo("Outgoing ICMP packet length: %d (%d)\n",
            dev->d_len, (ipv4->len[0] << 8) | ipv4->len[1]);

#ifdef CONFIG_NET_STATISTICS
      g_netstats.icmp.sent++;
      g_netstats.ipv4.sent++;
#endif
    }

#ifdef CONFIG_NET_ICMP_SOCKET
  /* If an ICMP echo reply is received then there should also be
   * a thread waiting to received the echo response.
   */

  else if (icmp->type == ICMP_ECHO_REPLY)
    {
      FAR struct icmp_conn_s *conn;
      uint16_t flags;

      /* Nothing consumed the ICMP reply.  That might because this is
       * an old, invalid reply or simply because the ping application
       * has not yet put its poll or recv in place.
       */

      /* Is there any connection that might expect this reply? */

      conn = icmp_findconn(dev, icmp->id);
      if (conn == NULL)
        {
          /* No.. drop the packet */

          goto drop;
        }

      flags = devif_conn_event(dev, ICMP_NEWDATA, conn->sconn.list);
      if ((flags & ICMP_NEWDATA) != 0)
        {
          uint16_t nbuffered;

          /* Add the ICMP echo reply to the IPPROTO_ICMP socket read-ahead
           * buffer.
           */

          nbuffered = icmp_datahandler(dev, conn);
          if (nbuffered == 0)
            {
              /* Could not buffer the data.. drop the packet */

              goto drop;
            }
        }
    }
#endif

  /* Otherwise the ICMP input was not processed */

  else
    {
      nwarn("WARNING: Unknown ICMP cmd: %d\n", icmp->type);
      goto typeerr;
    }

  return;

typeerr:
#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmp.typeerr++;
#endif

#ifdef CONFIG_NET_ICMP_SOCKET
drop:
#endif
#ifdef CONFIG_NET_STATISTICS
  g_netstats.icmp.drop++;
#endif
  dev->d_len = 0;
}

#endif /* CONFIG_NET_ICMP */
#endif /* CONFIG_NET */
