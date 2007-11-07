/****************************************************************************
 * net/uip/uip-tcpsend.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/types.h>
#include <debug.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#include "uip-internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define BUF ((struct uip_tcpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_tcpsendcomplete
 *
 * Description:
 *   Complete the final portions of the send operation.  This function sets
 *   up IP header and computes the TCP checksum
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

static void uip_tcpsendcomplete(struct uip_driver_s *dev)
{
  BUF->ttl         = UIP_TTL;

#ifdef CONFIG_NET_IPv6

  /* For IPv6, the IP length field does not include the IPv6 IP header
   * length.
   */

  BUF->len[0]      = ((dev->d_len - UIP_IPH_LEN) >> 8);
  BUF->len[1]      = ((dev->d_len - UIP_IPH_LEN) & 0xff);

#else /* CONFIG_NET_IPv6 */

  BUF->len[0]      = (dev->d_len >> 8);
  BUF->len[1]      = (dev->d_len & 0xff);

#endif /* CONFIG_NET_IPv6 */

  BUF->urgp[0]     = BUF->urgp[1] = 0;

  /* Calculate TCP checksum. */

  BUF->tcpchksum   = 0;
  BUF->tcpchksum   = ~(uip_tcpchksum(dev));

#ifdef CONFIG_NET_IPv6

  BUF->vtc         = 0x60;
  BUF->tcf         = 0x00;
  BUF->flow        = 0x00;

#else /* CONFIG_NET_IPv6 */

  BUF->vhl         = 0x45;
  BUF->tos         = 0;
  BUF->ipoffset[0] = 0;
  BUF->ipoffset[1] = 0;
  ++g_ipid;
  BUF->ipid[0]     = g_ipid >> 8;
  BUF->ipid[1]     = g_ipid & 0xff;

  /* Calculate IP checksum. */

  BUF->ipchksum    = 0;
  BUF->ipchksum    = ~(uip_ipchksum(dev));

#endif /* CONFIG_NET_IPv6 */

  vdbg("Outgoing TCP packet length: %d (%d)\n",
       dev->d_len, (BUF->len[0] << 8) | BUF->len[1]);

#ifdef CONFIG_NET_STATISTICS
  uip_stat.tcp.sent++;
  uip_stat.ip.sent++;
#endif
}

/****************************************************************************
 * Name: uip_tcpsendcommon
 *
 * Description:
 *   We're done with the input processing. We are now ready to send a reply
 *   Our job is to fill in all the fields of the TCP and IP headers before
 *   calculating the checksum and finally send the packet.
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

static void uip_tcpsendcommon(struct uip_driver_s *dev, struct uip_conn *conn)
{
  BUF->ackno[0] = conn->rcv_nxt[0];
  BUF->ackno[1] = conn->rcv_nxt[1];
  BUF->ackno[2] = conn->rcv_nxt[2];
  BUF->ackno[3] = conn->rcv_nxt[3];

  BUF->seqno[0] = conn->snd_nxt[0];
  BUF->seqno[1] = conn->snd_nxt[1];
  BUF->seqno[2] = conn->snd_nxt[2];
  BUF->seqno[3] = conn->snd_nxt[3];

  BUF->proto    = UIP_PROTO_TCP;

  BUF->srcport  = conn->lport;
  BUF->destport = conn->rport;

  uiphdr_ipaddr_copy(BUF->srcipaddr, &dev->d_ipaddr);
  uiphdr_ipaddr_copy(BUF->destipaddr, &conn->ripaddr);

  if (conn->tcpstateflags & UIP_STOPPED)
    {
      /* If the connection has issued uip_stop(), we advertise a zero
       * window so that the remote host will stop sending data.
       */

      BUF->wnd[0] = 0;
      BUF->wnd[1] = 0;
    }
  else
    {
      BUF->wnd[0] = ((UIP_RECEIVE_WINDOW) >> 8);
      BUF->wnd[1] = ((UIP_RECEIVE_WINDOW) & 0xff);
    }

  /* Finish the IP portion of the message, calculate checksums and send
   * the message.
   */

  uip_tcpsendcomplete(dev);

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: uip_tcpsend
 *
 * Description:
 *   Setup to send a TCP packet
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   flags  - flags to apply to the TCP header
 *   len    - length of the message
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void uip_tcpsend(struct uip_driver_s *dev, struct uip_conn *conn, uint8 flags, uint16 len)
{
  BUF->flags     = flags;
  dev->d_len     = len;
  BUF->tcpoffset = (UIP_TCPH_LEN / 4) << 4;
  uip_tcpsendcommon(dev, conn);
}

/****************************************************************************
 * Name: uip_tcpreset
 *
 * Description:
 *   Send a TCP reset (no-data) message
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void uip_tcpreset(struct uip_driver_s *dev)
{
  uint16 tmp16;
  uint8  seqbyte;

#ifdef CONFIG_NET_STATISTICS
  uip_stat.tcp.rst++;
#endif

  BUF->flags     = TCP_RST | TCP_ACK;
  dev->d_len     = UIP_IPTCPH_LEN;
  BUF->tcpoffset = 5 << 4;

  /* Flip the seqno and ackno fields in the TCP header. */

  seqbyte        = BUF->seqno[3];
  BUF->seqno[3]  = BUF->ackno[3];
  BUF->ackno[3]  = seqbyte;

  seqbyte        = BUF->seqno[2];
  BUF->seqno[2]  = BUF->ackno[2];
  BUF->ackno[2]  = seqbyte;

  seqbyte        = BUF->seqno[1];
  BUF->seqno[1]  = BUF->ackno[1];
  BUF->ackno[1]  = seqbyte;

  seqbyte        = BUF->seqno[0];
  BUF->seqno[0]  = BUF->ackno[0];
  BUF->ackno[0]  = seqbyte;

  /* We also have to increase the sequence number we are
   * acknowledging. If the least significant byte overflowed, we need
   * to propagate the carry to the other bytes as well.
   */

  if (++(BUF->ackno[3]) == 0)
    {
      if (++(BUF->ackno[2]) == 0)
        {
          if (++(BUF->ackno[1]) == 0)
            {
              ++(BUF->ackno[0]);
            }
        }
    }

  /* Swap port numbers. */

  tmp16         = BUF->srcport;
  BUF->srcport  = BUF->destport;
  BUF->destport = tmp16;

  /* Swap IP addresses. */

  uiphdr_ipaddr_copy(BUF->destipaddr, BUF->srcipaddr);
  uiphdr_ipaddr_copy(BUF->srcipaddr, dev->d_ipaddr);

  /* And send out the RST packet */

  uip_tcpsendcomplete(dev);
}

/****************************************************************************
 * Name: uip_tcpack
 *
 * Description:
 *   Send the SYN or SYNACK response.
 *
 * Parameters:
 *   dev    - The device driver structure to use in the send operation
 *   conn   - The TCP connection structure holding connection information
 *   ack    - The ACK response to send
 *
 * Return:
 *   None
 *
 * Assumptions:
 *   Called from the interrupt level or with interrupts disabled.
 *
 ****************************************************************************/

void uip_tcpack(struct uip_driver_s *dev, struct uip_conn *conn, uint8 ack)
{
  /* Save the ACK bits */

  BUF->flags = ack;

  /* We send out the TCP Maximum Segment Size option with our ack. */

  BUF->optdata[0] = TCP_OPT_MSS;
  BUF->optdata[1] = TCP_OPT_MSS_LEN;
  BUF->optdata[2] = (UIP_TCP_MSS) / 256;
  BUF->optdata[3] = (UIP_TCP_MSS) & 255;
  dev->d_len      = UIP_IPTCPH_LEN + TCP_OPT_MSS_LEN;
  BUF->tcpoffset  = ((UIP_TCPH_LEN + TCP_OPT_MSS_LEN) / 4) << 4;

  /* Complete the common portions of the TCP message */

  uip_tcpsendcommon(dev, conn);
}

#endif /* CONFIG_NET */
