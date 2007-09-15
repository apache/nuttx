/****************************************************************************
 * uip.c
 * The uIP TCP/IP stack code.
 * author Adam Dunkels <adam@dunkels.com>
 *
 * uIP is an implementation of the TCP/IP protocol stack intended for
 * small 8-bit and 16-bit microcontrollers.
 *
 * uIP provides the necessary protocols for Internet communication,
 * with a very small code footprint and RAM requirements - the uIP
 * code size is on the order of a few kilobytes and RAM usage is on
 * the order of a few hundred bytes.
 *
 * Copyright (c) 2001-2003, Adam Dunkels.
 * All rights reserved.
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
 * uIP is a small implementation of the IP, UDP and TCP protocols (as
 * well as some basic ICMP stuff). The implementation couples the IP,
 * UDP, TCP and the application layers very tightly. To keep the size
 * of the compiled code down, this code frequently uses the goto
 * statement. While it would be possible to break the uip_interrupt()
 * function into many smaller functions, this would increase the code
 * size because of the overhead of parameter passing and the fact that
 * the optimier would not be as efficient.
 *
 * The principle is that we have a small buffer, called the d_buf,
 * in which the device driver puts an incoming packet. The TCP/IP
 * stack parses the headers in the packet, and calls the
 * application. If the remote host has sent data to the application,
 * this data is present in the d_buf and the application read the
 * data from there. It is up to the application to put this data into
 * a byte stream if needed. The application will not be fed with data
 * that is out of sequence.
 *
 * If the application whishes to send data to the peer, it should put
 * its data into the d_buf. The d_appdata pointer points to the
 * first available byte. The TCP/IP stack will calculate the
 * checksums, and fill in the necessary header fields and finally send
 * the packet back to the peer.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <debug.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#ifdef CONFIG_NET_IPv6
# include "uip-neighbor.h"
#endif /* CONFIG_NET_IPv6 */

#include <string.h>

#if UIP_LOGGING == 1
#include <stdio.h>
extern void uip_log(char *msg);
# define UIP_LOG(m) uip_log(m)
#else
# define UIP_LOG(m)
#endif /* UIP_LOGGING == 1 */

#include "uip-internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define TCP_FIN 0x01
#define TCP_SYN 0x02
#define TCP_RST 0x04
#define TCP_PSH 0x08
#define TCP_ACK 0x10
#define TCP_URG 0x20
#define TCP_CTL 0x3f

#define TCP_OPT_END     0   /* End of TCP options list */
#define TCP_OPT_NOOP    1   /* "No-operation" TCP option */
#define TCP_OPT_MSS     2   /* Maximum segment size TCP option */

#define TCP_OPT_MSS_LEN 4   /* Length of TCP MSS option. */

#define ICMP_ECHO_REPLY 0
#define ICMP_ECHO       8

#define ICMP6_ECHO_REPLY             129
#define ICMP6_ECHO                   128
#define ICMP6_NEIGHBOR_SOLICITATION  135
#define ICMP6_NEIGHBOR_ADVERTISEMENT 136

#define ICMP6_FLAG_S (1 << 6)

#define ICMP6_OPTION_SOURCE_LINK_ADDRESS 1
#define ICMP6_OPTION_TARGET_LINK_ADDRESS 2

/* Macros. */

#define BUF     ((struct uip_tcpip_hdr *)&dev->d_buf[UIP_LLH_LEN])
#define FBUF    ((struct uip_tcpip_hdr *)&uip_reassbuf[0])
#define ICMPBUF ((struct uip_icmpip_hdr *)&dev->d_buf[UIP_LLH_LEN])
#define UDPBUF  ((struct uip_udpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* The IP address of this host. If it is defined to be fixed (by
   setting UIP_FIXEDADDR to 1 in uipopt.h), the address is set
   here. Otherwise, the address */
#if UIP_FIXEDADDR > 0
const uip_ipaddr_t uip_hostaddr =
  {HTONS((UIP_IPADDR0 << 8) | UIP_IPADDR1),
   HTONS((UIP_IPADDR2 << 8) | UIP_IPADDR3)};

const uip_ipaddr_t uip_draddr =
  {HTONS((UIP_DRIPADDR0 << 8) | UIP_DRIPADDR1),
   HTONS((UIP_DRIPADDR2 << 8) | UIP_DRIPADDR3)};

const uip_ipaddr_t uip_netmask =
  {HTONS((UIP_NETMASK0 << 8) | UIP_NETMASK1),
   HTONS((UIP_NETMASK2 << 8) | UIP_NETMASK3)};
#else
uip_ipaddr_t uip_hostaddr;
uip_ipaddr_t uip_draddr;
uip_ipaddr_t uip_netmask;
#endif /* UIP_FIXEDADDR */

#if UIP_URGDATA > 0
void *uip_urgdata;               /* The uip_urgdata pointer points to urgent data
                                  * (out-of-band data), if present. */
uint16 uip_urglen;
uint16 uip_surglen;
#endif /* UIP_URGDATA > 0 */

uint8  uip_flags;                /* The uip_flags variable is used for communication
                                  * between the TCP/IP stack and the application
                                  * program. */
struct uip_conn *uip_conn;       /* uip_conn always points to the current connection. */

uint16 uip_listenports[UIP_LISTENPORTS];
                                 /* The uip_listenports list all currently listening ports. */
#ifdef CONFIG_NET_UDP
struct uip_udp_conn *uip_udp_conn;
#endif   /* CONFIG_NET_UDP */

/* Temporary variables. */

uint8 uip_acc32[4];

#if UIP_STATISTICS == 1
struct uip_stats uip_stat;
# define UIP_STAT(s) s
#else
# define UIP_STAT(s)
#endif /* UIP_STATISTICS == 1 */

const uip_ipaddr_t all_ones_addr =
#ifdef CONFIG_NET_IPv6
  {0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff,0xffff};
#else /* CONFIG_NET_IPv6 */
  {0xffff,0xffff};
#endif /* CONFIG_NET_IPv6 */

const uip_ipaddr_t all_zeroes_addr =
#ifdef CONFIG_NET_IPv6
  {0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000,0x0000};
#else /* CONFIG_NET_IPv6 */
  {0x0000,0x0000};
#endif /* CONFIG_NET_IPv6 */

/****************************************************************************
 * Private Variables
 ****************************************************************************/

#if UIP_FIXEDETHADDR
const struct uip_eth_addr uip_ethaddr =
{{ UIP_ETHADDR0, UIP_ETHADDR1, UIP_ETHADDR2, UIP_ETHADDR3, UIP_ETHADDR4, UIP_ETHADDR5 }};
#else
struct uip_eth_addr uip_ethaddr = {{ 0,0,0,0,0,0 }};
#endif

static uint16 ipid;              /* Ths ipid variable is an increasing number that is
                                  * used for the IP ID field. */

/* Temporary variables. */

static uint8  c;
static uint8  opt;
static uint16 tmp16;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if !UIP_ARCH_CHKSUM
static uint16 chksum(uint16 sum, const uint8 *data, uint16 len)
{
  uint16 t;
  const uint8 *dataptr;
  const uint8 *last_byte;

  dataptr = data;
  last_byte = data + len - 1;

  while(dataptr < last_byte)
    {
      /* At least two more bytes */

      t = (dataptr[0] << 8) + dataptr[1];
      sum += t;
      if (sum < t)
        {
          sum++; /* carry */
        }
      dataptr += 2;
    }

  if (dataptr == last_byte)
    {
      t = (dataptr[0] << 8) + 0;
      sum += t;
      if (sum < t)
        {
          sum++; /* carry */
        }
    }

  /* Return sum in host byte order. */

  return sum;
}

static uint16 upper_layer_chksum(struct uip_driver_s *dev, uint8 proto)
{
  uint16 upper_layer_len;
  uint16 sum;

#ifdef CONFIG_NET_IPv6
  upper_layer_len = (((uint16)(BUF->len[0]) << 8) + BUF->len[1]);
#else /* CONFIG_NET_IPv6 */
  upper_layer_len = (((uint16)(BUF->len[0]) << 8) + BUF->len[1]) - UIP_IPH_LEN;
#endif /* CONFIG_NET_IPv6 */

  /* First sum pseudoheader. */

  /* IP protocol and length fields. This addition cannot carry. */
  sum = upper_layer_len + proto;

  /* Sum IP source and destination addresses. */
  sum = chksum(sum, (uint8 *)&BUF->srcipaddr[0], 2 * sizeof(uip_ipaddr_t));

  /* Sum TCP header and data. */
  sum = chksum(sum, &dev->d_buf[UIP_IPH_LEN + UIP_LLH_LEN], upper_layer_len);

  return (sum == 0) ? 0xffff : htons(sum);
}

#ifdef CONFIG_NET_IPv6
static uint16 uip_icmp6chksum(struct uip_driver_s *dev)
{
  return upper_layer_chksum(dev, UIP_PROTO_ICMP6);
}
#endif /* CONFIG_NET_IPv6 */

#endif /* UIP_ARCH_CHKSUM */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* This function may be used at boot time to set the initial ip_id.*/

void uip_setipid(uint16 id)
{
  ipid = id;
}

/* Calculate the Internet checksum over a buffer. */

#if !UIP_ARCH_ADD32
void uip_add32(uint8 *op32, uint16 op16)
{
  uip_acc32[3] = op32[3] + (op16 & 0xff);
  uip_acc32[2] = op32[2] + (op16 >> 8);
  uip_acc32[1] = op32[1];
  uip_acc32[0] = op32[0];

  if (uip_acc32[2] < (op16 >> 8))
    {
      ++uip_acc32[1];
      if (uip_acc32[1] == 0)
        {
          ++uip_acc32[0];
        }
    }

  if (uip_acc32[3] < (op16 & 0xff))
    {
      ++uip_acc32[2];
      if (uip_acc32[2] == 0)
        {
          ++uip_acc32[1];
          if (uip_acc32[1] == 0)
            {
              ++uip_acc32[0];
            }
        }
    }
}
#endif /* UIP_ARCH_ADD32 */

#if !UIP_ARCH_CHKSUM
uint16 uip_chksum(uint16 *data, uint16 len)
{
  return htons(chksum(0, (uint8 *)data, len));
}

/* Calculate the IP header checksum of the packet header in d_buf. */

#ifndef UIP_ARCH_IPCHKSUM
uint16 uip_ipchksum(struct uip_driver_s *dev)
{
  uint16 sum;

  sum = chksum(0, &dev->d_buf[UIP_LLH_LEN], UIP_IPH_LEN);
  dbg("uip_ipchksum: sum 0x%04x\n", sum);
  return (sum == 0) ? 0xffff : htons(sum);
}
#endif

/* Calculate the TCP checksum of the packet in d_buf and d_appdata. */

uint16 uip_tcpchksum(struct uip_driver_s *dev)
{
  return upper_layer_chksum(dev, UIP_PROTO_TCP);
}

/* Calculate the UDP checksum of the packet in d_buf and d_appdata. */

#ifdef CONFIG_NET_UDP_CHECKSUMS
uint16 uip_udpchksum(struct uip_driver_s *dev)
{
  return upper_layer_chksum(dev, UIP_PROTO_UDP);
}
#endif /* UIP_UDP_CHECKSUMS */
#endif /* UIP_ARCH_CHKSUM */

void uip_init(void)
{
  for (c = 0; c < UIP_LISTENPORTS; ++c)
    {
      uip_listenports[c] = 0;
    }

  /* Initialize the TCP/IP connection structures */

  uip_tcpinit();

  /* Initialize the UDP connection structures */

  uip_udpinit();

  /* IPv4 initialization. */
#if UIP_FIXEDADDR == 0
  /*  uip_hostaddr[0] = uip_hostaddr[1] = 0;*/
#endif /* UIP_FIXEDADDR */
}

void uip_unlisten(uint16 port)
{
  for (c = 0; c < UIP_LISTENPORTS; ++c)
    {
      if (uip_listenports[c] == port)
        {
          uip_listenports[c] = 0;
          return;
        }
    }
}

void uip_listen(uint16 port)
{
  for (c = 0; c < UIP_LISTENPORTS; ++c)
    {
      if (uip_listenports[c] == 0)
        {
          uip_listenports[c] = port;
          return;
        }
    }
}

/* IP fragment reassembly: not well-tested. */

#if UIP_REASSEMBLY && !defined(CONFIG_NET_IPv6)
#define UIP_REASS_BUFSIZE (UIP_BUFSIZE - UIP_LLH_LEN)
static uint8 uip_reassbuf[UIP_REASS_BUFSIZE];
static uint8 uip_reassbitmap[UIP_REASS_BUFSIZE / (8 * 8)];
static const uint8 bitmap_bits[8] = {0xff, 0x7f, 0x3f, 0x1f, 0x0f, 0x07, 0x03, 0x01};
static uint16 uip_reasslen;
static uint8 uip_reassflags;
#define UIP_REASS_FLAG_LASTFRAG 0x01
static uint8 uip_reasstmr;

#define IP_MF   0x20

static uint8 uip_reass(void)
{
  uint16 offset, len;
  uint16 i;

  /* If ip_reasstmr is zero, no packet is present in the buffer, so we
   * write the IP header of the fragment into the reassembly
   * buffer. The timer is updated with the maximum age.
   */

  if (uip_reasstmr == 0)
    {
      memcpy(uip_reassbuf, &BUF->vhl, UIP_IPH_LEN);
      uip_reasstmr = UIP_REASS_MAXAGE;
      uip_reassflags = 0;

      /* Clear the bitmap. */
      memset(uip_reassbitmap, 0, sizeof(uip_reassbitmap));
    }

  /* Check if the incoming fragment matches the one currently present
   * in the reasembly buffer. If so, we proceed with copying the
   * fragment into the buffer.
   */

  if (BUF->srcipaddr[0] == FBUF->srcipaddr[0] && BUF->srcipaddr[1] == FBUF->srcipaddr[1] &&
      BUF->destipaddr[0] == FBUF->destipaddr[0] && BUF->destipaddr[1] == FBUF->destipaddr[1] &&
      BUF->ipid[0] == FBUF->ipid[0] && BUF->ipid[1] == FBUF->ipid[1])
    {
      len = (BUF->len[0] << 8) + BUF->len[1] - (BUF->vhl & 0x0f) * 4;
      offset = (((BUF->ipoffset[0] & 0x3f) << 8) + BUF->ipoffset[1]) * 8;

      /* If the offset or the offset + fragment length overflows the
       * reassembly buffer, we discard the entire packet.
       */

      if (offset > UIP_REASS_BUFSIZE || offset + len > UIP_REASS_BUFSIZE)
        {
          uip_reasstmr = 0;
          goto nullreturn;
        }

      /* Copy the fragment into the reassembly buffer, at the right offset. */

      memcpy(&uip_reassbuf[UIP_IPH_LEN + offset], (char *)BUF + (int)((BUF->vhl & 0x0f) * 4), len);

    /* Update the bitmap. */

    if (offset / (8 * 8) == (offset + len) / (8 * 8))
      {
        /* If the two endpoints are in the same byte, we only update that byte. */

        uip_reassbitmap[offset / (8 * 8)] |=
          bitmap_bits[(offset / 8 ) & 7] & ~bitmap_bits[((offset + len) / 8 ) & 7];

      }
    else
      {
        /* If the two endpoints are in different bytes, we update the bytes
         * in the endpoints and fill the stuff inbetween with 0xff.
         */

        uip_reassbitmap[offset / (8 * 8)] |= bitmap_bits[(offset / 8 ) & 7];
        for (i = 1 + offset / (8 * 8); i < (offset + len) / (8 * 8); ++i)
          {
            uip_reassbitmap[i] = 0xff;
          }
        uip_reassbitmap[(offset + len) / (8 * 8)] |= ~bitmap_bits[((offset + len) / 8 ) & 7];
      }

    /* If this fragment has the More Fragments flag set to zero, we know that
     * this is the last fragment, so we can calculate the size of the entire
     * packet. We also set the IP_REASS_FLAG_LASTFRAG flag to indicate that
     * we have received the final fragment.
     */

    if ((BUF->ipoffset[0] & IP_MF) == 0)
      {
        uip_reassflags |= UIP_REASS_FLAG_LASTFRAG;
        uip_reasslen = offset + len;
      }

    /* Finally, we check if we have a full packet in the buffer. We do this
     * by checking if we have the last fragment and if all bits in the bitmap
     * are set.
     */

    if (uip_reassflags & UIP_REASS_FLAG_LASTFRAG)
      {
        /* Check all bytes up to and including all but the last byte in
         * the bitmap.
         */

        for (i = 0; i < uip_reasslen / (8 * 8) - 1; ++i)
          {
            if (uip_reassbitmap[i] != 0xff)
              {
                goto nullreturn;
              }
          }

        /* Check the last byte in the bitmap. It should contain just the
         * right amount of bits.
         */

        if (uip_reassbitmap[uip_reasslen / (8 * 8)] != (uint8)~bitmap_bits[uip_reasslen / 8 & 7])
          {
            goto nullreturn;
          }

        /* If we have come this far, we have a full packet in the buffer,
         * so we allocate a pbuf and copy the packet into it. We also reset
         * the timer.
         */

        uip_reasstmr = 0;
        memcpy(BUF, FBUF, uip_reasslen);

        /* Pretend to be a "normal" (i.e., not fragmented) IP packet from
         * now on.
         */

        BUF->ipoffset[0] = BUF->ipoffset[1] = 0;
        BUF->len[0] = uip_reasslen >> 8;
        BUF->len[1] = uip_reasslen & 0xff;
        BUF->ipchksum = 0;
        BUF->ipchksum = ~(uip_ipchksum(dev));

        return uip_reasslen;
      }
  }

nullreturn:
  return 0;
}
#endif /* UIP_REASSEMBLY */

static void uip_add_rcv_nxt(uint16 n)
{
  uip_add32(uip_conn->rcv_nxt, n);
  uip_conn->rcv_nxt[0] = uip_acc32[0];
  uip_conn->rcv_nxt[1] = uip_acc32[1];
  uip_conn->rcv_nxt[2] = uip_acc32[2];
  uip_conn->rcv_nxt[3] = uip_acc32[3];
}

static void uip_udp_callback(struct uip_driver_s *dev)
{
  /* Some sanity checking */

  if (uip_udp_conn && uip_udp_conn->event)
    {
      /* Perform the callback */

      uip_udp_conn->event(dev, uip_udp_conn->private);
    }
}

static void uip_tcp_callback(struct uip_driver_s *dev)
{
  /* Some sanity checking */

  if (uip_conn)
    {
      /* Check if there is a data callback */

      if (uip_conn->data_event)
      {
        /* Perform the callback */

        uip_conn->data_event(dev, uip_conn->data_private);
      }

      /* Check if there is a connection-related event and a connection
       * callback.
       */
      if (((uip_flags & UIP_CONN_EVENTS) != 0) && uip_conn->connection_event)
        {
          /* Perform the callback */

          uip_conn->connection_event(uip_conn->connection_private);
        }
    }
}

void uip_interrupt(struct uip_driver_s *dev, uint8 flag)
{
  register struct uip_conn *uip_connr = uip_conn;

#ifdef CONFIG_NET_UDP
  if (flag == UIP_UDP_SEND_CONN)
    {
      goto udp_send;
    }
#endif   /* CONFIG_NET_UDP */

  dev->d_snddata = dev->d_appdata = &dev->d_buf[UIP_IPTCPH_LEN + UIP_LLH_LEN];

  /* Check if we were invoked because of a poll request for a
   * particular connection.
   */

  if (flag == UIP_POLL_REQUEST)
    {
      if ((uip_connr->tcpstateflags & UIP_TS_MASK) == UIP_ESTABLISHED &&
           !uip_outstanding(uip_connr))
        {
          uip_flags = UIP_POLL;
          uip_tcp_callback(dev);
          goto appsend;
        }
        goto drop;
    }

  /* Check if we were invoked because of the perodic timer firing. */

  else if (flag == UIP_TIMER)
    {
#if UIP_REASSEMBLY
      if (uip_reasstmr != 0)
        {
          --uip_reasstmr;
        }
#endif /* UIP_REASSEMBLY */

      /* Increase the TCP sequence number */

      uip_tcpnextsequence();

      /* Reset the length variables. */

      dev->d_len  = 0;
      dev->d_sndlen = 0;

      /* Check if the connection is in a state in which we simply wait
       * for the connection to time out. If so, we increase the
       * connection's timer and remove the connection if it times
       * out.
       */

      if (uip_connr->tcpstateflags == UIP_TIME_WAIT ||
          uip_connr->tcpstateflags == UIP_FIN_WAIT_2)
        {
          ++(uip_connr->timer);
          if (uip_connr->timer == UIP_TIME_WAIT_TIMEOUT)
            {
              uip_connr->tcpstateflags = UIP_CLOSED;
            }
        }
      else if (uip_connr->tcpstateflags != UIP_CLOSED)
        {
          /* If the connection has outstanding data, we increase the
           * connection's timer and see if it has reached the RTO value
           * in which case we retransmit.
           */

        if (uip_outstanding(uip_connr))
          {
            if (uip_connr->timer-- == 0)
              {
                if (uip_connr->nrtx == UIP_MAXRTX ||
                    ((uip_connr->tcpstateflags == UIP_SYN_SENT ||
                    uip_connr->tcpstateflags == UIP_SYN_RCVD) &&
                    uip_connr->nrtx == UIP_MAXSYNRTX))
                  {
                    uip_connr->tcpstateflags = UIP_CLOSED;

                    /* We call uip_tcp_callback() with uip_flags set to
                     * UIP_TIMEDOUT to inform the application that the
                     * connection has timed out.
                     */

                    uip_flags = UIP_TIMEDOUT;
                    uip_tcp_callback(dev);

                    /* We also send a reset packet to the remote host. */

                    BUF->flags = TCP_RST | TCP_ACK;
                    goto tcp_send_nodata;
                  }

                /* Exponential backoff. */

                uip_connr->timer = UIP_RTO << (uip_connr->nrtx > 4 ? 4: uip_connr->nrtx);
                ++(uip_connr->nrtx);

                /* Ok, so we need to retransmit. We do this differently
                * depending on which state we are in. In ESTABLISHED, we
                 * call upon the application so that it may prepare the
                 * data for the retransmit. In SYN_RCVD, we resend the
                 * SYNACK that we sent earlier and in LAST_ACK we have to
                 * retransmit our FINACK.
                 */

                UIP_STAT(++uip_stat.tcp.rexmit);
                switch(uip_connr->tcpstateflags & UIP_TS_MASK)
                  {
                    case UIP_SYN_RCVD:
                      /* In the SYN_RCVD state, we should retransmit our
                       * SYNACK.
                       */

                      goto tcp_send_synack;

                    case UIP_SYN_SENT:
                      /* In the SYN_SENT state, we retransmit out SYN. */
                      BUF->flags = 0;
                      goto tcp_send_syn;

                    case UIP_ESTABLISHED:
                      /* In the ESTABLISHED state, we call upon the application
                      * to do the actual retransmit after which we jump into
                       * the code for sending out the packet (the apprexmit
                       * label).
                       */

                      uip_flags = UIP_REXMIT;
                      uip_tcp_callback(dev);
                      goto apprexmit;

                    case UIP_FIN_WAIT_1:
                    case UIP_CLOSING:
                    case UIP_LAST_ACK:
                      /* In all these states we should retransmit a FINACK. */
                      goto tcp_send_finack;

                  }
              }
          }
        else if ((uip_connr->tcpstateflags & UIP_TS_MASK) == UIP_ESTABLISHED)
          {
            /* If there was no need for a retransmission, we poll the
            * application for new data.
             */

            uip_flags = UIP_POLL;
            uip_tcp_callback(dev);
            goto appsend;
          }
      }
    goto drop;
  }

#ifdef CONFIG_NET_UDP
  if (flag == UIP_UDP_TIMER)
    {
      if (uip_udp_conn->lport != 0)
        {
          uip_conn = NULL;
          dev->d_snddata = dev->d_appdata = &dev->d_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN];
          dev->d_len = 0;
          dev->d_sndlen = 0;
          uip_flags = UIP_POLL;
          uip_udp_callback(dev);
          goto udp_send;
        }
      else
        {
          goto drop;
        }
    }
#endif

  /* This is where the input processing starts. */
  UIP_STAT(++uip_stat.ip.recv);

  /* Start of IP input header processing code. */

#ifdef CONFIG_NET_IPv6
  /* Check validity of the IP header. */
  if ((BUF->vtc & 0xf0) != 0x60) 
    {
      /* IP version and header length. */
      UIP_STAT(++uip_stat.ip.drop);
      UIP_STAT(++uip_stat.ip.vhlerr);
      UIP_LOG("ipv6: invalid version.");
      goto drop;
    }
#else /* CONFIG_NET_IPv6 */
    /* Check validity of the IP header. */
    if (BUF->vhl != 0x45)
      {
        /* IP version and header length. */
        UIP_STAT(++uip_stat.ip.drop);
        UIP_STAT(++uip_stat.ip.vhlerr);
        UIP_LOG("ip: invalid version or header length.");
        goto drop;
      }
#endif /* CONFIG_NET_IPv6 */

    /* Check the size of the packet. If the size reported to us in
       d_len is smaller the size reported in the IP header, we assume
       that the packet has been corrupted in transit. If the size of
       d_len is larger than the size reported in the IP packet header,
       the packet has been padded and we set d_len to the correct
       value.. */

    if ((BUF->len[0] << 8) + BUF->len[1] <= dev->d_len)
      {
        dev->d_len = (BUF->len[0] << 8) + BUF->len[1];
#ifdef CONFIG_NET_IPv6
        dev->d_len += 40; /* The length reported in the IPv6 header is the
                          length of the payload that follows the
                          header. However, uIP uses the d_len variable
                          for holding the size of the entire packet,
                          including the IP header. For IPv4 this is not a
                          problem as the length field in the IPv4 header
                          contains the length of the entire packet. But
                          for IPv6 we need to add the size of the IPv6
                          header (40 bytes). */
#endif /* CONFIG_NET_IPv6 */
      }
    else
      {
        UIP_LOG("ip: packet shorter than reported in IP header.");
        goto drop;
      }

#ifndef CONFIG_NET_IPv6
    /* Check the fragment flag. */

    if ((BUF->ipoffset[0] & 0x3f) != 0 ||
          BUF->ipoffset[1] != 0)
      {
#if UIP_REASSEMBLY
        dev->d_len = uip_reass();
        if (dev->d_len == 0)
          {
            goto drop;
          }
#else /* UIP_REASSEMBLY */
        UIP_STAT(++uip_stat.ip.drop);
        UIP_STAT(++uip_stat.ip.fragerr);
        UIP_LOG("ip: fragment dropped.");
        goto drop;
#endif /* UIP_REASSEMBLY */
      }
#endif /* CONFIG_NET_IPv6 */

    if (uip_ipaddr_cmp(uip_hostaddr, all_zeroes_addr))
      {
        /* If we are configured to use ping IP address configuration and
           hasn't been assigned an IP address yet, we accept all ICMP
           packets. */
#if UIP_PINGADDRCONF && !CONFIG_NET_IPv6
        if (BUF->proto == UIP_PROTO_ICMP)
          {
            UIP_LOG("ip: possible ping config packet received.");
            goto icmp_input;
          }
        else
          {
            UIP_LOG("ip: packet dropped since no address assigned.");
            goto drop;
          }
#endif /* UIP_PINGADDRCONF */

      }
    else
      {
        /* If IP broadcast support is configured, we check for a broadcast
           UDP packet, which may be destined to us. */
#if UIP_BROADCAST
        dbg("UDP IP checksum 0x%04x\n", uip_ipchksum(dev));
        if (BUF->proto == UIP_PROTO_UDP &&
             uip_ipaddr_cmp(BUF->destipaddr, all_ones_addr)
       /*&& uip_ipchksum(dev) == 0xffff*/)
          {
            goto udp_input;
          }
#endif /* UIP_BROADCAST */

        /* Check if the packet is destined for our IP address. */
#ifndef CONFIG_NET_IPv6
        if (!uip_ipaddr_cmp(BUF->destipaddr, uip_hostaddr))
          {
            UIP_STAT(++uip_stat.ip.drop);
          goto drop;
          }
#else /* CONFIG_NET_IPv6 */
        /* For IPv6, packet reception is a little trickier as we need to
           make sure that we listen to certain multicast addresses (all
           hosts multicast address, and the solicited-node multicast
           address) as well. However, we will cheat here and accept all
           multicast packets that are sent to the ff02::/16 addresses. */
        if (!uip_ipaddr_cmp(BUF->destipaddr, uip_hostaddr) &&
             BUF->destipaddr[0] != HTONS(0xff02))
          {
            UIP_STAT(++uip_stat.ip.drop);
            goto drop;
          }
#endif /* CONFIG_NET_IPv6 */
      }

#ifndef CONFIG_NET_IPv6
    if (uip_ipchksum(dev) != 0xffff)
      {
        /* Compute and check the IP header checksum. */

        UIP_STAT(++uip_stat.ip.drop);
        UIP_STAT(++uip_stat.ip.chkerr);
        UIP_LOG("ip: bad checksum.");
        goto drop;
      }
#endif /* CONFIG_NET_IPv6 */

    if (BUF->proto == UIP_PROTO_TCP)
      {
        /* Check for TCP packet. If so, proceed with TCP input processing. */

        goto tcp_input;
      }

#ifdef CONFIG_NET_UDP
    if (BUF->proto == UIP_PROTO_UDP)
      {
        goto udp_input;
      }
#endif   /* CONFIG_NET_UDP */

#ifndef CONFIG_NET_IPv6
    /* ICMPv4 processing code follows. */
    if (BUF->proto != UIP_PROTO_ICMP)
      {
        /* We only allow ICMP packets from here. */

        UIP_STAT(++uip_stat.ip.drop);
        UIP_STAT(++uip_stat.ip.protoerr);
        UIP_LOG("ip: neither tcp nor icmp.");
        goto drop;
      }

#if UIP_PINGADDRCONF
 icmp_input:
#endif /* UIP_PINGADDRCONF */
    UIP_STAT(++uip_stat.icmp.recv);

    /* ICMP echo (i.e., ping) processing. This is simple, we only change
      the ICMP type from ECHO to ECHO_REPLY and adjust the ICMP
       checksum before we return the packet. */
    if (ICMPBUF->type != ICMP_ECHO)
      {
        UIP_STAT(++uip_stat.icmp.drop);
        UIP_STAT(++uip_stat.icmp.typeerr);
        UIP_LOG("icmp: not icmp echo.");
        goto drop;
      }

    /* If we are configured to use ping IP address assignment, we use
      the destination IP address of this ping packet and assign it to
      ourself. */
#if UIP_PINGADDRCONF
    if ((uip_hostaddr[0] | uip_hostaddr[1]) == 0)
      {
        uip_hostaddr[0] = BUF->destipaddr[0];
        uip_hostaddr[1] = BUF->destipaddr[1];
      }
#endif /* UIP_PINGADDRCONF */

    ICMPBUF->type = ICMP_ECHO_REPLY;

    if (ICMPBUF->icmpchksum >= HTONS(0xffff - (ICMP_ECHO << 8)))
      {
        ICMPBUF->icmpchksum += HTONS(ICMP_ECHO << 8) + 1;
      }
    else
      {
        ICMPBUF->icmpchksum += HTONS(ICMP_ECHO << 8);
      }

    /* Swap IP addresses. */

    uip_ipaddr_copy(BUF->destipaddr, BUF->srcipaddr);
    uip_ipaddr_copy(BUF->srcipaddr, uip_hostaddr);

    UIP_STAT(++uip_stat.icmp.sent);
    goto send;

    /* End of IPv4 input header processing code. */
#else /* !CONFIG_NET_IPv6 */

    /* This is IPv6 ICMPv6 processing code. */
    dbg("icmp6_input: length %d\n", dev->d_len);

    if (BUF->proto != UIP_PROTO_ICMP6)
      {
        /* We only allow ICMPv6 packets from here. */

        UIP_STAT(++uip_stat.ip.drop);
        UIP_STAT(++uip_stat.ip.protoerr);
        UIP_LOG("ip: neither tcp nor icmp6.");
        goto drop;
      }

    UIP_STAT(++uip_stat.icmp.recv);

    /* If we get a neighbor solicitation for our address we should send
      a neighbor advertisement message back. */
    if (ICMPBUF->type == ICMP6_NEIGHBOR_SOLICITATION)
      {
        if (uip_ipaddr_cmp(ICMPBUF->icmp6data, uip_hostaddr))
          {
            if (ICMPBUF->options[0] == ICMP6_OPTION_SOURCE_LINK_ADDRESS)
              {
                /* Save the sender's address in our neighbor list. */
                uip_neighbor_add(ICMPBUF->srcipaddr, &(ICMPBUF->options[2]));
              }

            /* We should now send a neighbor advertisement back to where the
              neighbor solicication came from. */
            ICMPBUF->type = ICMP6_NEIGHBOR_ADVERTISEMENT;
            ICMPBUF->flags = ICMP6_FLAG_S; /* Solicited flag. */

            ICMPBUF->reserved1 = ICMPBUF->reserved2 = ICMPBUF->reserved3 = 0;

            uip_ipaddr_copy(ICMPBUF->destipaddr, ICMPBUF->srcipaddr);
            uip_ipaddr_copy(ICMPBUF->srcipaddr, uip_hostaddr);
            ICMPBUF->options[0] = ICMP6_OPTION_TARGET_LINK_ADDRESS;
            ICMPBUF->options[1] = 1;  /* Options length, 1 = 8 bytes. */
            memcpy(&(ICMPBUF->options[2]), &uip_ethaddr, sizeof(uip_ethaddr));
            ICMPBUF->icmpchksum = 0;
            ICMPBUF->icmpchksum = ~uip_icmp6chksum(dev);
            goto send;
          }
        goto drop;
      }
    else if (ICMPBUF->type == ICMP6_ECHO)
      {
        /* ICMP echo (i.e., ping) processing. This is simple, we only
           change the ICMP type from ECHO to ECHO_REPLY and update the
           ICMP checksum before we return the packet. */

        ICMPBUF->type = ICMP6_ECHO_REPLY;

        uip_ipaddr_copy(BUF->destipaddr, BUF->srcipaddr);
        uip_ipaddr_copy(BUF->srcipaddr, uip_hostaddr);
        ICMPBUF->icmpchksum = 0;
        ICMPBUF->icmpchksum = ~uip_icmp6chksum(dev);

        UIP_STAT(++uip_stat.icmp.sent);
        goto send;
      }
    else
      {
        dbg("Unknown icmp6 message type %d\n", ICMPBUF->type);
        UIP_STAT(++uip_stat.icmp.drop);
        UIP_STAT(++uip_stat.icmp.typeerr);
        UIP_LOG("icmp: unknown ICMP message.");
        goto drop;
      }

    /* End of IPv6 ICMP processing. */

#endif /* !CONFIG_NET_IPv6 */

#ifdef CONFIG_NET_UDP
    /* UDP input processing. */
 udp_input:
    /* UDP processing is really just a hack. We don't do anything to the
       UDP/IP headers, but let the UDP application do all the hard
       work. If the application sets d_sndlen, it has a packet to
       send. */
#ifdef CONFIG_NET_UDP_CHECKSUMS
    dev->d_len    -= UIP_IPUDPH_LEN;
    dev->d_appdata = &dev->d_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN];
    if (UDPBUF->udpchksum != 0 && uip_udpchksum(dev) != 0xffff)
      {
        UIP_STAT(++uip_stat.udp.drop);
        UIP_STAT(++uip_stat.udp.chkerr);
        UIP_LOG("udp: bad checksum.");
        goto drop;
      }
#else /* UIP_UDP_CHECKSUMS */
    dev->d_len -= UIP_IPUDPH_LEN;
#endif /* UIP_UDP_CHECKSUMS */

    /* Demultiplex this UDP packet between the UDP "connections". */

    uip_udp_conn = uip_udpactive(UDPBUF);
    if (uip_udp_conn)
      {
        goto udp_found;
      }

    UIP_LOG("udp: no matching connection found");
    goto drop;

 udp_found:
    uip_conn = NULL;
    uip_flags = UIP_NEWDATA;
    dev->d_snddata = dev->d_appdata = &dev->d_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN];
    dev->d_sndlen = 0;
    uip_udp_callback(dev);

 udp_send:
    if (dev->d_sndlen == 0)
      {
        goto drop;
      }
    dev->d_len = dev->d_sndlen + UIP_IPUDPH_LEN;

#ifdef CONFIG_NET_IPv6
    /* For IPv6, the IP length field does not include the IPv6 IP header
      length. */
    BUF->len[0] = ((dev->d_len - UIP_IPH_LEN) >> 8);
    BUF->len[1] = ((dev->d_len - UIP_IPH_LEN) & 0xff);
#else /* CONFIG_NET_IPv6 */
    BUF->len[0] = (dev->d_len >> 8);
    BUF->len[1] = (dev->d_len & 0xff);
#endif /* CONFIG_NET_IPv6 */

    BUF->ttl = uip_udp_conn->ttl;
    BUF->proto = UIP_PROTO_UDP;

    UDPBUF->udplen = HTONS(dev->d_sndlen + UIP_UDPH_LEN);
    UDPBUF->udpchksum = 0;

    BUF->srcport  = uip_udp_conn->lport;
    BUF->destport = uip_udp_conn->rport;

    uip_ipaddr_copy(BUF->srcipaddr, uip_hostaddr);
    uip_ipaddr_copy(BUF->destipaddr, uip_udp_conn->ripaddr);

    dev->d_appdata = &dev->d_buf[UIP_LLH_LEN + UIP_IPTCPH_LEN];

#ifdef CONFIG_NET_UDP_CHECKSUMS
    /* Calculate UDP checksum. */
    UDPBUF->udpchksum = ~(uip_udpchksum(dev));
    if (UDPBUF->udpchksum == 0)
      {
        UDPBUF->udpchksum = 0xffff;
      }
#endif /* UIP_UDP_CHECKSUMS */

    goto ip_send_nolen;
#endif   /* CONFIG_NET_UDP */

    /* TCP input processing. */
 tcp_input:
    UIP_STAT(++uip_stat.tcp.recv);

    /* Start of TCP input header processing code. */

    if (uip_tcpchksum(dev) != 0xffff)
      {
        /* Compute and check the TCP checksum. */

        UIP_STAT(++uip_stat.tcp.drop);
        UIP_STAT(++uip_stat.tcp.chkerr);
        UIP_LOG("tcp: bad checksum.");
        goto drop;
      }

    /* Demultiplex this segment. First check any active connections. */

    uip_connr = uip_tcpactive(BUF);
    if (uip_connr)
      {
        goto found;
      }

    /* If we didn't find and active connection that expected the packet,
     * either this packet is an old duplicate, or this is a SYN packet
     * destined for a connection in LISTEN. If the SYN flag isn't set,
     * it is an old packet and we send a RST.
     */

    if ((BUF->flags & TCP_CTL) != TCP_SYN)
      {
        goto reset;
      }

    tmp16 = BUF->destport;

    /* Next, check listening connections. */
    for (c = 0; c < UIP_LISTENPORTS; ++c)
      {
        if (tmp16 == uip_listenports[c])
          goto found_listen;
      }

    /* No matching connection found, so we send a RST packet. */
    UIP_STAT(++uip_stat.tcp.synrst);
 reset:

    /* We do not send resets in response to resets. */
    if (BUF->flags & TCP_RST)
      {
        goto drop;
      }

    UIP_STAT(++uip_stat.tcp.rst);

    BUF->flags = TCP_RST | TCP_ACK;
    dev->d_len = UIP_IPTCPH_LEN;
    BUF->tcpoffset = 5 << 4;

    /* Flip the seqno and ackno fields in the TCP header. */
    c = BUF->seqno[3];
    BUF->seqno[3] = BUF->ackno[3];
    BUF->ackno[3] = c;

    c = BUF->seqno[2];
    BUF->seqno[2] = BUF->ackno[2];
    BUF->ackno[2] = c;

    c = BUF->seqno[1];
    BUF->seqno[1] = BUF->ackno[1];
    BUF->ackno[1] = c;

    c = BUF->seqno[0];
    BUF->seqno[0] = BUF->ackno[0];
    BUF->ackno[0] = c;

    /* We also have to increase the sequence number we are
       acknowledging. If the least significant byte overflowed, we need
       to propagate the carry to the other bytes as well. */
    if (++BUF->ackno[3] == 0)
      {
        if (++BUF->ackno[2] == 0)
          {
            if (++BUF->ackno[1] == 0)
              {
                ++BUF->ackno[0];
              }
          }
      }

    /* Swap port numbers. */
    tmp16 = BUF->srcport;
    BUF->srcport = BUF->destport;
    BUF->destport = tmp16;

    /* Swap IP addresses. */
    uip_ipaddr_copy(BUF->destipaddr, BUF->srcipaddr);
    uip_ipaddr_copy(BUF->srcipaddr, uip_hostaddr);

    /* And send out the RST packet! */
    goto tcp_send_noconn;

    /* This label will be jumped to if we matched the incoming packet
     * with a connection in LISTEN. In that case, we should create a new
     * connection and send a SYNACK in return.
     */

found_listen:

    /* First allocate a new connection structure */

    uip_connr = uip_tcplistener(BUF);
    if (!uip_connr)
      {
        /* All connections are used already, we drop packet and hope that
         * the remote end will retransmit the packet at a time when we
         * have more spare connections.
         */

        UIP_STAT(++uip_stat.tcp.syndrop);
        UIP_LOG("tcp: found no unused connections.");
        goto drop;
      }

    uip_add_rcv_nxt(1);
    uip_conn = uip_connr;

    /* Parse the TCP MSS option, if present. */

    if ((BUF->tcpoffset & 0xf0) > 0x50)
      {
        for (c = 0; c < ((BUF->tcpoffset >> 4) - 5) << 2 ;)
          {
            opt = dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN + c];
            if (opt == TCP_OPT_END)
              {
                /* End of options. */
                break;
              }
            else if (opt == TCP_OPT_NOOP)
              {
                ++c;
                /* NOP option. */
              }
            else if (opt == TCP_OPT_MSS &&
                       dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN + 1 + c] == TCP_OPT_MSS_LEN)
              {
                /* An MSS option with the right option length. */
                tmp16 = ((uint16)dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN + 2 + c] << 8) |
                         (uint16)dev->d_buf[UIP_IPTCPH_LEN + UIP_LLH_LEN + 3 + c];
                uip_connr->initialmss = uip_connr->mss =
                        tmp16 > UIP_TCP_MSS? UIP_TCP_MSS: tmp16;

                /* And we are done processing options. */
                break;
              }
            else
              {
                /* All other options have a length field, so that we easily
                   can skip past them. */
                if (dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN + 1 + c] == 0)
                  {
                      /* If the length field is zero, the options are malformed
                         and we don't process them further. */
                      break;
                  }
                c += dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN + 1 + c];
              }
          }
      }

    /* Our response will be a SYNACK. */
tcp_send_synack:
    BUF->flags = TCP_ACK;

 tcp_send_syn:
    BUF->flags |= TCP_SYN;

    /* We send out the TCP Maximum Segment Size option with our
       SYNACK. */
    BUF->optdata[0] = TCP_OPT_MSS;
    BUF->optdata[1] = TCP_OPT_MSS_LEN;
    BUF->optdata[2] = (UIP_TCP_MSS) / 256;
    BUF->optdata[3] = (UIP_TCP_MSS) & 255;
    dev->d_len = UIP_IPTCPH_LEN + TCP_OPT_MSS_LEN;
    BUF->tcpoffset = ((UIP_TCPH_LEN + TCP_OPT_MSS_LEN) / 4) << 4;
    goto tcp_send;

    /* This label will be jumped to if we found an active connection. */
 found:
    uip_conn = uip_connr;
    uip_flags = 0;

    /* We do a very naive form of TCP reset processing; we just accept
     * any RST and kill our connection. We should in fact check if the
     * sequence number of this reset is wihtin our advertised window
     * before we accept the reset.
     */

    if (BUF->flags & TCP_RST)
      {
        uip_connr->tcpstateflags = UIP_CLOSED;
        UIP_LOG("tcp: got reset, aborting connection.");
        uip_flags = UIP_ABORT;
        uip_tcp_callback(dev);
        goto drop;
      }

    /* Calculated the length of the data, if the application has sent
     * any data to us.
     */

    c = (BUF->tcpoffset >> 4) << 2;

    /* d_len will contain the length of the actual TCP data. This is
     * calculated by subtracing the length of the TCP header (in
     * c) and the length of the IP header (20 bytes).
     */

    dev->d_len -= (c + UIP_IPH_LEN);

    /* First, check if the sequence number of the incoming packet is
     * what we're expecting next. If not, we send out an ACK with the
     * correct numbers in.
     */

    if (!(((uip_connr->tcpstateflags & UIP_TS_MASK) == UIP_SYN_SENT) &&
        ((BUF->flags & TCP_CTL) == (TCP_SYN | TCP_ACK))))
      {
        if ((dev->d_len > 0 || ((BUF->flags & (TCP_SYN | TCP_FIN)) != 0)) &&
            (BUF->seqno[0] != uip_connr->rcv_nxt[0] ||
            BUF->seqno[1] != uip_connr->rcv_nxt[1] ||
            BUF->seqno[2] != uip_connr->rcv_nxt[2] ||
            BUF->seqno[3] != uip_connr->rcv_nxt[3]))
          {
            goto tcp_send_ack;
          }
      }

    /* Next, check if the incoming segment acknowledges any outstanding
       data. If so, we update the sequence number, reset the length of
       the outstanding data, calculate RTT estimations, and reset the
       retransmission timer. */
    if ((BUF->flags & TCP_ACK) && uip_outstanding(uip_connr))
      {
        uip_add32(uip_connr->snd_nxt, uip_connr->len);

        if (BUF->ackno[0] == uip_acc32[0] &&
            BUF->ackno[1] == uip_acc32[1] &&
            BUF->ackno[2] == uip_acc32[2] &&
            BUF->ackno[3] == uip_acc32[3])
          {
            /* Update sequence number. */
            uip_connr->snd_nxt[0] = uip_acc32[0];
            uip_connr->snd_nxt[1] = uip_acc32[1];
            uip_connr->snd_nxt[2] = uip_acc32[2];
            uip_connr->snd_nxt[3] = uip_acc32[3];

            /* Do RTT estimation, unless we have done retransmissions. */
            if (uip_connr->nrtx == 0)
              {
                signed char m;
                m = uip_connr->rto - uip_connr->timer;

                /* This is taken directly from VJs original code in his paper */
                m = m - (uip_connr->sa >> 3);
                uip_connr->sa += m;
                if (m < 0)
                  {
                    m = -m;
                  }
                m = m - (uip_connr->sv >> 2);
                uip_connr->sv += m;
                uip_connr->rto = (uip_connr->sa >> 3) + uip_connr->sv;
              }

            /* Set the acknowledged flag. */
            uip_flags = UIP_ACKDATA;

            /* Reset the retransmission timer. */
            uip_connr->timer = uip_connr->rto;

            /* Reset length of outstanding data. */
            uip_connr->len = 0;
          }
      }

    /* Do different things depending on in what state the connection is. */
    switch(uip_connr->tcpstateflags & UIP_TS_MASK)
      {
        /* CLOSED and LISTEN are not handled here. CLOSE_WAIT is not
           implemented, since we force the application to close when the
           peer sends a FIN (hence the application goes directly from
           ESTABLISHED to LAST_ACK). */
        case UIP_SYN_RCVD:
          /* In SYN_RCVD we have sent out a SYNACK in response to a SYN, and
           * we are waiting for an ACK that acknowledges the data we sent
           * out the last time. Therefore, we want to have the UIP_ACKDATA
           * flag set. If so, we enter the ESTABLISHED state.
           */

          if (uip_flags & UIP_ACKDATA)
            {
              uip_connr->tcpstateflags = UIP_ESTABLISHED;
              uip_flags                = UIP_CONNECTED;
              uip_connr->len           = 0;

              if (dev->d_len > 0)
                {
                  uip_flags           |= UIP_NEWDATA;
                  uip_add_rcv_nxt(dev->d_len);
                }

              dev->d_sndlen            = 0;
              uip_tcp_callback(dev);
              goto appsend;
            }
          goto drop;

        case UIP_SYN_SENT:
          /* In SYN_SENT, we wait for a SYNACK that is sent in response to
             our SYN. The rcv_nxt is set to sequence number in the SYNACK
             plus one, and we send an ACK. We move into the ESTABLISHED
             state. */
          if ((uip_flags & UIP_ACKDATA) &&
              (BUF->flags & TCP_CTL) == (TCP_SYN | TCP_ACK))
            {
              /* Parse the TCP MSS option, if present. */
              if ((BUF->tcpoffset & 0xf0) > 0x50)
                {
                  for (c = 0; c < ((BUF->tcpoffset >> 4) - 5) << 2 ;)
                    {
                      opt = dev->d_buf[UIP_IPTCPH_LEN + UIP_LLH_LEN + c];
                      if (opt == TCP_OPT_END)
                        {
                          /* End of options. */
                          break;
                        }
                      else if (opt == TCP_OPT_NOOP)
                        {
                          ++c;
                          /* NOP option. */
                        }
                      else if (opt == TCP_OPT_MSS &&
                                dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN + 1 + c] == TCP_OPT_MSS_LEN)
                        {
                          /* An MSS option with the right option length. */
                          tmp16 = (dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN + 2 + c] << 8) |
                          dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN + 3 + c];
                          uip_connr->initialmss =
                            uip_connr->mss = tmp16 > UIP_TCP_MSS? UIP_TCP_MSS: tmp16;

                          /* And we are done processing options. */
                          break;
                        }
                      else
                        {
                          /* All other options have a length field, so that we easily
                             can skip past them. */
                          if (dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN + 1 + c] == 0)
                            {
                              /* If the length field is zero, the options are malformed
                                 and we don't process them further. */
                              break;
                            }
                          c += dev->d_buf[UIP_TCPIP_HLEN + UIP_LLH_LEN + 1 + c];
                        }
                    }
                }

              uip_connr->tcpstateflags = UIP_ESTABLISHED;
              uip_connr->rcv_nxt[0] = BUF->seqno[0];
              uip_connr->rcv_nxt[1] = BUF->seqno[1];
              uip_connr->rcv_nxt[2] = BUF->seqno[2];
              uip_connr->rcv_nxt[3] = BUF->seqno[3];
              uip_add_rcv_nxt(1);
              uip_flags = UIP_CONNECTED | UIP_NEWDATA;
              uip_connr->len = 0;
              dev->d_len = 0;
              dev->d_sndlen = 0;
              uip_tcp_callback(dev);
              goto appsend;
            }

          /* Inform the application that the connection failed */
          uip_flags = UIP_ABORT;
          uip_tcp_callback(dev);

          /* The connection is closed after we send the RST */
          uip_conn->tcpstateflags = UIP_CLOSED;
          goto reset;

        case UIP_ESTABLISHED:
          /* In the ESTABLISHED state, we call upon the application to feed
             data into the d_buf. If the UIP_ACKDATA flag is set, the
             application should put new data into the buffer, otherwise we are
             retransmitting an old segment, and the application should put that
             data into the buffer.

             If the incoming packet is a FIN, we should close the connection on
             this side as well, and we send out a FIN and enter the LAST_ACK
             state. We require that there is no outstanding data; otherwise the
             sequence numbers will be screwed up. */

          if (BUF->flags & TCP_FIN && !(uip_connr->tcpstateflags & UIP_STOPPED))
            {
              if (uip_outstanding(uip_connr))
                {
                  goto drop;
                }

              uip_add_rcv_nxt(dev->d_len + 1);
              uip_flags |= UIP_CLOSE;

              if (dev->d_len > 0)
                {
                  uip_flags |= UIP_NEWDATA;
                }

              uip_tcp_callback(dev);
              uip_connr->len = 1;
              uip_connr->tcpstateflags = UIP_LAST_ACK;
              uip_connr->nrtx = 0;

 tcp_send_finack:
              BUF->flags = TCP_FIN | TCP_ACK;
              goto tcp_send_nodata;
            }

          /* Check the URG flag. If this is set, the segment carries urgent
             data that we must pass to the application. */
          if ((BUF->flags & TCP_URG) != 0)
            {
#if UIP_URGDATA > 0
              uip_urglen = (BUF->urgp[0] << 8) | BUF->urgp[1];
              if (uip_urglen > dev->d_len)
                {
                  /* There is more urgent data in the next segment to come. */
                  uip_urglen = dev->d_len;
                }
              uip_add_rcv_nxt(uip_urglen);
              dev->d_len -= uip_urglen;
              uip_urgdata = dev->d_appdata;
              dev->d_appdata += uip_urglen;
            }
          else
            {
              uip_urglen = 0;
#else /* UIP_URGDATA > 0 */
              dev->d_appdata = ((char *)dev->d_appdata) + ((BUF->urgp[0] << 8) | BUF->urgp[1]);
              dev->d_len -= (BUF->urgp[0] << 8) | BUF->urgp[1];
#endif /* UIP_URGDATA > 0 */
            }

          /* If d_len > 0 we have TCP data in the packet, and we flag this
             by setting the UIP_NEWDATA flag and update the sequence number
             we acknowledge. If the application has stopped the dataflow
             using uip_stop(), we must not accept any data packets from the
             remote host. */
          if (dev->d_len > 0 && !(uip_connr->tcpstateflags & UIP_STOPPED))
            {
              uip_flags |= UIP_NEWDATA;
              uip_add_rcv_nxt(dev->d_len);
            }

          /* Check if the available buffer space advertised by the other end
             is smaller than the initial MSS for this connection. If so, we
             set the current MSS to the window size to ensure that the
             application does not send more data than the other end can
             handle.

             If the remote host advertises a zero window, we set the MSS to
             the initial MSS so that the application will send an entire MSS
             of data. This data will not be acknowledged by the receiver,
             and the application will retransmit it. This is called the
             "persistent timer" and uses the retransmission mechanim.
           */
          tmp16 = ((uint16)BUF->wnd[0] << 8) + (uint16)BUF->wnd[1];
          if (tmp16 > uip_connr->initialmss || tmp16 == 0)
            {
              tmp16 = uip_connr->initialmss;
            }
            uip_connr->mss = tmp16;

          /* If this packet constitutes an ACK for outstanding data (flagged
             by the UIP_ACKDATA flag, we should call the application since it
             might want to send more data. If the incoming packet had data
             from the peer (as flagged by the UIP_NEWDATA flag), the
             application must also be notified.

             When the application is called, the d_len field
             contains the length of the incoming data. The application can
             access the incoming data through the global pointer
             d_appdata, which usually points UIP_IPTCPH_LEN + UIP_LLH_LEN
             bytes into the d_buf array.

             If the application wishes to send any data, this data should be
             put into the d_appdata and the length of the data should be
             put into d_len. If the application don't have any data to
             send, d_len must be set to 0. */
          if (uip_flags & (UIP_NEWDATA | UIP_ACKDATA))
            {
              dev->d_sndlen = 0;
              uip_tcp_callback(dev);

 appsend:
              if (uip_flags & UIP_ABORT)
                {
                  dev->d_sndlen = 0;
                  uip_connr->tcpstateflags = UIP_CLOSED;
                  BUF->flags = TCP_RST | TCP_ACK;
                  goto tcp_send_nodata;
                }

              if (uip_flags & UIP_CLOSE)
                {
                  dev->d_sndlen = 0;
                  uip_connr->len = 1;
                  uip_connr->tcpstateflags = UIP_FIN_WAIT_1;
                  uip_connr->nrtx = 0;
                  BUF->flags = TCP_FIN | TCP_ACK;
                  goto tcp_send_nodata;
                }

              /* If d_sndlen > 0, the application has data to be sent. */
              if (dev->d_sndlen > 0)
                {
                  /* If the connection has acknowledged data, the contents of
                     the ->len variable should be discarded. */
                  if ((uip_flags & UIP_ACKDATA) != 0)
                    {
                      uip_connr->len = 0;
                    }

                  /* If the ->len variable is non-zero the connection has
                     already data in transit and cannot send anymore right
                     now. */
                  if (uip_connr->len == 0)
                    {
                      /* The application cannot send more than what is allowed by
                         the mss (the minumum of the MSS and the available
                         window). */
                      if (dev->d_sndlen > uip_connr->mss)
                        {
                          dev->d_sndlen = uip_connr->mss;
                        }

                      /* Remember how much data we send out now so that we know
                        when everything has been acknowledged. */
                      uip_connr->len = dev->d_sndlen;
                    }
                  else
                    {
                      /* If the application already had unacknowledged data, we
                         make sure that the application does not send (i.e.,
                         retransmit) out more than it previously sent out. */
                      dev->d_sndlen = uip_connr->len;
                    }
                }
              uip_connr->nrtx = 0;
 apprexmit:
              dev->d_appdata = dev->d_snddata;

              /* If the application has data to be sent, or if the incoming
                 packet had new data in it, we must send out a packet. */
              if (dev->d_sndlen > 0 && uip_connr->len > 0)
                {
                  /* Add the length of the IP and TCP headers. */
                  dev->d_len = uip_connr->len + UIP_TCPIP_HLEN;

                  /* We always set the ACK flag in response packets. */
                  BUF->flags = TCP_ACK | TCP_PSH;

                  /* Send the packet. */
                  goto tcp_send_noopts;
                }

              /* If there is no data to send, just send out a pure ACK if
                 there is newdata. */
              if (uip_flags & UIP_NEWDATA)
                {
                  dev->d_len = UIP_TCPIP_HLEN;
                  BUF->flags = TCP_ACK;
                  goto tcp_send_noopts;
                }
            }
          goto drop;

        case UIP_LAST_ACK:
          /* We can close this connection if the peer has acknowledged our
             FIN. This is indicated by the UIP_ACKDATA flag. */
          if (uip_flags & UIP_ACKDATA)
            {
              uip_connr->tcpstateflags = UIP_CLOSED;
              uip_flags = UIP_CLOSE;
              uip_tcp_callback(dev);
            }
          break;

        case UIP_FIN_WAIT_1:
          /* The application has closed the connection, but the remote host
             hasn't closed its end yet. Thus we do nothing but wait for a
             FIN from the other side. */
          if (dev->d_len > 0)
            {
              uip_add_rcv_nxt(dev->d_len);
            }
          if (BUF->flags & TCP_FIN)
            {
              if (uip_flags & UIP_ACKDATA)
                {
                  uip_connr->tcpstateflags = UIP_TIME_WAIT;
                  uip_connr->timer = 0;
                  uip_connr->len = 0;
                }
              else
                {
                  uip_connr->tcpstateflags = UIP_CLOSING;
                }

              uip_add_rcv_nxt(1);
              uip_flags = UIP_CLOSE;
              uip_tcp_callback(dev);
              goto tcp_send_ack;
            }
          else if (uip_flags & UIP_ACKDATA)
            {
              uip_connr->tcpstateflags = UIP_FIN_WAIT_2;
              uip_connr->len = 0;
              goto drop;
            }
          if (dev->d_len > 0)
            {
              goto tcp_send_ack;
            }
          goto drop;

        case UIP_FIN_WAIT_2:
          if (dev->d_len > 0)
            {
              uip_add_rcv_nxt(dev->d_len);
            }
          if (BUF->flags & TCP_FIN)
            {
              uip_connr->tcpstateflags = UIP_TIME_WAIT;
              uip_connr->timer = 0;
              uip_add_rcv_nxt(1);
              uip_flags = UIP_CLOSE;
              uip_tcp_callback(dev);
              goto tcp_send_ack;
            }
          if (dev->d_len > 0)
            {
              goto tcp_send_ack;
            }
          goto drop;

        case UIP_TIME_WAIT:
          goto tcp_send_ack;

        case UIP_CLOSING:
          if (uip_flags & UIP_ACKDATA)
            {
              uip_connr->tcpstateflags = UIP_TIME_WAIT;
              uip_connr->timer = 0;
            }
      }
    goto drop;

    /* We jump here when we are ready to send the packet, and just want
       to set the appropriate TCP sequence numbers in the TCP header. */
 tcp_send_ack:
    BUF->flags = TCP_ACK;
    tcp_send_nodata:
    dev->d_len = UIP_IPTCPH_LEN;
    tcp_send_noopts:
    BUF->tcpoffset = (UIP_TCPH_LEN / 4) << 4;
 tcp_send:
    /* We're done with the input processing. We are now ready to send a
       reply. Our job is to fill in all the fields of the TCP and IP
       headers before calculating the checksum and finally send the
       packet. */
    BUF->ackno[0] = uip_connr->rcv_nxt[0];
    BUF->ackno[1] = uip_connr->rcv_nxt[1];
    BUF->ackno[2] = uip_connr->rcv_nxt[2];
    BUF->ackno[3] = uip_connr->rcv_nxt[3];

    BUF->seqno[0] = uip_connr->snd_nxt[0];
    BUF->seqno[1] = uip_connr->snd_nxt[1];
    BUF->seqno[2] = uip_connr->snd_nxt[2];
    BUF->seqno[3] = uip_connr->snd_nxt[3];

    BUF->proto = UIP_PROTO_TCP;

    BUF->srcport  = uip_connr->lport;
    BUF->destport = uip_connr->rport;

    uip_ipaddr_copy(BUF->srcipaddr, uip_hostaddr);
    uip_ipaddr_copy(BUF->destipaddr, uip_connr->ripaddr);

    if (uip_connr->tcpstateflags & UIP_STOPPED)
      {
        /* If the connection has issued uip_stop(), we advertise a zero
           window so that the remote host will stop sending data. */
        BUF->wnd[0] = BUF->wnd[1] = 0;
      }
    else
      {
        BUF->wnd[0] = ((UIP_RECEIVE_WINDOW) >> 8);
        BUF->wnd[1] = ((UIP_RECEIVE_WINDOW) & 0xff);
      }

 tcp_send_noconn:
    BUF->ttl = UIP_TTL;
#ifdef CONFIG_NET_IPv6
    /* For IPv6, the IP length field does not include the IPv6 IP header
       length. */
    BUF->len[0] = ((dev->d_len - UIP_IPH_LEN) >> 8);
    BUF->len[1] = ((dev->d_len - UIP_IPH_LEN) & 0xff);
#else /* CONFIG_NET_IPv6 */
    BUF->len[0] = (dev->d_len >> 8);
    BUF->len[1] = (dev->d_len & 0xff);
#endif /* CONFIG_NET_IPv6 */

    BUF->urgp[0] = BUF->urgp[1] = 0;

    /* Calculate TCP checksum. */
    BUF->tcpchksum = 0;
    BUF->tcpchksum = ~(uip_tcpchksum(dev));

#ifdef CONFIG_NET_UDP
 ip_send_nolen:
#endif   /* CONFIG_NET_UDP */

#ifdef CONFIG_NET_IPv6
    BUF->vtc = 0x60;
    BUF->tcflow = 0x00;
    BUF->flow = 0x00;
#else /* CONFIG_NET_IPv6 */
    BUF->vhl = 0x45;
    BUF->tos = 0;
    BUF->ipoffset[0] = BUF->ipoffset[1] = 0;
    ++ipid;
    BUF->ipid[0] = ipid >> 8;
    BUF->ipid[1] = ipid & 0xff;

    /* Calculate IP checksum. */
    BUF->ipchksum = 0;
    BUF->ipchksum = ~(uip_ipchksum(dev));
    dbg("uip ip_send_nolen: chkecum 0x%04x\n", uip_ipchksum(dev));
#endif /* CONFIG_NET_IPv6 */

    UIP_STAT(++uip_stat.tcp.sent);
 send:
    dbg("Sending packet with length %d (%d)\n", dev->d_len,
                 (BUF->len[0] << 8) | BUF->len[1]);

    UIP_STAT(++uip_stat.ip.sent);

    /* Return and let the caller do the actual transmission. */
    uip_flags = 0;
    return;
 drop:
    dev->d_len = 0;
    uip_flags = 0;
    return;
}
