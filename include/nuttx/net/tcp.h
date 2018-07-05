/****************************************************************************
 * include/nuttx/net/tcp.h
 * Header file for the NuttX TCP/IP stack.
 *
 * This TCP/IP stack header file contains definitions for a number of C
 * macros that are used by internal network structures, TCP/IP header
 * structures and function declarations.
 *
 *   Copyright (C) 2007, 2009-2010, 2012-2014 Gregory Nutt. All rights
 *      reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was leveraged from uIP which also has a BSD-style license:
 *
 *   Author Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
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

#ifndef __INCLUDE_NUTTX_NET_TCP_H
#define __INCLUDE_NUTTX_NET_TCP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET_TCP

#include <stdint.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/arp.h>
#include <nuttx/net/ip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TCP definitions */

#define TCP_FIN           0x01
#define TCP_SYN           0x02
#define TCP_RST           0x04
#define TCP_PSH           0x08
#define TCP_ACK           0x10
#define TCP_URG           0x20
#define TCP_CTL           0x3f

#define TCP_OPT_END       0   /* End of TCP options list */
#define TCP_OPT_NOOP      1   /* "No-operation" TCP option */
#define TCP_OPT_MSS       2   /* Maximum segment size TCP option */

#define TCP_OPT_MSS_LEN   4   /* Length of TCP MSS option. */

/* The TCP states used in the struct tcp_conn_s tcpstateflags field */

#define TCP_STATE_MASK    0x0f /* Bits 0-3: TCP state */
#  define TCP_CLOSED      0x00 /* The connection is not in use and available */
#  define TCP_ALLOCATED   0x01 /* The connection is allocated, but not yet initialized */
#  define TCP_SYN_RCVD    0x02
#  define TCP_SYN_SENT    0x03
#  define TCP_ESTABLISHED 0x04
#  define TCP_FIN_WAIT_1  0x05
#  define TCP_FIN_WAIT_2  0x06
#  define TCP_CLOSING     0x07
#  define TCP_TIME_WAIT   0x08
#  define TCP_LAST_ACK    0x09
#  define TCP_STOPPED     0x10 /* Bit 4: stopped */
                               /* Bit 5-7: Unused, but not available */

/* TCP header sizes
 *
 * The minimum size header is 5 words and the maximum is 15 words thus
 * giving the minimum size of 20 bytes and maximum of 60 bytes, allowing for
 * up to 40 bytes of options in the header.
 */

#define TCP_HDRLEN        20                         /* Size of TCP header (minimum) */
#define TCP_OPT_HDRLEN(n) (20 + ((n) << 2))          /* Size of TCP header w/options */
#define TCP_MAX_HDRLEN    60                         /* Maximum size of TCP header */

#ifdef CONFIG_NET_IPv4
#  define IPv4TCP_HDRLEN  (TCP_HDRLEN + IPv4_HDRLEN) /* Size of IPv4 + TCP header */
#endif

#ifdef CONFIG_NET_IPv6
#  define IPv6TCP_HDRLEN  (TCP_HDRLEN + IPv6_HDRLEN) /* Size of IPv4 + TCP header */
#endif

/* Initial minimum MSS according to RFC 879
 *
 * The default TCP Maximum Segment Sizes (MSS) are defined below.is 536.  If
 * a host wishes to set the maximum segment size to a value other than the
 * default, the maximum segment size is specified as a TCP option, initially
 * in the TCP SYN packet during the TCP handshake.  The value cannot be
 * changed after the connection is established.
 *
 * These defaults correspond to the minimum MTU values:
 *
 *   IPv4:  MTU=576;  MSS=536  (MTU - IPv4_HDRLEN - TCP_HDRLEN)
 *   IPv6:  MTU=1280; MSS=1200 (MTU - IPv5_HDRLEN - TCP_HDRLEN)
 */

#define TCP_DEFAULT_IPv4_MSS  536
#define TCP_DEFAULT_IPv6_MSS  1200

/* However, we do need to make allowance for certain links such as SLIP that
 * have unusually small MTUs.
 */

#ifdef CONFIG_NET_IPv4
#  define TCP_IPv4_INITIAL_MSS(d) \
     (TCP_MSS(d,IPv4_HDRLEN) > 536 ? 536 : TCP_MSS(d,IPv4_HDRLEN))

#  define MIN_IPv4_TCP_INITIAL_MSS \
     (__MIN_TCP_MSS(IPv4_HDRLEN) > 536 ? 536 : __MIN_TCP_MSS(IPv4_HDRLEN))
#  define MAX_IPv4_TCP_INITIAL_MSS  \
     (__MAX_TCP_MSS(IPv4_HDRLEN) > 536 ? 536 : __MAX_TCP_MSS(h))
#endif

#ifdef CONFIG_NET_IPv6
#  define TCP_IPv6_INITIAL_MSS(d) \
     (TCP_MSS(d,IPv6_HDRLEN) > 1200 ? 1200 : TCP_MSS(d,IPv6_HDRLEN))
#  define MIN_IPv6_TCP_INITIAL_MSS \
     (__MIN_TCP_MSS(IPv6_HDRLEN) > 1200 ? 1200 : __MIN_TCP_MSS(IPv6_HDRLEN))
#  define MAX_IPv6_TCP_INITIAL_MSS  \
     (__MAX_TCP_MSS(IPv6_HDRLEN) > 1200 ? 1200 : __MAX_TCP_MSS(IPv6_HDRLEN))
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* TCP header */

struct tcp_hdr_s
{
  uint16_t srcport;
  uint16_t destport;
  uint8_t  seqno[4];
  uint8_t  ackno[4];
  uint8_t  tcpoffset;
  uint8_t  flags;
  uint8_t  wnd[2];
  uint16_t tcpchksum;
  uint8_t  urgp[2];
  uint8_t  optdata[4];
};

/* The structure holding the TCP/IP statistics that are gathered if
 * CONFIG_NET_STATISTICS is defined.
 */

#ifdef CONFIG_NET_STATISTICS
struct tcp_stats_s
{
  net_stats_t drop;       /* Number of dropped TCP segments */
  net_stats_t recv;       /* Number of received TCP segments */
  net_stats_t sent;       /* Number of sent TCP segments */
  net_stats_t chkerr;     /* Number of TCP segments with a bad checksum */
  net_stats_t ackerr;     /* Number of TCP segments with a bad ACK number */
  net_stats_t rst;        /* Number of received TCP RST (reset) segments */
  net_stats_t rexmit;     /* Number of retransmitted TCP segments */
  net_stats_t syndrop;    /* Number of dropped SYNs due to too few
                             available connections */
  net_stats_t synrst;     /* Number of SYNs for closed ports triggering a RST */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* CONFIG_NET_TCP */
#endif /* __INCLUDE_NUTTX_NET_TCP_H */
