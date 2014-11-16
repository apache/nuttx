/****************************************************************************
 * include/nuttx/net/netconfig.h
 * Configuration options for NuttX uIP-based networking.
 *
 * This file is used for tweaking various configuration options for
 * uIP. This is most assuring the correct default values are provided and
 * that configured options are valid.
 *
 * Note: Network configuration options the netconfig.h should not be changed,
 * but rather the per-project defconfig file.
 *
 *   Copyright (C) 2007, 2011, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This logic was leveraged from uIP which also has a BSD-style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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

#ifndef __INCLUDE_NUTTX_NET_NETCONFG_H
#define __INCLUDE_NUTTX_NET_NETCONFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <nuttx/config.h>

/****************************************************************************
 * Public Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Layer 2 Configuration Options ********************************************/

/* The default data link layer for uIP is Ethernet.  If CONFIG_NET_SLIP is
 * defined in the NuttX header file, then SLIP will be supported.  The basic
 * differences between the SLIP and Ethernet configurations is that when SLIP
 * is selected:
 *
 * - The link level header (that comes before the IP header) is omitted.
 * - All MAC address processing is suppressed.
 * - ARP is disabled.
 *
 * If CONFIG_NET_SLIP is not supported, then Ethernet will be used (there is
 * no need to define anything special in the configuration file to use
 * Ethernet -- it is the default).
 *
 * The "link level header" is the offset into the d_buf where the IP header
 * can be found. For Ethernet, this should be set to 14. For SLIP, this
 * should be set to 0.
 *
 * If CONFIG_NET_MULTILINK is defined, then multiple link protocols are
 * supported concurrently.  In this case, the size of link layer header
 * varies and is obtained from the network device structure.
 *
 * Support is also provided to select different MTU sizes for each different
 * link layer protocol.  A better solution would be to support device-by-
 * device MTU sizes.  This minimum support is require to support the
 * optimal SLIP MTU of 296 bytes and the standard Ethernet MTU of 1500
 * bytes.
 */

#ifdef CONFIG_NET_SLIP
#  ifndef CONFIG_NET_SLIP_MTU
#    define CONFIG_NET_SLIP_MTU 590
#  endif
#endif

#ifdef CONFIG_NET_ETHERNET
#  ifndef CONFIG_NET_ETH_MTU
#    define CONFIG_NET_ETH_MTU 590
#  endif
#endif

#if defined(CONFIG_NET_MULTILINK)
   /* We are supporting multiple network devices using different link layer
    * protocols.  Get the size of the link layer header from the device
    * structure.
    */

#  define NET_LL_HDRLEN(d) ((d)->d_llhdrlen)
#  define NET_LL_MTU(d)    ((d)->d_llmtu)

#  ifdef CONFIG_NET_ETHERNET
#    define _MIN_ETH_MTU  CONFIG_NET_ETH_MTU
#    define _MAX_ETH_MTU  CONFIG_NET_ETH_MTU
#  else
#    define _MIN_ETH_MTU  UINT16_MAX
#    define _MAX_ETH_MTU  0
#  endif

#  ifdef CONFIG_NET_SLIP
#    define _MIN_SLIP_MTU  MIN(_MIN_ETH_MTU,CONFIG_NET_SLIP_MTU)
#    define _MAX_SLIP_MTU  MAX(_MAX_ETH_MTU,CONFIG_NET_SLIP_MTU)
#  else
#    define _MIN_SLIP_MTU  _MIN_ETH_MTU
#    define _MAX_SLIP_MTU  _MAX_ETH_MTU
#  endif

#  define MIN_NET_LL_MTU   _MIN_SLIP_MTU
#  define MAX_NET_LL_MTU   _MAX_SLIP_MTU

#elif defined(CONFIG_NET_SLIP)
   /* There is no link layer header with SLIP */

#  ifdef CONFIG_NET_IPv6
#    error SLIP is not available for IPv6
#  endif
#  define NET_LL_HDRLEN(d) 0
#  define NET_LL_MTU(d)    CONFIG_NET_SLIP_MTU
#  define MIN_NET_LL_MTU   CONFIG_NET_SLIP_MTU
#  define MAX_NET_LL_MTU   CONFIG_NET_SLIP_MTU

#else /* if defined(CONFIG_NET_ETHERNET) */
   /* Assume standard Ethernet link layer header */

#  define NET_LL_HDRLEN(d) 14
#  define NET_LL_MTU(d)    CONFIG_NET_ETH_MTU
#  define MIN_NET_LL_MTU   CONFIG_NET_ETH_MTU
#  define MAX_NET_LL_MTU   CONFIG_NET_ETH_MTU

#endif /* MULTILINK or SLIP or ETHERNET */

/* Layer 3/4 Configuration Options ******************************************/

/* IP configuration options */

/* The IP TTL (time to live) of IP packets sent by uIP.
 *
 * This should normally not be changed.
 */

#define IP_TTL 64

#ifdef CONFIG_NET_TCP_REASSEMBLY
#  ifndef CONFIG_NET_TCP_REASS_MAXAGE
  /* The maximum time an IP fragment should wait in the reassembly
   * buffer before it is dropped.  Units are deci-seconds, the range
   * of the timer is 8-bits.
   */

#    define CONFIG_NET_TCP_REASS_MAXAGE (20*10) /* 20 seconds */
#  endif
#endif

/* Network drivers often receive packets with garbage at the end
 * and are longer than the size of packet in the TCP header.  The
 * following "fudge" factor increases the size of the I/O buffering
 * by a small amount to allocate slightly oversize packets.  After
 * receipt, the packet size will be chopped down to the size indicated
 * in the TCP header.
 */

#ifndef CONFIG_NET_GUARDSIZE
#  define CONFIG_NET_GUARDSIZE 2
#endif

/* ICMP configuration options */

#ifndef CONFIG_NET_ICMP
#  undef CONFIG_NET_ICMP_PING
#endif

/* UDP configuration options */

/* The maximum amount of concurrent UDP connection, Default: 10 */

#ifndef CONFIG_NET_UDP_CONNS
#  ifdef CONFIG_NET_UDP
#    define CONFIG_NET_UDP_CONNS 10
#  else
#    define CONFIG_NET_UDP_CONNS  0
#  endif
#endif

/* The UDP maximum packet size. This is should not be to set to more
 * than NET_LL_MTU(d) - NET_LL_HDRLEN(dev) - IPUDP_HDRLEN.
 */

#define UDP_MSS(d)    (CONFIG_NET_ETH_MTU - NET_LL_HDRLEN(d) - IPUDP_HDRLEN)

#ifdef CONFIG_NET_ETHERNET
#  define MIN_UDP_MSS (CONFIG_NET_ETH_MTU - ETH_HDRLEN - IPUDP_HDRLEN)
#else /* if defined(CONFIG_NET_SLIP) */
#  define MIN_UDP_MSS (CONFIG_NET_ETH_MTU - IPUDP_HDRLEN)
#endif

#ifdef CONFIG_NET_SLIP
#  define MAX_UDP_MSS (CONFIG_NET_ETH_MTU - IPUDP_HDRLEN)
#else /* if defined(CONFIG_NET_ETHERNET) */
#  define MAX_UDP_MSS (CONFIG_NET_ETH_MTU - ETH_HDRLEN - IPUDP_HDRLEN)
#endif

/* TCP configuration options */

/* The maximum number of simultaneously open TCP connections.
 *
 * Since the TCP connections are statically allocated, turning this
 * configuration knob down results in less RAM used. Each TCP
 * connection requires approximately 30 bytes of memory.
 */

#ifndef CONFIG_NET_TCP_CONNS
#  ifdef CONFIG_NET_TCP
#   define CONFIG_NET_TCP_CONNS 10
#  else
#   define CONFIG_NET_TCP_CONNS  0
#  endif
#endif

/* The maximum number of simultaneously listening TCP ports.
 *
 * Each listening TCP port requires 2 bytes of memory.
 */

#ifndef CONFIG_NET_MAX_LISTENPORTS
#  define CONFIG_NET_MAX_LISTENPORTS 20
#endif

/* Define the maximum number of concurrently active UDP and TCP
 * ports.  This number must be greater than the number of open
 * sockets in order to support multi-threaded read/write operations.
 */

#ifndef CONFIG_NET_NACTIVESOCKETS
#  define CONFIG_NET_NACTIVESOCKETS (CONFIG_NET_TCP_CONNS + CONFIG_NET_UDP_CONNS)
#endif

/* The initial retransmission timeout counted in timer pulses.
 *
 * This should not be changed.
 */

#define TCP_RTO 3

/* The maximum number of times a segment should be retransmitted
 * before the connection should be aborted.
 *
 * This should not be changed.
 */

#define TCP_MAXRTX  8

/* The maximum number of times a SYN segment should be retransmitted
 * before a connection request should be deemed to have been
 * unsuccessful.
 *
 * This should not need to be changed.
 */

#define TCP_MAXSYNRTX 5

/* The TCP maximum segment size. This is should not be set to more
 * than CONFIG_NET_ETH_MTU - NET_LL_HDRLEN(dev) - IPTCP_HDRLEN.
 *
 * In the case where there are multiple network devices with different
 * link layer protocols (CONFIG_NET_MULTILINK), each network device
 * may support a different UDP MSS value.  Here we arbitrarily select
 * the minimum MSS for that case.
 */

#define TCP_MSS(d)    (CONFIG_NET_ETH_MTU - NET_LL_HDRLEN(d) - IPTCP_HDRLEN)

#ifdef CONFIG_NET_ETHERNET
#  define MIN_TCP_MSS (CONFIG_NET_ETH_MTU - ETH_HDRLEN - IPTCP_HDRLEN)
#else /* if defined(CONFIG_NET_SLIP) */
#  define MIN_TCP_MSS (CONFIG_NET_ETH_MTU - IPTCP_HDRLEN)
#endif

#ifdef CONFIG_NET_SLIP
#  define MAX_TCP_MSS (CONFIG_NET_ETH_MTU - IPTCP_HDRLEN)
#else /* if defined(CONFIG_NET_ETHERNET) */
#  define MAX_TCP_MSS (CONFIG_NET_ETH_MTU - ETH_HDRLEN - IPTCP_HDRLEN)
#endif

/* The size of the advertised receiver's window.
 *
 * Should be set low (i.e., to the size of the d_buf buffer) is the
 * application is slow to process incoming data, or high (32768 bytes)
 * if the application processes data quickly.
 *
 * See the note above regarding the TCP MSS and CONFIG_NET_MULTILINK.
 */

#ifndef CONFIG_NET_RECEIVE_WINDOW
#  define CONFIG_NET_RECEIVE_WINDOW MIN_TCP_MSS
#endif

/* How long a connection should stay in the TIME_WAIT state.
 *
 * This configuration option has no real implication, and it should be
 * left untouched. Units: half second.
 */

#define TCP_TIME_WAIT_TIMEOUT (60*2)

/* ARP configuration options */

#ifndef CONFIG_NET_ARPTAB_SIZE
/* The size of the ARP table.
 *
 * This option should be set to a larger value if this uIP node will
 * have many connections from the local network.
 */

#  define CONFIG_NET_ARPTAB_SIZE 8
#endif

#ifndef CONFIG_NET_ARP_MAXAGE
/* The maximum age of ARP table entries measured in 10ths of seconds.
 *
 * An CONFIG_NET_ARP_MAXAGE of 120 corresponds to 20 minutes (BSD
 * default).
 */

#  define CONFIG_NET_ARP_MAXAGE 120
#endif

/* General configuration options */

/* Delay after receive to catch a following packet.  No delay should be
 * required if TCP/IP read-ahead buffering is enabled.
 */

#ifndef CONFIG_NET_TCP_RECVDELAY
#  ifdef CONFIG_NET_TCP_READAHEAD
#    define CONFIG_NET_TCP_RECVDELAY 0
#  else
#    define CONFIG_NET_TCP_RECVDELAY 5
#  endif
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Statistics datatype
 *
 * This typedef defines the dataype used for keeping statistics in
 * uIP.
 */

typedef uint16_t net_stats_t;

#endif /* __INCLUDE_NUTTX_NET_NETCONFG_H */

