/****************************************************************************
 * include/nuttx/net/netconfig.h
 * Configuration options for NuttX networking.
 *
 * This file is used for tweaking various configuration options for the
 * network. This is most assuring the correct default values are provided
 * and that configured options are valid.
 *
 * Note: Network configuration options the netconfig.h should not be changed,
 * but rather the per-project defconfig file.
 *
 *   Copyright (C) 2007, 2011, 2014-2015, 2017-2019 Gregory Nutt. All rights
 *     reserved.
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

#include <nuttx/config.h>

#include <stdint.h>
#include <sys/param.h>

#include <nuttx/net/ethernet.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Using the following definitions, the following socket() arguments should
 * provide a valid socket in all configurations:
 *
 *   ret = socket(NET_SOCK_FAMILY, NET_SOCK_TYPE,
 *                NET_SOCK_PROTOCOL);
 */

/* The address family that we used to create the socket really does not
 * matter.  It should, however, be valid in the current configuration.
 */

#if defined(CONFIG_NET_IPv4)
#  define NET_SOCK_FAMILY  AF_INET
#elif defined(CONFIG_NET_IPv6)
#  define NET_SOCK_FAMILY  AF_INET6
#elif defined(CONFIG_NET_LOCAL)
#  define NET_SOCK_FAMILY  AF_LOCAL
#elif defined(CONFIG_NET_PKT)
#  define NET_SOCK_FAMILY  AF_PACKET
#elif defined(CONFIG_NET_CAN)
#  define NET_SOCK_FAMILY  AF_CAN
#elif defined(CONFIG_NET_IEEE802154)
#  define NET_SOCK_FAMILY  AF_IEEE802154
#elif defined(CONFIG_WIRELESS_PKTRADIO)
#  define NET_SOCK_FAMILY  AF_PKTRADIO
#elif defined(CONFIG_NET_BLUETOOTH)
#  define NET_SOCK_FAMILY  AF_BLUETOOTH
#elif defined(CONFIG_NET_USRSOCK)
#  define NET_SOCK_FAMILY  AF_INET
#elif defined(CONFIG_NET_NETLINK)
#  define NET_SOCK_FAMILY  AF_NETLINK
#elif defined(CONFIG_NET_RPMSG)
#  define NET_SOCK_FAMILY  AF_RPMSG
#else
#  define NET_SOCK_FAMILY  AF_UNSPEC
#endif

/* Socket protocol of zero normally works */

#define NET_SOCK_PROTOCOL  0

/* SOCK_CTRL is the preferred socket type to use when we just want a
 * socket for performing driver ioctls.
 */

#define NET_SOCK_TYPE SOCK_CTRL

#if NET_SOCK_FAMILY == AF_INET
#  if !defined(CONFIG_NET_UDP) && !defined(CONFIG_NET_TCP) && \
      defined(CONFIG_NET_ICMP_SOCKET)
#   undef NET_SOCK_PROTOCOL
#   define NET_SOCK_PROTOCOL IPPROTO_ICMP
#  endif
#elif NET_SOCK_FAMILY == AF_INET6
#  if !defined(CONFIG_NET_UDP) && !defined(CONFIG_NET_TCP) && \
      defined(CONFIG_NET_ICMPv6_SOCKET)
#   undef NET_SOCK_PROTOCOL
#   define NET_SOCK_PROTOCOL IPPROTO_ICMP6
#  endif
#endif

/* Eliminate dependencies on other header files.  This should not harm
 * portability because these are well-known constants.
 */

#define __IPv4_HDRLEN 20  /* Must match IPv4_HDRLEN in include/nuttx/net/ip.h */
#define __IPv6_HDRLEN 40  /* Must match IPv6_HDRLEN in include/nuttx/net/ip.h */
#define __UDP_HDRLEN  8   /* Must match UDP_HDRLEN in include/nuttx/net/udp.h */
#define __TCP_HDRLEN  20  /* Must match TCP_HDRLEN in include/nuttx/net/tcp.h */
                          /* REVISIT: Not really a constant */

/* Layer 2 Configuration Options ********************************************/

/* The default link layer is Ethernet.  If CONFIG_NET_SLIP is defined in
 * the NuttX header file, then SLIP will be supported.  The basic
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
 * There are other device-specific features that at tied to the link layer:
 *
 *   - Maximum Transfer Unit (MTU)
 *   - TCP Receive Window size (See TCP configuration options below)
 *
 * A better solution would be to support device-by-device MTU and receive
 * window sizes.  This minimum support is require to support the optimal
 * SLIP MTU of 296 bytes and the standard Ethernet MTU of 1500
 * bytes.
 */

#ifdef CONFIG_NET_SLIP
#  ifndef CONFIG_NET_SLIP_PKTSIZE
#    define CONFIG_NET_SLIP_PKTSIZE 296
#  endif
#endif

#ifdef CONFIG_NET_TUN
#  ifndef CONFIG_NET_TUN_PKTSIZE
#    define CONFIG_NET_TUN_PKTSIZE 296
#  endif
#endif

#ifdef CONFIG_NET_ETHERNET
#  ifndef CONFIG_NET_ETH_PKTSIZE
#    define CONFIG_NET_ETH_PKTSIZE 590
#  endif
#endif

#ifndef CONFIG_NET_6LOWPAN_PKTSIZE
#  define CONFIG_NET_6LOWPAN_PKTSIZE  1294
#endif

/* We are supporting multiple network devices using different link layer
 * protocols.  Get the size of the link layer header from the device
 * structure.
 */

#define NET_LL_HDRLEN(d)       ((d)->d_llhdrlen)
#define NETDEV_PKTSIZE(d)      ((d)->d_pktsize)

#ifdef CONFIG_NET_ETHERNET
#  define _MIN_ETH_PKTSIZE     CONFIG_NET_ETH_PKTSIZE
#  define _MAX_ETH_PKTSIZE     CONFIG_NET_ETH_PKTSIZE
#else
#  define _MIN_ETH_PKTSIZE     UINT16_MAX
#  define _MAX_ETH_PKTSIZE     0
#endif

#ifdef CONFIG_NET_LOOPBACK
#  define _MIN_LO_PKTSIZE      MIN(_MIN_ETH_PKTSIZE, 574)
#  define _MAX_LO_PKTSIZE      MAX(_MAX_ETH_PKTSIZE, 1518)
#else
#  define _MIN_LO_PKTSIZE      _MIN_ETH_PKTSIZE
#  define _MAX_LO_PKTSIZE      _MAX_ETH_PKTSIZE
#endif

#ifdef CONFIG_NET_SLIP
#  define _MIN_SLIP_PKTSIZE    MIN(_MIN_LO_PKTSIZE, CONFIG_NET_SLIP_PKTSIZE)
#  define _MAX_SLIP_PKTSIZE    MAX(_MAX_LO_PKTSIZE, CONFIG_NET_SLIP_PKTSIZE)
#else
#  define _MIN_SLIP_PKTSIZE    _MIN_LO_PKTSIZE
#  define _MAX_SLIP_PKTSIZE    _MAX_LO_PKTSIZE
#endif

#ifdef CONFIG_NET_TUN
#  define _MIN_TUN_PKTSIZE      MIN(_MIN_SLIP_PKTSIZE, CONFIG_NET_TUN_PKTSIZE)
#  define _MAX_TUN_PKTSIZE      MAX(_MAX_SLIP_PKTSIZE, CONFIG_NET_TUN_PKTSIZE)
#else
#  define _MIN_TUN_PKTSIZE      _MIN_SLIP_PKTSIZE
#  define _MAX_TUN_PKTSIZE      _MAX_SLIP_PKTSIZE
#endif

#ifdef CONFIG_NET_6LOWPAN
#  define _MIN_6LOWPAN_PKTSIZE  MIN(_MIN_TUN_PKTSIZE, CONFIG_NET_6LOWPAN_PKTSIZE)
#  define _MAX_6LOWPAN_PKTSIZE  MAX(_MAX_TUN_PKTSIZE, CONFIG_NET_6LOWPAN_PKTSIZE)
#else
#  define _MIN_6LOWPAN_PKTSIZE  _MIN_TUN_PKTSIZE
#  define _MAX_6LOWPAN_PKTSIZE  _MAX_TUN_PKTSIZE
#endif

#define MIN_NETDEV_PKTSIZE      _MIN_6LOWPAN_PKTSIZE
#define MAX_NETDEV_PKTSIZE      _MAX_6LOWPAN_PKTSIZE

/* The loopback driver packet buffer should be quite large.  The larger the
 * loopback packet buffer, the better will be TCP performance of the loopback
 * transfers.  The Linux loopback device historically used packet buffers of
 * size 16Kb, but that was increased in recent Linux versions to 64Kb.  Those
 * sizes may be excessive for resource constrained MCUs, however.
 *
 * For the loopback driver, we enforce a lower limit that is the maximum
 * packet size of all enabled link layer protocols.
 */

#if CONFIG_NET_LOOPBACK_PKTSIZE < MAX_NETDEV_PKTSIZE
#  define NET_LO_PKTSIZE        MAX_NETDEV_PKTSIZE
#else
#  define NET_LO_PKTSIZE        CONFIG_NET_LOOPBACK_PKTSIZE
#endif

#ifndef CONFIG_NET_SEND_BUFSIZE
#define CONFIG_NET_SEND_BUFSIZE 0
#endif

/* Layer 3/4 Configuration Options ******************************************/

/* IP configuration options */

/* The IP TTL (time to live) of IP packets sent by the network stack.
 *
 * This should normally not be changed.
 */

#define IP_TTL_DEFAULT 64

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
#  undef CONFIG_NET_ICMP_SOCKET
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

/* The UDP maximum packet size. This should not be set to more than
 * NETDEV_PKTSIZE(d) - NET_LL_HDRLEN(dev) - __UDP_HDRLEN - IPv*_HDRLEN.
 */

#define UDP_MSS(d,h)               (NETDEV_PKTSIZE(d) - NET_LL_HDRLEN(d) - __UDP_HDRLEN - (h))

#ifdef CONFIG_NET_ETHERNET
#  define ETH_UDP_MSS(h)           (CONFIG_NET_ETH_PKTSIZE - ETH_HDRLEN - __UDP_HDRLEN - (h))
#endif

#ifdef CONFIG_NET_6LOWPAN
#  define IEEE802154_UDP_MSS(h)    (CONFIG_NET_6LOWPAN_PKTSIZE - __UDP_HDRLEN - (h))
#endif

#ifdef CONFIG_NET_LOOPBACK
#  define LO_UDP_MSS(h)            (NET_LO_PKTSIZE - __UDP_HDRLEN - (h))
#endif

#ifdef CONFIG_NET_SLIP
#  define SLIP_UDP_MSS(h)          (CONFIG_NET_SLIP_PKTSIZE - __UDP_HDRLEN - (h))
#endif

#ifdef CONFIG_NET_TUN
#  define TUN_UDP_MSS(h)           (CONFIG_NET_TUN_PKTSIZE - __UDP_HDRLEN - (h))
#endif

#ifdef CONFIG_NET_USRSOCK
#  define __MIN_UDP_MSS(h)         INT_MAX
#  define __MAX_UDP_MSS(h)         0
#endif

#ifdef CONFIG_NET_ETHERNET
#  undef  __MIN_UDP_MSS
#  undef  __MAX_UDP_MSS
#  define __MIN_UDP_MSS(h)         ETH_UDP_MSS(h)
#  define __MAX_UDP_MSS(h)         ETH_UDP_MSS(h)
#  define __ETH_MIN_UDP_MSS(h)     ETH_UDP_MSS(h)
#  define __ETH_MAX_UDP_MSS(h)     ETH_UDP_MSS(h)
#else
#  define __ETH_MIN_UDP_MSS(h)     INT_MAX
#  define __ETH_MAX_UDP_MSS(h)     0
#endif

#ifdef CONFIG_NET_6LOWPAN
#  undef  __MIN_UDP_MSS
#  undef  __MAX_UDP_MSS
#  define __MIN_UDP_MSS(h)         MIN(IEEE802154_UDP_MSS(h),__ETH_MIN_UDP_MSS(h))
#  define __MAX_UDP_MSS(h)         MAX(IEEE802154_UDP_MSS(h),__ETH_MAX_UDP_MSS(h))
#  define __6LOWPAN_MIN_UDP_MSS(h) MIN(IEEE802154_UDP_MSS(h),__ETH_MIN_UDP_MSS(h))
#  define __6LOWPAN_MAX_UDP_MSS(h) MAX(IEEE802154_UDP_MSS(h),__ETH_MAX_UDP_MSS(h))
#else
#  define __6LOWPAN_MIN_UDP_MSS(h) __ETH_MIN_UDP_MSS(h)
#  define __6LOWPAN_MAX_UDP_MSS(h) __ETH_MAX_UDP_MSS(h)
#endif

#ifdef CONFIG_NET_LOOPBACK
#  undef  __MIN_UDP_MSS
#  undef  __MAX_UDP_MSS
#  define __MIN_UDP_MSS(h)        MIN(LO_UDP_MSS(h),__6LOWPAN_MIN_UDP_MSS(h))
#  define __MAX_UDP_MSS(h)        MAX(LO_UDP_MSS(h),__6LOWPAN_MAX_UDP_MSS(h))
#  define __LOOP_MIN_UDP_MSS(h)   MIN(LO_UDP_MSS(h),__6LOWPAN_MIN_UDP_MSS(h))
#  define __LOOP_MAX_UDP_MSS(h)   MAX(LO_UDP_MSS(h),__6LOWPAN_MAX_UDP_MSS(h))
#else
#  define __LOOP_MIN_UDP_MSS(h) __6LOWPAN_MIN_UDP_MSS(h)
#  define __LOOP_MAX_UDP_MSS(h) __6LOWPAN_MAX_UDP_MSS(h)
#endif

#ifdef CONFIG_NET_SLIP
#  undef  __MIN_UDP_MSS
#  undef  __MAX_UDP_MSS
#  define __MIN_UDP_MSS(h)      MIN(SLIP_UDP_MSS(h),__LOOP_MIN_UDP_MSS(h))
#  define __MAX_UDP_MSS(h)      MAX(SLIP_UDP_MSS(h),__LOOP_MAX_UDP_MSS(h))
#  define __SLIP_MIN_UDP_MSS(h) MIN(SLIP_UDP_MSS(h),__LOOP_MIN_UDP_MSS(h))
#  define __SLIP_MAX_UDP_MSS(h) MAX(SLIP_UDP_MSS(h),__LOOP_MAX_UDP_MSS(h))
#else
#  define __SLIP_MIN_UDP_MSS(h) __LOOP_MIN_UDP_MSS(h)
#  define __SLIP_MAX_UDP_MSS(h) __LOOP_MAX_UDP_MSS(h)
#endif

#ifdef CONFIG_NET_TUN
#  undef  __MIN_UDP_MSS
#  undef  __MAX_UDP_MSS
#  define __MIN_UDP_MSS(h)      MIN(TUN_UDP_MSS(h),__SLIP_MIN_UDP_MSS(h))
#  define __MAX_UDP_MSS(h)      MAX(TUN_UDP_MSS(h),__SLIP_MAX_UDP_MSS(h))
#  define __TUN_MIN_UDP_MSS(h)  MIN(TUN_UDP_MSS(h),__SLIP_MIN_UDP_MSS(h))
#  define __TUN_MAX_UDP_MSS(h)  MAX(TUN_UDP_MSS(h),__SLIP_MAX_UDP_MSS(h))
#else
#  define __TUN_MIN_UDP_MSS(h)  __SLIP_MIN_UDP_MSS(h)
#  define __TUN_MAX_UDP_MSS(h)  __SLIP_MAX_UDP_MSS(h)
#endif

 #ifdef CONFIG_NET_IPv4
#  define UDP_IPv4_MSS(d)       UDP_MSS(d,__IPv4_HDRLEN)
#  define ETH_IPv4_UDP_MSS      ETH_UDP_MSS(__IPv4_HDRLEN)
#  define SLIP_IPv4_UDP_MSS     SLIP_UDP_MSS(__IPv4_HDRLEN)
#  define TUN_IPv4_UDP_MSS      TUN_UDP_MSS(__IPv4_HDRLEN)

#  define MIN_IPv4_UDP_MSS      __MIN_UDP_MSS(__IPv4_HDRLEN)
#  define MIN_UDP_MSS           __MIN_UDP_MSS(__IPv4_HDRLEN)

#  undef  MAX_UDP_MSS
#  define MAX_IPv4_UDP_MSS      __MAX_UDP_MSS(__IPv4_HDRLEN)
#  define MAX_UDP_MSS           __MAX_UDP_MSS(__IPv4_HDRLEN)
#endif

/* If IPv6 is supported, it will have the smaller MSS. */

#ifdef CONFIG_NET_IPv6
#  undef  MIN_UDP_MSS
#  define MIN_IPv6_UDP_MSS      __MIN_UDP_MSS(__IPv6_HDRLEN)
#  define MIN_UDP_MSS           __MIN_UDP_MSS(__IPv6_HDRLEN)
#  ifndef MAX_UDP_MSS
#    define MAX_UDP_MSS         __MAX_UDP_MSS(__IPv6_HDRLEN)
#  endif
#endif

/* TCP configuration options */

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
#  define CONFIG_NET_NACTIVESOCKETS (CONFIG_NET_TCP_PREALLOC_CONNS + \
                                     CONFIG_NET_UDP_CONNS)
#endif

/* The initial retransmission timeout counted in timer pulses.
 * REVISIT:  TCP RTO really should be calculated dynamically for each TCP
 * connection.
 */

#ifdef CONFIG_NET_TCP_RTO
#  define TCP_RTO CONFIG_NET_TCP_RTO
#else
#  define TCP_RTO 3
#endif

/* The maximum number of times a segment should be retransmitted
 * before the connection should be aborted.
 *
 * This should not be changed.
 */

#ifdef CONFIG_NET_TCP_MAXRTX
#  define TCP_MAXRTX CONFIG_NET_TCP_MAXRTX
#else
#  define TCP_MAXRTX 8
#endif

/* The maximum number of times a SYN segment should be retransmitted
 * before a connection request should be deemed to have been
 * unsuccessful.
 *
 * This should not need to be changed.
 */

#ifdef CONFIG_NET_TCP_MAXSYNRTX
#  define TCP_MAXSYNRTX CONFIG_NET_TCP_MAXSYNRTX
#else
#  define TCP_MAXSYNRTX 5
#endif

/* The TCP maximum segment size. This should not be set to more than
 * NETDEV_PKTSIZE(dev) - NET_LL_HDRLEN(dev) - IPvN_HDRLEN - __TCP_HDRLEN.
 *
 * In the case where there are multiple network devices with different
 * link layer protocols, each network device may support a different MSS
 * value.  Here we arbitrarily select the minimum MSS for that case.
 *
 * REVISIT: __TCP_HDRLEN is not really a constant!
 */

#define TCP_MSS(d,h)            (NETDEV_PKTSIZE(d) - NET_LL_HDRLEN(d) - __TCP_HDRLEN - (h))

/* Get the smallest and largest MSS */

#ifdef CONFIG_NET_ETHERNET
#  define ETH_TCP_MSS(h)        (CONFIG_NET_ETH_PKTSIZE - ETH_HDRLEN - __TCP_HDRLEN - (h))
#endif

#ifdef CONFIG_NET_6LOWPAN
#  define IEEE802154_TCP_MSS(h) (CONFIG_NET_6LOWPAN_PKTSIZE - __TCP_HDRLEN - (h))
#endif

#ifdef CONFIG_NET_LOOPBACK
#  define LO_TCP_MSS(h)         (NET_LO_PKTSIZE - __TCP_HDRLEN - (h))
#endif

#ifdef CONFIG_NET_SLIP
#  define SLIP_TCP_MSS(h)       (CONFIG_NET_SLIP_PKTSIZE - __TCP_HDRLEN - (h))
#endif

#ifdef CONFIG_NET_TUN
#  define TUN_TCP_MSS(h)        (CONFIG_NET_TUN_PKTSIZE - __TCP_HDRLEN - (h))
#endif

#ifdef CONFIG_NET_USRSOCK
#  define __MIN_TCP_MSS(h)         INT_MAX
#  define __MAX_TCP_MSS(h)         0
#endif

#ifdef CONFIG_NET_ETHERNET
#  undef  __MIN_TCP_MSS
#  undef  __MAX_TCP_MSS
#  define __MIN_TCP_MSS(h)         ETH_TCP_MSS(h)
#  define __MAX_TCP_MSS(h)         ETH_TCP_MSS(h)
#  define __ETH_MIN_TCP_MSS(h)     ETH_TCP_MSS(h)
#  define __ETH_MAX_TCP_MSS(h)     ETH_TCP_MSS(h)
#else
#  define __ETH_MIN_TCP_MSS(h)     INT_MAX
#  define __ETH_MAX_TCP_MSS(h)     0
#endif

#ifdef CONFIG_NET_6LOWPAN
#  undef  __MIN_TCP_MSS
#  undef  __MAX_TCP_MSS
#  define __MIN_TCP_MSS(h)         MIN(IEEE802154_TCP_MSS(h),__ETH_MIN_TCP_MSS(h))
#  define __MAX_TCP_MSS(h)         MAX(IEEE802154_TCP_MSS(h),__ETH_MAX_TCP_MSS(h))
#  define __6LOWPAN_MIN_TCP_MSS(h) MIN(IEEE802154_TCP_MSS(h),__ETH_MIN_TCP_MSS(h))
#  define __6LOWPAN_MAX_TCP_MSS(h) MAX(IEEE802154_TCP_MSS(h),__ETH_MAX_TCP_MSS(h))
#else
#  define __6LOWPAN_MIN_TCP_MSS(h) __ETH_MIN_TCP_MSS(h)
#  define __6LOWPAN_MAX_TCP_MSS(h) __ETH_MAX_TCP_MSS(h)
#endif

#ifdef CONFIG_NET_LOOPBACK
#  undef  __MIN_TCP_MSS
#  undef  __MAX_TCP_MSS
#  define __MIN_TCP_MSS(h)         MIN(LO_TCP_MSS(h),__6LOWPAN_MIN_TCP_MSS(h))
#  define __MAX_TCP_MSS(h)         MAX(LO_TCP_MSS(h),__6LOWPAN_MAX_TCP_MSS(h))
#  define __LOOP_MIN_TCP_MSS(h)    MIN(LO_TCP_MSS(h),__6LOWPAN_MIN_TCP_MSS(h))
#  define __LOOP_MAX_TCP_MSS(h)    MAX(LO_TCP_MSS(h),__6LOWPAN_MAX_TCP_MSS(h))
#else
#  define __LOOP_MIN_TCP_MSS(h)   __6LOWPAN_MIN_TCP_MSS(h)
#  define __LOOP_MAX_TCP_MSS(h)   __6LOWPAN_MAX_TCP_MSS(h)
#endif

#ifdef CONFIG_NET_SLIP
#  undef  __MIN_TCP_MSS
#  undef  __MAX_TCP_MSS
#  define __MIN_TCP_MSS(h)        MIN(SLIP_TCP_MSS(h),__LOOP_MIN_TCP_MSS(h))
#  define __MAX_TCP_MSS(h)        MAX(SLIP_TCP_MSS(h),__LOOP_MAX_TCP_MSS(h))
#  define __SLIP_MIN_TCP_MSS(h)   MIN(SLIP_TCP_MSS(h),__LOOP_MIN_TCP_MSS(h))
#  define __SLIP_MAX_TCP_MSS(h)   MAX(SLIP_TCP_MSS(h),__LOOP_MAX_TCP_MSS(h))
#else
#  define __SLIP_MIN_TCP_MSS(h)   __LOOP_MIN_TCP_MSS(h)
#  define __SLIP_MAX_TCP_MSS(h)   __LOOP_MAX_TCP_MSS(h)
#endif

#ifdef CONFIG_NET_TUN
#  undef  __MIN_TCP_MSS
#  undef  __MAX_TCP_MSS
#  define __MIN_TCP_MSS(h)        MIN(TUN_TCP_MSS(h),__SLIP_MIN_TCP_MSS(h))
#  define __MAX_TCP_MSS(h)        MAX(TUN_TCP_MSS(h),__SLIP_MAX_TCP_MSS(h))
#  define __TUN_MIN_TCP_MSS(h)    MIN(TUN_TCP_MSS(h),__SLIP_MIN_TCP_MSS(h))
#  define __TUN_MAX_TCP_MSS(h)    MAX(TUN_TCP_MSS(h),__SLIP_MAX_TCP_MSS(h))
#else
#  define __TUN_MIN_TCP_MSS(h)    __SLIP_MIN_TCP_MSS(h)
#  define __TUN_MAX_TCP_MSS(h)    __SLIP_MAX_TCP_MSS(h)
#endif

/* If IPv4 is supported, it will have the larger MSS.
 * NOTE: MSS calculation excludes the __TCP_HDRLEN.
 */

#ifdef CONFIG_NET_IPv6
#  define TCP_IPv6_MSS(d)       TCP_MSS(d,__IPv6_HDRLEN)
#  define ETH_IPv6_TCP_MSS      ETH_TCP_MSS(__IPv6_HDRLEN)
#  define SLIP_IPv6_TCP_MSS     SLIP_TCP_MSS(__IPv6_HDRLEN)
#  define TUN_IPv6_TCP_MSS      TUN_TCP_MSS(__IPv6_HDRLEN)
#  define MAX_TCP_MSS           __MAX_TCP_MSS(__IPv6_HDRLEN)
#endif

#ifdef CONFIG_NET_IPv4
#  define TCP_IPv4_MSS(d)       TCP_MSS(d,__IPv4_HDRLEN)
#  define ETH_IPv4_TCP_MSS      ETH_TCP_MSS(__IPv4_HDRLEN)
#  define SLIP_IPv4_TCP_MSS     SLIP_TCP_MSS(__IPv4_HDRLEN)
#  define TUN_IPv4_TCP_MSS      TUN_TCP_MSS(__IPv4_HDRLEN)
#  define MIN_TCP_MSS           __MIN_TCP_MSS(__IPv4_HDRLEN)
#  undef MAX_TCP_MSS
#  define MAX_TCP_MSS           __MAX_TCP_MSS(__IPv4_HDRLEN)
#endif

/* If IPv6 is supported, it will have the smaller MSS. */

#ifdef CONFIG_NET_IPv6
#  undef MIN_TCP_MSS
#  define MIN_TCP_MSS           __MIN_TCP_MSS(__IPv6_HDRLEN)
#endif

/* How long a connection should stay in the TIME_WAIT state (in units of
 * seconds).
 *
 * TIME_WAIT is often also known as the 2MSL wait state.  This is because
 * the socket that transitions to TIME_WAIT stays there for a period that
 * is 2 x Maximum Segment Lifetime in duration.  The MSL is the maximum
 * amount of time that any segment can remain valid on the network before
 * being discarded.  This time limit is ultimately bounded by the TTL field
 * in the IP datagram that is used to transmit the TCP segment.  RFC 793
 * specifies MSL as 2 minutes but most systems permit this value to be tuned.
 * Here a default TIME_WAIT (2MSL) 2 minutes is used, half the value
 * specified by RFC 793.
 */

#ifdef CONFIG_NET_TCP_WAIT_TIMEOUT
#  define TCP_TIME_WAIT_TIMEOUT CONFIG_NET_TCP_WAIT_TIMEOUT
#else
#  define TCP_TIME_WAIT_TIMEOUT (60*2)
#endif

/* ARP configuration options */

#ifndef CONFIG_NET_ARPTAB_SIZE
/* The size of the ARP table.
 *
 * This option should be set to a larger value if this network node will
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

/* Usrsock configuration options */

/* The maximum amount of concurrent usrsock connections, Default: 6 */

#ifndef CONFIG_NET_USRSOCK_CONNS
#  ifdef CONFIG_NET_USRSOCK
#    define CONFIG_NET_USRSOCK_CONNS 6
#  else
#    define CONFIG_NET_USRSOCK_CONNS 0
#  endif
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Statistics datatype
 *
 * This typedef defines the datatype used for keeping statistics in
 * the network.
 */

typedef uint16_t net_stats_t;

#endif /* __INCLUDE_NUTTX_NET_NETCONFG_H */
