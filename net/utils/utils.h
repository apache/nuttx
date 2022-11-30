/****************************************************************************
 * net/utils/utils.h
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

#ifndef __NET_UTILS_UTILS_H
#define __NET_UTILS_UTILS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* These values control the behavior of net_timeval2desc */

enum tv2ds_remainder_e
{
  TV2DS_TRUNC = 0, /* Truncate microsecond remainder */
  TV2DS_ROUND,     /* Round to the nearest full decisecond */
  TV2DS_CEIL       /* Force to next larger full decisecond */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct net_driver_s;      /* Forward reference */
struct timeval;           /* Forward reference */

/****************************************************************************
 * Name: net_breaklock
 *
 * Description:
 *   Break the lock, return information needed to restore re-entrant lock
 *   state.
 *
 ****************************************************************************/

int net_breaklock(FAR unsigned int *count);

/****************************************************************************
 * Name: net_restorelock
 *
 * Description:
 *   Restore the locked state
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failured (probably -ECANCELED).
 *
 ****************************************************************************/

int net_restorelock(unsigned int count);

/****************************************************************************
 * Name: net_dsec2timeval
 *
 * Description:
 *   Convert a decisecond value to a struct timeval.  Needed by getsockopt()
 *   to report timeout values.
 *
 * Input Parameters:
 *   dsec The decisecond value to convert
 *   tv   The struct timeval to receive the converted value
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void net_dsec2timeval(uint16_t dsec, FAR struct timeval *tv);

/****************************************************************************
 * Name: net_dsec2tick
 *
 * Description:
 *   Convert a decisecond value to a system clock ticks.  Used by IGMP logic.
 *
 * Input Parameters:
 *   dsec The decisecond value to convert
 *
 * Returned Value:
 *   The decisecond value expressed as system clock ticks
 *
 ****************************************************************************/

unsigned int net_dsec2tick(int dsec);

/****************************************************************************
 * Name: net_timeval2dsec
 *
 * Description:
 *   Convert a struct timeval to deciseconds.  Needed by setsockopt() to
 *   save new timeout values.
 *
 * Input Parameters:
 *   tv        - The struct timeval to convert
 *   remainder - Determines how to handler the microsecond remainder
 *
 * Returned Value:
 *   The converted value
 *
 * Assumptions:
 *
 ****************************************************************************/

unsigned int net_timeval2dsec(FAR struct timeval *tv,
                              enum tv2ds_remainder_e remainder);

/****************************************************************************
 * Name: net_ipv6_mask2pref
 *
 * Description:
 *   Convert a 128-bit netmask to a prefix length.  The NuttX IPv6
 *   networking uses 128-bit network masks internally.  This function
 *   converts the IPv6 netmask to a prefix length.
 *
 *   The prefix length is the number of MS '1' bits on in the netmask.
 *   This, of course, assumes that all MS bits are '1' and all LS bits are
 *   '0' with no intermixed 1's and 0's.  This function searches from the MS
 *   bit until the first '0' is found (this does not necessary mean that
 *   there might not be additional '1' bits following the firs '0', but that
 *   will be a malformed netmask.
 *
 * Input Parameters:
 *   mask   Points to an IPv6 netmask in the form of uint16_t[8]
 *
 * Returned Value:
 *   The prefix length, range 0-128 on success;  This function will not
 *   fail.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
uint8_t net_ipv6_mask2pref(FAR const uint16_t *mask);
#endif

/****************************************************************************
 * Name: net_ipv6_pref2mask
 *
 * Description:
 *   Convert a IPv6 prefix length to a network mask.  The prefix length
 *   specifies the number of MS bits under mask (0-128)
 *
 * Input Parameters:
 *   preflen  - Determines the width of the netmask (in bits).  Range 0-128
 *   mask  - The location to return the netmask.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void net_ipv6_pref2mask(uint8_t preflen, net_ipv6addr_t mask);
#endif

/****************************************************************************
 * Name: net_chksum_adjust
 *
 * Description:
 *   Adjusts the checksum of a packet without having to completely
 *   recalculate it, as described in RFC 3022, Section 4.2, Page 9.
 *
 * Input Parameters:
 *   chksum - points to the chksum in the packet
 *   optr   - points to the old data in the packet
 *   olen   - length of old data
 *   nptr   - points to the new data in the packet
 *   nlen   - length of new data
 *
 * Limitations:
 *   The algorithm is applicable only for even offsets and even lengths.
 *
 ****************************************************************************/

void net_chksum_adjust(FAR uint16_t *chksum,
                       FAR const uint16_t *optr, ssize_t olen,
                       FAR const uint16_t *nptr, ssize_t nlen);

/****************************************************************************
 * Name: tcp_chksum, tcp_ipv4_chksum, and tcp_ipv6_chksum
 *
 * Description:
 *   Calculate the TCP checksum of the packet in d_buf and d_appdata.
 *
 *   The TCP checksum is the Internet checksum of data contents of the
 *   TCP segment, and a pseudo-header as defined in RFC793.
 *
 *   Note: The d_appdata pointer that points to the packet data may
 *   point anywhere in memory, so it is not possible to simply calculate
 *   the Internet checksum of the contents of the d_buf buffer.
 *
 * Returned Value:
 *   The TCP checksum of the TCP segment in d_buf and pointed to by
 *   d_appdata.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
uint16_t tcp_ipv4_chksum(FAR struct net_driver_s *dev);
#endif

#ifdef CONFIG_NET_IPv6
uint16_t tcp_ipv6_chksum(FAR struct net_driver_s *dev);
#endif

#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
uint16_t tcp_chksum(FAR struct net_driver_s *dev);
#elif defined(CONFIG_NET_IPv4)
#  define tcp_chksum(d) tcp_ipv4_chksum(d)
#else /* if defined(CONFIG_NET_IPv6) */
#  define tcp_chksum(d) tcp_ipv6_chksum(d)
#endif

/****************************************************************************
 * Name: udp_ipv4_chksum
 *
 * Description:
 *   Calculate the UDP/IPv4 checksum of the packet in d_buf and d_appdata.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP_CHECKSUMS) && defined(CONFIG_NET_IPv4)
uint16_t udp_ipv4_chksum(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: udp_ipv6_chksum
 *
 * Description:
 *   Calculate the UDP/IPv6 checksum of the packet in d_buf and d_appdata.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_UDP_CHECKSUMS) && defined(CONFIG_NET_IPv6)
uint16_t udp_ipv6_chksum(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: icmp_chksum
 *
 * Description:
 *   Calculate the checksum of the IPv4 ICMP message
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP
uint16_t icmp_chksum(FAR struct net_driver_s *dev, int len);
#endif

/****************************************************************************
 * Name: icmpv6_chksum
 *
 * Description:
 *   Calculate the checksum of the ICMPv6 message
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
uint16_t icmpv6_chksum(FAR struct net_driver_s *dev, unsigned int iplen);
#endif

/****************************************************************************
 * Name: net_bound_device
 *
 * Description:
 *   If the socket is bound to a device, return the reference to the
 *   bound device.
 *
 * Input Parameters:
 *   sconn - Socket connection structure (not currently used).
 *
 * Returned Value:
 *   A reference to the bound device.  If the retained interface index no
 *   longer refers to a valid device, this function will unbind the device
 *   and return an arbitrary network device at the head of the list of
 *   registered devices.  This supports legacy IPv4 DHCPD behavior when
 *   there is only a single registered network device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_BINDTODEVICE
FAR struct net_driver_s *net_bound_device(FAR struct socket_conn_s *sconn);
#else
#  define net_bound_device(c) netdev_default();
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __NET_UTILS_UTILS_H */
