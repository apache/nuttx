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
 * Name: net_lockinitialize
 *
 * Description:
 *   Initialize the locking facility
 *
 ****************************************************************************/

void net_lockinitialize(void);

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
 * Name: chksum
 *
 * Description:
 *   Calculate the raw change some over the memory region described by
 *   data and len.
 *
 * Input Parameters:
 *   sum  - Partial calculations carried over from a previous call to chksum.
 *          This should be zero on the first time that check sum is called.
 *   data - Beginning of the data to include in the checksum.
 *   len  - Length of the data to include in the checksum.
 *
 * Returned Value:
 *   The updated checksum value.
 *
 ****************************************************************************/

uint16_t chksum(uint16_t sum, FAR const uint8_t *data, uint16_t len);

/****************************************************************************
 * Name: net_chksum
 *
 * Description:
 *   Calculate the Internet checksum over a buffer.
 *
 *   The Internet checksum is the one's complement of the one's complement
 *   sum of all 16-bit words in the buffer.
 *
 *   See RFC1071.
 *
 *   If CONFIG_NET_ARCH_CHKSUM is defined, then this function must be
 *   provided by architecture-specific logic.
 *
 * Input Parameters:
 *
 *   buf - A pointer to the buffer over which the checksum is to be computed.
 *
 *   len - Length of the buffer over which the checksum is to be computed.
 *
 * Returned Value:
 *   The Internet checksum of the buffer.
 *
 ****************************************************************************/

uint16_t net_chksum(FAR uint16_t *data, uint16_t len);

/****************************************************************************
 * Name: ipv4_upperlayer_chksum
 *
 * Description:
 *   Perform the checksum calcaultion over the IPv4, protocol headers, and
 *   data payload as necessary.
 *
 * Input Parameters:
 *   dev   - The network driver instance.  The packet data is in the d_buf
 *           of the device.
 *   proto - The protocol being supported
 *
 * Returned Value:
 *   The calculated checksum
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
uint16_t ipv4_upperlayer_chksum(FAR struct net_driver_s *dev, uint8_t proto);
#endif

/****************************************************************************
 * Name: ipv6_upperlayer_chksum
 *
 * Description:
 *   Perform the checksum calculation over the IPv6, protocol headers, and
 *   data payload as necessary.
 *
 * Input Parameters:
 *   dev   - The network driver instance.  The packet data is in the d_buf
 *           of the device.
 *   proto - The protocol being supported
 *   iplen - The size of the IPv6 header.  This may be larger than
 *           IPv6_HDRLEN the IPv6 header if IPv6 extension headers are
 *           present.
 *
 * Returned Value:
 *   The calculated checksum
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
uint16_t ipv6_upperlayer_chksum(FAR struct net_driver_s *dev,
                                uint8_t proto, unsigned int iplen);
#endif

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

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __NET_UTILS_UTILS_H */
