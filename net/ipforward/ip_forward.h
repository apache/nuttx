/****************************************************************************
 * net/ipforward/ip_forward.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __NET_IPFORWARD_IP_FORWARD_H
#define __NET_IPFORWARD_IP_FORWARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/net/ip.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/tcp.h>
#include <nuttx/net/icmp.h>
#include <nuttx/net/icmpv6.h>

#include "udp/udp.h"
#include "tcp/tcp.h"
#include "icmpv6/icmpv6.h"

#undef HAVE_FWDALLOC
#ifdef CONFIG_NET_IPFORWARD

/* Must of the logic in this header file applies only for configurations
 * will multiple network devices.
 */

#ifdef CONFIG_NETDEV_MULTINIC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HAVE_FWDALLOC 1

#ifndef CONFIG_NET_IPFORWARD_NSTRUCT
#  define CONFIG_NET_IPFORWARD_NSTRUCT 4
#endif

#define FWD_HEADER(fwd) (FAR union fwd_iphdr_u *)((fwd)->f_iob->io_data)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IPv4 + L2 header */

#ifdef CONFIG_NET_IPv4
struct fwd_ipv4hdr_u
{
  struct ipv4_hdr_s       l2;
  union
  {
#ifdef CONFIG_NET_TCP
    uint8_t               pad[TCP_MAX_HDRLEN];
    struct tcp_hdr_s      tcp;
#endif
#ifdef CONFIG_NET_UDP
    struct udp_hdr_s      udp;
#endif
#ifdef CONFIG_NET_ICMPv6
    struct icmp_hdr_s     icmp;
#endif
  } l3;
};
#endif

/* IPv6 + L2 header */

#ifdef CONFIG_NET_IPv6
struct fwd_ipv6hdr_u
{
  struct ipv6_hdr_s       l2;
  union
  {
#ifdef CONFIG_NET_TCP
    uint8_t               pad[TCP_MAX_HDRLEN];
    struct tcp_hdr_s      tcp;
#endif
#ifdef CONFIG_NET_UDP
    struct udp_hdr_s      udp;
#endif
#ifdef CONFIG_NET_ICMPv6
    struct icmpv6_hdr_s   icmpv6;
#endif
  } l3;
};
#endif

/* IPv4 or IPv6 + L2 header */

union fwd_iphdr_u
{
#ifdef CONFIG_NET_IPv4
  struct fwd_ipv4hdr_u  ipv4;
#endif
#ifdef CONFIG_NET_IPv6
  struct fwd_ipv6hdr_u  ipv6;
#endif
};

/* Connection structures */

union fwd_conn_u
{
#ifdef CONFIG_NET_TCP
  struct tcp_conn_s    tcp;
#endif
#ifdef CONFIG_NET_UDP
  struct udp_conn_s    udp;
#endif
#ifdef CONFIG_NET_ICMPv6
  struct icmpv6_conn_s icmpv6;
#endif
};

/* This is the send state structure */

struct devif_callback_s; /* Forward refernce */
struct net_driver_s;     /* Forward reference */
struct iob_s;            /* Forward reference */

struct forward_s
{
  FAR struct forward_s        *f_flink;   /* Supports a singly linked list */
  FAR struct net_driver_s     *f_dev;     /* Forwarding device */
  FAR struct iob_s            *f_iob;     /* IOB chain containing the packet */
  FAR struct devif_callback_s *f_cb;      /* Reference to callback instance */
  union fwd_conn_u             f_conn;    /* Protocol-specific connection struct */
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
  uint8_t                      f_domain;  /* Domain: PF_INET or PF_INET6 */
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ip_forward_initialize
 *
 * Description:
 *   Initialize the struct forward_s allocator.
 *
 * Assumptions:
 *   Called early in system initialization.
 *
 ****************************************************************************/

void ip_forward_initialize(void);

/****************************************************************************
 * Name: ip_forward_alloc
 *
 * Description:
 *   Allocate a forwarding structure by removing a pre-allocated entry from
 *   a free list.
 *
 * Assumptions:
 *   Caller holds the network lock.  Mutually excluvive access to the free
 *   list is assured by this lock.
 *
 ****************************************************************************/

FAR struct forward_s *ip_forward_alloc(void);

/****************************************************************************
 * Name: ip_forward_free
 *
 * Description:
 *   Free a forwarding structure by adding it to a free list.
 *
 * Assumptions:
 *   Caller holds the network lock.  Mutually excluvive access to the free
 *   list is assured by this lock.
 *
 ****************************************************************************/

void ip_forward_free(FAR struct forward_s *fwd);

/****************************************************************************
 * Name: ipv4_forward_broadcast
 *
 * Description:
 *   This function is called from ipv4_input when a broadcast or multicast
 *   packet is received.  If CONFIG_NET_IPFORWARD_BROADCAST is enabled, this
 *   function will forward the broadcast packet to other networks through
 *   other network devices.
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv4 packet.
 *   ipv4  - A convenience pointer to the IPv4 header in within the IPv4
 *           packet
 *
 *   On input:
 *   - dev->d_buf holds the received packet.
 *   - dev->d_len holds the length of the received packet MINUS the
 *     size of the L1 header.  That was subtracted out by ipv4_input.
 *   - ipv4 points to the IPv4 header with dev->d_buf.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPFORWARD_BROADCAST
void ipv4_forward_broadcast(FAR struct net_driver_s *dev,
                            FAR struct ipv4_hdr_s *ipv4);
#endif

/****************************************************************************
 * Name: ipv6_forward_broadcast
 *
 * Description:
 *   This function is called from ipv6_input when a broadcast or multicast
 *   packet is received.  If CONFIG_NET_IPFORWARD_BROADCAST is enabled, this
 *   function will forward the broadcast packet to other networks through
 *   other network devices.
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv6 packet.
 *   ipv6  - A convenience pointer to the IPv6 header in within the IPv6
 *           packet
 *
 *   On input:
 *   - dev->d_buf holds the received packet.
 *   - dev->d_len holds the length of the received packet MINUS the
 *     size of the L1 header.  That was subtracted out by ipv6_input.
 *   - ipv6 points to the IPv6 header with dev->d_buf.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPFORWARD_BROADCAST
void ipv6_forward_broadcast(FAR struct net_driver_s *dev,
                            FAR struct ipv6_hdr_s *ipv6);
#endif

/****************************************************************************
 * Name: devif_forward
 *
 * Description:
 *   Called from protocol-specific IP forwarding logic to re-send a packet.
 *
 * Input Parameters:
 *   fwd - An initialized instance of the common forwarding structure that
 *         includes everything needed to perform the forwarding operation.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void devif_forward(FAR struct forward_s *fwd);

#endif /* CONFIG_NETDEV_MULTINIC */

/****************************************************************************
 * Name: ipv4_forward
 *
 * Description:
 *   This function is called from ipv4_input when a packet is received that
 *   is not destined for us.  In this case, the packet may need to be
 *   forwarded to another device (or sent back out the same device)
 *   depending configuration, routing table information, and the IPv4
 *   networks served by various network devices.
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv4 packet.
 *   ipv4  - A convenience pointer to the IPv4 header in within the IPv4
 *           packet
 *
 *   On input:
 *   - dev->d_buf holds the received packet.
 *   - dev->d_len holds the length of the received packet MINUS the
 *     size of the L1 header.  That was subtracted out by ipv4_input.
 *   - ipv4 points to the IPv4 header with dev->d_buf.
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forward;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller (ipv4_input()) should drop the packet.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int ipv4_forward(FAR struct net_driver_s *dev, FAR struct ipv4_hdr_s *ipv4);
#endif

/****************************************************************************
 * Name: ipv6_forward
 *
 * Description:
 *   This function is called from ipv6_input when a packet is received that
 *   is not destined for us.  In this case, the packet may need to be
 *   forwarded to another device (or sent back out the same device)
 *   depending configuration, routing table information, and the IPv6
 *   networks served by various network devices.
 *
 * Input Parameters:
 *   dev   - The device on which the packet was received and which contains
 *           the IPv6 packet.
 *   ipv6  - A convenience pointer to the IPv6 header in within the IPv6
 *           packet
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forward;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller (ipv6_input()) should drop the packet.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
int ipv6_forward(FAR struct net_driver_s *dev, FAR struct ipv6_hdr_s *ipv6);
#endif

#endif /* CONFIG_NET_IPFORWARD */
#endif /* __NET_IPFORWARD_IP_FORWARD_H */
