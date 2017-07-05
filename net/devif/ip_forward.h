/****************************************************************************
 * net/devif/ip_forward.h
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

#ifndef  __NET_DEVIF_IP_FORWARD_H
#define  __NET_DEVIF_IP_FORWARD_H

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
#if defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NETDEV_MULTINIC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HAVE_FWDALLOC 1

#ifndef CONFIG_NET_IPFORWARD_NSTRUCT
#  define CONFIG_NET_IPFORWARD_NSTRUCT 4
#endif

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
  FAR struct iob_s            *f_iob;     /* IOBs containing the data payload */
  FAR struct devif_callback_s *f_cb;      /* Reference to callback instance */
  union fwd_iphdr_u            f_hdr;     /* Copy of original L2+L3 headers */
  union fwd_conn_u             f_conn;    /* Protocol-specific connectin struct */
  uint8_t                      f_hdrsize; /* The size of the L2+L3 headers */
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

#endif /* CONFIG_NET_IPFORWARD && CONFIG_NETDEV_MULTINIC */
#endif /* __NET_DEVIF_IP_FORWARD_H */
