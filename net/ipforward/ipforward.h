/****************************************************************************
 * net/ipforward/ipforward.h
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

#ifndef __NET_IPFORWARD_IPFORWARD_H
#define __NET_IPFORWARD_IPFORWARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#undef HAVE_FWDALLOC
#ifdef CONFIG_NET_IPFORWARD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HAVE_FWDALLOC 1

#ifndef CONFIG_NET_IPFORWARD_NSTRUCT
#  define CONFIG_NET_IPFORWARD_NSTRUCT 4
#endif

/* Allocate a new IP forwarding data callback */

#define ipfwd_callback_alloc(dev)   devif_callback_alloc(dev, &(dev)->d_conncb)
#define ipfwd_callback_free(dev,cb) devif_dev_callback_free(dev, cb)

/****************************************************************************
 * Public Types
 ****************************************************************************/

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
#if defined(CONFIG_NET_IPv4) && defined(CONFIG_NET_IPv6)
  uint8_t                      f_domain;  /* Domain: PF_INET or PF_INET6 */
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct ipv4_hdr_s; /* Forward reference */
struct ipv6_hdr_s; /* Forward reference */

/****************************************************************************
 * Name: ipfwd_initialize
 *
 * Description:
 *   Initialize the struct forward_s allocator.
 *
 * Assumptions:
 *   Called early in system initialization.
 *
 ****************************************************************************/

void ipfwd_initialize(void);

/****************************************************************************
 * Name: ipfwd_alloc
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

FAR struct forward_s *ipfwd_alloc(void);

/****************************************************************************
 * Name: ipfwd_free
 *
 * Description:
 *   Free a forwarding structure by adding it to a free list.
 *
 * Assumptions:
 *   Caller holds the network lock.  Mutually excluvive access to the free
 *   list is assured by this lock.
 *
 ****************************************************************************/

void ipfwd_free(FAR struct forward_s *fwd);

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

/****************************************************************************
 * Name: ipfwd_forward
 *
 * Description:
 *   Called by the IP forwarding logic when a packet is received on one
 *   network device, but must be forwarded on another network device.
 *
 *   Set up to forward the packet on the specified device.  This function
 *   will set up a send event handler that will perform the actual send
 *   asynchronously and must return without waiting for the send to
 *   complete.
 *
 * Input Parameters:
 *   fwd - An initialized instance of the common forwarding structure that
 *         includes everything needed to perform the forwarding operation.
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forwarded;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller should free the IOB list and drop the packet.
 *
 ****************************************************************************/

int ipfwd_forward(FAR struct forward_s *fwd);

/****************************************************************************
 * Name: ipfwd_poll
 *
 * Description:
 *   Poll all pending transfer for ARP requests to send.
 *
 * Assumptions:
 *   This function is called from the MAC device driver indirectly through
 *   devif_poll() and devif_timer().
 *
 ****************************************************************************/

void ipfwd_poll(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: ipfwd_dropstats
 *
 * Description:
 *   Update statistics for a dropped packet.
 *
 * Input Parameters:
 *   fwd - The forwarding state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_STATISTICS
void ipfwd_dropstats(FAR struct forward_s *fwd);
#else
#  define ipfwd_dropstats(fwd)
#endif

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

/****************************************************************************
 * Name: ipv6_dropstats
 *
 * Description:
 *   Update statistics for a dropped Ipv6 packet.
 *
 * Input Parameters:
 *   ipv6  - A pointer to the IPv6 header in within the IPv6 packet to be
 *           dropped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_NET_STATISTICS) && defined(CONFIG_NET_IPv6)
void ipv6_dropstats(FAR struct ipv6_hdr_s *ipv6);
#else
#  define ipv6_dropstats(ipv6)
#endif

/****************************************************************************
 * Name: ipv4_dropstats
 *
 * Description:
 *   Update statistics for a dropped Ipv4 packet.
 *
 * Input Parameters:
 *   ipv4  - A pointer to the IPv4 header in within the IPv4 packet to be
 *           dropped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_NET_STATISTICS) && defined(CONFIG_NET_IPv4)
void ipv4_dropstats(FAR struct ipv4_hdr_s *ipv4);
#else
#  define ipv4_dropstats(ipv4)
#endif

#endif /* CONFIG_NET_IPFORWARD */
#endif /* __NET_IPFORWARD_IPFORWARD_H */
