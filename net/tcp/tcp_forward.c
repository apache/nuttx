/****************************************************************************
 * net/tcp/tcp_forward.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>

#include "tcp/tcp.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_IPFORWARD) && defined(CONFIG_NET_TCP)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_ipv6_forward
 *
 * Description:
 *   Set up to forward the TCP packet on the specified device.  This
 *   function will set up a send "interrupt" handler that will perform
 *   the actual send asynchronously and must return without waiting for the
 *   send to complete.
 *
 * Input Parameters:
 *   dev   - The device on which the packet should be forwarded.
 *   ipv6  - A pointer to the IPv6 header in within the IPv6 packet.  This
 *           is immeidately followed by the TCP header.
 *   iob   - A list of IOBs containing the data payload to be sent.
 *
 * Returned Value:
 *   Zero is returned if the packet was successfully forwarded;  A negated
 *   errno value is returned if the packet is not forwardable.  In that
 *   latter case, the caller should free the IOB list and drop the packet.
 *
 ****************************************************************************/

#if defined(CONFIG_NET_IPv6) && defined(CONFIG_NETDEV_MULTINIC)
int tcp_ipv6_forward(FAR struct net_driver_s *dev,
                     FAR struct ipv6_hdr_s *ipv6, FAR struct iob_s *iob)
{
  /* Notify the forwarding device that TX data is available */

  /* Set up to send the packet when the selected device polls for TX data.
   * TCP packets must obey ACK and windowing rules.
   */

#warning Missing logic

  /* REVISIT:  For Ethernet we may have to fix up the Ethernet header:
   * - source MAC, the MAC of the current device.
   * - dest MAC, the MAC associated with the destination IPv6 adress.
   *   This will involve ICMPv6 and Neighbor Discovery.
   * - Because of TCP window, the packet may have to be sent in smaller
   *   pieces.
   */

  nwarn("WARNING: TCP packet forwarding not yet supported\n");
  return -ENOSYS;
}
#endif /* CONFIG_NET_IPv6 && CONFIG_NETDEV_MULTINIC */
#endif /* CONFIG_NET && CONFIG_NET_IPFORWARD && CONFIG_NET_TCP */
