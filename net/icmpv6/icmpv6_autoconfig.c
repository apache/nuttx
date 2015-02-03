/****************************************************************************
 * net/icmpv6/icmpv6_autoconfig.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifdef CONFIG_NET_ICMPv6_AUTOCONF

#include <errno.h>

#include "icmpv6/icmpv6.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_autoconfig
 *
 * Description:
 *   Perform IPv6 auto-configuration to assign an IPv6 address to this
 *   device.
 *
 *   Stateless auto-configuration exploits several other features in IPv6,
 *   including link-local addresses, multi-casting, the Neighbor Discovery
 *   protocol, and the ability to generate the interface identifier of an
 *   address from the underlying data link layer address. The general idea
 *   is to have a device generate a temporary address until it can determine
 *   the characteristics of the network it is on, and then create a permanent
 *   address it can use based on that information.
 *
 * Parameters:
 *   dev - The device driver structure to assign the address to
 *
 * Return:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int icmpv6_autoconfig(FAR struct net_driver_s *dev)
{
  /* IPv6 Stateless Autoconfiguration
   *
   * The following is a summary of the steps a device takes when using
   * stateless auto-configuration:
   *
   * 1. Link-Local Address Generation: The device generates a link-local
   *    address. Recall that this is one of the two types of local-use IPv6
   *    addresses. Link-local addresses have "1111 1110 10" for the first
   *    ten bits. The generated address uses those ten bits followed by 54
   *    zeroes and then the 64 bit interface identifier. Typically this
   *    will be derived from the data link layer (MAC) address.
   *
   *    IEEE 802 MAC addresses, used by Ethernet and other IEEE 802 Project
   *    networking technologies, have 48 bits.  The IEEE has also defined a
   *    format called the 64-bit extended unique identifier, abbreviated
   *    EUI-64.  To get the modified EUI-64 interface ID for a device, you
   *    simply take the EUI-64 address and change the 7th bit from the left
   *    (the"universal/local" or "U/L" bit) from a zero to a one.
   *
   * 2. Link-Local Address Uniqueness Test: The node tests to ensure that
   *    the address it generated isn't for some reason already in use on the
   *    local network. (This is very unlikely to be an issue if the link-local
   *    address came from a MAC address but more likely if it was based on a
   *    generated token.) It sends a Neighbor Solicitation message using the
   *    Neighbor Discovery (ND) protocol. It then listens for a Neighbor
   *    Advertisement in response that indicates that another device is
   *    already using its link-local address; if so, either a new address
   *    must be generated, or auto-configuration fails and another method
   *    must be employed.
   *
   * 3. Link-Local Address Assignment: Assuming the uniqueness test passes,
   *    the device assigns the link-local address to its IP interface. This
   *    address can be used for communication on the local network, but not
   *    on the wider Internet (since link-local addresses are not routed).
   *
   * 4. Router Contact: The node next attempts to contact a local router for
   *    more information on continuing the configuration. This is done either
   *    by listening for Router Advertisement messages sent periodically by
   *    routers, or by sending a specific Router Solicitation to ask a router
   *    for information on what to do next.
   *
   * 5. Router Direction: The router provides direction to the node on how to
   *    proceed with the auto-configuration. It may tell the node that on this
   *    network "stateful" auto-configuration is in use, and tell it the
   *    address of a DHCP server to use. Alternately, it will tell the host
   *    how to determine its global Internet address.
   *
   * 6. Global Address Configuration: Assuming that stateless auto-
   *    configuration is in use on the network, the host will configure
   *    itself with its globally-unique Internet address. This address is
   *    generally formed from a network prefix provided to the host by the
   *    router, combined with the device's identifier as generated in the
   *    first step.
   *
   * Reference: http://www.tcpipguide.com/free/t_IPv6AutoconfiguratinoandRenumbering.htm
   */

#warning Missing logic
  return -ENOSYS;
}

#endif /* CONFIG_NET_ICMPv6_AUTOCONF */
