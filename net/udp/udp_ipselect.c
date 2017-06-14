/****************************************************************************
 * net/udp/udp_ipselect.c
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
#ifdef CONFIG_NET

#include <stdint.h>
#include <debug.h>

#include <net/if.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/udp.h>

#include "udp/udp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_ipv4_select
 *
 * Description:
 *   Configure to send or receive an UDP IPv4 packet
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
void udp_ipv4_select(FAR struct net_driver_s *dev)
{
  /* Clear a bit in the d_flags to distinguish this from an IPv6 packet */

  IFF_SET_IPv4(dev->d_flags);

  /* Set the offset to the beginning of the UDP data payload */

  dev->d_appdata = &dev->d_buf[IPv4UDP_HDRLEN + NET_LL_HDRLEN(dev)];
}
#endif /* CONFIG_NET_IPv4 */

/****************************************************************************
 * Name: udp_ipv6_select
 *
 * Description:
 *   Configure to send or receive an UDP IPv6 packet
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
void udp_ipv6_select(FAR struct net_driver_s *dev)
{
  /* Set a bit in the d_flags to distinguish this from an IPv6 packet */

  IFF_SET_IPv6(dev->d_flags);

  /* Set the offset to the beginning of the UDP data payload */

  dev->d_appdata = &dev->d_buf[IPv6UDP_HDRLEN + NET_LL_HDRLEN(dev)];
}
#endif /* CONFIG_NET_IPv6 */

#endif /* CONFIG_NET */
