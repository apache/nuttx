/****************************************************************************
 * net/uip/uip-chksum.c
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
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

#include <sys/types.h>
#include <debug.h>

#include <net/uip/uipopt.h>
#include <net/uip/uip.h>
#include <net/uip/uip-arch.h>

#include "uip-internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define BUF ((struct uip_tcpip_hdr *)&dev->d_buf[UIP_LLH_LEN])

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if !UIP_ARCH_CHKSUM
static uint16 chksum(uint16 sum, const uint8 *data, uint16 len)
{
  uint16 t;
  const uint8 *dataptr;
  const uint8 *last_byte;

  dataptr = data;
  last_byte = data + len - 1;

  while(dataptr < last_byte)
    {
      /* At least two more bytes */

      t = (dataptr[0] << 8) + dataptr[1];
      sum += t;
      if (sum < t)
        {
          sum++; /* carry */
        }
      dataptr += 2;
    }

  if (dataptr == last_byte)
    {
      t = (dataptr[0] << 8) + 0;
      sum += t;
      if (sum < t)
        {
          sum++; /* carry */
        }
    }

  /* Return sum in host byte order. */

  return sum;
}

static uint16 upper_layer_chksum(struct uip_driver_s *dev, uint8 proto)
{
  uint16 upper_layer_len;
  uint16 sum;

#ifdef CONFIG_NET_IPv6
  upper_layer_len = (((uint16)(BUF->len[0]) << 8) + BUF->len[1]);
#else /* CONFIG_NET_IPv6 */
  upper_layer_len = (((uint16)(BUF->len[0]) << 8) + BUF->len[1]) - UIP_IPH_LEN;
#endif /* CONFIG_NET_IPv6 */

  /* First sum pseudoheader. */

  /* IP protocol and length fields. This addition cannot carry. */

  sum = upper_layer_len + proto;

  /* Sum IP source and destination addresses. */

  sum = chksum(sum, (uint8 *)&BUF->srcipaddr, 2 * sizeof(uip_ipaddr_t));

  /* Sum TCP header and data. */

  sum = chksum(sum, &dev->d_buf[UIP_IPH_LEN + UIP_LLH_LEN], upper_layer_len);

  return (sum == 0) ? 0xffff : htons(sum);
}

#ifdef CONFIG_NET_IPv6
static uint16 uip_icmp6chksum(struct uip_driver_s *dev)
{
  return upper_layer_chksum(dev, UIP_PROTO_ICMP6);
}
#endif /* CONFIG_NET_IPv6 */

#endif /* UIP_ARCH_CHKSUM */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Calculate the Internet checksum over a buffer. */

#if !UIP_ARCH_ADD32
static void uip_carry32(uint8 *sum, uint16 op16)
{
  if (sum[2] < (op16 >> 8))
    {
      ++sum[1];
      if (sum[1] == 0)
        {
          ++sum[0];
        }
    }

  if (sum[3] < (op16 & 0xff))
    {
      ++sum[2];
      if (sum[2] == 0)
        {
          ++sum[1];
          if (sum[1] == 0)
            {
              ++sum[0];
            }
        }
    }
}

void uip_add32(const uint8 *op32, uint16 op16, uint8 *sum)
{
  /* op32 and the sum are in network order (big-endian); op16 is host order. */

  sum[3] = op32[3] + (op16 & 0xff);
  sum[2] = op32[2] + (op16 >> 8);
  sum[1] = op32[1];
  sum[0] = op32[0];
  uip_carry32(sum, op16);
}

void uip_incr32(uint8 *op32, uint16 op16)
{
  op32[3] += (op16 & 0xff);
  op32[2] += (op16 >> 8);
  uip_carry32(op32, op16);
}

#endif /* UIP_ARCH_ADD32 */

#if !UIP_ARCH_CHKSUM
uint16 uip_chksum(uint16 *data, uint16 len)
{
  return htons(chksum(0, (uint8 *)data, len));
}

/* Calculate the IP header checksum of the packet header in d_buf. */

#ifndef UIP_ARCH_IPCHKSUM
uint16 uip_ipchksum(struct uip_driver_s *dev)
{
  uint16 sum;

  sum = chksum(0, &dev->d_buf[UIP_LLH_LEN], UIP_IPH_LEN);
  return (sum == 0) ? 0xffff : htons(sum);
}
#endif

/* Calculate the TCP checksum of the packet in d_buf and d_appdata. */

uint16 uip_tcpchksum(struct uip_driver_s *dev)
{
  return upper_layer_chksum(dev, UIP_PROTO_TCP);
}

/* Calculate the UDP checksum of the packet in d_buf and d_appdata. */

#ifdef CONFIG_NET_UDP_CHECKSUMS
uint16 uip_udpchksum(struct uip_driver_s *dev)
{
  return upper_layer_chksum(dev, UIP_PROTO_UDP);
}
#endif /* UIP_UDP_CHECKSUMS */
#endif /* UIP_ARCH_CHKSUM */

#endif /* CONFIG_NET */
