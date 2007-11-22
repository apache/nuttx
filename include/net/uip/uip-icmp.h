/****************************************************************************
 * net/uip/uip-icmp.h
 * Header file for the uIP ICMP stack.
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * This logic was leveraged from uIP which also has a BSD-style license:
 *
 *   Author Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2001-2003, Adam Dunkels.
 *   All rights reserved.
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

#ifndef __NET_UIP_UIP_ICMP_H
#define __NET_UIP_UIP_ICMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <net/uip/uipopt.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* The ICMP and IP headers */

struct uip_icmpip_hdr
{
#ifdef CONFIG_NET_IPv6

  /* IPv6 Ip header */

  uint8  vtc;               /* Bits 0-3: version, bits 4-7: traffic class (MS) */
  uint8  tcf;               /* Bits 0-3: traffic class (LS), bits 4-7: flow label (MS) */
  uint16 flow;              /* 16-bit flow label (LS) */
  uint8  len[2];            /* 16-bit Payload length */
  uint8  proto;             /*  8-bit Next header (same as IPv4 protocol field) */
  uint8  ttl;               /*  8-bit Hop limit (like IPv4 TTL field) */
  uip_ip6addr_t srcipaddr;  /* 128-bit Source address */
  uip_ip6addr_t destipaddr; /* 128-bit Destination address */

#else /* CONFIG_NET_IPv6 */

  /* IPv4 IP header */

  uint8  vhl;              /*  8-bit Version (4) and header length (5 or 6) */
  uint8  tos;              /*  8-bit Type of service (e.g., 6=TCP) */
  uint8  len[2];           /* 16-bit Total length */
  uint8  ipid[2];          /* 16-bit Identification */
  uint8  ipoffset[2];      /* 16-bit IP flags + fragment offset */
  uint8  ttl;              /*  8-bit Time to Live */
  uint8  proto;            /*  8-bit Protocol */
  uint16 ipchksum;         /* 16-bit Header checksum */
  uint16 srcipaddr[2];     /* 32-bit Source IP address */
  uint16 destipaddr[2];    /* 32-bit Destination IP address */

#endif /* CONFIG_NET_IPv6 */

  /* ICMP (echo) header */

  uint8  type;
  uint8  icode;
  uint16 icmpchksum;

#ifndef CONFIG_NET_IPv6

  uint16 id;
  uint16 seqno;

#else /* !CONFIG_NET_IPv6 */

  uint8 flags;
  uint8 reserved1;
  uint8 reserved2;
  uint8 reserved3;
  uint8 icmp6data[16];
  uint8 options[1];

#endif /* !CONFIG_NET_IPv6 */
};

/* The structure holding the ICMP statistics that are gathered if
 * CONFIG_NET_STATISTICS is defined.
 */

#ifdef CONFIG_NET_STATISTICS
struct uip_icmp_stats_s
{
  uip_stats_t drop;       /* Number of dropped ICMP packets */
  uip_stats_t recv;       /* Number of received ICMP packets */
  uip_stats_t sent;       /* Number of sent ICMP packets */
  uip_stats_t typeerr;    /* Number of ICMP packets with a wrong type */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __NET_UIP_UIP_ICMP_H */
