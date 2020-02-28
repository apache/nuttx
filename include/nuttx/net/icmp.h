/****************************************************************************
 * include/nuttx/net/icmp.h
 * Header file for the NuttX ICMP stack.
 *
 *   Copyright (C) 2007-2009, 2012, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_NET_ICMP_H
#define __INCLUDE_NUTTX_NET_ICMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/ip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ICMP definitions */

/* ICMP Message Types */

#define ICMP_ECHO_REPLY              0    /* RFC 792 */
#define ICMP_DEST_UNREACHABLE        3    /* RFC 792 */
#define ICMP_SRC_QUENCH              4    /* RFC 792 */
#define ICMP_REDIRECT                5    /* RFC 792 */
#define ICMP_ALT_HOST_ADDRESS        6
#define ICMP_ECHO_REQUEST            8    /* RFC 792 */
#define ICMP_ROUTER_ADVERTISEMENT    9    /* RFC 1256 */
#define ICMP_ROUTER_SOLICITATION     10   /* RFC 1256 */
#define ICMP_TIME_EXCEEDED           11   /* RFC 792 */
#define ICMP_PARAMETER_PROBLEM       12
#define ICMP_TIMESTAMP_REQUEST       13
#define ICMP_TIMESTAMP_REPLY         14
#define ICMP_INFORMATION_REQUEST     15
#define ICMP_INFORMATION_REPLY       16
#define ICMP_ADDRESS_MASK_REQUEST    17
#define ICMP_ADDRESS_MASK_REPLY      18
#define ICMP_TRACEROUTE              30
#define ICMP_CONVERSION_ERROR        31
#define ICMP_MOBILE_HOST_REDIRECT    32
#define ICMP_IPV6_WHEREAREYOU        33
#define ICMP_IPV6_IAMHERE            34
#define ICMP_MOBILE_REGIS_REQUEST    35
#define ICMP_MOBILE_REGIS_REPLY      36
#define ICMP_DOMAIN_NAME_REQUEST     37
#define ICMP_DOMAIN_NAME_REPLY       38
#define ICMP_SKIP_DISCOVERY_PROTO    39
#define ICMP_PHOTURIS_SECURITY_FAIL  40
#define ICMP_EXP_MOBILE_PROTO        41   /* RFC 4065 */

/* Header sizes */

#define ICMP_HDRLEN    8                           /* Size of ICMP header */
#define IPICMP_HDRLEN  (ICMP_HDRLEN + IPv4_HDRLEN) /* Size of IPv4 + ICMP header */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct icmp_hdr_s
{
  /* ICMP header */

  uint8_t  type;            /* Defines the format of the ICMP message */
  uint8_t  icode;           /* Further qualifies the ICMP message */
  uint16_t icmpchksum;      /* Checksum of ICMP header and data */

  /* All ICMP packets have an 8-byte header and variable-sized data section.
   * The first 4 bytes of the header have fixed format, while the last 4 bytes
   * depend on the type/code of that ICMP packet.
   */

  /* ICMP_ECHO_REQUEST and ICMP_ECHO_REPLY data */

  uint16_t id;               /* Used to match requests with replies */
  uint16_t seqno;            /* "  " "" "   " "      " "  " "     " */
};

/* The structure holding the ICMP statistics that are gathered if
 * CONFIG_NET_STATISTICS is defined.
 */

#ifdef CONFIG_NET_STATISTICS
struct icmp_stats_s
{
  net_stats_t drop;       /* Number of dropped ICMP packets */
  net_stats_t recv;       /* Number of received ICMP packets */
  net_stats_t sent;       /* Number of sent ICMP packets */
  net_stats_t typeerr;    /* Number of ICMP packets with a wrong type */
};
#endif

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

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __INCLUDE_NUTTX_NET_ICMP_H */
