/****************************************************************************
 * include/nuttx/net/udp.h
 * Header file for the uIP UDP stack.
 *
 * The uIP UDP stack header file contains definitions for a number
 * of C macros that are used by uIP programs as well as internal uIP
 * structures, UDP header structures and function declarations.
 *
 *   Copyright (C) 2007, 2009, 2012, 2014 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_NET_UDP_H
#define __INCLUDE_NUTTX_NET_UDP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/net/netconfig.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Header sizes */

#define UIP_UDPH_LEN    8     /* Size of UDP header */
#define UIP_IPUDPH_LEN (UIP_UDPH_LEN + UIP_IPH_LEN)    /* Size of IP + UDP header */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Representation of a uIP UDP connection */

struct net_driver_s;      /* Forward reference */
struct devif_callback_s;    /* Forward reference */
struct udp_conn_s
{
  dq_entry_t node;        /* Supports a doubly linked list */
  net_ipaddr_t ripaddr;   /* The IP address of the remote peer */
  uint16_t lport;         /* The local port number in network byte order */
  uint16_t rport;         /* The remote port number in network byte order */
  uint8_t  ttl;           /* Default time-to-live */
  uint8_t  crefs;         /* Reference counts on this instance */

  /* Defines the list of UDP callbacks */

  struct devif_callback_s *list;
};

/* The UDP and IP headers */

struct udp_iphdr_s
{
#ifdef CONFIG_NET_IPv6

  /* IPv6 Ip header */

  uint8_t  vtc;             /* Bits 0-3: version, bits 4-7: traffic class (MS) */
  uint8_t  tcf;             /* Bits 0-3: traffic class (LS), 4-bits: flow label (MS) */
  uint16_t flow;            /* 16-bit flow label (LS) */
  uint8_t  len[2];          /* 16-bit Payload length */
  uint8_t  proto;           /*  8-bit Next header (same as IPv4 protocol field) */
  uint8_t  ttl;             /*  8-bit Hop limit (like IPv4 TTL field) */
  net_ip6addr_t srcipaddr;  /* 128-bit Source address */
  net_ip6addr_t destipaddr; /* 128-bit Destination address */

#else /* CONFIG_NET_IPv6 */

  /* IPv4 header */

  uint8_t  vhl;             /*  8-bit Version (4) and header length (5 or 6) */
  uint8_t  tos;             /*  8-bit Type of service (e.g., 6=TCP) */
  uint8_t  len[2];          /* 16-bit Total length */
  uint8_t  ipid[2];         /* 16-bit Identification */
  uint8_t  ipoffset[2];     /* 16-bit IP flags + fragment offset */
  uint8_t  ttl;             /*  8-bit Time to Live */
  uint8_t  proto;           /*  8-bit Protocol */
  uint16_t ipchksum;        /* 16-bit Header checksum */
  uint16_t srcipaddr[2];    /* 32-bit Source IP address */
  uint16_t destipaddr[2];   /* 32-bit Destination IP address */

#endif /* CONFIG_NET_IPv6 */

  /* UDP header */

  uint16_t srcport;
  uint16_t destport;
  uint16_t udplen;
  uint16_t udpchksum;
};

/* The structure holding the UDP statistics that are gathered if
 * CONFIG_NET_STATISTICS is defined.
 */

#ifdef CONFIG_NET_STATISTICS
struct udp_stats_s
{
  net_stats_t drop;         /* Number of dropped UDP segments */
  net_stats_t recv;         /* Number of recived UDP segments */
  net_stats_t sent;         /* Number of sent UDP segments */
  net_stats_t chkerr;       /* Number of UDP segments with a bad checksum */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* uIP application functions
 *
 * Functions used by an application running of top of uIP. This includes
 * functions for opening and closing connections, sending and receiving
 * data, etc.
 *
 * Find a free connection structure and allocate it for use. This is
 * normally something done by the implementation of the socket() API
 */

FAR struct udp_conn_s *udp_alloc(void);

/* Free a connection structure that is no longer in use. This should
 * be done by the implementation of close()
 */

void udp_free(FAR struct udp_conn_s *conn);

/* Bind a UDP connection to a local address */

#ifdef CONFIG_NET_IPv6
int udp_bind(FAR struct udp_conn_s *conn,
             FAR const struct sockaddr_in6 *addr);
#else
int udp_bind(FAR struct udp_conn_s *conn,
             FAR const struct sockaddr_in *addr);
#endif

/* This function sets up a new UDP connection. The function will
 * automatically allocate an unused local port for the new
 * connection. However, another port can be chosen by using the
 * udp_bind() call, after the udp_connect() function has been
 * called.
 *
 * This function is called as part of the implementation of sendto
 * and recvfrom.
 *
 * addr The address of the remote host.
 */

#ifdef CONFIG_NET_IPv6
int udp_connect(FAR struct udp_conn_s *conn,
                FAR const struct sockaddr_in6 *addr);
#else
int udp_connect(FAR struct udp_conn_s *conn,
                FAR const struct sockaddr_in *addr);
#endif

/* Enable/disable UDP callbacks on a connection */

void udp_enable(FAR struct udp_conn_s *conn);
void udp_disable(FAR struct udp_conn_s *conn);

#endif /* __INCLUDE_NUTTX_NET_UDP_H */
