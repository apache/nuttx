/****************************************************************************
 * net/arp/arp.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __NET_ARP_ARP_H
#define __NET_ARP_ARP_H

/* The Address Resolution Protocol ARP is used for mapping between IP
 * addresses and link level addresses such as the Ethernet MAC
 * addresses. ARP uses broadcast queries to ask for the link level
 * address of a known IP address and the host which is configured with
 * the IP address for which the query was meant, will respond with its
 * link level address.
 *
 * Note: This ARP implementation only supports Ethernet.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_DEBUG
#  undef CONFIG_NET_ARP_DUMP
#endif

/* ARP Definitions **********************************************************/

#define ARP_REQUEST    1
#define ARP_REPLY      2

#define ARP_HWTYPE_ETH 1

#define RASIZE         4  /* Size of ROUTER ALERT */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ARP header -- Size 28 bytes */

struct arp_hdr_s
{
  uint16_t ah_hwtype;        /* 16-bit Hardware type (Ethernet=0x001) */
  uint16_t ah_protocol;      /* 16-bit Protocol type (IP=0x0800) */
  uint8_t  ah_hwlen;         /*  8-bit Hardware address size (6) */
  uint8_t  ah_protolen;      /*  8-bit Procotol address size (4) */
  uint16_t ah_opcode;        /* 16-bit Operation */
  uint8_t  ah_shwaddr[6];    /* 48-bit Sender hardware address */
  uint16_t ah_sipaddr[2];    /* 32-bit Sender IP adress */
  uint8_t  ah_dhwaddr[6];    /* 48-bit Target hardware address */
  uint16_t ah_dipaddr[2];    /* 32-bit Target IP address */
};

/* IP header -- Size 20 or 24 bytes */

struct arp_iphdr_s
{
  uint8_t  eh_vhl;           /*  8-bit Version (4) and header length (5 or 6) */
  uint8_t  eh_tos;           /*  8-bit Type of service (e.g., 6=TCP) */
  uint8_t  eh_len[2];        /* 16-bit Total length */
  uint8_t  eh_ipid[2];       /* 16-bit Identification */
  uint8_t  eh_ipoffset[2];   /* 16-bit IP flags + fragment offset */
  uint8_t  eh_ttl;           /*  8-bit Time to Live */
  uint8_t  eh_proto;         /*  8-bit Protocol */
  uint16_t eh_ipchksum;      /* 16-bit Header checksum */
  uint16_t eh_srcipaddr[2];  /* 32-bit Source IP address */
  uint16_t eh_destipaddr[2]; /* 32-bit Destination IP address */
  uint16_t eh_ipoption[2];   /* (optional) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

 #ifdef CONFIG_NET_ARP
/****************************************************************************
 * Name: arp_reset
 *
 * Description:
 *   Re-initialize the ARP table.
 *
 ****************************************************************************/

void arp_reset(void);

/****************************************************************************
 * Function: arp_timer_initialize
 *
 * Description:
 *   Initialized the 10 second timer that is need by the ARP logic in order
 *   to age ARP address associations
 *
 * Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called once at system initialization time
 *
 ****************************************************************************/

void arp_timer_initialize(void);

/****************************************************************************
 * Name: arp_timer
 *
 * Description:
 *   This function performs periodic timer processing in the ARP module
 *   and should be called at regular intervals.  The recommended interval
 *   is 10 seconds between the calls.  It is responsible for flushing old
 *   entries in the ARP table.
 *
 ****************************************************************************/

void arp_timer(void);

/****************************************************************************
 * Name: arp_dump
 *
 * Description:
 *   Dump the contents of an ARP packet to the SYSLOG device
 *
 * Input Parameters:
 *   arp - A reference to the ARP header to be dumped.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ARP_DUMP
void arp_dump(FAR struct arp_hdr_s *arp);
#else
# define arp_dump(arp)
#endif

#else /* CONFIG_NET_ARP */

/* If ARP is disabled, stub out all ARP interfaces */

# define arp_reset()
# define arp_timer_initialize(void)
# define arp_timer()
# define arp_dump(arp)

#endif /* CONFIG_NET_ARP */
#endif /* __NET_ARP_ARP_H */
