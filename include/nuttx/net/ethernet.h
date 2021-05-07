/****************************************************************************
 * include/nuttx/net/ethernet.h
 * Macros and definitions for the Ethernet link layer.
 *
 *   Copyright (C) 2007, 2009-2012, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from uIP with has a similar BSD-styple license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
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

#ifndef __INCLUDE_NUTTX_NET_ETHERNET_H
#define __INCLUDE_NUTTX_NET_ETHERNET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <net/ethernet.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Recognized values of the type bytes in the Ethernet header */

#define ETHTYPE_ARP      0x0806 /* Address resolution protocol */
#define ETHTYPE_IP       0x0800 /* IP protocol */
#define ETHTYPE_IP6      0x86dd /* IP protocol version 6 */

/* Tag protocol identifier (TPID) of 0x8100 identifies the frame as an
 * IEEE 802.1Q-tagged frame.  This field is located at the same position as
 * the Ethernet type field in untagged frames and is thus used to
 * distinguish the frame from untagged frames.
 */

#define TPID_8021QVLAN   0x8100

/* These are some of the types associated with QVLAN tagged
 * Ethernet packets.
 */

#define ETHTYPE_AVBTP    0x22f0 /* Audio/Video bridging type */

/* Size of the Ethernet headers */

#define ETH_HDRLEN       14     /* Header size: 2*6 + 2 */
#define ETH_8021Q_HDRLEN 18     /* Header size: 2*6 + 4 + 2 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* The Ethernet header -- 14 bytes. The first two fields are type 'struct
 * ether_addr but are represented as a simple byte array here because
 * some compilers refuse to pack 6 byte structures.
 */

struct eth_hdr_s
{
  uint8_t  dest[6]; /* Ethernet destination address (6 bytes) */
  uint8_t  src[6];  /* Ethernet source address (6 bytes) */
  uint16_t type;    /* Type code (2 bytes) */
};

/* IEEE 802.1Q adds a 32-bit field between the source MAC address and the
 * type fields of the original Ethernet header.  Two bytes are used for the
 * tag protocol identifier (TPID), the other two bytes for tag control
 * information  TCI).  The TCI field is further divided into PCP, DEI, and
 * VID.
 */

struct eth_8021qhdr_s
{
  uint8_t  dest[6]; /* Ethernet destination address (6 bytes) */
  uint8_t  src[6];  /* Ethernet source address (6 bytes) */
  uint16_t tpid;    /* TCI: Tag protocol identifier (2 bytes) */
  uint16_t tci;     /* TCI: Tag control information: PCP, DEI, VID (2 bytes) */
  uint16_t type;    /* Type code (2 bytes) */
};

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

#if defined(CONFIG_NET_ICMPv6_AUTOCONF) || defined(CONFIG_NET_ICMPv6_ROUTER)
EXTERN const struct ether_addr g_ipv6_ethallnodes;    /* All link local nodes */
EXTERN const struct ether_addr g_ipv6_ethallrouters;  /* All link local routers */
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_ETHERNET_H */
