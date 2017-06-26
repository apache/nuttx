/****************************************************************************
 * include/nuttx/net/ieee802154.h
 *
 *   Copyright (C) 2017, Gregory Nutt, all rights reserved
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives from Contiki:
 *
 *   Copyright (c) 2008, Swedish Institute of Computer Science.
 *   All rights reserved.
 *   Authors: Adam Dunkels <adam@sics.se>
 *            Nicolas Tsiftes <nvt@sics.se>
 *            Niclas Finne <nfi@sics.se>
 *            Mathilde Durvy <mdurvy@cisco.com>
 *            Julien Abeille <jabeille@cisco.com>
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_IEEE802154_H
#define __INCLUDE_NUTTX_NET_IEEE802154_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_NET_6LOWPAN

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* By default, a 2-byte short address is used for the IEEE802.15.4 MAC
 * device's link layer address.  If CONFIG_NET_6LOWPAN_EXTENDEDADDR
 * is selected, then an 8-byte extended address will be used.
 */

#define NET_6LOWPAN_SADDRSIZE  2
#define NET_6LOWPAN_EADDRSIZE  8

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
#  define NET_6LOWPAN_ADDRSIZE NET_6LOWPAN_EADDRSIZE
#else
#  define NET_6LOWPAN_ADDRSIZE NET_6LOWPAN_SADDRSIZE
#endif

/* This maximum size of an IEEE802.15.4 frame.  Certain, non-standard
 * devices may exceed this value, however.
 */

#define SIXLOWPAN_MAC_STDFRAME 127

/* Space for a two byte FCS must be reserved at the end of the frame */

#define SIXLOWPAN_MAC_FCSSIZE  2

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IEEE 802.15.4 address representations */

struct sixlowpan_saddr_s
{
  uint8_t u8[NET_6LOWPAN_SADDRSIZE];
};

struct sixlowpan_eaddr_s
{
  uint8_t u8[NET_6LOWPAN_EADDRSIZE];
};

union sixlowpan_anyaddr_u
{
  struct sixlowpan_saddr_s saddr;
  struct sixlowpan_eaddr_s eaddr;
};

struct sixlowpan_tagaddr_s
{
  bool extended;
  union sixlowpan_anyaddr_u u;
};

/* Represents the configured address size */

struct sixlowpan_addr_s
{
  uint8_t u8[NET_6LOWPAN_ADDRSIZE];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* CONFIG_NET_6LOWPAN */
#endif /* __INCLUDE_NUTTX_NET_IEEE802154_H */
