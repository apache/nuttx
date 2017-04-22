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

/* By default, a 2-byte Rime address is used for the IEEE802.15.4 MAC
 * device's link  layer address.  If CONFIG_NET_6LOWPAN_RIMEADDR_EXTENDED
 * is selected, then an 8-byte Rime address will be used.
 */

#ifdef CONFIG_NET_6LOWPAN_RIMEADDR_EXTENDED
#  define NET_6LOWPAN_RIMEADDR_SIZE 8
#else
#  define NET_6LOWPAN_RIMEADDR_SIZE 2
#endif

/* Frame format definitions *************************************************/
/* These are some definitions of element values used in the FCF.  See the
 * IEEE802.15.4 spec for details.
 */

#define FRAME802154_FRAMETYPE_SHIFT      (0)  /* Bits 0-2: Frame type */
#define FRAME802154_FRAMETYPE_MASK       (7 << FRAME802154_FRAMETYPE_SHIFT)
#define FRAME802154_SECENABLED_SHIFT     (3)  /* Bit 3: Security enabled */
#define FRAME802154_FRAMEPENDING_SHIFT   (4)  /* Bit 4: Frame pending */
#define FRAME802154_ACKREQUEST_SHIFT     (5)  /* Bit 5: ACK request */
#define FRAME802154_PANIDCOMP_SHIFT      (6)  /* Bit 6: PANID compression */
                                              /* Bits 7-9: Reserved */
#define FRAME802154_DSTADDR_SHIFT        (2)  /* Bits 10-11: Dest address mode */
#define FRAME802154_DSTADDR_MASK         (3 << FRAME802154_DSTADDR_SHIFT)
#define FRAME802154_VERSION_SHIFT        (4)  /* Bit 12-13: Frame version */
#define FRAME802154_VERSION_MASK         (3 << FRAME802154_VERSION_SHIFT)
#define FRAME802154_SRCADDR_SHIFT        (6)  /* Bits 14-15: Source address mode */
#define FRAME802154_SRCADDR_MASK         (3 << FRAME802154_SRCADDR_SHIFT)

/* Unshifted values for use in struct frame802154_fcf_s */

#define FRAME802154_BEACONFRAME          (0)
#define FRAME802154_DATAFRAME            (1)
#define FRAME802154_ACKFRAME             (2)
#define FRAME802154_CMDFRAME             (3)

#define FRAME802154_BEACONREQ            (7)

#define FRAME802154_IEEERESERVED         (0)
#define FRAME802154_NOADDR               (0)  /* Only valid for ACK or Beacon frames */
#define FRAME802154_SHORTADDRMODE        (2)
#define FRAME802154_LONGADDRMODE         (3)

#define FRAME802154_NOBEACONS            0x0f

#define FRAME802154_BROADCASTADDR        0xffff
#define FRAME802154_BROADCASTPANDID      0xffff

#define FRAME802154_IEEE802154_2003      (0)
#define FRAME802154_IEEE802154_2006      (1)

#define FRAME802154_SECURITY_LEVEL_NONE  (0)
#define FRAME802154_SECURITY_LEVEL_128   (3)

/* This maximum size of an IEEE802.15.4 frame.  Certain, non-standard
 * devices may exceed this value, however.
 */

#define SIXLOWPAN_MAC_STDFRAME 127

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Rime address representation */

struct rimeaddr_s
{
  uint8_t u8[NET_6LOWPAN_RIMEADDR_SIZE];
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* CONFIG_NET_6LOWPAN */
#endif /* __INCLUDE_NUTTX_NET_IEEE802154_H */
