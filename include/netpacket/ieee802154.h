/****************************************************************************
 * include/netpacket/ieee802154.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NETPACKET_IEEE802154_H
#define __INCLUDE_NETPACKET_IEEE802154_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Well known address */

#define IEEE802154_PANID_BROADCAST 0xffff
#define IEEE802154_ADDR_BROADCAST  0xffff
#define IEEE802154_ADDR_UNDEF      0xfffe

/* struct sockaddr_ieee802154_s union selectors */

#define s_saddr                    s_u.saddr
#define s_eaddr                    s_u.eaddr

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* See include/uttx/wireless/ieee802154/ieee802154_mac.h for address
 * definitions.  Particularly, enum ieee802154_addrmode_e, the address type
 *
 * See also the definitions above for portable union selectors.
 */

struct ieee802154_saddr_s
{
  uint8_t s_mode;                        /* Address mode (see enum
                                          * ieee802154_addrmode_e) */
  uint8_t s_panid[IEEE802154_PANIDSIZE]; /* PAN identifier */
  union
  {
    uint8_t saddr[IEEE802154_SADDRSIZE]; /* Short address */
    uint8_t eaddr[IEEE802154_EADDRSIZE]; /* Extended address */
  } s_u;
};

/* Socket address used with:
 *
 * bind()    - Associates local address with socket
 * connect() - Associates a remote address with the socket (for send())
 * sendto()  - Send to specified remote address
 * recvfrom()- Receive from indicated remote address.
 */

struct sockaddr_ieee802154_s
{
  sa_family_t sa_family;                 /* AF_IEEE802154 */
  struct ieee802154_saddr_s sa_addr;     /* Radio address */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NETPACKET_IEEE802154_H */
