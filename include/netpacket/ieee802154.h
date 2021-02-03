/****************************************************************************
 * include/netpacket/ieee802154.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
