/****************************************************************************
 * include/nuttx/wireless/ieee802154/ieee802154.h
 *
 *   Copyright (C) 2014-2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *   Author: Anthony Merlino <anthony@vergeaero.com>
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ieee802154_packet_s
{
  uint8_t len;
  uint8_t data[127];
  uint8_t lqi;
  uint8_t rssi;
};

/* IEEE 802.15.4 Device address
 * The addresses in ieee802154 have several formats:
 * No address                : [none]
 * Short address + PAN id    : PPPP/SSSS
 * Extended address + PAN id : PPPP/LLLLLLLLLLLLLLLL
 */

enum ieee802154_addr_mode_e {
  IEEE802154_ADDRMODE_NONE = 0,
  IEEE802154_ADDRMODE_SHORT = 2,
  IEEE802154_ADDRMODE_EXTENDED
};

struct ieee802154_addr_s
{
  enum ieee802154_addr_mode_e ia_mode;  /* Address mode. Short or Extended */
  uint16_t ia_panid;                    /* PAN identifier, can be IEEE802154_PAN_UNSPEC */
  union
  {
    uint16_t _ia_saddr;                 /* short address */
    uint8_t  _ia_eaddr[8];              /* extended address */
  } ia_addr;

#define ia_saddr ia_addr._ia_saddr
#define ia_eaddr ia_addr._ia_eaddr
};

#define IEEE802154_ADDRSTRLEN 22 /* (2*2+1+8*2, PPPP/EEEEEEEEEEEEEEEE) */

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

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_H*/
