/************************************************************************************
 * arch/hc/src/mc9s12ne64/mc9s12ne64_emacv1.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_EMACV1_H
#define __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_EMACV1_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_EMAC_NETCTL_OFFSET       0x0000 /* Network Control (8-bit) */
#define HCS12_EMAC_RXCTS_OFFSET        0x0003 /* Receive Control and Status (8-bit) */
#define HCS12_EMAC_TXCTS_OFFSET        0x0004 /* Transmit Control and Status (8-bit) */
#define HCS12_EMAC_ETCTL_OFFSET        0x0005 /* Ethertype Control (8-bit) */
#define HCS12_EMAC_ETYPE_OFFSET        0x0006 /* Programmable Ethertype (16-bit) */
#define HCS12_EMAC_PTIME_OFFSET        0x0008 /* PAUSE Timer Value and Counter (16-bit) */
#define HCS12_EMAC_IEVENT_OFFSET       0x000a /* Interrupt Event (16-bit) */
#define HCS12_EMAC_IMASK_OFFSET        0x000c /* Interrupt Mask (16-bit) */
#define HCS12_EMAC_SWRST_OFFSET        0x000e /* Software Reset (8-bit) */
#define HCS12_EMAC_MPADR_OFFSET        0x0010 /* MII Management PHY Address (8-bit) */
#define HCS12_EMAC_MRADR_OFFSET        0x0011 /* MII Management Register Address (8-bit) */
#define HCS12_EMAC_MWDATA_OFFSET       0x0012 /* MII Management Write Data (16-bit) */
#define HCS12_EMAC_MRDATA_OFFSET       0x0014 /* MII Management Read Data (16-bit) */
#define HCS12_EMAC_MCMST_OFFSET        0x0016 /* MII Management Command and Status (8-bit) */
#define HCS12_EMAC_BUFCFG_OFFSET       0x0018 /* Ethernet Buffer Configuration (16-bit) */
#define HCS12_EMAC_RXAEFP_OFFSET       0x001a /* Receive A End-of-Frame Pointer (16-bit) */
#define HCS12_EMAC_RXBEFP_OFFSET       0x001c /* Receive B End-of-Frame Pointer (16-bit) */
#define HCS12_EMAC_TXEFP_OFFSET        0x001e /* Transmit End-of-Frame Pointer (16-bit) */
#define HCS12_EMAC_MCHASH48_OFFSET     0x0020 /* Multicast Hash Table 48-63 (16-bit) */
#define HCS12_EMAC_MCHASH32_OFFSET     0x0022 /* Multicast Hash Table 32-47 (16-bit) */
#define HCS12_EMAC_MCHASH16_OFFSET     0x0024 /* Multicast Hash Table 16-31 (16-bit) */
#define HCS12_EMAC_MCHASH0_OFFSET      0x0026 /* Multicast Hash Table 0:15 (16-bit) */
#define HCS12_EMAC_MACAD32_OFFSET      0x0028 /* MAC Unicast AAddress 32-47 (16-bit) */
#define HCS12_EMAC_MACAD16_OFFSET      0x002a /* MAC Unicast AAddress 16-31 (16-bit) */
#define HCS12_EMAC_MACAD0_OFFSET       0x002c /* MAC Unicast AAddress 0-15 (16-bit) */
#define HCS12_EMAC_EMISC _OFFSET       0x002e /* Miscellaneous (16-bit) */

/* Register Addresses ***************************************************************/

#define HCS12_EMAC_NETCTL              (HCS12_EMAC_BASE+HCS12_EMAC_NETCTL_OFFSET)
#define HCS12_EMAC_RXCTS               (HCS12_EMAC_BASE+HCS12_EMAC_RXCTS_OFFSET)
#define HCS12_EMAC_TXCTS               (HCS12_EMAC_BASE+HCS12_EMAC_TXCTS_OFFSET)
#define HCS12_EMAC_ETCTL               (HCS12_EMAC_BASE+HCS12_EMAC_ETCTL_OFFSET)
#define HCS12_EMAC_ETYPE               (HCS12_EMAC_BASE+HCS12_EMAC_ETYPE_OFFSET)
#define HCS12_EMAC_PTIME               (HCS12_EMAC_BASE+HCS12_EMAC_PTIME_OFFSET)
#define HCS12_EMAC_IEVENT              (HCS12_EMAC_BASE+HCS12_EMAC_IEVENT_OFFSET)
#define HCS12_EMAC_IMASK               (HCS12_EMAC_BASE+HCS12_EMAC_IMASK_OFFSET)
#define HCS12_EMAC_SWRST               (HCS12_EMAC_BASE+HCS12_EMAC_SWRST_OFFSET)
#define HCS12_EMAC_MPADR               (HCS12_EMAC_BASE+HCS12_EMAC_MPADR_OFFSET)
#define HCS12_EMAC_MRADR               (HCS12_EMAC_BASE+HCS12_EMAC_MRADR_OFFSET)
#define HCS12_EMAC_MWDATA              (HCS12_EMAC_BASE+HCS12_EMAC_MWDATA_OFFSET)
#define HCS12_EMAC_MRDATA              (HCS12_EMAC_BASE+HCS12_EMAC_MRDATA_OFFSET)
#define HCS12_EMAC_MCMST               (HCS12_EMAC_BASE+HCS12_EMAC_MCMST_OFFSET)
#define HCS12_EMAC_BUFCFG              (HCS12_EMAC_BASE+HCS12_EMAC_BUFCFG_OFFSET)
#define HCS12_EMAC_RXAEFP              (HCS12_EMAC_BASE+HCS12_EMAC_RXAEFP_OFFSET)
#define HCS12_EMAC_RXBEFP              (HCS12_EMAC_BASE+HCS12_EMAC_RXBEFP_OFFSET)
#define HCS12_EMAC_TXEFP               (HCS12_EMAC_BASE+HCS12_EMAC_TXEFP_OFFSET)
#define HCS12_EMAC_MCHASH48            (HCS12_EMAC_BASE+HCS12_EMAC_MCHASH48_OFFSET)
#define HCS12_EMAC_MCHASH32            (HCS12_EMAC_BASE+HCS12_EMAC_MCHASH32_OFFSET)
#define HCS12_EMAC_MCHASH16            (HCS12_EMAC_BASE+HCS12_EMAC_MCHASH16_OFFSET)
#define HCS12_EMAC_MCHASH0             (HCS12_EMAC_BASE+HCS12_EMAC_MCHASH0_OFFSET)
#define HCS12_EMAC_MACAD32             (HCS12_EMAC_BASE+HCS12_EMAC_MACAD32_OFFSET)
#define HCS12_EMAC_MACAD16             (HCS12_EMAC_BASE+HCS12_EMAC_MACAD16_OFFSET)
#define HCS12_EMAC_MACAD0              (HCS12_EMAC_BASE+HCS12_EMAC_MACAD0_OFFSET)
#define HCS12_EMAC_EMISC               (HCS12_EMAC_BASE+HCS12_EMAC_EMISC_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* Network Control (8-bit) */
#define EMAC_NETCTL_

/* Receive Control and Status (8-bit) */
#define EMAC_RXCTS_

/* Transmit Control and Status (8-bit) */
#define EMAC_TXCTS_

/* Ethertype Control (8-bit) */
#define EMAC_ETCTL_

/* Programmable Ethertype (16-bit) -- 16-bit Ethernet type data */
/* PAUSE Timer Value and Counter (16-bit) -- 16-bit PAUSER timer value */

/* Interrupt Event (16-bit) */
#define EMAC_IEVENT_

/* Interrupt Mask (16-bit) */
#define EMAC_IMASK_

/* Software Reset (8-bit) */

#define EMAC_SWRST_MACRST              (1<< 7)  /* Bit 7: EMAC is reset */

/* MII Management PHY Address (8-bit) */

#define EMAC_MPADR_MASK                (0x1f)

/* MII Management Register Address (8-bit) */

#define EMAC_MRADR_MASK                (0x1f)

/* MII Management Write Data (16-bit) -- 16-bit write data */
/* MII Management Read Data (16-bit) -- 16-bit read data */

/* MII Management Command and Status (8-bit) */
#define EMAC_MCMST_

/* Ethernet Buffer Configuration (16-bit) */
#define EMAC_BUFCFG_

/* Receive A End-of-Frame Pointer (16-bit) */

#define EMAC_RXAEFP_MASK               (0x07ff)

/* Receive B End-of-Frame Pointer (16-bit) */

#define EMAC_RXBEFP_MASK               (0x07ff)

/* Transmit End-of-Frame Pointer (16-bit) */

#define EMAC_TXEFP_MASK                (0x07ff)

/* Multicast Hash Table 48-63 (16-bit) -- 16-bits of MAC address */
/* Multicast Hash Table 32-47 (16-bit) -- 16-bits of MAC address */
/* Multicast Hash Table 16-31 (16-bit) -- 16-bits of MAC address */
/* Multicast Hash Table  0:15 (16-bit) -- 16-bits of MAC address */

/* MAC Unicast Address 32-47 (16-bit) -- 16-bits of address */
/* MAC Unicast AAddress 16-31 (16-bit) -- 16-bits of address */
/* MAC Unicast AAddress 0-15 (16-bit) -- 16-bits of address */

/* Miscellaneous (16-bit) */
#define EMAC_EMISC_

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_EMACV1_H */
