/************************************************************************************
 * arch/hc/src/mc9s12ne64/mc9s12ne64_phyv2.h
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

#ifndef __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_PHYV2_H
#define __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_PHYV2_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define HCS12_PHY_EPHYCTL0_OFFSET      0x0000 /* Ethernet Physical Transceiver Control Register 0 */
#define HCS12_PHY_EPHYCTL1_OFFSET      0x0001 /* Ethernet Physical Transceiver Control Register 1 */
#define HCS12_PHY_EPHYSR_OFFSET        0x0002 /* Ethernet Physical Transceiver Status Register */

/* Register Addresses ***************************************************************/

#define HCS12_PHY_EPHYCTL0             (HCS12_EPHY_BASE+HCS12_PHY_EPHYCTL0_OFFSET)
#define HCS12_PHY_EPHYCTL1             (HCS12_EPHY_BASE+HCS12_PHY_EPHYCTL1_OFFSET)
#define HCS12_PHY_EPHYSR               (HCS12_EPHY_BASE+HCS12_PHY_EPHYSR_OFFSET)

/* Register Bit-Field Definitions ***************************************************/

/* Ethernet Physical Transceiver Control Register 0 */
#define PHY_EPHYCTL0_

/* Ethernet Physical Transceiver Control Register 1 */
#define PHY_EPHYCTL1_

/* Ethernet Physical Transceiver Status Register */
#define PHY_EPHYSR_

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_HC_SRC_MC9S12NE64_MC9S12NE64_PHYV2_H */
