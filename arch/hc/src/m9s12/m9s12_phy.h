/****************************************************************************
 * arch/hc/src/m9s12/m9s12_phy.h
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

#ifndef __ARCH_HC_SRC_M9S12_M9S12_PHY_H
#define __ARCH_HC_SRC_M9S12_M9S12_PHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define HCS12_PHY_EPHYCTL0_OFFSET      0x0000 /* Ethernet Physical Transceiver Control Register 0 */
#define HCS12_PHY_EPHYCTL1_OFFSET      0x0001 /* Ethernet Physical Transceiver Control Register 1 */
#define HCS12_PHY_EPHYSR_OFFSET        0x0002 /* Ethernet Physical Transceiver Status Register */

/* Register Addresses *******************************************************/

#define HCS12_PHY_EPHYCTL0             (HCS12_EPHY_BASE+HCS12_PHY_EPHYCTL0_OFFSET)
#define HCS12_PHY_EPHYCTL1             (HCS12_EPHY_BASE+HCS12_PHY_EPHYCTL1_OFFSET)
#define HCS12_PHY_EPHYSR               (HCS12_EPHY_BASE+HCS12_PHY_EPHYSR_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Ethernet Physical Transceiver Control Register 0 */

#define PHY_EPHYCTL0_EPHYIEN           (1 << 0)  /* Bit 0: EPHY Interrupt Enable */
#define PHY_EPHYCTL0_EPHYWAI           (1 << 2)  /* Bit 2: EPHY Module Stops While in Wait */
#define PHY_EPHYCTL0_LEDEN             (1 << 3)  /* Bit 3: LED Drive Enable */
#define PHY_EPHYCTL0_DIS10             (1 << 4)  /* Bit 4: Disable 10BASE-T PLL */
#define PHY_EPHYCTL0_DIS100            (1 << 5)  /* Bit 5: Disable 100 BASE-TX PLL */
#define PHY_EPHYCTL0_ANDIS             (1 << 6)  /* Bit 6: Auto Negotiation Disable */
#define PHY_EPHYCTL0_EPHYEN            (1 << 7)  /* Bit 7: EPHY Enable */

/* Ethernet Physical Transceiver Control Register 1 */

#define PHY_EPHYCTL1_PHYADD_SHIFT      (0)       /* Bits 0-4: EPHY Address for MII Requests */
#define PHY_EPHYCTL1_PHYADD_MASK       (0x1f)

/* Ethernet Physical Transceiver Status Register */

#define PHY_EPHYSR_EPHYI               (1 << 0)  /* Bit 0: EPHY Interrupt Flag */
#define PHY_EPHYSR_10DIS               (1 << 4)  /* Bit 4: EPHY Port 10BASE-T mode status */
#define PHY_EPHYSR_100DIS              (1 << 5)  /* Bit 5: EPHY Port 100BASE-TX mode status */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_HC_SRC_M9S12_M9S12_PHY_H */
