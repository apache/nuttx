/****************************************************************************
 * arch/arm/src/stm32n6/hardware/stm32n6xxx_usbphyc.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_USBPHYC_H
#define __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_USBPHYC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/stm32n6xxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define STM32_USBPHYC_CR_OFFSET      0x0000 /* USB HS PHY control register */
#define STM32_USBPHYC_TRIM1CR_OFFSET 0x0004 /* USB HS PHY trimming 1 */
#define STM32_USBPHYC_TRIM2CR_OFFSET 0x0008 /* USB HS PHY trimming 2 */

/* Register Addresses *******************************************************/

#define STM32_USB1_HS_PHYC_CR        (STM32_USB1_HS_PHYC_BASE + \
                                      STM32_USBPHYC_CR_OFFSET)
#define STM32_USB2_HS_PHYC_CR        (STM32_USB2_HS_PHYC_BASE + \
                                      STM32_USBPHYC_CR_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* USB HS PHY control register (USBPHYC_CR) */

#define USBPHYC_CR_RETENABLEN1       (1 << 0)  /* Bit 0: Retention mode enable */
#define USBPHYC_CR_AUTORSMENB1       (1 << 1)  /* Bit 1: Auto-resume mode enable */
#define USBPHYC_CR_CMN               (1 << 2)  /* Bit 2: Power down of analog
                                                * blocks in Suspend and Sleep */
#define USBPHYC_CR_FSEL_SHIFT        (4)       /* Bits 4-6: Reference clock
                                                * 19.2, 20 or 24 MHz */
#define USBPHYC_CR_FSEL_MASK         (7 << USBPHYC_CR_FSEL_SHIFT)
#  define USBPHYC_CR_FSEL_19_2MHZ    (0 << USBPHYC_CR_FSEL_SHIFT)
#  define USBPHYC_CR_FSEL_20MHZ      (1 << USBPHYC_CR_FSEL_SHIFT)
#  define USBPHYC_CR_FSEL_24MHZ      (2 << USBPHYC_CR_FSEL_SHIFT)
#define USBPHYC_CR_OTGDISABLE0       (1 << 16) /* Bit 16: OTG block disable */
#define USBPHYC_CR_DRVVBUS0          (1 << 17) /* Bit 17: Drive VBUS */
#define USBPHYC_CR_SELOTGDBG         (1 << 31) /* Bit 31: Select HS PHY debug
                                                * port */

#endif /* __ARCH_ARM_SRC_STM32N6_HARDWARE_STM32N6XXX_USBPHYC_H */
