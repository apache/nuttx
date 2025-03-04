/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/nxxx_port.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_PORT_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_PORT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/nxxx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NXXX_PORT1               0
#define NXXX_PORT2               1
#define NXXX_PORT3               2
#define NXXX_PORT4               3
#define NXXX_PORT5               4
#define NXXX_PORT6               5

/* PORT Register Offsets ****************************************************/
#define NXXX_PORT_GPCLR_OFFSET   0x0010                               /* Global Pin Control Low Register */
#define NXXX_PORT_GPCHR_OFFSET   0x0014                               /* Global Pin Control High Register */
#define NXXX_PORT_PCR0_OFFSET    0x0080                               /* Pin Control Register 0 */
#define NXXX_PORT_PCR_OFFSET(n)  (NXXX_PORT_PCR0_OFFSET + ((n) << 2)) /* Pin Control Register n=0..31 */

/* PORT Register Addresses **************************************************/

#define NXXX_PORT_BASE(n)        (NXXX_PORT0_BASE + ((n) << 12)) /* Port n multiplexing control */
#define NXXX_PORT_PCR_BASE(p,n)  (NXXX_PORT_BASE(p) + NXXX_PORT_PCR_OFFSET(n))
#define NXXX_PORT_GPCLR(p)       (NXXX_PORT_BASE(p) + NXXX_PORT_GPCLR_OFFSET)
#define NXXX_PORT_GPCHR(p)       (NXXX_PORT_BASE(p) + NXXX_PORT_GPCHR_OFFSET)

/* PORT Register Bitfield Definitions ***************************************/

/* Pin Control Register n=0..31 */

#define PORT_PCR_PS                 (1 << 0)  /* Bit 0:  Pull Select */
#  define PORT_PCR_PULLDOWN         (0)       /*         Enable internal pulldown */
#  define PORT_PCR_PULLUP           (1 << 0)  /*         Enable internal pullup */
#define PORT_PCR_PE                 (1 << 1)  /* Bit 1:  Pull Enable */
#define PORT_PCR_PV                 (1 << 2)  /* Bit 2:  Pull Value */
#define PORT_PCR_SRE                (1 << 3)  /* Bit 3:  Slew Rate Enable */
#define PORT_PCR_PFE                (1 << 4)  /* Bit 4:  Passive Filter Enable */
#define PORT_PCR_ODE                (1 << 5)  /* Bit 5:  Open Drain Enable */
#define PORT_PCR_DSE                (1 << 6)  /* Bit 6:  Drive Strength Enable */
#define PORT_PCR_MUX_SHIFT          (8)       /* Bits 8-10:  Pin Mux Control */
#define PORT_PCR_MUX_MASK           (15 << PORT_PCR_MUX_SHIFT)
#  define PORT_PCR_MUX_GPIO         (0 << PORT_PCR_MUX_SHIFT)  /* Alternative 0 (GPIO) */
#  define PORT_PCR_MUX_ALT1         (1 << PORT_PCR_MUX_SHIFT)  /* Alternative 1 (chip-specific) */
#  define PORT_PCR_MUX_ALT2         (2 << PORT_PCR_MUX_SHIFT)  /* Alternative 2 (chip-specific) */
#  define PORT_PCR_MUX_ALT3         (3 << PORT_PCR_MUX_SHIFT)  /* Alternative 3 (chip-specific) */
#  define PORT_PCR_MUX_ALT4         (4 << PORT_PCR_MUX_SHIFT)  /* Alternative 4 (chip-specific) */
#  define PORT_PCR_MUX_ALT5         (5 << PORT_PCR_MUX_SHIFT)  /* Alternative 5 (chip-specific) */
#  define PORT_PCR_MUX_ALT6         (6 << PORT_PCR_MUX_SHIFT)  /* Alternative 6 (chip-specific) */
#  define PORT_PCR_MUX_ALT7         (7 << PORT_PCR_MUX_SHIFT)  /* Alternative 7 (chip-specific) */
#  define PORT_PCR_MUX_ALT8         (8 << PORT_PCR_MUX_SHIFT)  /* Alternative 8 (chip-specific) */
#  define PORT_PCR_MUX_ALT9         (9 << PORT_PCR_MUX_SHIFT)  /* Alternative 9 (chip-specific) */
#  define PORT_PCR_MUX_ALT10        (10 << PORT_PCR_MUX_SHIFT) /* Alternative 10 (chip-specific) */
#  define PORT_PCR_MUX_ALT11        (11 << PORT_PCR_MUX_SHIFT) /* Alternative 11 (chip-specific) */
#  define PORT_PCR_MUX_ALT12        (12 << PORT_PCR_MUX_SHIFT) /* Alternative 12 (chip-specific) */
#  define PORT_PCR_MUX_ALT13        (13 << PORT_PCR_MUX_SHIFT) /* Alternative 13 (chip-specific) */

#define PORT_PCR_IBE                (1 << 12) /* Bit 12: Input Buffer Enable */
#define PORT_PCR_INV                (1 << 13) /* Bit 13: Invert Input */
#define PORT_PCR_LK                 (1 << 15) /* Bit 15: Lock Register */

/* Global Pin Control Low Register */

#define PORT_GPCLR_GPWD_SHIFT       (0)       /* Bits 0-15: Global Pin Write Data */
#define PORT_GPCLR_GPWD_MASK        (0xffff << PORT_GPCLR_GPWD_SHIFT)
#  define PORT_GPCLR_GPWD_PIN(n)    ((uint32_t)(n) << PORT_GPCLR_GPWD_SHIFT) /* Pin n=0..15 */

#define PORT_GPCLR_GPWE_SHIFT       (16)      /* Bits 16-31: Global Pin Write Enable */
#define PORT_GPCLR_GPWE_MASK        (0xffff << PORT_GPCLR_GPWE_SHIFT)
#  define PORT_GPCLR_GPWE_PIN(n)    ((uint32_t)(n) << PORT_GPCLR_GPWE_SHIFT) /* Pin n=0..15 */

/* Global Pin Control High Register */

#define PORT_GPCHR_GPWD_SHIFT       (0)       /* Bits 0-15: Global Pin Write Data */
#define PORT_GPCHR_GPWD_MASK        (0xffff << PORT_GPCHR_GPWD_SHIFT)
#  define PORT_GPCHR_GPWD_PIN(n)    ((uint32_t)((n) - 16) << PORT_GPCHR_GPWD_SHIFT) /* Pin n=16..31 */

#define PORT_GPCHR_GPWE_SHIFT       (16)      /* Bits 16-31: Global Pin Write Enable */
#define PORT_GPCHR_GPWE_MASK        (0xffff << PORT_GPCHR_GPWE_SHIFT)
#  define PORT_GPCHR_GPWE_PIN(n)    ((uint32_t)((n) - 16) << PORT_GPCHR_GPWE_SHIFT) /* Pin n=16..31 */

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_NXXX_PORT_H */
