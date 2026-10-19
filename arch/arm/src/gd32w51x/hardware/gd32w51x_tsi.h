/****************************************************************************
 * arch/arm/src/gd32w51x/hardware/gd32w51x_tsi.h
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

#ifndef __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_TSI_H
#define __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_TSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32w51x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The touch sensing interface measures a capacitance by charge transfer:
 * the pins are organized in 3 groups of 4 (GxPy).  In each group, one pin
 * is wired to a sampling capacitor and the others to electrodes; the count
 * of transfer cycles needed to charge the sampling capacitor drops when an
 * electrode is touched.
 */

#define GD32W51X_TSI_NGROUPS          3
#define GD32W51X_TSI_PIN(g, p)        (1 << ((g) * 4 + (p)))

/* Register offsets *********************************************************/

#define GD32W51X_TSI_CTL0_OFFSET      0x0000  /* Control register 0 */
#define GD32W51X_TSI_INTEN_OFFSET     0x0004  /* Interrupt enable */
#define GD32W51X_TSI_INTC_OFFSET      0x0008  /* Interrupt flag clear */
#define GD32W51X_TSI_INTF_OFFSET      0x000c  /* Interrupt flag */
#define GD32W51X_TSI_PHM_OFFSET       0x0010  /* Pin hysteresis mode */
#define GD32W51X_TSI_ASW_OFFSET       0x0018  /* Analog switch */
#define GD32W51X_TSI_SAMPCFG_OFFSET   0x0020  /* Sample pin configuration */
#define GD32W51X_TSI_CHCFG_OFFSET     0x0028  /* Channel pin configuration */
#define GD32W51X_TSI_GCTL_OFFSET      0x0030  /* Group control */
#define GD32W51X_TSI_CTL1_OFFSET      0x0300  /* Control register 1 */

/* Cycle count of a group */

#define GD32W51X_TSI_GCYCN_OFFSET(g)  (0x0034 + ((g) << 2))

/* Register addresses *******************************************************/

#define GD32W51X_TSI_CTL0     (GD32W51X_TSI_BASE + GD32W51X_TSI_CTL0_OFFSET)
#define GD32W51X_TSI_INTEN    (GD32W51X_TSI_BASE + GD32W51X_TSI_INTEN_OFFSET)
#define GD32W51X_TSI_INTC     (GD32W51X_TSI_BASE + GD32W51X_TSI_INTC_OFFSET)
#define GD32W51X_TSI_INTF     (GD32W51X_TSI_BASE + GD32W51X_TSI_INTF_OFFSET)
#define GD32W51X_TSI_PHM      (GD32W51X_TSI_BASE + GD32W51X_TSI_PHM_OFFSET)
#define GD32W51X_TSI_ASW      (GD32W51X_TSI_BASE + GD32W51X_TSI_ASW_OFFSET)
#define GD32W51X_TSI_SAMPCFG \
  (GD32W51X_TSI_BASE + GD32W51X_TSI_SAMPCFG_OFFSET)
#define GD32W51X_TSI_CHCFG    (GD32W51X_TSI_BASE + GD32W51X_TSI_CHCFG_OFFSET)
#define GD32W51X_TSI_GCTL     (GD32W51X_TSI_BASE + GD32W51X_TSI_GCTL_OFFSET)
#define GD32W51X_TSI_GCYCN(g) \
  (GD32W51X_TSI_BASE + GD32W51X_TSI_GCYCN_OFFSET(g))
#define GD32W51X_TSI_CTL1     (GD32W51X_TSI_BASE + GD32W51X_TSI_CTL1_OFFSET)

/* Register bit definitions *************************************************/

/* Control register 0 */

#define TSI_CTL0_TSIEN                (1 << 0)  /* TSI enable */
#define TSI_CTL0_TSIS                 (1 << 1)  /* Start a sequence */
#define TSI_CTL0_TRGMOD               (1 << 2)  /* Hardware trigger mode */
#define TSI_CTL0_EGSEL                (1 << 3)  /* Rising edge trigger */
#define TSI_CTL0_PINMOD               (1 << 4)  /* Floating pins when idle */
#define TSI_CTL0_MCN_SHIFT            5         /* Max cycles 2^(8+n)-1 */
#define TSI_CTL0_MCN_MASK             (7 << TSI_CTL0_MCN_SHIFT)
#  define TSI_CTL0_MCN(n)             ((n) << TSI_CTL0_MCN_SHIFT)
#define TSI_CTL0_CTCDIV_SHIFT         12        /* CTCLK divider, bits 2:0 */
#define TSI_CTL0_CTCDIV_MASK          (7 << TSI_CTL0_CTCDIV_SHIFT)
#define TSI_CTL0_ECDIV                (1 << 15) /* ECCLK divider, bit 0 */
#define TSI_CTL0_ECEN                 (1 << 16) /* Extend charge enable */
#define TSI_CTL0_ECDT_SHIFT           17        /* Extend charge duration */
#define TSI_CTL0_ECDT_MASK            (0x7f << TSI_CTL0_ECDT_SHIFT)
#define TSI_CTL0_CTDT_SHIFT           24        /* Transfer duration - 1 */
#define TSI_CTL0_CTDT_MASK            (15 << TSI_CTL0_CTDT_SHIFT)
#  define TSI_CTL0_CTDT(n)            (((n) - 1) << TSI_CTL0_CTDT_SHIFT)
#define TSI_CTL0_CDT_SHIFT            28        /* Charge duration - 1 */
#define TSI_CTL0_CDT_MASK             (15 << TSI_CTL0_CDT_SHIFT)
#  define TSI_CTL0_CDT(n)             (((n) - 1) << TSI_CTL0_CDT_SHIFT)

/* Interrupt enable, flag clear and flag registers */

#define TSI_INT_CTCF                  (1 << 0)  /* Charge transfer complete */
#define TSI_INT_MNERR                 (1 << 1)  /* Max cycle number error */

/* Group control register */

#define TSI_GCTL_GE(g)                (1 << (g))        /* Group enable */
#define TSI_GCTL_GC(g)                (1 << ((g) + 16)) /* Group complete */

/* Group cycle number registers */

#define TSI_GCYCN_MASK                0x3fff

/* Control register 1 */

#define TSI_CTL1_CTCDIV               (1 << 24) /* CTCLK divider, bit 3 */
#define TSI_CTL1_ECDIV_SHIFT          28        /* ECCLK divider, bits 2:1 */
#define TSI_CTL1_ECDIV_MASK           (3 << TSI_CTL1_ECDIV_SHIFT)

#endif /* __ARCH_ARM_SRC_GD32W51X_HARDWARE_GD32W51X_TSI_H */
