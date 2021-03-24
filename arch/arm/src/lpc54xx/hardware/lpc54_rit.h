/****************************************************************************
 * arch/arm/src/lpc54xx/hardware/lpc54_rit.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_RIT_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_RIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC54_RIT_COMPVAL_OFFSET  0x0000  /* LS 48-bit Compare register */
#define LPC54_RIT_MASK_OFFSET     0x0004  /* LS 48-bit Mask register */
#define LPC54_RIT_CTRL_OFFSET     0x0008  /* Control register */
#define LPC54_RIT_COUNTER_OFFSET  0x000c  /* LS 48-bit counter */
#define LPC54_RIT_COMPVALH_OFFSET 0x0010  /* MS 48-bit Compare register */
#define LPC54_RIT_MASKH_OFFSET    0x0014  /* MS 48-bit Mask register */
#define LPC54_RIT_COUNTERH_OFFSET 0x001c  /* MS 48-bit counter */

/* Register addresses *******************************************************/

#define LPC54_RIT_COMPVAL         (LPC54_RIT_BASE+LPC54_RIT_COMPVAL_OFFSET)
#define LPC54_RIT_MASK            (LPC54_RIT_BASE+LPC54_RIT_MASK_OFFSET)
#define LPC54_RIT_CTRL            (LPC54_RIT_BASE+LPC54_RIT_CTRL_OFFSET)
#define LPC54_RIT_COUNTER         (LPC54_RIT_BASE+LPC54_RIT_COUNTER_OFFSET)
#define LPC54_RIT_COMPVALH        (LPC54_RIT_BASE+LPC54_RIT_COMPVALH_OFFSET)
#define LPC54_RIT_MASKH           (LPC54_RIT_BASE+LPC54_RIT_MASKH_OFFSET)
#define LPC54_RIT_COUNTERH        (LPC54_RIT_BASE+LPC54_RIT_COUNTERH_OFFSET)

/* Register bit definitions *************************************************/

/* LS Compare register (Bits 0-31: value compared to the counter) */

/* MS Compare register (Bits 21-47: value compared to the counter) */

/* LS Mask register (Bits 0-31: 32-bit mask value) */

/* MS Mask register (Bits 32-47: 16-bit mask value) */

/* Control register */

#define RIT_CTRL_INT             (1 << 0)  /* Bit 0: Interrupt flag */
#define RIT_CTRL_ENCLR           (1 << 1)  /* Bit 1: Timer enable clear */
#define RIT_CTRL_ENBR            (1 << 2)  /* Bit 2: Timer enable for debug */
#define RIT_CTRL_EN              (1 << 3)  /* Bit 3: Timer enable */
                                           /* Bits 4-31: Reserved */

/* LS 48-bit counter (Bits 0-31: 48-bit up counter) */

/* MS 48-bit counter (Bits 32-47: 48-bit up counter) */

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_RIT_H */
