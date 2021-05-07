/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_rit.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_RIT_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_RIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC17_40_RIT_COMPVAL_OFFSET 0x0000  /* Compare register */
#define LPC17_40_RIT_MASK_OFFSET    0x0004  /* Mask register */
#define LPC17_40_RIT_CTRL_OFFSET    0x0008  /* Control register */
#define LPC17_40_RIT_COUNTER_OFFSET 0x000c  /* 32-bit counter */

/* Register addresses *******************************************************/

#define LPC17_40_RIT_COMPVAL        (LPC17_40_RIT_BASE+LPC17_40_RIT_COMPVAL_OFFSET)
#define LPC17_40_RIT_MASK           (LPC17_40_RIT_BASE+LPC17_40_RIT_MASK_OFFSET)
#define LPC17_40_RIT_CTRL           (LPC17_40_RIT_BASE+LPC17_40_RIT_CTRL_OFFSET)
#define LPC17_40_RIT_COUNTER        (LPC17_40_RIT_BASE+LPC17_40_RIT_COUNTER_OFFSET)

/* Register bit definitions *************************************************/

/* Compare register (Bits 0-31: value compared to the counter) */

/* Mask register (Bits 0-31: 32-bit mask value) */

/* Control register */

#define RIT_CTRL_INT                (1 << 0)  /* Bit 0: Interrupt flag */
#define RIT_CTRL_ENCLR              (1 << 1)  /* Bit 1: Timer enable clear */
#define RIT_CTRL_ENBR               (1 << 2)  /* Bit 2: Timer enable for debug */
#define RIT_CTRL_EN                 (1 << 3)  /* Bit 3: Timer enable */
                                              /* Bits 4-31: Reserved */

/* 32-bit counter (Bits 0-31: 32-bit up counter) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_RIT_H */
