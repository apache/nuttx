/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_rit.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_RIT_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_RIT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC43_RIT_COMPVAL_OFFSET 0x0000  /* Compare register */
#define LPC43_RIT_MASK_OFFSET    0x0004  /* Mask register */
#define LPC43_RIT_CTRL_OFFSET    0x0008  /* Control register */
#define LPC43_RIT_COUNTER_OFFSET 0x000c  /* 32-bit counter */

/* Register addresses *******************************************************/

#define LPC43_RIT_COMPVAL        (LPC43_RIT_BASE+LPC43_RIT_COMPVAL_OFFSET)
#define LPC43_RIT_MASK           (LPC43_RIT_BASE+LPC43_RIT_MASK_OFFSET)
#define LPC43_RIT_CTRL           (LPC43_RIT_BASE+LPC43_RIT_CTRL_OFFSET)
#define LPC43_RIT_COUNTER        (LPC43_RIT_BASE+LPC43_RIT_COUNTER_OFFSET)

/* Register bit definitions *************************************************/

/* Compare register (Bits 0-31: value compared to the counter) */

/* Mask register (Bits 0-31: 32-bit mask value) */

/* Control register */

#define RIT_CTRL_INT             (1 << 0)  /* Bit 0: Interrupt flag */
#define RIT_CTRL_ENCLR           (1 << 1)  /* Bit 1: Timer enable clear */
#define RIT_CTRL_ENBR            (1 << 2)  /* Bit 2: Timer enable for debug */
#define RIT_CTRL_EN              (1 << 3)  /* Bit 3: Timer enable */
                                           /* Bits 4-31: Reserved */

/* 32-bit counter (Bits 0-31: 32-bit up counter) */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_RIT_H */
