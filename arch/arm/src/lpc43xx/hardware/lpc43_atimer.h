/****************************************************************************
 * arch/arm/src/lpc43xx/hardware/lpc43_atimer.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_ATIMER_H
#define __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_ATIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define LPC43_ATIMER_COUNT_OFFSET    0x0000 /* Downcounter register */
#define LPC43_ATIMER_PRESET_OFFSET   0x0004 /* Preset value register */
#define LPC43_ATIMER_CLREN_OFFSET    0x0fd8 /* Interrupt clear enable register */
#define LPC43_ATIMER_SETEN_OFFSET    0x0fdc /* Interrupt set enable register */
#define LPC43_ATIMER_STATUS_OFFSET   0x0fe0 /* Status register */
#define LPC43_ATIMER_ENABLE_OFFSET   0x0fe4 /* Enable register */
#define LPC43_ATIMER_CLRSTAT_OFFSET  0x0fe8 /* Clear register */
#define LPC43_ATIMER_SETSTAT_OFFSET  0x0fec /* Set register */

/* Register Addresses *******************************************************/

#define LPC43_ATIMER_COUNT           (LPC43_ATIMER_BASE+LPC43_ATIMER_COUNT_OFFSET)
#define LPC43_ATIMER_PRESET          (LPC43_ATIMER_BASE+LPC43_ATIMER_PRESET_OFFSET)
#define LPC43_ATIMER_CLREN           (LPC43_ATIMER_BASE+LPC43_ATIMER_CLREN_OFFSET)
#define LPC43_ATIMER_SETEN           (LPC43_ATIMER_BASE+LPC43_ATIMER_SETEN_OFFSET)
#define LPC43_ATIMER_STATUS          (LPC43_ATIMER_BASE+LPC43_ATIMER_STATUS_OFFSET)
#define LPC43_ATIMER_ENABLE          (LPC43_ATIMER_BASE+LPC43_ATIMER_ENABLE_OFFSET)
#define LPC43_ATIMER_CLRSTAT         (LPC43_ATIMER_BASE+LPC43_ATIMER_CLRSTAT_OFFSET)
#define LPC43_ATIMER_SETSTAT         (LPC43_ATIMER_BASE+LPC43_ATIMER_SETSTAT_OFFSET)

/* Register Bit Definitions *************************************************/

/* Downcounter register */

#define ATIMER_COUNT_MASK            0xffff    /* Bits 0-15: Down counter */
                                               /* Bits 16-31: Reserved */

/* Preset value register */

#define ATIMER_PRESET_MASK           0xffff    /* Bits 0-15: Counter reload value */
                                               /* Bits 16-31: Reserved */

/* Interrupt clear enable register */

#define ATIMER_CLREN                 (1 << 0)  /* Bit 0:  Clear interrupt enable */
                                               /* Bits 1-31: Reserved */

/* Interrupt set enable register */

#define ATIMER_SETEN                 (1 << 0)  /* Bit 0:  Set interrupt enable */
                                               /* Bits 1-31: Reserved */

/* Status register */

#define ATIMER_STATUS                (1 << 0)  /* Bit 0:  Interrupt status */
                                               /* Bits 1-31: Reserved */

/* Enable register */

#define ATIMER_ENABLE                (1 << 0)  /* Bit 0:  Interrupt enable status */
                                               /* Bits 1-31: Reserved */

/* Clear register */

#define ATIMER_CLRSTAT               (1 << 0)  /* Bit 0:  Clear interrupt status */
                                               /* Bits 1-31: Reserved */

/* Set register */

#define ATIMER_SETSTAT               (1 << 0)  /* Bit 0:  Set interrupt status */
                                               /* Bits 1-31: Reserved */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_HARDWARE_LPC43_ATIMER_H */
