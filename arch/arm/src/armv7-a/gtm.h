/****************************************************************************
 * arch/arm/src/armv7-a/gtm.h
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

/* Reference:
 *   Cortex™-A9 MPCore, Revision: r4p1, Technical Reference Manual, ARM DDI
 *   0407I (ID091612).
 */

#ifndef __ARCH_ARM_SRC_ARMV7_A_GTM_H
#define __ARCH_ARM_SRC_ARMV7_A_GTM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nuttx/config.h"
#include <stdint.h>
#include "mpcore.h"

#ifdef CONFIG_ARMV7A_HAVE_GTM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GTM Register Offsets *****************************************************/

#define GTM_COUNT0_OFFSET      0x0000 /* Global Timer Counter Register 0 */
#define GTM_COUNT1_OFFSET      0x0004 /* Global Timer Counter Register 1 */
#define GTM_CTRL_OFFSET        0x0008 /* Global Timer Control Register */
#define GTM_STA_OFFSET         0x000c /* Global Timer Interrupt Status Register */
#define GTM_COMP0_OFFSET       0x0010 /* Comparator Value Register 0 */
#define GTM_COMP1_OFFSET       0x0014 /* Comparator Value Register 1 */
#define GTM_AUTO_OFFSET        0x0018 /* Auto-increment Register */

/* GTM Register Addresses ***************************************************/

#define GTM_COUNT0             (MPCORE_GTM_VBASE+GTM_COUNT0_OFFSET)
#define GTM_COUNT1             (MPCORE_GTM_VBASE+GTM_COUNT1_OFFSET)
#define GTM_CTRL               (MPCORE_GTM_VBASE+GTM_CTRL_OFFSET)
#define GTM_STA                (MPCORE_GTM_VBASE+GTM_STA_OFFSET)
#define GTM_COMP0              (MPCORE_GTM_VBASE+GTM_COMP0_OFFSET)
#define GTM_COMP1              (MPCORE_GTM_VBASE+GTM_COMP1_OFFSET)
#define GTM_AUTO               (MPCORE_GTM_VBASE+GTM_AUTO_OFFSET)

/* GTM Register Bit Definitions *********************************************/

/* Global Timer Counter Register 0/1 -- 64-bit timer counter value */

/* Global Timer Control Register */

#define GTM_CTRL_TIMEN         (1 << 0)  /* Bit 0:  Timer comparator */
#define GTM_CTRL_CMPEN         (1 << 1)  /* Bit 1:  Enable comparator */
#define GTM_CTRL_INTEN         (1 << 2)  /* Bit 2:  Enable timer interrupt ID 27 */
#define GTM_CTRL_AUTO          (1 << 3)  /* Bit 3:  Auto-increment comparator register */
                                         /* Bits 4-7: Reserved */
#define GTM_CTRL_PRESC_SHIFT   (8)       /* Bits 8-15: PERIPHCLK prescaler */
#define GTM_CTRL_PRESC_MASK    (0xff << GTM_CTRL_PRESC_SHIFT)
#  define GTM_CTRL_PRESC(n)    ((uint32_t)(n) << GTM_CTRL_PRESC_SHIFT)
                                         /* Bits 16-31: Reserved */

/* Global Timer Interrupt Status Register */

#define GTM_STA_EVENT          (1 << 0)  /* Timer event flag */
                                         /* Bits 1-31: Reserved */

/* Comparator Value Register 0/1 -- 64-bit timer compare value */

/* Auto-increment Register -- 32-bit auto-increment value */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Clocking:
 *   CLK - This is the main clock of the Cortex-A9 processor.  All Cortex-A9
 *     processors in the Cortex-A9 MPCore processor and the SCU are clocked
 *     with a distributed version of CLK.
 *   PERIPHCLK - The Interrupt Controller, global timer, private timers, and
 *     watchdogs are clocked with PERIPHCLK.  PERIPHCLK must be synchronous
 *     with CLK, and the PERIPHCLK clock period, N, must be configured as a
 *     multiple of the CLK clock period. This multiple N must be equal to,
 *     or greater than two.
 */

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_ARMV7A_HAVE_GTM */
#endif /* __ARCH_ARM_SRC_ARMV7_A_GTM_H */
