/****************************************************************************
 * arch/arm/src/armv7-a/gtm.h
 * Global Timer Definitions
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference:
 *   Cortex™-A9 MPCore, Revision: r4p1, Technical Reference Manual, ARM DDI
 *   0407I (ID091612).
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
 ****************************************************************************/

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
#define GTM_COMP1              (MPCORE_GTM_VBASE+COMPARE1_OFFSET)
#define GTM_AUTO               (MPCORE_GTM_VBASE+AUTO_OFFSET)

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
