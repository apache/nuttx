/****************************************************************************
 * arch/arm/src/armv7-a/scu.h
 * Generic Interrupt Controller Definitions
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

#ifndef __ARCH_ARM_SRC_ARMV7_A_SCU_H
#define __ARCH_ARM_SRC_ARMV7_A_SCU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "mpcore.h"                         /* For MPCORE_SCU_VBASE */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define SCU_CTRL_OFFSET              0x0000 /* SCU Control Register (Implementation defined) */
#define SCU_CONFIG_OFFSET            0x0004 /* SCU Configuration Register (Implementation defined) */
#define SCU_PWRSTATUS_OFFSET         0x0008 /* SCU CPU Power Status Register */
#define SCU_INVALIDATE_OFFSET        0x000c /* SCU Invalidate All Registers in Secure State */
#define SCU_FILTERSTART_OFFSET       0x0040 /* Filtering Start Address Register Defined by FILTERSTART input */
#define SCU_FILTEREND_OFFSET         0x0044 /* Filtering End Address Register Defined by FILTEREND input */
#define SCU_SAC_OFFSET               0x0050 /* SCU Access Control (SAC) Register */
#define SCU_SNSAC_OFFSET             0x0054 /* SCU Non-secure Access Control (SNSAC) Register */

/* Register addresses *******************************************************/

#define SCU_CTRL                     (MPCORE_SCU_VBASE+SCU_CTRL_OFFSET)
#define SCU_CONFIG                   (MPCORE_SCU_VBASE+SCU_CONFIG_OFFSET)
#define SCU_PWRSTATUS                (MPCORE_SCU_VBASE+SCU_PWRSTATUS_OFFSET)
#define SCU_INVALIDATE               (MPCORE_SCU_VBASE+SCU_INVALIDATE_OFFSET)
#define SCU_FILTERSTART              (MPCORE_SCU_VBASE+SCU_FILTERSTART_OFFSET)
#define SCU_FILTEREND                (MPCORE_SCU_VBASE+SCU_FILTEREND_OFFSET)
#define SCU_SAC                      (MPCORE_SCU_VBASE+SCU_SAC_OFFSET)
#define SCU_SNSAC                    (MPCORE_SCU_VBASE+SCU_SNSAC_OFFSET)

/* Register bit-field definitions *******************************************/

/* SCU Control Register (Implementation defined) */

#define SCU_CTRL_ENABLE              (1 << 0)  /* SCU enable */
#define SCU_CTRL_ADDRFILTER          (1 << 1)  /* Address filtering enable */
#define SCU_CTRL_RAMPARITY           (1 << 2)  /* SCU RAMs parity enable */
#define SCU_CTRL_LINFILL             (1 << 3)  /* SCU speculative linefill enable */
#define SCU_CTRL_PORT0               (1 << 4)  /* Force all device to port0 enable */
#define SCU_CTRL_STANDBY             (1 << 5)  /* SCU standby enable */
#define SCU_CTRL_ICSTANDBY           (1 << 6)  /* IC standby enable */

/* SCU Configuration Register (Implementation defined) */

#define SCU_CONFIG_NCPUS_SHIFT       0         /* CPU number Number of CPUs present */
#define SCU_CONFIG_NCPUS_MASK        (3 << SCU_CONFIG_NCPUS_SHIFT)
#  define SCU_CONFIG_NCPUS(r)        ((((uint32_t)(r) &  SCU_CONFIG_NCPUS_MASK) >> SCU_CONFIG_NCPUS_SHIFT) + 1)
#define SCU_CONFIG_SMPCPUS_SHIFT     4         /* Processors that are in SMP or AMP mode */
#define SCU_CONFIG_SMPCPUS_MASK      (15 << SCU_CONFIG_SMPCPUS_SHIFT)
#  define SCU_CONFIG_CPU_SMP(n)      (1 << ((n)+4))
#  define SCU_CONFIG_CPU0_SMP        (1 << 4)
#  define SCU_CONFIG_CPU1_SMP        (1 << 5)
#  define SCU_CONFIG_CPU2_SMP        (1 << 6)
#  define SCU_CONFIG_CPU3_SMP        (1 << 7)

#define SCU_CONFIG_TAGRAM_16KB       0
#define SCU_CONFIG_TAGRAM_32KB       1
#define SCU_CONFIG_TAGRAM_64KB       2

#define SCU_CONFIG_CPU0_TAGRAM_SHIFT 8         /* CPU 0 tag RAM size */
#define SCU_CONFIG_CPU0_TAGRAM_MASK  (3 << SCU_CONFIG_CPU0_TAGRAM_SHIFT)
#define SCU_CONFIG_CPU1_TAGRAM_SHIFT 10        /* CPU 1 tag RAM size */
#define SCU_CONFIG_CPU1_TAGRAM_MASK  (3 << SCU_CONFIG_CPU0_TAGRAM_SHIFT)
#define SCU_CONFIG_CPU2_TAGRAM_SHIFT 12        /* CPU 1 tag RAM size */
#define SCU_CONFIG_CPU2_TAGRAM_MASK  (3 << SCU_CONFIG_CPU0_TAGRAM_SHIFT)
#define SCU_CONFIG_CPU3_TAGRAM_SHIFT 14        /* CPU 1 tag RAM size */
#define SCU_CONFIG_CPU3_TAGRAM_MASK  (3 << SCU_CONFIG_CPU0_TAGRAM_SHIFT)

/* SCU CPU Power Status Register */

#define SCU_PWRSTATUS_NORMAL         0
#define SCU_PWRSTATUS_DORMANT        2
#define SCU_PWRSTATUS_PWROFF         3

#define SCU_PWRSTATUS_CPU0_SHIFT     0         /* CPU0 status Power status */
#define SCU_PWRSTATUS_CPU0_MASK      (3 << SCU_PWRSTATUS_CPU0_SHIFT)
#define SCU_PWRSTATUS_CPU1_SHIFT     8         /* CPU1 status Power status */
#define SCU_PWRSTATUS_CPU1_MASK      (3 << SCU_PWRSTATUS_CPU1_SHIFT)
#define SCU_PWRSTATUS_CPU2_SHIFT     16        /* CPU2 status Power status */
#define SCU_PWRSTATUS_CPU2_MASK      (3 << SCU_PWRSTATUS_CPU2_SHIFT)
#define SCU_PWRSTATUS_CPU3_SHIFT     24        /* CPU3 status Power status */
#define SCU_PWRSTATUS_CPU3_MASK      (3 << SCU_PWRSTATUS_CPU3_SHIFT)

/* SCU Invalidate All Registers in Secure State */

#define SCU_INVALIDATE_ALL_WAYS      15
#define SCU_INVALIDATE_CPU0_SHIFT    0         /* Ways that must be invalidated for CPU0 */
#define SCU_INVALIDATE_CPU0_MASK     (15 << SCU_INVALIDATE_CPU0_SHIFT)
#define SCU_INVALIDATE_CPU1_SHIFT    4         /* Ways that must be invalidated for CPU1 */
#define SCU_INVALIDATE_CPU1_MASK     (15 << SCU_INVALIDATE_CPU1_SHIFT)
#define SCU_INVALIDATE_CPU2_SHIFT    8         /* Ways that must be invalidated for CPU2 */
#define SCU_INVALIDATE_CPU2_MASK     (15 << SCU_INVALIDATE_CPU2_SHIFT)
#define SCU_INVALIDATE_CPU3_SHIFT    12        /* Ways that must be invalidated for CPU3 */
#define SCU_INVALIDATE_CPU3_MASK     (15 << SCU_INVALIDATE_CPU3_SHIFT)

/* Filtering Start Address Register Defined by FILTERSTART input */

#define SCU_FILTERSTART_SHIFT        10        /* Filtering start address */
#define SCU_FILTERSTART_MASK         (0xfff << SCU_FILTERSTART_SHIFT)

/* Filtering End Address Register Defined by FILTEREND input */

#define SCU_FILTEREND_SHIFT          10        /* Filtering start address */
#define SCU_FILTEREND_MASK           (0xfff << SCU_FILTEREND_SHIFT)

/* SCU Access Control (SAC) Register */

#define SCU_SAC_CPU(n)               (1 << (n)) /* CPUn may access components */

/* SCU Non-secure Access Control (SNSAC) Register */

#define SCU_SNSAC_COMP_CPU(n)        (1 << (n))     /* CPUn has non-secure access to components */
#define SCU_SNSAC_PTIM_CPU(n)        (1 << ((n)+4)) /* CPUn has non-secure access to private timers */
#define SCU_SNSAC_GTIM_CPU(n)        (1 << ((n)+8)) /* CPUn has non-secure access to global timer */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_enable_smp
 *
 * Description:
 *   Enable the SCU and make certain that current CPU is participating in
 *   the SMP cache coherency.
 *
 ****************************************************************************/

void arm_enable_smp(int cpu);

#endif /* __ARCH_ARM_SRC_ARMV7_A_SCU_H */
