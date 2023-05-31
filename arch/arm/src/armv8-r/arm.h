/****************************************************************************
 * arch/arm/src/armv8-r/arm.h
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

/* References:
 *
 *  "ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition",
 *   Copyright 1996-1998, 2000, 2004-2012 ARM.
 *   All rights reserved. ARM DDI 0406C.c (ID051414)
 */

#ifndef __ARCH_ARM_SRC_ARMV8_R_ARM_H
#define __ARCH_ARM_SRC_ARMV8_R_ARM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BIT(n)          ((1UL) << (n))
#define BIT64(n)        ((1ULL) << (n))

/* Bit mask with bits 0 through n-1 (inclusive) set,
 * or 0 if n is 0.
 */
#define BIT_MASK(n)     (BIT(n) - 1)
#define BIT64_MASK(n)   (BIT64(n) - 1ULL)

/* ARMv8-R ******************************************************************/

/* PSR bits */

#define PSR_MODE_SHIFT    (0)       /* Bits 0-4: Mode fields */
#define PSR_MODE_MASK     (31 << PSR_MODE_SHIFT)
#  define PSR_MODE_USR    (16 << PSR_MODE_SHIFT) /* User mode */
#  define PSR_MODE_FIQ    (17 << PSR_MODE_SHIFT) /* FIQ mode */
#  define PSR_MODE_IRQ    (18 << PSR_MODE_SHIFT) /* IRQ mode */
#  define PSR_MODE_SVC    (19 << PSR_MODE_SHIFT) /* Supervisor mode */
#  define PSR_MODE_ABT    (23 << PSR_MODE_SHIFT) /* Abort mode */
#  define PSR_MODE_HYP    (26 << PSR_MODE_SHIFT) /* Hypervisor mode */
#  define PSR_MODE_UND    (27 << PSR_MODE_SHIFT) /* Undefined mode */
#  define PSR_MODE_SYS    (31 << PSR_MODE_SHIFT) /* System mode */

#define PSR_T_BIT         (1 << 5)  /* Bit 5: Thumb execution state bit */
#define PSR_MASK_SHIFT    (6)       /* Bits 6-8: Mask Bits */
#define PSR_MASK_MASK     (7 << PSR_GE_SHIFT)
#  define PSR_F_BIT       (1 << 6)  /* Bit 6: FIQ mask bit */
#  define PSR_I_BIT       (1 << 7)  /* Bit 7: IRQ mask bit */
#  define PSR_A_BIT       (1 << 8)  /* Bit 8: Asynchronous abort mask */
#define PSR_E_BIT         (1 << 9)  /* Bit 9:  Endianness execution state bit */
#define PSR_IT27_SHIFT    (10)      /* Bits 10-15:  If-Then execution state bits IT[2:7] */
#define PSR_IT27_MASK     (0x3f << PSR_IT27_SHIFT)
#define PSR_GE_SHIFT      (16)      /* Bits 16-19: Greater than or Equal flags */
#define PSR_GE_MASK       (15 << PSR_GE_SHIFT)
                                    /* Bits 20-23: Reserved. RAZ/SBZP */
#define PSR_J_BIT         (1 << 24) /* Bit 24: Jazelle state bit */
#define PSR_IT01_SHIFT    (25)      /* Bits 25-26:  If-Then execution state bits IT[0:1] */
#define PSR_IT01_MASK     (3 << PSR_IT01_SHIFT)
#define PSR_Q_BIT         (1 << 27) /* Bit 27: Cumulative saturation bit */
#define PSR_V_BIT         (1 << 28) /* Bit 28: Overflow condition flag */
#define PSR_C_BIT         (1 << 29) /* Bit 29: Carry condition flag */
#define PSR_Z_BIT         (1 << 30) /* Bit 30: Zero condition flag */
#define PSR_N_BIT         (1 << 31) /* Bit 31: Negative condition flag */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

/****************************************************************************
 * Name: arm_boot
 *
 * Description:
 *   Complete boot operations started in arm_head.S
 *
 * Boot Sequence
 *
 *   1.  The __start entry point in armv8-r/arm_head.S is invoked upon power-
 *       on reset.
 *   2.  __start prepares CPU for code execution.
 *
 ****************************************************************************/

void arm_boot(void);

/****************************************************************************
 * Name: arm_data_initialize
 *
 * Description:
 *   Clear all of .bss to zero; set .data to the correct initial values.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void arm_data_initialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_ARMV8_R_ARM_H */
