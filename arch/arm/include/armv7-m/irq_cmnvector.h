/****************************************************************************
 * arch/arm/include/armv7-m/irq_cmnvector.h
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

#ifndef __ARCH_ARM_INCLUDE_ARMV7_M_IRQ_CMNVECTOR_H
#define __ARCH_ARM_INCLUDE_ARMV7_M_IRQ_CMNVECTOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ Stack Frame Format: */

/* The following additional registers are stored by the interrupt handling
 * logic.
 */

#define REG_R13             (0)  /* R13 = SP at time of interrupt */
#ifdef CONFIG_ARMV7M_USEBASEPRI
#  define REG_BASEPRI       (1)  /* BASEPRI */
#else
#  define REG_PRIMASK       (1)  /* PRIMASK */
#endif
#define REG_R4              (2)  /* R4 */
#define REG_R5              (3)  /* R5 */
#define REG_R6              (4)  /* R6 */
#define REG_R7              (5)  /* R7 */
#define REG_R8              (6)  /* R8 */
#define REG_R9              (7)  /* R9 */
#define REG_R10             (8)  /* R10 */
#define REG_R11             (9)  /* R11 */
#define REG_EXC_RETURN      (10) /* EXC_RETURN */
#define SW_INT_REGS         (11)

#ifdef CONFIG_ARCH_FPU

/* If the MCU supports a floating point unit, then it will be necessary
 * to save the state of the non-volatile registers before calling code
 * that may save and overwrite them.
 */

#  define REG_S16           (SW_INT_REGS + 0)  /* S16 */
#  define REG_S17           (SW_INT_REGS + 1)  /* S17 */
#  define REG_S18           (SW_INT_REGS + 2)  /* S18 */
#  define REG_S19           (SW_INT_REGS + 3)  /* S19 */
#  define REG_S20           (SW_INT_REGS + 4)  /* S20 */
#  define REG_S21           (SW_INT_REGS + 5)  /* S21 */
#  define REG_S22           (SW_INT_REGS + 6)  /* S22 */
#  define REG_S23           (SW_INT_REGS + 7)  /* S23 */
#  define REG_S24           (SW_INT_REGS + 8)  /* S24 */
#  define REG_S25           (SW_INT_REGS + 9)  /* S25 */
#  define REG_S26           (SW_INT_REGS + 10) /* S26 */
#  define REG_S27           (SW_INT_REGS + 11) /* S27 */
#  define REG_S28           (SW_INT_REGS + 12) /* S28 */
#  define REG_S29           (SW_INT_REGS + 13) /* S29 */
#  define REG_S30           (SW_INT_REGS + 14) /* S30 */
#  define REG_S31           (SW_INT_REGS + 15) /* S31 */
#  define SW_FPU_REGS       (16)
#else
#  define SW_FPU_REGS       (0)
#endif

/* The total number of registers saved by software */

#define SW_XCPT_REGS        (SW_INT_REGS + SW_FPU_REGS)
#define SW_XCPT_SIZE        (4 * SW_XCPT_REGS)

/* On entry into an IRQ, the hardware automatically saves the following
 * registers on the stack in this (address) order:
 */

#define REG_R0              (SW_XCPT_REGS + 0) /* R0 */
#define REG_R1              (SW_XCPT_REGS + 1) /* R1 */
#define REG_R2              (SW_XCPT_REGS + 2) /* R2 */
#define REG_R3              (SW_XCPT_REGS + 3) /* R3 */
#define REG_R12             (SW_XCPT_REGS + 4) /* R12 */
#define REG_R14             (SW_XCPT_REGS + 5) /* R14 = LR */
#define REG_R15             (SW_XCPT_REGS + 6) /* R15 = PC */
#define REG_XPSR            (SW_XCPT_REGS + 7) /* xPSR */
#define HW_INT_REGS         (8)

#ifdef CONFIG_ARCH_FPU

/* If the FPU is enabled, the hardware also saves the volatile FP registers.
 */

#  define REG_S0            (SW_XCPT_REGS + 8)  /* S0 */
#  define REG_S1            (SW_XCPT_REGS + 9)  /* S1 */
#  define REG_S2            (SW_XCPT_REGS + 10) /* S2 */
#  define REG_S3            (SW_XCPT_REGS + 11) /* S3 */
#  define REG_S4            (SW_XCPT_REGS + 12) /* S4 */
#  define REG_S5            (SW_XCPT_REGS + 13) /* S5 */
#  define REG_S6            (SW_XCPT_REGS + 14) /* S6 */
#  define REG_S7            (SW_XCPT_REGS + 15) /* S7 */
#  define REG_S8            (SW_XCPT_REGS + 16) /* S8 */
#  define REG_S9            (SW_XCPT_REGS + 17) /* S9 */
#  define REG_S10           (SW_XCPT_REGS + 18) /* S10 */
#  define REG_S11           (SW_XCPT_REGS + 19) /* S11 */
#  define REG_S12           (SW_XCPT_REGS + 20) /* S12 */
#  define REG_S13           (SW_XCPT_REGS + 21) /* S13 */
#  define REG_S14           (SW_XCPT_REGS + 22) /* S14 */
#  define REG_S15           (SW_XCPT_REGS + 23) /* S15 */
#  define REG_FPSCR         (SW_XCPT_REGS + 24) /* FPSCR */
#  define REG_FP_RESERVED   (SW_XCPT_REGS + 25) /* Reserved */
#  define HW_FPU_REGS       (18)
#else
#  define HW_FPU_REGS       (0)
#endif

#define HW_XCPT_REGS        (HW_INT_REGS + HW_FPU_REGS)
#define HW_XCPT_SIZE        (4 * HW_XCPT_REGS)

#define XCPTCONTEXT_REGS    (HW_XCPT_REGS + SW_XCPT_REGS)
#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_INCLUDE_ARMV7_M_IRQ_CMNVECTOR_H */
