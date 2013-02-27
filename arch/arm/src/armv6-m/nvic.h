/****************************************************************************************************
 * arch/arm/src/armv6-m/nvic.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_COMMON_ARMV6_M_NVIC_H
#define __ARCH_ARM_SRC_COMMON_ARMV6_M_NVIC_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Base addresses ***********************************************************************************/

#define ARMV6M_SYSCON1_BASE            0xe000e008 /* 0xe000e008-0xe000e00f System Control Block */
                                                  /* 0xe000e010-0xe000e01f Reserved */
#define ARMV6M_SYSTICK_BASE            0xe000e010 /* 0xe000e010-0xe000e01f SysTick system timer */
#define ARMV6M_NVIC1_BASE              0xe000e100 /* 0xe000e100-0xe000e4ef Nested Vectored Interrupt Controller */
#define ARMV6M_SYSCON2_BASE            0xe000ed00 /* 0xe000ed00-0xe000ed3f System Control Block */
#define ARMV6M_NVIC2_BASE              0xe000ef00 /* 0xe000ef00-0xe000ef03 Nested Vectored Interrupt Controller */

/* NVIC register offsets ****************************************************************************/
/* NVIC register offsets (all relative to ARMV6M_NVIC1_BASE) */

#define ARMV6M_NVIC_ISER_OFFSET        0x0000  /* Interrupt set-enable register */
#define ARMV6M_NVIC_ICER_OFFSET        0x0080  /* Interrupt clear-enable register */
#define ARMV6M_NVIC_ISPR_OFFSET        0x0100  /* Interrupt set-pending register */
#define ARMV6M_NVIC_ICPR_OFFSET        0x0180  /* Interrupt clear-pending register */
#define ARMV6M_NVIC_IPR_OFFSET(n)      (0x0300 + ((n) << 2))
#  define ARMV6M_NVIC_IPR0_OFFSET      0x0300  /* Interrupt priority register 0 */
#  define ARMV6M_NVIC_IPR1_OFFSET      0x0304  /* Interrupt priority register 1 */
#  define ARMV6M_NVIC_IPR2_OFFSET      0x0308  /* Interrupt priority register 2 */
#  define ARMV6M_NVIC_IPR3_OFFSET      0x030c  /* Interrupt priority register 3 */
#  define ARMV6M_NVIC_IPR4_OFFSET      0x0310  /* Interrupt priority register 4 */
#  define ARMV6M_NVIC_IPR5_OFFSET      0x0314  /* Interrupt priority register 5 */
#  define ARMV6M_NVIC_IPR6_OFFSET      0x0318  /* Interrupt priority register 6 */
#  define ARMV6M_NVIC_IPR7_OFFSET      0x031c  /* Interrupt priority register 7 */

/* System control register offsets (all relative to ARMV6M_SYSCON2_BASE) */

#define ARMV6M_SYSCON_CPUID_OFFSET     0x0000  /* CPUID Register */
#define ARMV6M_SYSCON_ICSR_OFFSET      0x0004  /* Interrupt control and state register  */
#define ARMV6M_SYSCON_AIRCR_OFFSET     0x000c  /* Application interrupt and reset control register */
#define ARMV6M_SYSCON_SCR_OFFSET       0x0010  /* System control register */
#define ARMV6M_SYSCON_CCR_OFFSET       0x0014  /* Configuration and control register */
#define ARMV6M_SYSCON_SHPR2_OFFSET     0x001c  /* System handler priority register 2 */
#define ARMV6M_SYSCON_SHPR3_OFFSET     0x0020  /* System handler priority register 3 */

/* SYSTICK register offsets (all relatvive to ARMV6M_SYSTICK_BASE) */

#define ARMV6M_SYSTICK_CSR_OFFSET      0x0000  /* SysTick control and status register */
#define ARMV6M_SYSTICK_RVR_OFFSET      0x0004  /* SysTick reload value register */
#define ARMV6M_SYSTICK_CVR_OFFSET      0x0008  /* SysTick current value register */
#define ARMV6M_SYSTICK_CALIB_OFFSET    0x000c  /* SysTick calibration value register */

/* Register addresses *******************************************************************************/
/* NVIC register addresses */

#define ARMV6M_NVIC_ISER               (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_ISER_OFFSET)
#define ARMV6M_NVIC_ICER               (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_ICER_OFFSET)
#define ARMV6M_NVIC_ISPR               (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_ISPR_OFFSET)
#define ARMV6M_NVIC_ICPR               (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_ICPR_OFFSET)
#define ARMV6M_NVIC_IPR(n)             (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_IPR_OFFSET(n))
#  define ARMV6M_NVIC_IPR0             (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_IPR0_OFFSET)
#  define ARMV6M_NVIC_IPR1             (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_IPR1_OFFSET)
#  define ARMV6M_NVIC_IPR2             (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_IPR2_OFFSET)
#  define ARMV6M_NVIC_IPR3             (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_IPR3_OFFSET)
#  define ARMV6M_NVIC_IPR4             (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_IPR4_OFFSET)
#  define ARMV6M_NVIC_IPR5             (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_IPR5_OFFSET)
#  define ARMV6M_NVIC_IPR6             (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_IPR6_OFFSET)
#  define ARMV6M_NVIC_IPR7             (ARMV6M_NVIC1_BASE+ARMV6M_NVIC_IPR7_OFFSET)

/* System control register addresses */

#define ARMV6M_SYSCON_CPUID            (ARMV6M_SYSCON2_BASE+ARMV6M_SYSCON_CPUID_OFFSET)
#define ARMV6M_SYSCON_ICSR             (ARMV6M_SYSCON2_BASE+ARMV6M_SYSCON_ICSR_OFFSET)
#define ARMV6M_SYSCON_AIRCR            (ARMV6M_SYSCON2_BASE+ARMV6M_SYSCON_AIRCR_OFFSET)
#define ARMV6M_SYSCON_SCR              (ARMV6M_SYSCON2_BASE+ARMV6M_SYSCON_SCR_OFFSET)
#define ARMV6M_SYSCON_CCR              (ARMV6M_SYSCON2_BASE+ARMV6M_SYSCON_CCR_OFFSET)
#define ARMV6M_SYSCON_SHPR2            (ARMV6M_SYSCON2_BASE+ARMV6M_SYSCON_SHPR2_OFFSET)
#define ARMV6M_SYSCON_SHPR3            (ARMV6M_SYSCON2_BASE+ARMV6M_SYSCON_SHPR3_OFFSET)

/* SYSTICK register addresses */

#define ARMV6M_SYSTICK_CSR             (ARMV6M_SYSTICK_BASE+ARMV6M_SYSTICK_CSR_OFFSET)
#define ARMV6M_SYSTICK_RVR             (ARMV6M_SYSTICK_BASE+ARMV6M_SYSTICK_RVR_OFFSET)
#define ARMV6M_SYSTICK_CVR             (ARMV6M_SYSTICK_BASE+ARMV6M_SYSTICK_CVR_OFFSET)
#define ARMV6M_SYSTICK_CALIB           (ARMV6M_SYSTICK_BASE+ARMV6M_SYSTICK_CALIB_OFFSET)

/* Register bit definitions *************************************************************************/

/* Interrupt set-enable register */

#define NVIC_ISER(n)                   (1 << (n)) /* n=0..31 */

/* Interrupt clear-enable register */

#define NVIC_ICER(n)                   (1 << (n)) /* n=0..31 */

/* Interrupt set-pending register */

#define NVIC_ISPR(n)                   (1 << (n)) /* n=0..31 */

/* Interrupt clear-pending register */

#define NVIC_ICPR(n)                   (1 << (n)) /* n=0..31 */

/* Interrupt priority registers 0-7 */

#define NVIC_IPR_0_SHIFT               (0)        /* Bits 0-7:  PRI_(4n) */
#define NVIC_IPR_0_MASK                (0xff << NVIC_IPR_0_SHIFT)
#  define NVIC_IPR_0(p)                ((p) << NVIC_IPR_0_SHIFT)
#define NVIC_IPR_1_SHIFT               (8)        /* Bits 8-15:  PRI_(4n+1) */
#define NVIC_IPR_1_MASK                (0xff << NVIC_IPR_1_SHIFT)
#  define NVIC_IPR_1(p)                ((p) << NVIC_IPR_1_SHIFT)
#define NVIC_IPR_2_SHIFT               (16)       /* Bits 16-23:  PRI_(4n+2) */
#define NVIC_IPR_2_MASK                (0xff << NVIC_IPR_2_SHIFT)
#  define NVIC_IPR_2(p)                ((p) << NVIC_IPR_2_SHIFT)
#define NVIC_IPR_3_SHIFT               (24)       /* Bits 24-31:  PRI_(4n+3) */
#define NVIC_IPR_3_MASK                (0xff << NVIC_IPR_3_SHIFT)
#  define NVIC_IPR_3(p)                ((p) << NVIC_IPR_3_SHIFT)

#define NVIC_IPR0_0_SHIFT              (0)        /* Bits 0-7:   PRI_0 */
#define NVIC_IPR0_0_MASK               (0xff << NVIC_IPR0_0_SHIFT)
#  define NVIC_IPR0_0(p)               ((p) << NVIC_IPR0_0_SHIFT)
#define NVIC_IPR0_1_SHIFT              (8)        /* Bits 8-15:  PRI_1 */
#define NVIC_IPR0_1_MASK               (0xff << NVIC_IPR0_1_SHIFT)
#  define NVIC_IPR0_1(p)               ((p) << NVIC_IPR0_1_SHIFT)
#define NVIC_IPR0_2_SHIFT              (16)       /* Bits 16-23: PRI_2 */
#define NVIC_IPR0_2_MASK               (0xff << NVIC_IPR0_2_SHIFT)
#  define NVIC_IPR0_2(p)               ((p) << NVIC_IPR0_2_SHIFT)
#define NVIC_IPR0_3_SHIFT              (24)       /* Bits 24-31: PRI_3 */
#define NVIC_IPR0_3_MASK               (0xff << NVIC_IPR0_3_SHIFT)
#  define NVIC_IPR0_3(p)               ((p) << NVIC_IPR0_3_SHIFT)

#define NVIC_IPR1_4_SHIFT              (0)        /* Bits 0-7:   PRI_4 */
#define NVIC_IPR1_4_MASK               (0xff << NVIC_IPR1_4_SHIFT)
#  define NVIC_IPR1_4(p)               ((p) << NVIC_IPR1_4_SHIFT)
#define NVIC_IPR1_5_SHIFT              (8)        /* Bits 8-15:  PRI_5 */
#define NVIC_IPR1_5_MASK               (0xff << NVIC_IPR1_5_SHIFT)
#  define NVIC_IPR1_5(p)               ((p) << NVIC_IPR1_5_SHIFT)
#define NVIC_IPR1_6_SHIFT              (16)       /* Bits 16-23: PRI_6 */
#define NVIC_IPR1_6_MASK               (0xff << NVIC_IPR1_6_SHIFT)
#  define NVIC_IPR1_6(p)               ((p) << NVIC_IPR1_6_SHIFT)
#define NVIC_IPR1_7_SHIFT              (24)       /* Bits 24-31: PRI_7 */
#define NVIC_IPR1_7_MASK               (0xff << NVIC_IPR1_7_SHIFT)
#  define NVIC_IPR1_7(p)               ((p) << NVIC_IPR1_7_SHIFT)

#define NVIC_IPR2_8_SHIFT              (0)        /* Bits 0-7:   PRI_8 */
#define NVIC_IPR2_8_MASK               (0xff << NVIC_IPR2_8_SHIFT)
#  define NVIC_IPR2_8(p)               ((p) << NVIC_IPR2_8_SHIFT)
#define NVIC_IPR2_9_SHIFT              (8)        /* Bits 8-15:  PRI_9 */
#define NVIC_IPR2_9_MASK               (0xff << NVIC_IPR2_9_SHIFT)
#  define NVIC_IPR2_9(p)               ((p) << NVIC_IPR2_9_SHIFT)
#define NVIC_IPR2_10_SHIFT             (16)       /* Bits 16-23: PRI_10 */
#define NVIC_IPR2_10_MASK              (0xff << NVIC_IPR2_10_SHIFT)
#  define NVIC_IPR2_10(p)              ((p) << NVIC_IPR2_10_SHIFT)
#define NVIC_IPR2_11_SHIFT             (24)       /* Bits 24-31: PRI_11 */
#define NVIC_IPR2_11_MASK              (0xff << NVIC_IPR2_11_SHIFT)
#  define NVIC_IPR2_11(p)              ((p) << NVIC_IPR2_11_SHIFT)

#define NVIC_IPR3_12_SHIFT             (0)        /* Bits 0-7:   PRI_12 */
#define NVIC_IPR3_12_MASK              (0xff << NVIC_IPR3_12_SHIFT)
#  define NVIC_IPR3_12(p)              ((p) << NVIC_IPR3_12_SHIFT)
#define NVIC_IPR3_13_SHIFT             (8)        /* Bits 8-15:  PRI_13 */
#define NVIC_IPR3_13_MASK              (0xff << NVIC_IPR3_13_SHIFT)
#  define NVIC_IPR3_13(p)              ((p) << NVIC_IPR3_13_SHIFT)
#define NVIC_IPR3_14_SHIFT             (16)       /* Bits 16-23: PRI_14 */
#define NVIC_IPR3_14_MASK              (0xff << NVIC_IPR3_14_SHIFT)
#  define NVIC_IPR3_14(p)              ((p) << NVIC_IPR3_14_SHIFT)
#define NVIC_IPR3_15_SHIFT             (24)       /* Bits 24-31: PRI_15 */
#define NVIC_IPR3_15_MASK              (0xff << NVIC_IPR3_15_SHIFT)
#  define NVIC_IPR3_15(p)              ((p) << NVIC_IPR3_15_SHIFT)

#define NVIC_IPR4_16_SHIFT             (0)        /* Bits 0-7:   PRI_16 */
#define NVIC_IPR4_16_MASK              (0xff << NVIC_IPR4_16_SHIFT)
#  define NVIC_IPR4_16(p)              ((p) << NVIC_IPR4_16_SHIFT)
#define NVIC_IPR4_17_SHIFT             (8)        /* Bits 8-15:  PRI_17 */
#define NVIC_IPR4_17_MASK              (0xff << NVIC_IPR4_17_SHIFT)
#  define NVIC_IPR4_17(p)              ((p) << NVIC_IPR4_17_SHIFT)
#define NVIC_IPR4_18_SHIFT             (16)       /* Bits 16-23: PRI_18 */
#define NVIC_IPR4_18_MASK              (0xff << NVIC_IPR4_18_SHIFT)
#  define NVIC_IPR4_18(p)              ((p) << NVIC_IPR4_18_SHIFT)
#define NVIC_IPR4_19_SHIFT             (24)       /* Bits 24-31: PRI_19 */
#define NVIC_IPR4_19_MASK              (0xff << NVIC_IPR4_19_SHIFT)
#  define NVIC_IPR4_19(p)              ((p) << NVIC_IPR4_19_SHIFT)

#define NVIC_IPR5_20_SHIFT             (0)        /* Bits 0-7:   PRI_20 */
#define NVIC_IPR5_20_MASK              (0xff << NVIC_IPR5_20_SHIFT)
#  define NVIC_IPR5_20(p)              ((p) << NVIC_IPR5_20_SHIFT)
#define NVIC_IPR5_21_SHIFT             (8)        /* Bits 8-15:  PRI_21 */
#define NVIC_IPR5_21_MASK              (0xff << NVIC_IPR5_21_SHIFT)
#  define NVIC_IPR5_21(p)              ((p) << NVIC_IPR5_21_SHIFT)
#define NVIC_IPR5_22_SHIFT             (16)       /* Bits 16-23: PRI_22 */
#define NVIC_IPR5_22_MASK              (0xff << NVIC_IPR5_22_SHIFT)
#  define NVIC_IPR5_22(p)              ((p) << NVIC_IPR5_22_SHIFT)
#define NVIC_IPR5_23_SHIFT             (24)       /* Bits 24-31: PRI_23 */
#define NVIC_IPR5_23_MASK              (0xff << NVIC_IPR5_23_SHIFT)
#  define NVIC_IPR5_23(p)              ((p) << NVIC_IPR5_23_SHIFT)

#define NVIC_IPR6_24_SHIFT             (0)        /* Bits 0-7:   PRI_24 */
#define NVIC_IPR6_24_MASK              (0xff << NVIC_IPR6_24_SHIFT)
#  define NVIC_IPR6_24(p)              ((p) << NVIC_IPR6_24_SHIFT)
#define NVIC_IPR6_25_SHIFT             (8)        /* Bits 8-15:  PRI_25 */
#define NVIC_IPR6_25_MASK              (0xff << NVIC_IPR6_25_SHIFT)
#  define NVIC_IPR6_25(p)              ((p) << NVIC_IPR6_25_SHIFT)
#define NVIC_IPR6_26_SHIFT             (16)       /* Bits 16-23: PRI_26 */
#define NVIC_IPR6_26_MASK              (0xff << NVIC_IPR6_26_SHIFT)
#  define NVIC_IPR6_26(p)              ((p) << NVIC_IPR6_26_SHIFT)
#define NVIC_IPR6_27_SHIFT             (24)       /* Bits 24-31: PRI_27 */
#define NVIC_IPR6_27_MASK              (0xff << NVIC_IPR6_27_SHIFT)
#  define NVIC_IPR6_27(p)              ((p) << NVIC_IPR6_27_SHIFT)

#define NVIC_IPR7_28_SHIFT             (0)        /* Bits 0-7:   PRI_28 */
#define NVIC_IPR7_28_MASK              (0xff << NVIC_IPR7_28_SHIFT)
#  define NVIC_IPR7_28(p)              ((p) << NVIC_IPR7_28_SHIFT)
#define NVIC_IPR7_29_SHIFT             (8)        /* Bits 8-15:  PRI_29 */
#define NVIC_IPR7_29_MASK              (0xff << NVIC_IPR7_29_SHIFT)
#  define NVIC_IPR7_29(p)              ((p) << NVIC_IPR7_29_SHIFT)
#define NVIC_IPR7_30_SHIFT             (16)       /* Bits 16-23: PRI_30 */
#define NVIC_IPR7_30_MASK              (0xff << NVIC_IPR7_30_SHIFT)
#  define NVIC_IPR7_30(p)              ((p) << NVIC_IPR7_30_SHIFT)
#define NVIC_IPR7_31_SHIFT             (24)       /* Bits 24-31: PRI_31 */
#define NVIC_IPR7_31_MASK              (0xff << NVIC_IPR7_31_SHIFT)
#  define NVIC_IPR7_31(p)              ((p) << NVIC_IPR7_31_SHIFT)

/* System control register addresses */

/* CPUID Register */

#define SYSCON_CPUID_REVISION_SHIFT    (0)     /* Bits 0-3:  Revision number */
#define SYSCON_CPUID_REVISION_MASK     (15 << SYSCON_CPUID_REVISION_SHIFT)
#define SYSCON_CPUID_PARTNO_SHIFT      (4)     /* Bits 4-15: Part number of the processor */
#define SYSCON_CPUID_PARTNO_MASK       (0x0fff << SYSCON_CPUID_PARTNO_SHIFT)
#  define SYSCON_CPUID_PARTNO_CORTEXM0 (0x0c20 << SYSCON_CPUID_PARTNO_SHIFT)
#define SYSCON_CPUID_CONSTANT_SHIFT    (16)    /* Bits 16-19: Constant that defines the architecture
                                                *             of the processor */
#define SYSCON_CPUID_CONSTANT_MASK     (15 << SYSCON_CPUID_CONSTANT_SHIFT)
#  define SYSCON_CPUID_CONSTANT_ARMV6M (12 << SYSCON_CPUID_PARTNO_SHIFT)
#define SYSCON_CPUID_VARIANT_SHIFT     (20)    /* Bits 20-23: Variant number, the r value in the
                                                *             rnpn product revision identifier */
#define SYSCON_CPUID_VARIANT_MASK      (15 << SYSCON_CPUID_VARIANT_SHIFT)
#define SYSCON_CPUID_IMPLEMENTER_SHIFT (24)    /* Bits 24-31: Implementer code */
#define SYSCON_CPUID_IMPLEMENTER_MASK  (0xff << SYSCON_CPUID_IMPLEMENTER_SHIFT)
#  define SYSCON_CPUID_IMPLEMENTER_ARM (0x41 << SYSCON_CPUID_IMPLEMENTER_SHIFT)

/* Interrupt control and state register  */

#define SYSCON_ICSR_VECTACTIVE_SHIFT   (0)       /* Bits 0-5: Contains the active exception number */
#define SYSCON_ICSR_VECTACTIVE_MASK    (63 << SYSCON_ICSR_VECTACTIVE_SHIFT)
#define SYSCON_ICSR_VECTPENDING_SHIFT  (12)      /* Bits 12-17: Indicates the exception number of
                                                  *             the highest priority pending enabled
                                                  *             exception */
#define SYSCON_ICSR_VECTPENDING_MASK   (63 << SYSCON_ICSR_VECTPENDING_SHIFT)
#define SYSCON_ICSR_ISRPENDING         (1 << 22) /* Bit 22: Interrupt pending flag, excluding NMI
                                                  *         and Faults */
#define SYSCON_ICSR_PENDSTCLR          (1 << 25) /* Bit 25: SysTick exception clear-pending bit */
#define SYSCON_ICSR_PENDSTSET          (1 << 26) /* Bit 26: SysTick exception set-pending bit */
#define SYSCON_ICSR_PENDSVCLR          (1 << 27) /* Bit 27: PendSV clear-pending bit */
#define SYSCON_ICSR_PENDSVSET          (1 << 28) /* Bit 28: PendSV set-pending bit */
#define SYSCON_ICSR_NMIPENDSET         (1 << 31) /* Bit 31: NMI set-pending bit */

/* Application interrupt and reset control register */

#define SYSCON_AIRCR_VECTCLRACTIVE     (1 << 1)  /* Bit 1: Reserved for debug use */
#define SYSCON_AIRCR_SYSRESETREQ       (1 << 2)  /* Bit 2: System reset request */
#define SYSCON_AIRCR_ENDIANESS         (1 << 15) /* Bit 15: Data endianness implemented */
#define SYSCON_AIRCR_VECTKEY_SHIFT     (16)      /* Bits 16-31: Register key */
#define SYSCON_AIRCR_VECTKEY_MASK      (0xffff << SYSCON_AIRCR_VECTKEY_SHIFT)
#  define SYSCON_AIRCR_VECTKEY(n)      ((n) << SYSCON_AIRCR_VECTKEY_SHIFT)

/* System control register */

#define SYSCON_SCR_SLEEPONEXIT         (1 << 1) /* Bit 1: Sleep-on-exit when returning from handler
                                                 *        to thread mode */
#define SYSCON_SCR_SLEEPDEEP           (1 << 2) /* Bit 2: Use deep sleep as the low power mode */
#define SYSCON_SCR_SEVONPEND           (1 << 4) /* Bit 4: Send event on pending bit */

/* Configuration and control register */

#define SYSCON_CCR_UNALIGN_TRP         (1 << 3) /* Bit 3: Unaligned accesses generate a hardfault */
#define SYSCON_CCR_STKALIGN            (1 << 9) /* Bit 9: 8-byte stack alignment on exception entry */

/* System handler priority register 2 */

#define SYSCON_SHPR2_PRI_11_SHIFT      (24)      /* Bits 24-31: Priority of system handler 11,
                                                  *             SVCall */
#define SYSCON_SHPR2_PRI_11_MASK       (0xff << SYSCON_SHPR2_PRI_11_SHIFT)

/* System handler priority register 3 */

#define SYSCON_SHPR3_PRI_14_SHIFT      (16)      /* Bits 16-23: Priority of system handler 14,
                                                  *             PendSV */
#define SYSCON_SHPR3_PRI_14_MASK       (0xff << SYSCON_SHPR3_PRI_14_SHIFT)
#define SYSCON_SHPR3_PRI_15_SHIFT      (24)      /* Bits 24-31: Priority of system handler 15,
                                                  *             SysTick exception */
#define SYSCON_SHPR3_PRI_15_MASK       (0xff << SYSCON_SHPR3_PRI_15_SHIFT)

/* SYSTICK register addresses */

/* SysTick control and status register */

#define SYSTICK_CSR_ENABLE             (1 << 0)  /* Bit 0:  Enables the counter */
#define SYSTICK_CSR_TICKINT            (1 << 1)  /* Bit 1:  Enables SysTick exception request */
#define SYSTICK_CSR_CLKSOURCE          (1 << 2)  /* Bit 2:  Selects the SysTick timer clock source */
#define SYSTICK_CSR_COUNTFLAG          (1 << 16) /* Bit 16: Returns 1 if timer counted to 0 since
                                                  *         the last read of this register */
/* SysTick reload value register */

#define SYSTICK_RVR_MASK               (0x0fffffff) /* Bits 0-23 */

/* SysTick current value register */

#define SYSTICK_CVR_MASK               (0x0fffffff) /* Bits 0-23 */

/* SysTick calibration value register */

#define SYSTICK_CALIB_TENMS_SHIFT      (0)       /* Bits 0-23: Reload value for 10ms (100Hz) timing */
#define SYSTICK_CALIB_TENMS_MASK       (0x0fffffff << SYSTICK_CALIB_TENMS_SHIFT)
#define SYSTICK_CALIB_SKEW             (1 << 30) /* Bit 30: TENMS value is exact */
#define SYSTICK_CALIB_NOREF            (1 << 31) /* Bit 31: Device provides a reference clock */

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public Data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__
#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" 
{
#else
#define EXTERN extern
#endif

/****************************************************************************************************
 * Public Function Prototypes
 ****************************************************************************************************/

/****************************************************************************************************
 * Function:  up_dumpnvic
 *
 * Description:
 *   Dump all NVIC and SYSCON registers along with a user message.
 *
 ****************************************************************************************************/

#ifdef CONFIG_DEBUG
void up_dumpnvic(FAR const char *msg);
#else
#  define up_dumpnvic(m)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_COMMON_ARMV6_M_NVIC_H */
