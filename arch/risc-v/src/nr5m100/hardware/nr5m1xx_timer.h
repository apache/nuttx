/****************************************************************************
 * arch/risc-v/src/nr5m100/hardware/nr5_timer.h
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#ifndef ARCH_RISCV_SRC_NR5M100_CHIP_NR5M1XX_TIMERA_H
#define ARCH_RISCV_SRC_NR5M100_CHIP_NR5M1XX_TIMERA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nr5m1xx_memorymap.h"

/* The timers used in the NR5M100 are functionally equivalent to
 * Timer A in the MSP430.  The hardware peripheral in the FPGA uses the BSD
 * licensed RTL code from the OpenMSP430 project on opencores.org.
 */

/* TimerA offet definitions */

#define NR5_TIMERA_TACTL_OFFSET           0x00
#define NR5_TIMERA_TAR_OFFSET             0x04
#define NR5_TIMERA_TACCTL0_OFFSET         0x08
#define NR5_TIMERA_TACCR0_OFFSET          0x0C
#define NR5_TIMERA_TACCTL1_OFFSET         0x10
#define NR5_TIMERA_TACCR1_OFFSET          0x14
#define NR5_TIMERA_TACCTL2_OFFSET         0x18
#define NR5_TIMERA_TACCR2_OFFSET          0x1C
#define NR5_TIMERA_TAIV_OFFSET            0x20

/* Timer 1 address definitions */

#ifdef CONFIG_NR5_TIMER1
#define NR5_TIMER1_TACTL_OFFSET           (NR5_TIMER1_BASE+NR5_TIMERA_TACTL_OFFSET)
#define NR5_TIMER1_TAR_OFFSET             (NR5_TIMER1_BASE+NR5_TIMERA_TAR_OFFSET)
#define NR5_TIMER1_TACCTL0_OFFSET         (NR5_TIMER1_BASE+NR5_TIMERA_TACCTL0_OFFSET)
#define NR5_TIMER1_TACCR0_OFFSET          (NR5_TIMER1_BASE+NR5_TIMERA_TACCR0_OFFSET)
#define NR5_TIMER1_TACCTL1_OFFSET         (NR5_TIMER1_BASE+NR5_TIMERA_TACCTL1_OFFSET)
#define NR5_TIMER1_TACCR1_OFFSET          (NR5_TIMER1_BASE+NR5_TIMERA_TACCR1_OFFSET)
#define NR5_TIMER1_TACCTL2_OFFSET         (NR5_TIMER1_BASE+NR5_TIMERA_TACCTL2_OFFSET)
#define NR5_TIMER1_TACCR2_OFFSET          (NR5_TIMER1_BASE+NR5_TIMERA_TACCR2_OFFSET)
#define NR5_TIMER1_TAIV_OFFSET            (NR5_TIMER1_BASE+NR5_TIMERA_TAIV_OFFSET)
#endif

/* Timer 2 address definitions */

#ifdef CONFIG_NR5_TIMER2
#define NR5_TIMER2_TACTL_OFFSET           (NR5_TIMER2_BASE+NR5_TIMERA_TACTL_OFFSET)
#define NR5_TIMER2_TAR_OFFSET             (NR5_TIMER2_BASE+NR5_TIMERA_TAR_OFFSET)
#define NR5_TIMER2_TACCTL0_OFFSET         (NR5_TIMER2_BASE+NR5_TIMERA_TACCTL0_OFFSET)
#define NR5_TIMER2_TACCR0_OFFSET          (NR5_TIMER2_BASE+NR5_TIMERA_TACCR0_OFFSET)
#define NR5_TIMER2_TACCTL1_OFFSET         (NR5_TIMER2_BASE+NR5_TIMERA_TACCTL1_OFFSET)
#define NR5_TIMER2_TACCR1_OFFSET          (NR5_TIMER2_BASE+NR5_TIMERA_TACCR1_OFFSET)
#define NR5_TIMER2_TACCTL2_OFFSET         (NR5_TIMER2_BASE+NR5_TIMERA_TACCTL2_OFFSET)
#define NR5_TIMER2_TACCR2_OFFSET          (NR5_TIMER2_BASE+NR5_TIMERA_TACCR2_OFFSET)
#define NR5_TIMER2_TAIV_OFFSET            (NR5_TIMER2_BASE+NR5_TIMERA_TAIV_OFFSET)
#endif

/* Timer 3 address definitions */

#ifdef CONFIG_NR5_TIMER3
#define NR5_TIMER3_TACTL_OFFSET           (NR5_TIMER3_BASE+NR5_TIMERA_TACTL_OFFSET)
#define NR5_TIMER3_TAR_OFFSET             (NR5_TIMER3_BASE+NR5_TIMERA_TAR_OFFSET)
#define NR5_TIMER3_TACCTL0_OFFSET         (NR5_TIMER3_BASE+NR5_TIMERA_TACCTL0_OFFSET)
#define NR5_TIMER3_TACCR0_OFFSET          (NR5_TIMER3_BASE+NR5_TIMERA_TACCR0_OFFSET)
#define NR5_TIMER3_TACCTL1_OFFSET         (NR5_TIMER3_BASE+NR5_TIMERA_TACCTL1_OFFSET)
#define NR5_TIMER3_TACCR1_OFFSET          (NR5_TIMER3_BASE+NR5_TIMERA_TACCR1_OFFSET)
#define NR5_TIMER3_TACCTL2_OFFSET         (NR5_TIMER3_BASE+NR5_TIMERA_TACCTL2_OFFSET)
#define NR5_TIMER3_TACCR2_OFFSET          (NR5_TIMER3_BASE+NR5_TIMERA_TACCR2_OFFSET)
#define NR5_TIMER3_TAIV_OFFSET            (NR5_TIMER3_BASE+NR5_TIMERA_TAIV_OFFSET)
#endif

/* Timer 4 address definitions */

#ifdef CONFIG_NR5_TIMER4
#define NR5_TIMER4_TACTL_OFFSET           (NR5_TIMER4_BASE+NR5_TIMERA_TACTL_OFFSET)
#define NR5_TIMER4_TAR_OFFSET             (NR5_TIMER4_BASE+NR5_TIMERA_TAR_OFFSET)
#define NR5_TIMER4_TACCTL0_OFFSET         (NR5_TIMER4_BASE+NR5_TIMERA_TACCTL0_OFFSET)
#define NR5_TIMER4_TACCR0_OFFSET          (NR5_TIMER4_BASE+NR5_TIMERA_TACCR0_OFFSET)
#define NR5_TIMER4_TACCTL1_OFFSET         (NR5_TIMER4_BASE+NR5_TIMERA_TACCTL1_OFFSET)
#define NR5_TIMER4_TACCR1_OFFSET          (NR5_TIMER4_BASE+NR5_TIMERA_TACCR1_OFFSET)
#define NR5_TIMER4_TACCTL2_OFFSET         (NR5_TIMER4_BASE+NR5_TIMERA_TACCTL2_OFFSET)
#define NR5_TIMER4_TACCR2_OFFSET          (NR5_TIMER4_BASE+NR5_TIMERA_TACCR2_OFFSET)
#define NR5_TIMER4_TAIV_OFFSET            (NR5_TIMER4_BASE+NR5_TIMERA_TAIV_OFFSET)
#endif

/* Timer 5 address definitions */

#ifdef CONFIG_NR5_TIMER5
#define NR5_TIMER5_TACTL_OFFSET           (NR5_TIMER5_BASE+NR5_TIMERA_TACTL_OFFSET)
#define NR5_TIMER5_TAR_OFFSET             (NR5_TIMER5_BASE+NR5_TIMERA_TAR_OFFSET)
#define NR5_TIMER5_TACCTL0_OFFSET         (NR5_TIMER5_BASE+NR5_TIMERA_TACCTL0_OFFSET)
#define NR5_TIMER5_TACCR0_OFFSET          (NR5_TIMER5_BASE+NR5_TIMERA_TACCR0_OFFSET)
#define NR5_TIMER5_TACCTL1_OFFSET         (NR5_TIMER5_BASE+NR5_TIMERA_TACCTL1_OFFSET)
#define NR5_TIMER5_TACCR1_OFFSET          (NR5_TIMER5_BASE+NR5_TIMERA_TACCR1_OFFSET)
#define NR5_TIMER5_TACCTL2_OFFSET         (NR5_TIMER5_BASE+NR5_TIMERA_TACCTL2_OFFSET)
#define NR5_TIMER5_TACCR2_OFFSET          (NR5_TIMER5_BASE+NR5_TIMERA_TACCR2_OFFSET)
#define NR5_TIMER5_TAIV_OFFSET            (NR5_TIMER5_BASE+NR5_TIMERA_TAIV_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************/

/* Control register TACTL Bit definitions */

#define TIMERA_TACTL_TAIFG                (1 << 0)    /* Bit 0: Interrupt Pending Flag */
#define TIMERA_TACTL_TAIE                 (1 << 1)    /* Bit 1: Interrupt Enable */
#define TIMERA_TACTL_TACLR                (1 << 2)    /* Bit 2: TAR counter clear */
#define TIMERA_TACTL_MC_SHIFT             4           /* Bits 4-5: Mode Control */
#  define TIMERA_TACTL_MC_MASK            (3 << TIMERA_TACTL_MC_SHIFT)
#  define TIMERA_TACTL_STOP               (0 << TIMERA_TACTL_MC_SHIFT)
#  define TIMERA_TACTL_UP                 (1 << TIMERA_TACTL_MC_SHIFT)
#  define TIMERA_TACTL_CONTINUOUS         (2 << TIMERA_TACTL_MC_SHIFT)
#  define TIMERA_TACTL_UPDOWN             (3 << TIMERA_TACTL_MC_SHIFT)
#define TIMERA_TACTL_ID_SHIFT             6           /* Bits 6-7: Input Divider */
#  define TIMERA_TACTL_DIV_1              (0 << TIMERA_TACTL_ID_SHIFT)
#  define TIMERA_TACTL_DIV_2              (1 << TIMERA_TACTL_ID_SHIFT)
#  define TIMERA_TACTL_DIV_4              (2 << TIMERA_TACTL_ID_SHIFT)
#  define TIMERA_TACTL_DIV_8              (3 << TIMERA_TACTL_ID_SHIFT)
#define TIMERA_TACTL_TASSEL_SHIFT         8           /* Bits 8-9: Clock Source Select */
#  define TIMERA_TACTL_MED_CLOCK          (0 << TIMERA_TACTL_TASSEL_SHIFT)
#  define TIMERA_TACTL_SLOW_CLOCK         (1 << TIMERA_TACTL_TASSEL_SHIFT)
#  define TIMERA_TACTL_SYS_CLOCK          (2 << TIMERA_TACTL_TASSEL_SHIFT)
#  define TIMERA_TACTL_EXT_CLOCK          (3 << TIMERA_TACTL_TASSEL_SHIFT)
#define TIMERA_TACTL_TAPRE_SHIFT          10          /* Bits 10-15: Clock Prescaler */
#define TIMERA_TACTL_TAPRE_MASK           0x3F
#  define TIMERA_TACTL_TAPRE(x)           (((x) & TIMERA_TACTL_TAPRE_MASK) << TIMERA_TACTL_TAPRE_SHIFT)

/* Capture / Compare register bit definitions */

#define TIMERA_TACCTL_CCIFG               (1 << 0)    /* Bit 0: Capture/compare interrupt Flag */
#define TIMERA_TACCTL_COV                 (1 << 1)    /* Bit 1: Capture overflow */
#define TIMERA_TACCTL_OUTVAL              (1 << 2)    /* Bit 2: Output value */
#define TIMERA_TACCTL_CCI                 (1 << 3)    /* Bit 3: Capture/compare input value */
#define TIMERA_TACCTL_CCIE                (1 << 4)    /* Bit 4: Capture/Compare interrupt Enable */
#define TIMERA_TACCTL_OUTMOD_SHIFT        5           /* Bits 5-7: Output Mode */
#  define TIMERA_TACCTL_OUT               (0 << TIMERA_TACCTL_OUTMOD_SHIFT)
#  define TIMERA_TACCTL_SET               (1 << TIMERA_TACCTL_OUTMOD_SHIFT)
#  define TIMERA_TACCTL_TOGGLE_RESET      (2 << TIMERA_TACCTL_OUTMOD_SHIFT)
#  define TIMERA_TACCTL_SET_RESET         (3 << TIMERA_TACCTL_OUTMOD_SHIFT)
#  define TIMERA_TACCTL_TOGGLE            (4 << TIMERA_TACCTL_OUTMOD_SHIFT)
#  define TIMERA_TACCTL_RESET             (5 << TIMERA_TACCTL_OUTMOD_SHIFT)
#  define TIMERA_TACCTL_TOGGLE_SET        (6 << TIMERA_TACCTL_OUTMOD_SHIFT)
#  define TIMERA_TACCTL_RESET_SET         (7 << TIMERA_TACCTL_OUTMOD_SHIFT)
#define TIMERA_TACCTL_CAP                 (1 << 8)    /* Bit 8: Capture mode select  */
#define TIMERA_TACCTL_SCCI                (1 << 10)   /* Bit 10: Synchronized capture input */
#define TIMERA_TACCTL_SCS                 (1 << 11)   /* Bit 11: Synchronize capture source */
#define TIMERA_TACCTL_CCIS_SHIFT          12          /* Bits 12-13: Capture Input Select */
#  define TIMERA_TACCTL_CCIS_CCIA         (0 << TIMERA_TACCTL_CCIS_SHIFT)
#  define TIMERA_TACCTL_CCIS_CCIB         (1 << TIMERA_TACCTL_CCIS_SHIFT)
#  define TIMERA_TACCTL_CCIS_GND          (2 << TIMERA_TACCTL_CCIS_SHIFT)
#  define TIMERA_TACCTL_CCIS_VCC          (3 << TIMERA_TACCTL_CCIS_SHIFT)
#define TIMERA_TACCTL_CM_SHIFT            14          /* Bits 14-15: Capture Mode */
#  define TIMERA_TACCTL_CM_NO_CAPTURE     (0 << TIMERA_TACCTL_SM_SHIFT)
#  define TIMERA_TACCTL_CM_RISING         (1 << TIMERA_TACCTL_SM_SHIFT)
#  define TIMERA_TACCTL_CM_FALLING        (2 << TIMERA_TACCTL_SM_SHIFT)
#  define TIMERA_TACCTL_CM_BOTH           (3 << TIMERA_TACCTL_SM_SHIFT)

/* Interrupt Vector Register */

#define TIMERA_TAIV_TAIV_SHIFT            1           /* Bits 1-3: Interrupt Source */
#  define TIMERA_TAIV_TACCR1              (1 << TIMERA_TAIV_TAIV_SHIFT)
#  define TIMERA_TAIV_TACCR2              (2 << TIMERA_TAIV_TAIV_SHIFT)
#  define TIMERA_TAIV_TAIFG               (5 << TIMERA_TAIV_TAIV_SHIFT)

#endif /* _ARCH_RISCV_SRC_NR5M100_CHIP_NR5M1XX_TIMERA_H */
