/************************************************************************************
 * arch/arm/src/tiva/hardware/lm/lm3s_timer.h
 *
 * Originally:
 *
 *   Copyright (C) 2012, 2014 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <macscomp@gmail.com>
 *
 * Ongoing support and major revision to support the TM4C129 family
 * (essentially a full file replacement):
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Some bitfield definitions taken from a header file provided by:
 *
 *   Copyright (C) 2014 TRD2 Inc. All rights reserved.
 *   Author: Calvin Maguranis <calvin.maguranis@trd2inc.com>
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM3S_TIMER_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM3S_TIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/tiva_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* GPTM register offsets ************************************************************/

#define TIVA_TIMER_CFG_OFFSET          0x0000 /* GPTM Configuration */
#define TIVA_TIMER_TAMR_OFFSET         0x0004 /* GPTM Timer A Mode */
#define TIVA_TIMER_TBMR_OFFSET         0x0008 /* GPTM Timer B Mode */
#define TIVA_TIMER_CTL_OFFSET          0x000c /* GPTM Control */
#define TIVA_TIMER_IMR_OFFSET          0x0018 /* GPTM Interrupt Mask */
#define TIVA_TIMER_RIS_OFFSET          0x001c /* GPTM Raw Interrupt Status */
#define TIVA_TIMER_MIS_OFFSET          0x0020 /* GPTM Masked Interrupt Status */
#define TIVA_TIMER_ICR_OFFSET          0x0024 /* GPTM Interrupt Clear */
#define TIVA_TIMER_TAILR_OFFSET        0x0028 /* GPTM Timer A Interval Load */
#define TIVA_TIMER_TBILR_OFFSET        0x002c /* GPTM Timer B Interval Load */
#define TIVA_TIMER_TAMATCHR_OFFSET     0x0030 /* GPTM Timer A Match */
#define TIVA_TIMER_TBMATCHR_OFFSET     0x0034 /* GPTM Timer B Match */
#define TIVA_TIMER_TAPR_OFFSET         0x0038 /* GPTM Timer A Prescale */
#define TIVA_TIMER_TBPR_OFFSET         0x003c /* GPTM Timer B Prescale */
#define TIVA_TIMER_TAPMR_OFFSET        0x0040 /* GPTM TimerA Prescale Match */
#define TIVA_TIMER_TBPMR_OFFSET        0x0044 /* GPTM TimerB Prescale Match */
#define TIVA_TIMER_TAR_OFFSET          0x0048 /* GPTM Timer A */
#define TIVA_TIMER_TBR_OFFSET          0x004c /* GPTM Timer B */

/* GPTM register addresses **********************************************************/

#if TIVA_NTIMERS > 0
#define TIVA_TIMER0_CFG                (TIVA_TIMER0_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER0_TAMR               (TIVA_TIMER0_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER0_TBMR               (TIVA_TIMER0_BASE + TIVA_TIMER_TBMR_OFFSET)
#define TIVA_TIMER0_CTL                (TIVA_TIMER0_BASE + TIVA_TIMER_CTL_OFFSET)
#define TIVA_TIMER0_IMR                (TIVA_TIMER0_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER0_RIS                (TIVA_TIMER0_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER0_MIS                (TIVA_TIMER0_BASE + TIVA_TIMER_MIS_OFFSET)
#define TIVA_TIMER0_ICR                (TIVA_TIMER0_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER0_TAILR              (TIVA_TIMER0_BASE + TIVA_TIMER_TAILR_OFFSET)
#define TIVA_TIMER0_TBILR              (TIVA_TIMER0_BASE + TIVA_TIMER_TBILR_OFFSET)
#define TIVA_TIMER0_TAMATCHR           (TIVA_TIMER0_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#define TIVA_TIMER0_TBMATCHR           (TIVA_TIMER0_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#define TIVA_TIMER0_TAPR               (TIVA_TIMER0_BASE + TIVA_TIMER_TAPR_OFFSET)
#define TIVA_TIMER0_TBPR               (TIVA_TIMER0_BASE + TIVA_TIMER_TBPR_OFFSET)
#define TIVA_TIMER0_TAPMR              (TIVA_TIMER0_BASE + TIVA_TIMER_TAPMR_OFFSET)
#define TIVA_TIMER0_TBPMR              (TIVA_TIMER0_BASE + TIVA_TIMER_TBPMR_OFFSET)
#define TIVA_TIMER0_TAR                (TIVA_TIMER0_BASE + TIVA_TIMER_TAR_OFFSET)
#define TIVA_TIMER0_TBR                (TIVA_TIMER0_BASE + TIVA_TIMER_TBR_OFFSET)
#endif /* TIVA_NTIMERS > 0 */

#if TIVA_NTIMERS > 1
#define TIVA_TIMER1_CFG                (TIVA_TIMER1_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER1_TAMR               (TIVA_TIMER1_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER1_TBMR               (TIVA_TIMER1_BASE + TIVA_TIMER_TBMR_OFFSET)
#define TIVA_TIMER1_CTL                (TIVA_TIMER1_BASE + TIVA_TIMER_CTL_OFFSET)
#define TIVA_TIMER1_IMR                (TIVA_TIMER1_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER1_RIS                (TIVA_TIMER1_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER1_MIS                (TIVA_TIMER1_BASE + TIVA_TIMER_MIS_OFFSET)
#define TIVA_TIMER1_ICR                (TIVA_TIMER1_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER1_TAILR              (TIVA_TIMER1_BASE + TIVA_TIMER_TAILR_OFFSET)
#define TIVA_TIMER1_TBILR              (TIVA_TIMER1_BASE + TIVA_TIMER_TBILR_OFFSET)
#define TIVA_TIMER1_TAMATCHR           (TIVA_TIMER1_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#define TIVA_TIMER1_TBMATCHR           (TIVA_TIMER1_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#define TIVA_TIMER1_TAPR               (TIVA_TIMER1_BASE + TIVA_TIMER_TAPR_OFFSET)
#define TIVA_TIMER1_TBPR               (TIVA_TIMER1_BASE + TIVA_TIMER_TBPR_OFFSET)
#define TIVA_TIMER1_TAPMR              (TIVA_TIMER1_BASE + TIVA_TIMER_TAPMR_OFFSET)
#define TIVA_TIMER1_TBPMR              (TIVA_TIMER1_BASE + TIVA_TIMER_TBPMR_OFFSET)
#define TIVA_TIMER1_TAR                (TIVA_TIMER1_BASE + TIVA_TIMER_TAR_OFFSET)
#define TIVA_TIMER1_TBR                (TIVA_TIMER1_BASE + TIVA_TIMER_TBR_OFFSET)
#endif /* TIVA_NTIMERS > 1 */

#if TIVA_NTIMERS > 2
#define TIVA_TIMER2_CFG                (TIVA_TIMER2_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER2_TAMR               (TIVA_TIMER2_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER2_TBMR               (TIVA_TIMER2_BASE + TIVA_TIMER_TBMR_OFFSET)
#define TIVA_TIMER2_CTL                (TIVA_TIMER2_BASE + TIVA_TIMER_CTL_OFFSET)
#define TIVA_TIMER2_IMR                (TIVA_TIMER2_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER2_RIS                (TIVA_TIMER2_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER2_MIS                (TIVA_TIMER2_BASE + TIVA_TIMER_MIS_OFFSET)
#define TIVA_TIMER2_ICR                (TIVA_TIMER2_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER2_TAILR              (TIVA_TIMER2_BASE + TIVA_TIMER_TAILR_OFFSET)
#define TIVA_TIMER2_TBILR              (TIVA_TIMER2_BASE + TIVA_TIMER_TBILR_OFFSET)
#define TIVA_TIMER2_TAMATCHR           (TIVA_TIMER2_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#define TIVA_TIMER2_TBMATCHR           (TIVA_TIMER2_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#define TIVA_TIMER2_TAPR               (TIVA_TIMER2_BASE + TIVA_TIMER_TAPR_OFFSET)
#define TIVA_TIMER2_TBPR               (TIVA_TIMER2_BASE + TIVA_TIMER_TBPR_OFFSET)
#define TIVA_TIMER2_TAPMR              (TIVA_TIMER2_BASE + TIVA_TIMER_TAPMR_OFFSET)
#define TIVA_TIMER2_TBPMR              (TIVA_TIMER2_BASE + TIVA_TIMER_TBPMR_OFFSET)
#define TIVA_TIMER2_TAR                (TIVA_TIMER2_BASE + TIVA_TIMER_TAR_OFFSET)
#define TIVA_TIMER2_TBR                (TIVA_TIMER2_BASE + TIVA_TIMER_TBR_OFFSET)
#endif /* TIVA_NTIMERS > 2 */

#if TIVA_NTIMERS > 3
#define TIVA_TIMER3_CFG                (TIVA_TIMER3_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER3_TAMR               (TIVA_TIMER3_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER3_TBMR               (TIVA_TIMER3_BASE + TIVA_TIMER_TBMR_OFFSET)
#define TIVA_TIMER3_CTL                (TIVA_TIMER3_BASE + TIVA_TIMER_CTL_OFFSET)
#define TIVA_TIMER3_IMR                (TIVA_TIMER3_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER3_RIS                (TIVA_TIMER3_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER3_MIS                (TIVA_TIMER3_BASE + TIVA_TIMER_MIS_OFFSET)
#define TIVA_TIMER3_ICR                (TIVA_TIMER3_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER3_TAILR              (TIVA_TIMER3_BASE + TIVA_TIMER_TAILR_OFFSET)
#define TIVA_TIMER3_TBILR              (TIVA_TIMER3_BASE + TIVA_TIMER_TBILR_OFFSET)
#define TIVA_TIMER3_TAMATCHR           (TIVA_TIMER3_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#define TIVA_TIMER3_TBMATCHR           (TIVA_TIMER3_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#define TIVA_TIMER3_TAPR               (TIVA_TIMER3_BASE + TIVA_TIMER_TAPR_OFFSET)
#define TIVA_TIMER3_TBPR               (TIVA_TIMER3_BASE + TIVA_TIMER_TBPR_OFFSET)
#define TIVA_TIMER3_TAPMR              (TIVA_TIMER3_BASE + TIVA_TIMER_TAPMR_OFFSET)
#define TIVA_TIMER3_TBPMR              (TIVA_TIMER3_BASE + TIVA_TIMER_TBPMR_OFFSET)
#define TIVA_TIMER3_TAR                (TIVA_TIMER3_BASE + TIVA_TIMER_TAR_OFFSET)
#define TIVA_TIMER3_TBR                (TIVA_TIMER3_BASE + TIVA_TIMER_TBR_OFFSET)
#endif /* TIVA_NTIMERS > 3 */

#if TIVA_NTIMERS > 4
#define TIVA_TIMER4_CFG                (TIVA_TIMER4_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER4_TAMR               (TIVA_TIMER4_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER4_TBMR               (TIVA_TIMER4_BASE + TIVA_TIMER_TBMR_OFFSET)
#define TIVA_TIMER4_CTL                (TIVA_TIMER4_BASE + TIVA_TIMER_CTL_OFFSET)
#define TIVA_TIMER4_IMR                (TIVA_TIMER4_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER4_RIS                (TIVA_TIMER4_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER4_MIS                (TIVA_TIMER4_BASE + TIVA_TIMER_MIS_OFFSET)
#define TIVA_TIMER4_ICR                (TIVA_TIMER4_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER4_TAILR              (TIVA_TIMER4_BASE + TIVA_TIMER_TAILR_OFFSET)
#define TIVA_TIMER4_TBILR              (TIVA_TIMER4_BASE + TIVA_TIMER_TBILR_OFFSET)
#define TIVA_TIMER4_TAMATCHR           (TIVA_TIMER4_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#define TIVA_TIMER4_TBMATCHR           (TIVA_TIMER4_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#define TIVA_TIMER4_TAPR               (TIVA_TIMER4_BASE + TIVA_TIMER_TAPR_OFFSET)
#define TIVA_TIMER4_TBPR               (TIVA_TIMER4_BASE + TIVA_TIMER_TBPR_OFFSET)
#define TIVA_TIMER4_TAPMR              (TIVA_TIMER4_BASE + TIVA_TIMER_TAPMR_OFFSET)
#define TIVA_TIMER4_TBPMR              (TIVA_TIMER4_BASE + TIVA_TIMER_TBPMR_OFFSET)
#define TIVA_TIMER4_TAR                (TIVA_TIMER4_BASE + TIVA_TIMER_TAR_OFFSET)
#define TIVA_TIMER4_TBR                (TIVA_TIMER4_BASE + TIVA_TIMER_TBR_OFFSET)
#endif /* TIVA_NTIMERS > 4 */

#if TIVA_NTIMERS > 5
#define TIVA_TIMER5_CFG                (TIVA_TIMER5_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER5_TAMR               (TIVA_TIMER5_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER5_TBMR               (TIVA_TIMER5_BASE + TIVA_TIMER_TBMR_OFFSET)
#define TIVA_TIMER5_CTL                (TIVA_TIMER5_BASE + TIVA_TIMER_CTL_OFFSET)
#define TIVA_TIMER5_IMR                (TIVA_TIMER5_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER5_RIS                (TIVA_TIMER5_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER5_MIS                (TIVA_TIMER5_BASE + TIVA_TIMER_MIS_OFFSET)
#define TIVA_TIMER5_ICR                (TIVA_TIMER5_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER5_TAILR              (TIVA_TIMER5_BASE + TIVA_TIMER_TAILR_OFFSET)
#define TIVA_TIMER5_TBILR              (TIVA_TIMER5_BASE + TIVA_TIMER_TBILR_OFFSET)
#define TIVA_TIMER5_TAMATCHR           (TIVA_TIMER5_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#define TIVA_TIMER5_TBMATCHR           (TIVA_TIMER5_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#define TIVA_TIMER5_TAPR               (TIVA_TIMER5_BASE + TIVA_TIMER_TAPR_OFFSET)
#define TIVA_TIMER5_TBPR               (TIVA_TIMER5_BASE + TIVA_TIMER_TBPR_OFFSET)
#define TIVA_TIMER5_TAPMR              (TIVA_TIMER5_BASE + TIVA_TIMER_TAPMR_OFFSET)
#define TIVA_TIMER5_TBPMR              (TIVA_TIMER5_BASE + TIVA_TIMER_TBPMR_OFFSET)
#define TIVA_TIMER5_TAR                (TIVA_TIMER5_BASE + TIVA_TIMER_TAR_OFFSET)
#define TIVA_TIMER5_TBR                (TIVA_TIMER5_BASE + TIVA_TIMER_TBR_OFFSET)
#endif /* TIVA_NTIMERS > 5 */

#if TIVA_NTIMERS > 6
#define TIVA_TIMER6_CFG                (TIVA_TIMER6_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER6_TAMR               (TIVA_TIMER6_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER6_TBMR               (TIVA_TIMER6_BASE + TIVA_TIMER_TBMR_OFFSET)
#define TIVA_TIMER6_CTL                (TIVA_TIMER6_BASE + TIVA_TIMER_CTL_OFFSET)
#define TIVA_TIMER6_IMR                (TIVA_TIMER6_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER6_RIS                (TIVA_TIMER6_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER6_MIS                (TIVA_TIMER6_BASE + TIVA_TIMER_MIS_OFFSET)
#define TIVA_TIMER6_ICR                (TIVA_TIMER6_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER6_TAILR              (TIVA_TIMER6_BASE + TIVA_TIMER_TAILR_OFFSET)
#define TIVA_TIMER6_TBILR              (TIVA_TIMER6_BASE + TIVA_TIMER_TBILR_OFFSET)
#define TIVA_TIMER6_TAMATCHR           (TIVA_TIMER6_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#define TIVA_TIMER6_TBMATCHR           (TIVA_TIMER6_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#define TIVA_TIMER6_TAPR               (TIVA_TIMER6_BASE + TIVA_TIMER_TAPR_OFFSET)
#define TIVA_TIMER6_TBPR               (TIVA_TIMER6_BASE + TIVA_TIMER_TBPR_OFFSET)
#define TIVA_TIMER6_TAPMR              (TIVA_TIMER6_BASE + TIVA_TIMER_TAPMR_OFFSET)
#define TIVA_TIMER6_TBPMR              (TIVA_TIMER6_BASE + TIVA_TIMER_TBPMR_OFFSET)
#define TIVA_TIMER6_TAR                (TIVA_TIMER6_BASE + TIVA_TIMER_TAR_OFFSET)
#define TIVA_TIMER6_TBR                (TIVA_TIMER6_BASE + TIVA_TIMER_TBR_OFFSET)
#endif /* TIVA_NTIMERS > 6 */

#if TIVA_NTIMERS > 7
#define TIVA_TIMER7_CFG                (TIVA_TIMER7_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER7_TAMR               (TIVA_TIMER7_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER7_TBMR               (TIVA_TIMER7_BASE + TIVA_TIMER_TBMR_OFFSET)
#define TIVA_TIMER7_CTL                (TIVA_TIMER7_BASE + TIVA_TIMER_CTL_OFFSET)
#define TIVA_TIMER7_IMR                (TIVA_TIMER7_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER7_RIS                (TIVA_TIMER7_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER7_MIS                (TIVA_TIMER7_BASE + TIVA_TIMER_MIS_OFFSET)
#define TIVA_TIMER7_ICR                (TIVA_TIMER7_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER7_TAILR              (TIVA_TIMER7_BASE + TIVA_TIMER_TAILR_OFFSET)
#define TIVA_TIMER7_TBILR              (TIVA_TIMER7_BASE + TIVA_TIMER_TBILR_OFFSET)
#define TIVA_TIMER7_TAMATCHR           (TIVA_TIMER7_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#define TIVA_TIMER7_TBMATCHR           (TIVA_TIMER7_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#define TIVA_TIMER7_TAPR               (TIVA_TIMER7_BASE + TIVA_TIMER_TAPR_OFFSET)
#define TIVA_TIMER7_TBPR               (TIVA_TIMER7_BASE + TIVA_TIMER_TBPR_OFFSET)
#define TIVA_TIMER7_TAPMR              (TIVA_TIMER7_BASE + TIVA_TIMER_TAPMR_OFFSET)
#define TIVA_TIMER7_TBPMR              (TIVA_TIMER7_BASE + TIVA_TIMER_TBPMR_OFFSET)
#define TIVA_TIMER7_TAR                (TIVA_TIMER7_BASE + TIVA_TIMER_TAR_OFFSET)
#define TIVA_TIMER7_TBR                (TIVA_TIMER7_BASE + TIVA_TIMER_TBR_OFFSET)
#endif /* TIVA_NTIMERS > 7 */

/* GPTM register bit definitions ****************************************************/
/* GPTM Configuration (CFG) */

#define TIMER_CFG_CFG_SHIFT            0         /* Bits 2-0:  Configuration */
#define TIMER__CFG_MASK                (7 << TIMER_CFG_CFG_SHIFT)
#  define TIMER_CFG_CFG_32             (0 << TIMER_CFG_CFG_SHIFT) /* 32-bit timer configuration */
#  define TIMER_CFG_CFG_RTC            (1 << TIMER_CFG_CFG_SHIFT) /* 32-bit real-time clock (RTC) counter configuration */
#  define TIMER_CFG_CFG_16             (4 << TIMER_CFG_CFG_SHIFT) /* 16-bit timer configuration */

/* GPTM Timer A/B Mode (TAMR and TBMR) */

#define TIMER_TnMR_TnMR_SHIFT          (0)       /* Bits 1-0:  Timer A/B Mode */
#define TIMER_TnMR_TnMR_MASK           (3 << TIMER_TnMR_TnMR_SHIFT) /* Bits 1-0:  Timer A/B Mode */
#  define TIMER_TnMR_TnMR_ONESHOT      (1 << TIMER_TnMR_TnMR_SHIFT) /* One-Shot Timer mode */
#  define TIMER_TnMR_TnMR_PERIODIC     (2 << TIMER_TnMR_TnMR_SHIFT) /* Periodic Timer mode */
#  define TIMER_TnMR_TnMR_CAPTURE      (3 << TIMER_TnMR_TnMR_SHIFT) /* Capture mode */
#define TIMER_TnMR_TnCMR_SHIFT         (2)       /* Bit 2:  Timer A/B Capture Mode */
#define TIMER_TnMR_TnCMR               (1 << TIMER_TnMR_TnCMR_SHIFT) /* Bit 2:  Timer A/B Capture Mode */
#  define TIMER_TnMR_TnCMR_EDGECOUNT   (0 << TIMER_TnMR_TnCMR_SHIFT) /* Edge-Count mode */
#  define TIMER_TnMR_TnCMR_EDGETIME    (1 << TIMER_TnMR_TnCMR_SHIFT) /* Edge-Time mode */
#define TIMER_TnMR_TnAMS_SHIFT         (3)       /* Bit 3:  Timer A/B Alternate Mode Select */
#define TIMER_TnMR_TnAMS               (1 << TIMER_TnMR_TnAMS_SHIFT) /* Bit 3:  Timer A/B Alternate Mode Select */
#  define TIMER_TnMR_TnAMS_CAPTURE     (0 << TIMER_TnMR_TnAMS_SHIFT) /* Capture mode is enabled */
#  define TIMER_TnMR_TnAMS_PWM         (1 << TIMER_TnMR_TnAMS_SHIFT) /* PWM mode is enabled */

/* GPTM Control (CTL) */

#define TIMER_CTL_TAEN                 (1 << 0)  /* Bit 0:  Timer A Enable */
#define TIMER_CTL_TASTALL              (1 << 1)  /* Bit 1:  Timer A Stall Enable */
#define TIMER_CTL_TAEVENT_SHIFT        (2)       /* Bits 2-3: GPTM Timer A Event Mode */
#define TIMER_CTL_TAEVENT_MASK         (3 << TIMER_CTL_TAEVENT_SHIFT)
#  define TIMER_CTL_TAEVENT_POS        (0 << TIMER_CTL_TAEVENT_SHIFT) /* Positive edge */
#  define TIMER_CTL_TAEVENT_NEG        (1 << TIMER_CTL_TAEVENT_SHIFT) /* Negative edge */
#  define TIMER_CTL_TAEVENT_BOTH       (3 << TIMER_CTL_TAEVENT_SHIFT) /* Both edges */
#define TIMER_CTL_RTCEN                (1 << 4)  /* Bit 4:  GPTM RTC Stall Enable */
#define TIMER_CTL_TAOTE                (1 << 5)  /* Bit 5:  GPTM Timer A Output Trigger Enable */
#define TIMER_CTL_TAPWML               (1 << 6)  /* Bit 6:  GPTM Timer A PWM Output Level */
#define TIMER_CTL_TBEN                 (1 << 8)  /* Bit 8:  GPTM Timer B Enable */
#define TIMER_CTL_TBSTALL              (1 << 9)  /* Bit 9:  GPTM Timer B Stall Enable */
#define TIMER_CTL_TBEVENT_SHFIT        (10)      /* Bits 10-11: GPTM Timer B Event Mode */
#define TIMER_CTL_TBEVENT_MASK         (3 << TIMER_CTL_TBEVENT_SHFIT)
#  define TIMER_CTL_TBEVENT_POS        (0 << TIMER_CTL_TBEVENT_SHFIT) /* Positive edge */
#  define TIMER_CTL_TBEVENT_NEG        (1 << TIMER_CTL_TBEVENT_SHFIT) /* Negative edge */
#  define TIMER_CTL_TBEVENT_BOTH       (3 << TIMER_CTL_TBEVENT_SHFIT) /* Both edges */
#define TIMER_CTL_TBOTE                (1 << 13) /* Bit 13: GPTM Timer B Output Trigger Enable */
#define TIMER_CTL_TBPWML               (1 << 14) /* Bit 14: GPTM Timer B PWM Output Level */

/* Common bit definitions used with:
 *
 * - GPTM Interrupt Mask (IMR)
 * - GPTM Raw Interrupt Status (RIS)
 * - GPTM Masked Interrupt Status (MIS)
 * - GPTM Interrupt Clear (ICR)
 */

#define TIMER_INT_TATO                 (1 << 0)  /* Bit 0:  Timer A Time-Out Interrupt */
#define TIMER_INT_TBTO                 (1 << 8)  /* Bit 8:  GPTM Timer B Time-Out Interrupt */
#define TIMERA_INTS                    0x00000001
#define TIMERB_INTS                    0x00000100
#define TIMER_ALLINTS                  0x00000101

/* GPTM Timer A Interval Load (TAILR) (32-bit value) */
/* GPTM Timer B Interval Load (TBILR) (32-bit value) */
/* GPTM Timer A Match (TAMATCHR) (32-bit value) */
/* GPTM Timer B Match (TBMATCHR) (32-bit value) */

/* GPTM Timer A/B Prescale (TnPR) */

#define TIMER_TnPR_TnPSR_SHIFT         (0)       /* Bits 0-8: GPTM Timer A/B Prescale */
#define TIMER_TnPR_TnPSR_MASK          (0xff << TIMER_TnPR_TnPSR_SHIFT)
#  define TIMER_TnPR_TnPSR(n)          ((uint32_t)(n) << TIMER_TnPR_TnPSR_SHIFT)

/* GPTM Timer A/B Prescale Match (TnPMR) */

#define TIMER_TnPMR_TnPSMR_SHIFT       (0)       /* Bits 0-8:  GPTM Timer A/B Prescale Match */
#define TIMER_TnPMR_TnPSMR_MASK        (0xff << TIMER_TnPMR_TnPSMR_SHIFT)
#  define TIMER_TnPMR_TnPSMR(n)        ((uint32_t)(n) << TIMER_TnPMR_TnPSMR_SHIFT)

/* GPTM Timer A (TAR) (16/32-bit value) */
/* GPTM Timer B (TBR) (16/32-bit value) */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_LM_LM3S_TIMER_H */
