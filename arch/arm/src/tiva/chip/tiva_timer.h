/************************************************************************************
 * arch/arm/src/tiva/chip/tiva_timer.h
 *
 * Originally:
 *
 *   Copyright (C) 2012, 2014 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <macscomp@gmail.com>
 *
 * Ongoing support and major revision to support the TM4C129 family
 * (essentially a full file replacement):
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_TIVA_CHIP_TIVA_TIMER_H
#define __ARCH_ARM_SRC_TIVA_CHIP_TIVA_TIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <arch/tiva/chip.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* GPTM register offsets ************************************************************/

#define TIVA_TIMER_CFG_OFFSET          0x0000 /* GPTM Configuration */
#define TIVA_TIMER_TAMR_OFFSET         0x0004 /* GPTM Timer A Mode */
#define TIVA_TIMER_TBMR_OFFSET         0x0008 /* GPTM Timer B Mode */
#define TIVA_TIMER_CTL_OFFSET          0x000c /* GPTM Control */

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER_SYNC_OFFSET       0x0010 /* GPTM Synchronize (GPTM0 only) */
#endif

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

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER_TAV_OFFSET        0x0050 /* GPTM Timer A Value */
#  define TIVA_TIMER_TBV_OFFSET        0x0054 /* GPTM Timer B Value */
#  define TIVA_TIMER_RTCPD_OFFSET      0x0058 /* GPTM RTC Predivide */
#  define TIVA_TIMER_TAPS_OFFSET       0x005c /* GPTM Timer A Prescale Snapshot */
#  define TIVA_TIMER_TBPS_OFFSET       0x0060 /* GPTM Timer B Prescale Snapshot */
#  define TIVA_TIMER_TAPV_OFFSET       0x0064 /* GPTM Timer A Prescale Value */
#  define TIVA_TIMER_TBPV_OFFSET       0x0068 /* GPTM Timer B Prescale Value */
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER_DMAEV_OFFSET      0x006c /* GPTM DMA Event */
#  define TIVA_TIMER_ADCEV_OFFSET      0x0070 /* GPTM ADC Event */
#endif

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER_PP_OFFSET         0x0fc0 /* GPTM Peripheral Properties */
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER_CC_OFFSET         0x0fc8 /* GPTM Clock Configuration */
#endif

/* GPTM register addresses **********************************************************/

#if TIVA_NTIMERS > 0
#define TIVA_TIMER0_CFG                (TIVA_TIMER0_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER0_TAMR               (TIVA_TIMER0_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER0_TBMR               (TIVA_TIMER0_BASE + TIVA_TIMER_TBMR_OFFSET)
#define TIVA_TIMER0_CTL                (TIVA_TIMER0_BASE + TIVA_TIMER_CTL_OFFSET)

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER0_SYNC             (TIVA_TIMER0_BASE + TIVA_TIMER_SYNC_OFFSET)
#endif

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

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER0_TAV              (TIVA_TIMER0_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER0_TBV              (TIVA_TIMER0_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER0_RTCPD            (TIVA_TIMER0_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER0_TAPS             (TIVA_TIMER0_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER0_TBPS             (TIVA_TIMER0_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER0_TAPV             (TIVA_TIMER0_BASE + TIVA_TIMER_TAPV_OFFSET)
#  define TIVA_TIMER0_TBPV             (TIVA_TIMER0_BASE + TIVA_TIMER_TBPV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER0_DMAEV            (TIVA_TIMER0_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER0_ADCEV            (TIVA_TIMER0_BASE + TIVA_TIMER_ADCEV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER0_PP               (TIVA_TIMER0_BASE + TIVA_TIMER_PP_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER0_CC               (TIVA_TIMER0_BASE + TIVA_TIMER_CC_OFFSET)
#endif
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

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER1_TAV              (TIVA_TIMER1_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER1_TBV              (TIVA_TIMER1_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER1_RTCPD            (TIVA_TIMER1_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER1_TAPS             (TIVA_TIMER1_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER1_TBPS             (TIVA_TIMER1_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER1_TAPV             (TIVA_TIMER1_BASE + TIVA_TIMER_TAPV_OFFSET)
#  define TIVA_TIMER1_TBPV             (TIVA_TIMER1_BASE + TIVA_TIMER_TBPV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER1_DMAEV            (TIVA_TIMER1_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER1_ADCEV            (TIVA_TIMER1_BASE + TIVA_TIMER_ADCEV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER1_PP               (TIVA_TIMER1_BASE + TIVA_TIMER_PP_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER1_CC               (TIVA_TIMER1_BASE + TIVA_TIMER_CC_OFFSET)
#endif
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

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER2_TAV              (TIVA_TIMER2_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER2_TBV              (TIVA_TIMER2_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER2_RTCPD            (TIVA_TIMER2_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER2_TAPS             (TIVA_TIMER2_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER2_TBPS             (TIVA_TIMER2_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER2_TAPV             (TIVA_TIMER2_BASE + TIVA_TIMER_TAPV_OFFSET)
#  define TIVA_TIMER2_TBPV             (TIVA_TIMER2_BASE + TIVA_TIMER_TBPV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER2_DMAEV            (TIVA_TIMER2_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER2_ADCEV            (TIVA_TIMER2_BASE + TIVA_TIMER_ADCEV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER2_PP               (TIVA_TIMER2_BASE + TIVA_TIMER_PP_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER2_CC               (TIVA_TIMER2_BASE + TIVA_TIMER_CC_OFFSET)
#endif
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

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER3_TAV              (TIVA_TIMER3_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER3_TBV              (TIVA_TIMER3_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER3_RTCPD            (TIVA_TIMER3_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER3_TAPS             (TIVA_TIMER3_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER3_TBPS             (TIVA_TIMER3_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER3_TAPV             (TIVA_TIMER3_BASE + TIVA_TIMER_TAPV_OFFSET)
#  define TIVA_TIMER3_TBPV             (TIVA_TIMER3_BASE + TIVA_TIMER_TBPV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER3_DMAEV            (TIVA_TIMER3_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER3_ADCEV            (TIVA_TIMER3_BASE + TIVA_TIMER_ADCEV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER3_PP               (TIVA_TIMER3_BASE + TIVA_TIMER_PP_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER3_CC               (TIVA_TIMER3_BASE + TIVA_TIMER_CC_OFFSET)
#endif
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

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER4_TAV              (TIVA_TIMER4_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER4_TBV              (TIVA_TIMER4_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER4_RTCPD            (TIVA_TIMER4_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER4_TAPS             (TIVA_TIMER4_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER4_TBPS             (TIVA_TIMER4_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER4_TAPV             (TIVA_TIMER4_BASE + TIVA_TIMER_TAPV_OFFSET)
#  define TIVA_TIMER4_TBPV             (TIVA_TIMER4_BASE + TIVA_TIMER_TBPV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER4_DMAEV            (TIVA_TIMER4_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER4_ADCEV            (TIVA_TIMER4_BASE + TIVA_TIMER_ADCEV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER4_PP               (TIVA_TIMER4_BASE + TIVA_TIMER_PP_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER4_CC               (TIVA_TIMER4_BASE + TIVA_TIMER_CC_OFFSET)
#endif
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

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER5_TAV              (TIVA_TIMER5_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER5_TBV              (TIVA_TIMER5_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER5_RTCPD            (TIVA_TIMER5_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER5_TAPS             (TIVA_TIMER5_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER5_TBPS             (TIVA_TIMER5_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER5_TAPV             (TIVA_TIMER5_BASE + TIVA_TIMER_TAPV_OFFSET)
#  define TIVA_TIMER5_TBPV             (TIVA_TIMER5_BASE + TIVA_TIMER_TBPV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER5_DMAEV            (TIVA_TIMER5_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER5_ADCEV            (TIVA_TIMER5_BASE + TIVA_TIMER_ADCEV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER5_PP               (TIVA_TIMER5_BASE + TIVA_TIMER_PP_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER5_CC               (TIVA_TIMER5_BASE + TIVA_TIMER_CC_OFFSET)
#endif
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

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER6_TAV              (TIVA_TIMER6_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER6_TBV              (TIVA_TIMER6_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER6_RTCPD            (TIVA_TIMER6_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER6_TAPS             (TIVA_TIMER6_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER6_TBPS             (TIVA_TIMER6_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER6_TAPV             (TIVA_TIMER6_BASE + TIVA_TIMER_TAPV_OFFSET)
#  define TIVA_TIMER6_TBPV             (TIVA_TIMER6_BASE + TIVA_TIMER_TBPV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER6_DMAEV            (TIVA_TIMER6_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER6_ADCEV            (TIVA_TIMER6_BASE + TIVA_TIMER_ADCEV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER6_PP               (TIVA_TIMER6_BASE + TIVA_TIMER_PP_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER6_CC               (TIVA_TIMER6_BASE + TIVA_TIMER_CC_OFFSET)
#endif
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

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER7_TAV              (TIVA_TIMER7_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER7_TBV              (TIVA_TIMER7_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER7_RTCPD            (TIVA_TIMER7_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER7_TAPS             (TIVA_TIMER7_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER7_TBPS             (TIVA_TIMER7_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER7_TAPV             (TIVA_TIMER7_BASE + TIVA_TIMER_TAPV_OFFSET)
#  define TIVA_TIMER7_TBPV             (TIVA_TIMER7_BASE + TIVA_TIMER_TBPV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER7_DMAEV            (TIVA_TIMER7_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER7_ADCEV            (TIVA_TIMER7_BASE + TIVA_TIMER_ADCEV_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIVA_TIMER7_PP               (TIVA_TIMER7_BASE + TIVA_TIMER_PP_OFFSET)
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIVA_TIMER7_CC               (TIVA_TIMER7_BASE + TIVA_TIMER_CC_OFFSET)
#endif
#endif /* TIVA_NTIMERS > 7 */

/* GPTM register bit definitions ****************************************************/
/* GPTM Configuration (CFG) */

#define TIMER_CFG_CFG_SHIFT            0         /* Bits 2-0:  Configuration */
#define TIMER__CFG_MASK                (7 << TIMER_CFG_CFG_SHIFT)
#  define TIMER_CFG_CFG_32             (0 << TIMER_CFG_CFG_SHIFT) /* 32-bit timer configuration */
#  define TIMER_CFG_CFG_RTC            (1 << TIMER_CFG_CFG_SHIFT) /* 32-bit real-time clock (RTC) counter configuration */
#  define TIMER_CFG_CFG_16             (4 << TIMER_CFG_CFG_SHIFT) /* 16-bit timer configuration */

/* GPTM Timer A/B Mode (TAMR and TBMR) */

#define TIMER_TnMR_TnMR_SHIFT          0         /* Bits 1-0:  Timer A/B Mode */
#define TIMER_TnMR_TnMR_MASK           (3 << TIMER_TnMR_TnMR_SHIFT)
#  define TIMER_TnMR_TnMR_ONESHOT      (1 << TIMER_TnMR_TnMR_SHIFT) /* One-Shot Timer mode */
#  define TIMER_TnMR_TnMR_PERIODIC     (2 << TIMER_TnMR_TnMR_SHIFT) /* Periodic Timer mode */
#  define TIMER_TnMR_TnMR_CAPTURE      (3 << TIMER_TnMR_TnMR_SHIFT) /* Capture mode */
#define TIMER_TnMR_TnCMR               (1 << 2)  /* Bit 2:  Timer A/B Capture Mode */
#  define TIMER_TnMR_TnCMR_EDGECOUNT   (0 << TIMER_TnMR_TnCMR_SHIFT) /* Edge-Count mode */
#  define TIMER_TnMR_TnCMR_EDGETIME    (1 << TIMER_TnMR_TnCMR_SHIFT) /* Edge-Time mode */
#define TIMER_TnMR_TnAMS               (1 << 3)  /* Bit 3:  Timer A/B Alternate Mode Select */
#  define TIMER_TnMR_TnAMS_CAPTURE     (0 << TIMER_TnMR_TnAMS_SHIFT) /* Capture mode is enabled */
#  define TIMER_TnMR_TnAMS_PWM         (1 << TIMER_TnMR_TnAMS_SHIFT) /* PWM mode is enabled */

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIMER_TnMR_TnCDIR            (1 << 4)  /* Bit 4:  Timer A/B Count Direction */
#    define TIMER_TnMR_TnCDIR_DOWN     (0)               /* Timer counts down */
#    define TIMER_TnMR_TnCDIR_UP       TIMER_TnMR_TnCDIR /* Timer counts up (one-shot/periodic modes) */
#  define TIMER_TnMR_TnMIE             (1 << 5)  /* Bit 5:  Timer A/B Match Interrupt Enable */
#  define TIMER_TnMR_TnWOT             (1 << 6)  /* Bit 6:  GPTM Timer A/B Wait-on-Trigger */
#  define TIMER_TnMR_TnSNAPS           (1 << 7)  /* Bit 7:  GPTM Timer A/B Snap-Shot Mode */
#  define TIMER_TnMR_TnILD             (1 << 8)  /* Bit 8:  GPTM Timer A/B Interval Load Write */
#  define TIMER_TnMR_TnPWMIE           (1 << 9)  /* Bit 9:  GPTM Timer A/B PWM Interrupt Enable */
#  define TIMER_TnMR_TnMRSU            (1 << 10) /* Bit 10: GPTM Timer A/B Match Register Update */
#  define TIMER_TnMR_TnPLO             (1 << 11) /* Bit 11: GPTM Timer A/B PWM Legacy Operation */
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIMER_TnMR_TnCINTD           (1 << 12) /* Bit 12: One-shot/Periodic Interrupt Disable */
#  define TIMER_TnMR_TCACT_SHIFT       (13)      /* Bits 13-15: Timer Compare Action Select */
#  define TIMER_TnMR_TCACT_MASK        (7 << TIMER_TnMR_TCACT_SHIFT)
#    define TIMER_TnMR_TCACT_NONE      (0 << TIMER_TnMR_TCACT_SHIFT) /* Disable compare operations */
#    define TIMER_TnMR_TCACT_TOGGLE    (1 << TIMER_TnMR_TCACT_SHIFT) /* Toggle state on timeout */
#    define TIMER_TnMR_TCACT_CLRTO     (2 << TIMER_TnMR_TCACT_SHIFT) /* Clear CCP on timeout */
#    define TIMER_TnMR_TCACT_SETTO     (3 << TIMER_TnMR_TCACT_SHIFT) /* Set CCP on timeout */
#    define TIMER_TnMR_TCACT_SETTOGTO  (4 << TIMER_TnMR_TCACT_SHIFT) /* Set CCP and toggle on TimeOut */
#    define TIMER_TnMR_TCACT_CLRTOGTO  (5 << TIMER_TnMR_TCACT_SHIFT) /* Clear CCP and toggle on TimeOut */
#    define TIMER_TnMR_TCACT_SETCLRTO  (6 << TIMER_TnMR_TCACT_SHIFT) /* Set CCP and clear on timeout */
#    define TIMER_TnMR_TCACT_CLRSETTO  (7 << TIMER_TnMR_TCACT_SHIFT) /* Clear CCP and set on timeout */
#endif

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

/* GPTM Synchronize (GPTM0 only) */

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIMER_SYNC_NONE              0         /* GPTMn is not affected */
#  define TIMER_SYNC_TA                1         /* Timer A timeout event triggered */
#  define TIMER_SYNC_TB                2         /* Timer B timeout event triggered */
#  define TIMER_SYNC_TATB              3         /* Both  Timer A/B timeout event triggered */

#  define TIMER_SYNC_SYNCT_SHIFT(i)    ((i) << 1) /* Synchronize GPTMi timer i */
#  define TIMER_SYNC_SYNCT_MASK(i)     (3 << TIMER_SYNC_SYNCT_SHIFT(i))
#    define TIMER_SYNC_SYNCT(i,n)      ((uint32_t)(n) << TIMER_SYNC_SYNCT_SHIFT(i))
#    define TIMER_SYNC_SYNCT_NONE(i)   TIMER_SYNC_SYNCT(i,TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCT_TA(i)     TIMER_SYNC_SYNCT(i,TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCT_TB(i)     TIMER_SYNC_SYNCT(i,TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCT_TATB(i)   TIMER_SYNC_SYNCT(i,TIMER_SYNC_TATB)

#  define TIMER_SYNC_SYNCT0_SHIFT      (0)       /* Bits 0-1: Synchronize GPTM timer 0 */
#  define TIMER_SYNC_SYNCT0_MASK       (3 << TIMER_SYNC_SYNCT0_SHIFT)
#    define TIMER_SYNC_SYNCT0(n)       ((uint32_t)(n) << TIMER_SYNC_SYNCT0_SHIFT)
#    define TIMER_SYNC_SYNCT0_NONE     TIMER_SYNC_SYNCT0(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCT0_TA       TIMER_SYNC_SYNCT0(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCT0_TB       TIMER_SYNC_SYNCT0(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCT0_TATB     TIMER_SYNC_SYNCT0(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCT1_SHIFT      (2)       /* Synchronize GPTM timer 1 */
#  define TIMER_SYNC_SYNCT1_MASK       (3 << TIMER_SYNC_SYNCT1_SHIFT)
#    define TIMER_SYNC_SYNCT1(n)       ((uint32_t)(n) << TIMER_SYNC_SYNCT1_SHIFT)
#    define TIMER_SYNC_SYNCT1_NONE     TIMER_SYNC_SYNCT1(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCT1_TA       TIMER_SYNC_SYNCT1(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCT1_TB       TIMER_SYNC_SYNCT1(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCT1_TATB     TIMER_SYNC_SYNCT1(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCT2_SHIFT      (4)       /* Synchronize GPTM timer 2 */
#  define TIMER_SYNC_SYNCT2_MASK       (3 << )
#    define TIMER_SYNC_SYNCT2(n)       ((uint32_t)(n) << TIMER_SYNC_SYNCT2_SHIFT)
#    define TIMER_SYNC_SYNCT2_NONE     TIMER_SYNC_SYNCT2(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCT2_TA       TIMER_SYNC_SYNCT2(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCT2_TB       TIMER_SYNC_SYNCT2(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCT2_TATB     TIMER_SYNC_SYNCT2(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCT3_SHIFT      (6)       /* Synchronize GPTM timer 3 */
#  define TIMER_SYNC_SYNCT3_MASK       (3 << )
#    define TIMER_SYNC_SYNCT3(n)       ((uint32_t)(n) << TIMER_SYNC_SYNCT3_SHIFT)
#    define TIMER_SYNC_SYNCT3_NONE     TIMER_SYNC_SYNCT3(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCT3_TA       TIMER_SYNC_SYNCT3(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCT3_TB       TIMER_SYNC_SYNCT3(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCT3_TATB     TIMER_SYNC_SYNCT3(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCT4_SHIFT      (8)       /* Synchronize GPTM timer 4 */
#  define TIMER_SYNC_SYNCT4_MASK       (3 << )
#    define TIMER_SYNC_SYNCT4(n)       ((uint32_t)(n) << TIMER_SYNC_SYNCT4_SHIFT)
#    define TIMER_SYNC_SYNCT4_NONE     TIMER_SYNC_SYNCT4(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCT4_TA       TIMER_SYNC_SYNCT4(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCT4_TB       TIMER_SYNC_SYNCT4(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCT4_TATB     TIMER_SYNC_SYNCT4(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCT5_SHIFT      (10)      /* Synchronize GPTM timer 5 */
#  define TIMER_SYNC_SYNCT5_MASK       (3 << )
#    define TIMER_SYNC_SYNCT5(n)       ((uint32_t)(n) << TIMER_SYNC_SYNCT5_SHIFT)
#    define TIMER_SYNC_SYNCT5_NONE     TIMER_SYNC_SYNCT5(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCT5_TA       TIMER_SYNC_SYNCT5(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCT5_TB       TIMER_SYNC_SYNCT5(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCT5_TATB     TIMER_SYNC_SYNCT5(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCT6_SHIFT      (12)      /* Synchronize GPTM timer 6 */
#  define TIMER_SYNC_SYNCT6_MASK       (3 << )
#    define TIMER_SYNC_SYNCT6(n)       ((uint32_t)(n) << TIMER_SYNC_SYNCT6_SHIFT)
#    define TIMER_SYNC_SYNCT6_NONE     TIMER_SYNC_SYNCT6(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCT6_TA       TIMER_SYNC_SYNCT6(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCT6_TB       TIMER_SYNC_SYNCT6(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCT6_TATB     TIMER_SYNC_SYNCT6(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCT7_SHIFT      (14)      /* Synchronize GPTM timer 7 */
#  define TIMER_SYNC_SYNCT7_MASK       (3 << )
#    define TIMER_SYNC_SYNCT7(n)       ((uint32_t)(n) << TIMER_SYNC_SYNCT7_SHIFT)
#    define TIMER_SYNC_SYNCT7_NONE     TIMER_SYNC_SYNCT7(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCT7_TA       TIMER_SYNC_SYNCT7(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCT7_TB       TIMER_SYNC_SYNCT7(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCT7_TATB     TIMER_SYNC_SYNCT7(TIMER_SYNC_TATB)

#  define TIMER_SYNC_SYNCWT_SHIFT(i)   ((i) << 1) /* Synchronize GPTMi 32/64-Bit Timer i */
#  define TIMER_SYNC_SYNCWT_MASK(i)    (3 << TIMER_SYNC_SYNCT_SHIFT(i))
#    define TIMER_SYNC_SYNCWT(i,n)     ((uint32_t)(n) << TIMER_SYNC_SYNCT_SHIFT(i))
#    define TIMER_SYNC_SYNCWT_NONE(i)  TIMER_SYNC_SYNCT(i,TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCWT_TA(i)    TIMER_SYNC_SYNCT(i,TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCWT_TB(i)    TIMER_SYNC_SYNCT(i,TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCWT_TATB(i)  TIMER_SYNC_SYNCT(i,TIMER_SYNC_TATB)

#  define TIMER_SYNC_SYNCWT0_SHIFT     (12)      /* Synchronize WTM 32/64-Bit wide timer 0 */
#  define TIMER_SYNC_SYNCWT0_MASK      (3 << TIMER_SYNC_SYNCWT0_SHIFT)
#    define TIMER_SYNC_SYNCWT0(n)      ((uint32_t)(n) << TIMER_SYNC_SYNCWT0_SHIFT)
#    define TIMER_SYNC_SYNCWT0_NONE    TIMER_SYNC_SYNCWT0(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCWT0_TA      TIMER_SYNC_SYNCWT0(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCWT0_TB      TIMER_SYNC_SYNCWT0(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCWT0_TATB    TIMER_SYNC_SYNCWT0(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCWT1_SHIFT     (14)      /* Synchronize WTM 32/64-Bit wide timer 1 */
#  define TIMER_SYNC_SYNCWT1_MASK      (3 << )
#    define TIMER_SYNC_SYNCWT1(n)      ((uint32_t)(n) << TIMER_SYNC_SYNCWT1_SHIFT)
#    define TIMER_SYNC_SYNCWT1_NONE    TIMER_SYNC_SYNCWT1(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCWT1_TA      TIMER_SYNC_SYNCWT1(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCWT1_TB      TIMER_SYNC_SYNCWT1(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCWT1_TATB    TIMER_SYNC_SYNCWT1(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCWT2_SHIFT     (16)      /* Synchronize WTM 32/64-Bit wide timer 2 */
#  define TIMER_SYNC_SYNCWT2_MASK      (3 << )
#    define TIMER_SYNC_SYNCWT2(n)      ((uint32_t)(n) << TIMER_SYNC_SYNCWT2_SHIFT)
#    define TIMER_SYNC_SYNCWT2_NONE    TIMER_SYNC_SYNCWT2(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCWT2_TA      TIMER_SYNC_SYNCWT2(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCWT2_TB      TIMER_SYNC_SYNCWT2(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCWT2_TATB    TIMER_SYNC_SYNCWT2(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCWT3_SHIFT     (18)      /* Synchronize WTM 32/64-Bit wide timer 3 */
#  define TIMER_SYNC_SYNCWT3_MASK      (3 << )
#    define TIMER_SYNC_SYNCWT3(n)      ((uint32_t)(n) << TIMER_SYNC_SYNCWT3_SHIFT)
#    define TIMER_SYNC_SYNCWT3_NONE    TIMER_SYNC_SYNCWT3(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCWT3_TA      TIMER_SYNC_SYNCWT3(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCWT3_TB      TIMER_SYNC_SYNCWT3(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCWT3_TATB    TIMER_SYNC_SYNCWT3(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCWT4_SHIFT     (20)      /* Synchronize WTM 32/64-Bit wide timer 4 */
#  define TIMER_SYNC_SYNCWT4_MASK      (3 << )
#    define TIMER_SYNC_SYNCWT4(n)      ((uint32_t)(n) << TIMER_SYNC_SYNCWT4_SHIFT)
#    define TIMER_SYNC_SYNCWT4_NONE    TIMER_SYNC_SYNCWT4(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCWT4_TA      TIMER_SYNC_SYNCWT4(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCWT4_TB      TIMER_SYNC_SYNCWT4(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCWT4_TATB    TIMER_SYNC_SYNCWT4(TIMER_SYNC_TATB)
#  define TIMER_SYNC_SYNCWT5_SHIFT     (22)      /* Synchronize WTM 32/64-Bit wide timer 5 */
#  define TIMER_SYNC_SYNCWT5_MASK      (3 << )
#    define TIMER_SYNC_SYNCWT5(n)      ((uint32_t)(n) << TIMER_SYNC_SYNCWT5_SHIFT)
#    define TIMER_SYNC_SYNCWT5_NONE    TIMER_SYNC_SYNCWT5(TIMER_SYNC_NONE)
#    define TIMER_SYNC_SYNCWT5_TA      TIMER_SYNC_SYNCWT5(TIMER_SYNC_TA)
#    define TIMER_SYNC_SYNCWT5_TB      TIMER_SYNC_SYNCWT5(TIMER_SYNC_TB)
#    define TIMER_SYNC_SYNCWT5_TATB    TIMER_SYNC_SYNCWT5(TIMER_SYNC_TATB)
#endif

/* Common bit definitions used with:
 *
 * - GPTM Interrupt Mask (IMR)
 * - GPTM Raw Interrupt Status (RIS)
 * - GPTM Masked Interrupt Status (MIS)
 * - GPTM Interrupt Clear (ICR)
 */

#define TIMER_INT_TATO                 (1 << 0)  /* Bit 0:  Timer A Time-Out Interrupt */

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIMER_INT_CAM                (1 << 1)  /* Bit 1:  GPTM Timer A Capture Mode Match Interrupt */
#  define TIMER_INT_CAE                (1 << 2)  /* Bit 2:  GPTM Timer A Capture Mode Event Interrupt */
#  define TIMER_INT_RTC                (1 << 3)  /* Bit 3:  GPTM RTC Interrupt */
#  define TIMER_INT_TAM                (1 << 4)  /* Bit 4:  GPTM Timer A Match Interrupt */
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIMER_INT_DMAA               (1 << 5)  /* Bit 5:  GPTM Timer A DMA Done Interrupt */
#endif

#define TIMER_INT_TBTO                 (1 << 8)  /* Bit 8:  GPTM Timer B Time-Out Interrupt */

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIMER_INT_CBM                (1 << 9)  /* Bit 9:  GPTM Timer B Capture Mode Match Interrupt */
#  define TIMER_INT_CBE                (1 << 10) /* Bit 10: GPTM Timer B Capture Mode Event Interrupt */
#  define TIMER_INT_TBM                (1 << 11) /* Bit 11: GPTM Timer B Match Interrupt */
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIMER_INT_DMAB               (1 << 13) /* Bit 13: GPTM Timer B DMA Done Interrupt */
#elif defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIMER_INT_WUE                (1 << 16) /* Bit 16: 32/64-Bit Wide GPTM Write Update Error Interrupt */
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIMERA_INTS                  0x0000003f
#  define TIMERB_INTS                  0x00002f00
#  define TIMER_ALLINTS                0x00002f3f
#elif defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIMERA_INTS                  0x0000001f
#  define TIMERB_INTS                  0x00000f00
#  define TIMER_ALLINTS                0x00010f1f
#else
#  define TIMERA_INTS                  0x00000001
#  define TIMERB_INTS                  0x00000100
#  define TIMER_ALLINTS                0x00000101
#endif

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
/* GPTM Timer A Value (TAV) (16/32-bit value) */
/* GPTM Timer B Value (TBV) (16/32-bit value) */

/* GPTM RTC Predivide (RTCPD) */

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIMER_RTCPD_SHIFT            (0)       /* Bits 0-15: RTC Predivide Counter Value */
#  define TIMER_RTCPD_MASK             (0xffff << TIMER_RTCPD_SHIFT)
#    define TIMER_RTCPD(n)             ((uint32_t)(n) << TIMER_RTCPD_SHIFT)
#endif

/* GPTM Timer A/B Prescale Snapshot (TnPS) */

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIMER_TnPS_PSS_SHIFT         (0)       /* Bits 0-15: GPTM Timer A/B Prescaler Snapshot */
#  define TIMER_TnPS_PSS_MASK          (0xffff << TIMER_TnPS_PSS_SHIFT)
#    define TIMER_TnPS_PSS(n)          ((uint32_t)(n) << TIMER_TnPS_PSS_SHIFT)
#endif

/* GPTM Timer A/B Prescale Value (TnPV) */

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIMER_TnPV_PSS_SHIFT         (0)       /* Bits 0-15: GPTM Timer A/B Prescaler Value */
#  define TIMER_TnPS_PSS_MASK          (0xffff << TIMER_TnPS_PSS_SHIFT)
#    define TIMER_TnPS_PSS(n)          ((uint32_t)(n) << TIMER_TnPS_PSS_SHIFT)
#endif

/* GPTM DMA Event (DMAEV) */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIMER_DMAEV_TATODMAEN        (1 << 0)  /* Bit 0:  GPTM A Time-Out Event DMA Trigger Enable */
#  define TIMER_DMAEV_CAMDMAEN         (1 << 1)  /* Bit 1:  GPTM A Capture Match Event DMA Trigger Enable */
#  define TIMER_DMAEV_CAEDMAEN         (1 << 2)  /* Bit 2:  GPTM A Capture Event DMA Trigger Enable */
#  define TIMER_DMAEV_RTCDMAEN         (1 << 3)  /* Bit 3:  GPTM A RTC Match Event DMA Trigger Enable */
#  define TIMER_DMAEV_TAMDMAEN         (1 << 4)  /* Bit 4:  GPTM A Mode Match Event DMA Trigger Enable */
#  define TIMER_DMAEV_TBTODMAEN        (1 << 8)  /* Bit 8:  GPTM B Time-Out Event DMA Trigger Enable */
#  define TIMER_DMAEV_CBMDMAEN         (1 << 9)  /* Bit 9:  GPTM B Capture Match Event DMA Trigger Enable */
#  define TIMER_DMAEV_CBEDMAEN         (1 << 10) /* Bit 10: GPTM B Capture Event DMA Trigger Enable */
#  define TIMER_DMAEV_TBMDMAEN         (1 << 11) /* Bit 11: GPTM B Mode Match Event DMA Trigger Enable */
#endif

/* GPTM ADC Event (ADCEV) */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIMER_ADCEV_TATOADCEN        (1 << 0)  /* Bit 0:  GPTM A Time-Out Event ADC Trigger Enable */
#  define TIMER_ADCEV_CAMADCEN         (1 << 1)  /* Bit 1:  GPTM A Capture Match Event ADC Trigger Enable */
#  define TIMER_ADCEV_CAEADCEN         (1 << 2)  /* Bit 2:  GPTM A Capture Event ADC Trigger Enable */
#  define TIMER_ADCEV_RTCADCEN         (1 << 3)  /* Bit 3:  GPTM RTC Match Event ADC Trigger Enable */
#  define TIMER_ADCEV_TAMADCEN         (1 << 4)  /* Bit 4:  GPTM A Mode Match Event ADC Trigger Enable */
#  define TIMER_ADCEV_TBTOADCEN        (1 << 8)  /* Bit 8:  GPTM B Time-Out Event ADC Trigger Enable */
#  define TIMER_ADCEV_CBMADCEN         (1 << 9)  /* Bit 9:  GPTM B Capture Match Event ADC Trigger Enable */
#  define TIMER_ADCEV_CBEADCEN         (1 << 10) /* Bit 10: GPTM B Capture Event ADC Trigger Enable */
#  define TIMER_ADCEV_TBMADCEN         (1 << 11) /* Bit 11: GPTM B Mode Match Event ADC Trigger Enable */
#endif

/* GPTM Peripheral Properties (PP) */

#if defined(CONFIG_ARCH_CHIP_LM4F) || defined(CONFIG_ARCH_CHIP_TM4C)
#  define TIMER_PP_SIZE_SHIFT          (0)       /* Bits 0-3: Count Size */
#  define TIMER_PP_SIZE_MASK           (15 << TIMER_PP_SIZE_SHIFT)
#    define TIMER_PP_SIZE_16           (0 << TIMER_PP_SIZE_SHIFT) /* Timer A/B 16 bits with 8-bit prescale */
#    define TIMER_PP_SIZE_32           (1 << TIMER_PP_SIZE_SHIFT) /* Timer A/B 32 bits with 16-bit prescale */
#endif

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIMER_PP_CHAIN               (1 << 4)  /* Bit 4:  Chain with Other Timers */
#  define TIMER_PP_SYNCCNT             (1 << 5)  /* Bit 5:  Synchronize Start */
#  define TIMER_PP_ALTCLK              (1 << 6)  /* Bit 6:  Alternate Clock Source */
#endif

/* GPTM Clock Configuration */

#if defined(CONFIG_ARCH_CHIP_TM4C129)
#  define TIMER_CC_ALTCLK              (1 << 0)  /* Bit 0: Alternate Clock Source */
#endif

#endif /* __ARCH_ARM_SRC_TIVA_CHIP_TIVA_TIMER_H */
