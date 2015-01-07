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

#define TIVA_TIMER_CFG_OFFSET        0x0000 /* GPTM Configuration */
#define TIVA_TIMER_TAMR_OFFSET       0x0004 /* GPTM Timer A Mode */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_TBMR_OFFSET     0x0008 /* GPTM Timer B Mode */
#endif

#define TIVA_TIMER_CTL_OFFSET        0x000c /* GPTM Control */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_SYNC_OFFSET     0x0010 /* GPTM Synchronize */
#endif

#define TIVA_TIMER_IMR_OFFSET        0x0018 /* GPTM Interrupt Mask */
#define TIVA_TIMER_RIS_OFFSET        0x001c /* GPTM Raw Interrupt Status */
#define TIVA_TIMER_ICR_OFFSET        0x0024 /* GPTM Interrupt Clear */
#define TIVA_TIMER_TAILR_OFFSET      0x0028 /* GPTM Timer A Interval Load */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_TBILR_OFFSET    0x002c /* GPTM Timer B Interval Load */
#  define TIVA_TIMER_TAMATCHR_OFFSET 0x0030 /* GPTM Timer A Match */
#  define TIVA_TIMER_TBMATCHR_OFFSET 0x0034 /* GPTM Timer B Match */
#  define TIVA_TIMER_TAPR_OFFSET     0x0038 /* GPTM Timer A Prescale */
#  define TIVA_TIMER_TBPR_OFFSET     0x003c /* GPTM Timer B Prescale */
#  define TIVA_TIMER_TAPMR_OFFSET    0x0040 /* GPTM TimerA Prescale Match */
#  define TIVA_TIMER_TBPMR_OFFSET    0x0044 /* GPTM TimerB Prescale Match */
#endif

#define TIVA_TIMER_TAR_OFFSET        0x0048 /* GPTM Timer A */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_TBR_OFFSET      0x004c /* GPTM Timer B */
#  define TIVA_TIMER_TAV_OFFSET      0x0050 /* GPTM Timer A Value */
#  define TIVA_TIMER_TBV_OFFSET      0x0054 /* GPTM Timer B Value */
#  define TIVA_TIMER_RTCPD_OFFSET    0x0058 /* GPTM RTC Predivide */
#  define TIVA_TIMER_TAPS_OFFSET     0x005c /* GPTM Timer A Prescale Snapshot */
#  define TIVA_TIMER_TBPS_OFFSET     0x0060 /* GPTM Timer B Prescale Snapshot */
#  define TIVA_TIMER_DMAEV_OFFSET    0x006c /* GPTM DMA Event */
#  define TIVA_TIMER_ADCEV_OFFSET    0x0070 /* GPTM ADC Event */
#  define TIVA_TIMER_PP_OFFSET       0x0fc0 /* GPTM Peripheral Properties */
#  define TIVA_TIMER_CC_OFFSET       0x0fc8 /* GPTM Clock Configuration */
#endif

/* GPTM register addresses **********************************************************/

#if TIVA_NTIMERS > 0
#define TIVA_TIMER0_CFG              (TIVA_TIMER0_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER0_TAMR             (TIVA_TIMER0_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER0_CTL              (TIVA_TIMER0_BASE + TIVA_TIMER_CTL_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER0_TBMR           (TIVA_TIMER0_BASE + TIVA_TIMER_TBMR_OFFSET)
#endif

#define TIVA_TIMER0_IMR              (TIVA_TIMER0_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER0_RIS              (TIVA_TIMER0_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER0_ICR              (TIVA_TIMER0_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER0_TAILR            (TIVA_TIMER0_BASE + TIVA_TIMER_TAILR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER0_TBILR          (TIVA_TIMER0_BASE + TIVA_TIMER_TBILR_OFFSET)
#  define TIVA_TIMER0_TAMATCHR       (TIVA_TIMER0_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#  define TIVA_TIMER0_TBMATCHR       (TIVA_TIMER0_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#  define TIVA_TIMER0_TAPR           (TIVA_TIMER0_BASE + TIVA_TIMER_TAPR_OFFSET)
#  define TIVA_TIMER0_TBPR           (TIVA_TIMER0_BASE + TIVA_TIMER_TBPR_OFFSET)
#  define TIVA_TIMER0_TAPMR          (TIVA_TIMER0_BASE + TIVA_TIMER_TAPMR_OFFSET)
#  define TIVA_TIMER0_TBPMR          (TIVA_TIMER0_BASE + TIVA_TIMER_TBPMR_OFFSET)
#endif

#define TIVA_TIMER0_TAR              (TIVA_TIMER0_BASE + TIVA_TIMER_TAR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER0_TBR            (TIVA_TIMER0_BASE + TIVA_TIMER_TBR_OFFSET)
#  define TIVA_TIMER0_TAV            (TIVA_TIMER0_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER0_TBV            (TIVA_TIMER0_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER0_RTCPD          (TIVA_TIMER0_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER0_TAPS           (TIVA_TIMER0_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER0_TBPS           (TIVA_TIMER0_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER0_DMAEV          (TIVA_TIMER0_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER0_ADCEV          (TIVA_TIMER0_BASE + TIVA_TIMER_ADCEV_OFFSET)
#  define TIVA_TIMER0_PP             (TIVA_TIMER0_BASE + TIVA_TIMER_PP_OFFSET)
#  define TIVA_TIMER0_CC             (TIVA_TIMER0_BASE + TIVA_TIMER_CC_OFFSET)
#endif
#endif /* TIVA_NTIMERS > 0 */

#if TIVA_NTIMERS > 1
#define TIVA_TIMER1_CFG              (TIVA_TIMER1_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER1_TAMR             (TIVA_TIMER1_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER1_CTL              (TIVA_TIMER1_BASE + TIVA_TIMER_CTL_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER1_TBMR           (TIVA_TIMER1_BASE + TIVA_TIMER_TBMR_OFFSET)
#endif

#define TIVA_TIMER1_IMR              (TIVA_TIMER1_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER1_RIS              (TIVA_TIMER1_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER1_ICR              (TIVA_TIMER1_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER1_TAILR            (TIVA_TIMER1_BASE + TIVA_TIMER_TAILR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER1_TBILR          (TIVA_TIMER1_BASE + TIVA_TIMER_TBILR_OFFSET)
#  define TIVA_TIMER1_TAMATCHR       (TIVA_TIMER1_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#  define TIVA_TIMER1_TBMATCHR       (TIVA_TIMER1_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#  define TIVA_TIMER1_TAPR           (TIVA_TIMER1_BASE + TIVA_TIMER_TAPR_OFFSET)
#  define TIVA_TIMER1_TBPR           (TIVA_TIMER1_BASE + TIVA_TIMER_TBPR_OFFSET)
#  define TIVA_TIMER1_TAPMR          (TIVA_TIMER1_BASE + TIVA_TIMER_TAPMR_OFFSET)
#  define TIVA_TIMER1_TBPMR          (TIVA_TIMER1_BASE + TIVA_TIMER_TBPMR_OFFSET)
#endif

#define TIVA_TIMER1_TAR              (TIVA_TIMER1_BASE + TIVA_TIMER_TAR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER1_TBR            (TIVA_TIMER1_BASE + TIVA_TIMER_TBR_OFFSET)
#  define TIVA_TIMER1_TAV            (TIVA_TIMER1_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER1_TBV            (TIVA_TIMER1_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER1_RTCPD          (TIVA_TIMER1_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER1_TAPS           (TIVA_TIMER1_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER1_TBPS           (TIVA_TIMER1_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER1_DMAEV          (TIVA_TIMER1_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER1_ADCEV          (TIVA_TIMER1_BASE + TIVA_TIMER_ADCEV_OFFSET)
#  define TIVA_TIMER1_PP             (TIVA_TIMER1_BASE + TIVA_TIMER_PP_OFFSET)
#  define TIVA_TIMER1_CC             (TIVA_TIMER1_BASE + TIVA_TIMER_CC_OFFSET)
#endif
#endif /* TIVA_NTIMERS > 1 */

#if TIVA_NTIMERS > 2
#define TIVA_TIMER2_CFG              (TIVA_TIMER2_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER2_TAMR             (TIVA_TIMER2_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER2_CTL              (TIVA_TIMER2_BASE + TIVA_TIMER_CTL_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER2_TBMR           (TIVA_TIMER2_BASE + TIVA_TIMER_TBMR_OFFSET)
#endif

#define TIVA_TIMER2_IMR              (TIVA_TIMER2_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER2_RIS              (TIVA_TIMER2_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER2_ICR              (TIVA_TIMER2_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER2_TAILR            (TIVA_TIMER2_BASE + TIVA_TIMER_TAILR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER2_TBILR          (TIVA_TIMER2_BASE + TIVA_TIMER_TBILR_OFFSET)
#  define TIVA_TIMER2_TAMATCHR       (TIVA_TIMER2_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#  define TIVA_TIMER2_TBMATCHR       (TIVA_TIMER2_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#  define TIVA_TIMER2_TAPR           (TIVA_TIMER2_BASE + TIVA_TIMER_TAPR_OFFSET)
#  define TIVA_TIMER2_TBPR           (TIVA_TIMER2_BASE + TIVA_TIMER_TBPR_OFFSET)
#  define TIVA_TIMER2_TAPMR          (TIVA_TIMER2_BASE + TIVA_TIMER_TAPMR_OFFSET)
#  define TIVA_TIMER2_TBPMR          (TIVA_TIMER2_BASE + TIVA_TIMER_TBPMR_OFFSET)
#endif

#define TIVA_TIMER2_TAR              (TIVA_TIMER2_BASE + TIVA_TIMER_TAR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER2_TBR            (TIVA_TIMER2_BASE + TIVA_TIMER_TBR_OFFSET)
#  define TIVA_TIMER2_TAV            (TIVA_TIMER2_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER2_TBV            (TIVA_TIMER2_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER2_RTCPD          (TIVA_TIMER2_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER2_TAPS           (TIVA_TIMER2_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER2_TBPS           (TIVA_TIMER2_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER2_DMAEV          (TIVA_TIMER2_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER2_ADCEV          (TIVA_TIMER2_BASE + TIVA_TIMER_ADCEV_OFFSET)
#  define TIVA_TIMER2_PP             (TIVA_TIMER2_BASE + TIVA_TIMER_PP_OFFSET)
#  define TIVA_TIMER2_CC             (TIVA_TIMER2_BASE + TIVA_TIMER_CC_OFFSET)
#endif
#endif /* TIVA_NTIMERS > 2 */

#if TIVA_NTIMERS > 3
#define TIVA_TIMER3_CFG              (TIVA_TIMER3_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER3_TAMR             (TIVA_TIMER3_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER3_CTL              (TIVA_TIMER3_BASE + TIVA_TIMER_CTL_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER3_TBMR           (TIVA_TIMER3_BASE + TIVA_TIMER_TBMR_OFFSET)
#endif

#define TIVA_TIMER3_IMR              (TIVA_TIMER3_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER3_RIS              (TIVA_TIMER3_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER3_ICR              (TIVA_TIMER3_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER3_TAILR            (TIVA_TIMER3_BASE + TIVA_TIMER_TAILR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER3_TBILR          (TIVA_TIMER3_BASE + TIVA_TIMER_TBILR_OFFSET)
#  define TIVA_TIMER3_TAMATCHR       (TIVA_TIMER3_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#  define TIVA_TIMER3_TBMATCHR       (TIVA_TIMER3_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#  define TIVA_TIMER3_TAPR           (TIVA_TIMER3_BASE + TIVA_TIMER_TAPR_OFFSET)
#  define TIVA_TIMER3_TBPR           (TIVA_TIMER3_BASE + TIVA_TIMER_TBPR_OFFSET)
#  define TIVA_TIMER3_TAPMR          (TIVA_TIMER3_BASE + TIVA_TIMER_TAPMR_OFFSET)
#  define TIVA_TIMER3_TBPMR          (TIVA_TIMER3_BASE + TIVA_TIMER_TBPMR_OFFSET)
#endif

#define TIVA_TIMER3_TAR              (TIVA_TIMER3_BASE + TIVA_TIMER_TAR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER3_TBR            (TIVA_TIMER3_BASE + TIVA_TIMER_TBR_OFFSET)
#  define TIVA_TIMER3_TAV            (TIVA_TIMER3_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER3_TBV            (TIVA_TIMER3_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER3_RTCPD          (TIVA_TIMER3_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER3_TAPS           (TIVA_TIMER3_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER3_TBPS           (TIVA_TIMER3_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER3_DMAEV          (TIVA_TIMER3_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER3_ADCEV          (TIVA_TIMER3_BASE + TIVA_TIMER_ADCEV_OFFSET)
#  define TIVA_TIMER3_PP             (TIVA_TIMER3_BASE + TIVA_TIMER_PP_OFFSET)
#  define TIVA_TIMER3_CC             (TIVA_TIMER3_BASE + TIVA_TIMER_CC_OFFSET)
#endif
#endif /* TIVA_NTIMERS > 3 */

#if TIVA_NTIMERS > 4
#define TIVA_TIMER4_CFG              (TIVA_TIMER4_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER4_TAMR             (TIVA_TIMER4_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER4_CTL              (TIVA_TIMER4_BASE + TIVA_TIMER_CTL_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER4_TBMR           (TIVA_TIMER4_BASE + TIVA_TIMER_TBMR_OFFSET)
#endif

#define TIVA_TIMER4_IMR              (TIVA_TIMER4_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER4_RIS              (TIVA_TIMER4_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER4_ICR              (TIVA_TIMER4_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER4_TAILR            (TIVA_TIMER4_BASE + TIVA_TIMER_TAILR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER4_TBILR          (TIVA_TIMER4_BASE + TIVA_TIMER_TBILR_OFFSET)
#  define TIVA_TIMER4_TAMATCHR       (TIVA_TIMER4_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#  define TIVA_TIMER4_TBMATCHR       (TIVA_TIMER4_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#  define TIVA_TIMER4_TAPR           (TIVA_TIMER4_BASE + TIVA_TIMER_TAPR_OFFSET)
#  define TIVA_TIMER4_TBPR           (TIVA_TIMER4_BASE + TIVA_TIMER_TBPR_OFFSET)
#  define TIVA_TIMER4_TAPMR          (TIVA_TIMER4_BASE + TIVA_TIMER_TAPMR_OFFSET)
#  define TIVA_TIMER4_TBPMR          (TIVA_TIMER4_BASE + TIVA_TIMER_TBPMR_OFFSET)
#endif

#define TIVA_TIMER4_TAR              (TIVA_TIMER4_BASE + TIVA_TIMER_TAR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER4_TBR            (TIVA_TIMER4_BASE + TIVA_TIMER_TBR_OFFSET)
#  define TIVA_TIMER4_TAV            (TIVA_TIMER4_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER4_TBV            (TIVA_TIMER4_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER4_RTCPD          (TIVA_TIMER4_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER4_TAPS           (TIVA_TIMER4_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER4_TBPS           (TIVA_TIMER4_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER4_DMAEV          (TIVA_TIMER4_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER4_ADCEV          (TIVA_TIMER4_BASE + TIVA_TIMER_ADCEV_OFFSET)
#  define TIVA_TIMER4_PP             (TIVA_TIMER4_BASE + TIVA_TIMER_PP_OFFSET)
#  define TIVA_TIMER4_CC             (TIVA_TIMER4_BASE + TIVA_TIMER_CC_OFFSET)
#endif
#endif /* TIVA_NTIMERS > 4 */

#if TIVA_NTIMERS > 5
#define TIVA_TIMER5_CFG              (TIVA_TIMER5_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER5_TAMR             (TIVA_TIMER5_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER5_CTL              (TIVA_TIMER5_BASE + TIVA_TIMER_CTL_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER5_TBMR           (TIVA_TIMER5_BASE + TIVA_TIMER_TBMR_OFFSET)
#endif

#define TIVA_TIMER5_IMR              (TIVA_TIMER5_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER5_RIS              (TIVA_TIMER5_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER5_ICR              (TIVA_TIMER5_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER5_TAILR            (TIVA_TIMER5_BASE + TIVA_TIMER_TAILR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER5_TBILR          (TIVA_TIMER5_BASE + TIVA_TIMER_TBILR_OFFSET)
#  define TIVA_TIMER5_TAMATCHR       (TIVA_TIMER5_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#  define TIVA_TIMER5_TBMATCHR       (TIVA_TIMER5_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#  define TIVA_TIMER5_TAPR           (TIVA_TIMER5_BASE + TIVA_TIMER_TAPR_OFFSET)
#  define TIVA_TIMER5_TBPR           (TIVA_TIMER5_BASE + TIVA_TIMER_TBPR_OFFSET)
#  define TIVA_TIMER5_TAPMR          (TIVA_TIMER5_BASE + TIVA_TIMER_TAPMR_OFFSET)
#  define TIVA_TIMER5_TBPMR          (TIVA_TIMER5_BASE + TIVA_TIMER_TBPMR_OFFSET)
#endif

#define TIVA_TIMER5_TAR              (TIVA_TIMER5_BASE + TIVA_TIMER_TAR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER5_TBR            (TIVA_TIMER5_BASE + TIVA_TIMER_TBR_OFFSET)
#  define TIVA_TIMER5_TAV            (TIVA_TIMER5_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER5_TBV            (TIVA_TIMER5_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER5_RTCPD          (TIVA_TIMER5_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER5_TAPS           (TIVA_TIMER5_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER5_TBPS           (TIVA_TIMER5_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER5_DMAEV          (TIVA_TIMER5_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER5_ADCEV          (TIVA_TIMER5_BASE + TIVA_TIMER_ADCEV_OFFSET)
#  define TIVA_TIMER5_PP             (TIVA_TIMER5_BASE + TIVA_TIMER_PP_OFFSET)
#  define TIVA_TIMER5_CC             (TIVA_TIMER5_BASE + TIVA_TIMER_CC_OFFSET)
#endif
#endif /* TIVA_NTIMERS > 5 */

#if TIVA_NTIMERS > 6
#define TIVA_TIMER6_CFG              (TIVA_TIMER6_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER6_TAMR             (TIVA_TIMER6_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER6_CTL              (TIVA_TIMER6_BASE + TIVA_TIMER_CTL_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER6_TBMR           (TIVA_TIMER6_BASE + TIVA_TIMER_TBMR_OFFSET)
#endif

#define TIVA_TIMER6_IMR              (TIVA_TIMER6_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER6_RIS              (TIVA_TIMER6_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER6_ICR              (TIVA_TIMER6_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER6_TAILR            (TIVA_TIMER6_BASE + TIVA_TIMER_TAILR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER6_TBILR          (TIVA_TIMER6_BASE + TIVA_TIMER_TBILR_OFFSET)
#  define TIVA_TIMER6_TAMATCHR       (TIVA_TIMER6_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#  define TIVA_TIMER6_TBMATCHR       (TIVA_TIMER6_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#  define TIVA_TIMER6_TAPR           (TIVA_TIMER6_BASE + TIVA_TIMER_TAPR_OFFSET)
#  define TIVA_TIMER6_TBPR           (TIVA_TIMER6_BASE + TIVA_TIMER_TBPR_OFFSET)
#  define TIVA_TIMER6_TAPMR          (TIVA_TIMER6_BASE + TIVA_TIMER_TAPMR_OFFSET)
#  define TIVA_TIMER6_TBPMR          (TIVA_TIMER6_BASE + TIVA_TIMER_TBPMR_OFFSET)
#endif

#define TIVA_TIMER6_TAR              (TIVA_TIMER6_BASE + TIVA_TIMER_TAR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER6_TBR            (TIVA_TIMER6_BASE + TIVA_TIMER_TBR_OFFSET)
#  define TIVA_TIMER6_TAV            (TIVA_TIMER6_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER6_TBV            (TIVA_TIMER6_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER6_RTCPD          (TIVA_TIMER6_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER6_TAPS           (TIVA_TIMER6_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER6_TBPS           (TIVA_TIMER6_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER6_DMAEV          (TIVA_TIMER6_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER6_ADCEV          (TIVA_TIMER6_BASE + TIVA_TIMER_ADCEV_OFFSET)
#  define TIVA_TIMER6_PP             (TIVA_TIMER6_BASE + TIVA_TIMER_PP_OFFSET)
#  define TIVA_TIMER6_CC             (TIVA_TIMER6_BASE + TIVA_TIMER_CC_OFFSET)
#endif
#endif /* TIVA_NTIMERS > 6 */

#if TIVA_NTIMERS > 7
#define TIVA_TIMER7_CFG              (TIVA_TIMER7_BASE + TIVA_TIMER_CFG_OFFSET)
#define TIVA_TIMER7_TAMR             (TIVA_TIMER7_BASE + TIVA_TIMER_TAMR_OFFSET)
#define TIVA_TIMER7_CTL              (TIVA_TIMER7_BASE + TIVA_TIMER_CTL_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER7_TBMR           (TIVA_TIMER7_BASE + TIVA_TIMER_TBMR_OFFSET)
#endif

#define TIVA_TIMER7_IMR              (TIVA_TIMER7_BASE + TIVA_TIMER_IMR_OFFSET)
#define TIVA_TIMER7_RIS              (TIVA_TIMER7_BASE + TIVA_TIMER_RIS_OFFSET)
#define TIVA_TIMER7_ICR              (TIVA_TIMER7_BASE + TIVA_TIMER_ICR_OFFSET)
#define TIVA_TIMER7_TAILR            (TIVA_TIMER7_BASE + TIVA_TIMER_TAILR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER7_TBILR          (TIVA_TIMER7_BASE + TIVA_TIMER_TBILR_OFFSET)
#  define TIVA_TIMER7_TAMATCHR       (TIVA_TIMER7_BASE + TIVA_TIMER_TAMATCHR_OFFSET)
#  define TIVA_TIMER7_TBMATCHR       (TIVA_TIMER7_BASE + TIVA_TIMER_TBMATCHR_OFFSET)
#  define TIVA_TIMER7_TAPR           (TIVA_TIMER7_BASE + TIVA_TIMER_TAPR_OFFSET)
#  define TIVA_TIMER7_TBPR           (TIVA_TIMER7_BASE + TIVA_TIMER_TBPR_OFFSET)
#  define TIVA_TIMER7_TAPMR          (TIVA_TIMER7_BASE + TIVA_TIMER_TAPMR_OFFSET)
#  define TIVA_TIMER7_TBPMR          (TIVA_TIMER7_BASE + TIVA_TIMER_TBPMR_OFFSET)
#endif

#define TIVA_TIMER7_TAR              (TIVA_TIMER7_BASE + TIVA_TIMER_TAR_OFFSET)

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER7_TBR            (TIVA_TIMER7_BASE + TIVA_TIMER_TBR_OFFSET)
#  define TIVA_TIMER7_TAV            (TIVA_TIMER7_BASE + TIVA_TIMER_TAV_OFFSET)
#  define TIVA_TIMER7_TBV            (TIVA_TIMER7_BASE + TIVA_TIMER_TBV_OFFSET)
#  define TIVA_TIMER7_RTCPD          (TIVA_TIMER7_BASE + TIVA_TIMER_RTCPD_OFFSET)
#  define TIVA_TIMER7_TAPS           (TIVA_TIMER7_BASE + TIVA_TIMER_TAPS_OFFSET)
#  define TIVA_TIMER7_TBPS           (TIVA_TIMER7_BASE + TIVA_TIMER_TBPS_OFFSET)
#  define TIVA_TIMER7_DMAEV          (TIVA_TIMER7_BASE + TIVA_TIMER_DMAEV_OFFSET)
#  define TIVA_TIMER7_ADCEV          (TIVA_TIMER7_BASE + TIVA_TIMER_ADCEV_OFFSET)
#  define TIVA_TIMER7_PP             (TIVA_TIMER7_BASE + TIVA_TIMER_PP_OFFSET)
#  define TIVA_TIMER7_CC             (TIVA_TIMER7_BASE + TIVA_TIMER_CC_OFFSET)
#endif
#endif /* TIVA_NTIMERS > 7 */

/* GPTM register bit definitions ****************************************************/
/* GPTM Configuration (CFG) */

#define TIMER_CFG_CFG_SHIFT          0         /* Bits 2-0:  Configuration */
#define TIMER__CFG_MASK              (7 << TIMER_CFG_CFG_SHIFT)
#  define TIMER_CFG_CFG_32           (0 << TIMER_CFG_CFG_SHIFT) /* 32-bit timer configuration */
#  define TIMER_CFG_CFG_RTC          (1 << TIMER_CFG_CFG_SHIFT) /* 32-bit real-time clock (RTC) counter configuration */
#  define TIMER_CFG_CFG_16           (1 << TIMER_CFG_CFG_SHIFT) /* 16-bit timer configuration */

/* GPTM Timer A Mode (TAMR) */

#define TIMER_TAMR_TAMR_SHIFT        0         /* Bits 1-0:  Timer A Mode */
#define TIMER_TAMR_TAMR_MASK         (3 << TIMER_TAMR_TAMR_SHIFT)
#  define TIMER_TAMR_TAMR_ONESHOT    (1 << TIMER_TAMR_TAMR_SHIFT) /* One-Shot Timer mode */
#  define TIMER_TAMR_TAMR_PERIODIC   (2 << TIMER_TAMR_TAMR_SHIFT) /* Periodic Timer mode */
#  define TIMER_TAMR_TAMR_CAPTURE    (3 << TIMER_TAMR_TAMR_SHIFT) /* Capture mode */
#define TIMER_TAMR_TACMR             (1 << 2)  /* Bit 2:  Timer A Capture Mode */
#  define TIMER_TAMR_TACMR_EDGECOUNT (0 << TIMER_TAMR_TACMR_SHIFT) /* Edge-Count mode */
#  define TIMER_TAMR_TACMR_EDGETIME  (1 << TIMER_TAMR_TACMR_SHIFT) /* Edge-Time mode */
#define TIMER_TAMR_TAAMS             (1 << 3)  /* Bit 3:  Timer A Alternate Mode Select */
#  define TIMER_TAMR_TAAMS_CAPTURE   (0 << TIMER_TAMR_TAAMS_SHIFT) /* Capture mode is enabled */
#  define TIMER_TAMR_TAAMS_PWM       (1 << TIMER_TAMR_TAAMS_SHIFT) /* PWM mode is enabled */
#define TIMER_TAMR_TACDIR            (1 << 4)  /* Bit 4:  Timer A Count Direction */
#  define TIMER_TAMR_TACDIR_DOWN     (0 << TIMER_TAMR_TACDIR_SHIFT) /* The timer counts down */
#  define TIMER_TAMR_TACDIR_UP       (1 << TIMER_TAMR_TACDIR_SHIFT) /* When in one-shot or periodic mode, the timer counts up */
#define TIMER_TAMR_TAMIE             (1 << 5)  /* Bit 5:  Timer A Match Interrupt Enable */

/* GPTM Timer B Mode (TBMR) */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIMER_TBMR_
#endif

/* GPTM Control (CTL) */

#define TIMER_CTL_TAEN               (1 << 0)  /* Bit 0:  Timer A Enable */
#define TIMER_CTL_TASTALL_SHIFT      (1 << 1)  /* Bit 1:  Timer A Stall Enable */

/* GPTM Interrupt Mask (IMR) */

#define TIMER_IMR_TATOIM_SHIFT       (1 << 0)  /* Bit 0:  Timer A Time-Out Interrupt Mask */

/* GPTM Raw Interrupt Status (RIS) */

#define TIMER_RIS_TATORIS_SHIFT      (1 << 0)  /* Bit 0:  Timer A Time-Out Raw Interrupt */

/* GPTM Interrupt Clear (ICR) */

#define TIMER_ICR_TATOCINT_SHIFT     (1 << 0)  /* Bit 0:  Timer A Time-Out Raw Interrupt Clear*/

/* GPTM Timer B Interval Load */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIMER_TBILR_
#endif

/* GPTM Timer A Match */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIMER_TAMATCHR_
#endif

/* GPTM Timer B Match */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIMER_TBMATCHR_
#endif

/* GPTM Timer A Prescale */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIMER_TAPR_
#endif

/* GPTM Timer B Prescale */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIMER_TBPR_
#endif

/* GPTM TimerA Prescale Match */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIMER_TAPMR_
#endif

/* GPTM TimerB Prescale Match */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIMER_TBPMR_
#endif

/* GPTM Timer A (TAR) */
#define TIMER_TAR_

/* GPTM Timer B (TBR) */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIMER_TBR_
#endif

/* GPTM Timer A Value */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_TAV_
#endif

/* GPTM Timer B Value */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_TBV_
#endif

/* GPTM RTC Predivide */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_RTCPD_
#endif

/* GPTM Timer A Prescale Snapshot */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_TAPS_
#endif

/* GPTM Timer B Prescale Snapshot */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_TBPS_
#endif

/* GPTM DMA Event */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_DMAEV_
#endif

/* GPTM ADC Event */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_ADCEV_
#endif

/* GPTM Peripheral Properties */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_PP_
#endif

/* GPTM Clock Configuration */

#if defined(CONFIG_ARCH_CHIP_TM4C129XNC)
#  define TIVA_TIMER_CC_
#endif

#endif /* __ARCH_ARM_SRC_TIVA_CHIP_TIVA_TIMER_H */
