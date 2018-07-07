/************************************************************************************
 * arch/arm/src/stm32/stm32_hrtim.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Mateusz Szafoni <raiden00@railab.me>
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_HRTIM_H
#define __ARCH_ARM_SRC_STM32_STM32_HRTIM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "chip.h"

#ifdef CONFIG_STM32_HRTIM1

#if defined(CONFIG_STM32_STM32F33XX)
#  include "chip/stm32f33xxx_hrtim.h"
#  include "chip/stm32f33xxx_rcc.h"
#else
#  error
#endif

/************************************************************************************
 * Pre-processor definitions
 ************************************************************************************/

#if defined(CONFIG_STM32_HRTIM_TIMA) || defined(CONFIG_STM32_HRTIM_TIMB) || \
    defined(CONFIG_STM32_HRTIM_TIMC) || defined(CONFIG_STM32_HRTIM_TIMD) || \
    defined(CONFIG_STM32_HRTIM_TIME)
#  define HRTIM_HAVE_SLAVE 1
#endif

#if defined(CONFIG_STM32_HRTIM_TIMA_PWM) || defined(CONFIG_STM32_HRTIM_TIMB_PWM) || \
    defined(CONFIG_STM32_HRTIM_TIMC_PWM) || defined(CONFIG_STM32_HRTIM_TIMD_PWM) || \
    defined(CONFIG_STM32_HRTIM_TIME_PWM)
#   ifndef CONFIG_STM32_HRTIM_PWM
#     error "CONFIG_STM32_HRTIM_PWM must be set"
#   endif
#endif

#if defined(CONFIG_STM32_HRTIM_TIMA_CAP) || defined(CONFIG_STM32_HRTIM_TIMB_CAP) || \
    defined(CONFIG_STM32_HRTIM_TIMC_CAP) || defined(CONFIG_STM32_HRTIM_TIMD_CAP) || \
    defined(CONFIG_STM32_HRTIM_TIME_CAP)
#   ifndef CONFIG_STM32_HRTIM_CAPTURE
#     error "CONFIG_STM32_HRTIM_CAPTURE must be set"
#   endif
#endif

#if defined(CONFIG_STM32_HRTIM_TIMA_DT) || defined(CONFIG_STM32_HRTIM_TIMB_DT) || \
    defined(CONFIG_STM32_HRTIM_TIMC_DT) || defined(CONFIG_STM32_HRTIM_TIMD_DT) || \
    defined(CONFIG_STM32_HRTIM_TIME_DT)
#   ifndef CONFIG_STM32_HRTIM_DEADTIME
#     error "CONFIG_STM32_HRTIM_DEADTIME must be set"
#   endif
#endif

#if defined(CONFIG_STM32_HRTIM_TIMA_CHOP) || defined(CONFIG_STM32_HRTIM_TIMB_CHOP) || \
    defined(CONFIG_STM32_HRTIM_TIMC_CHOP) || defined(CONFIG_STM32_HRTIM_TIMD_CHOP) || \
    defined(CONFIG_STM32_HRTIM_TIME_CHOP)
#   ifndef CONFIG_STM32_HRTIM_CHOPPER
#     error "CONFIG_STM32_HRTIM_CHOPPER must be set"
#   endif
#endif

#if defined(CONFIG_STM32_HRTIM_TIMA_BURST) || defined(CONFIG_STM32_HRTIM_TIMB_BURST) || \
    defined(CONFIG_STM32_HRTIM_TIMC_BURST) || defined(CONFIG_STM32_HRTIM_TIMD_BURST) || \
    defined(CONFIG_STM32_HRTIM_TIME_BURST)
#   ifndef CONFIG_STM32_HRTIM_BURST
#     error "CONFIG_STM32_HRTIM_BURST must be set"
#   endif
#endif

#if defined(CONFIG_STM32_HRTIM_SCOUT) || defined(CONFIG_STM32_HRTIM_SCIN)
#   ifndef CONFIG_STM32_HRTIM_SYNC
#     error "CONFIG_STM32_HRTIM_SYNC must be set"
#   endif
#endif

#if defined(CONFIG_STM32_HRTIM_FAULT1) || defined(CONFIG_STM32_HRTIM_FAULT2) || \
    defined(CONFIG_STM32_HRTIM_FAULT3) || defined(CONFIG_STM32_HRTIM_FAULT4) || \
    defined(CONFIG_STM32_HRTIM_FAULT5)
#   ifndef CONFIG_STM32_HRTIM_FAULTS
#     error "CONFIG_STM32_HRTIM_FAULTS must be set"
#   endif
#endif

#if defined(CONFIG_STM32_HRTIM_EEV1) || defined(CONFIG_STM32_HRTIM_EEV2) || \
    defined(CONFIG_STM32_HRTIM_EEV3) || defined(CONFIG_STM32_HRTIM_EEV4) || \
    defined(CONFIG_STM32_HRTIM_EEV5) || defined(CONFIG_STM32_HRTIM_EEV6) || \
    defined(CONFIG_STM32_HRTIM_EEV7) || defined(CONFIG_STM32_HRTIM_EEV8) || \
    defined(CONFIG_STM32_HRTIM_EEV9) || defined(CONFIG_STM32_HRTIM_EEV10)
#   ifndef CONFIG_STM32_HRTIM_EVENTS
#     error "CONFIG_STM32_HRTIM_EVENTS must be set"
#   endif
#endif

#if defined(CONFIG_STM32_HRTIM_MASTER_IRQ) || defined(CONFIG_STM32_HRTIM_TIMA_IRQ) || \
    defined(CONFIG_STM32_HRTIM_TIMB_IRQ)   || defined(CONFIG_STM32_HRTIM_TIMC_IRQ) || \
    defined(CONFIG_STM32_HRTIM_TIMD_IRQ)   || defined(CONFIG_STM32_HRTIM_TIME_IRQ) || \
    defined(CONFIG_STM32_HRTIM_CMN_IRQ)
#   ifndef CONFIG_STM32_HRTIM_INTERRUPTS
#     error "CONFIG_STM32_HRTIM_INTERRUPTS must be set"
#   endif
#endif

#if defined(CONFIG_STM32_HRTIM_ADC_TRG1) || defined(CONFIG_STM32_HRTIM_ADC_TRG2) || \
    defined(CONFIG_STM32_HRTIM_ADC_TRG3) || defined(CONFIG_STM32_HRTIM_ADC_TRG4)
#   ifndef CONFIG_STM32_HRTIM_ADC
#     error "CONFIG_STM32_HRTIM_ADC must be set"
#   endif
#endif

/* TIMX PWM configuration checking */

#ifdef CONFIG_STM32_HRTIM_TIMA_PWM
#  if !defined(CONFIG_STM32_HRTIM_TIMA_PWM_CH1) && \
      !defined(CONFIG_STM32_HRTIM_TIMA_PWM_CH2)
#    error "HRTIM TIMA PWM set but no channel selected"
#  endif
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB_PWM
#  if !defined(CONFIG_STM32_HRTIM_TIMB_PWM_CH1) &&  \
      !defined(CONFIG_STM32_HRTIM_TIMB_PWM_CH2)
#    error "HRTIM TIMB PWM set but no channel selected"
#  endif
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC_PWM
#  if !defined(CONFIG_STM32_HRTIM_TIMC_PWM_CH1) &&  \
      !defined(CONFIG_STM32_HRTIM_TIMC_PWM_CH2)
#    error "HRTIM TIMC PWM set but no channel selected"
#  endif
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD_PWM
#  if !defined(CONFIG_STM32_HRTIM_TIMD_PWM_CH1) &&  \
      !defined(CONFIG_STM32_HRTIM_TIMD_PWM_CH2)
#    error "HRTIM TIMD PWM set but no channel selected"
#  endif
#endif
#ifdef CONFIG_STM32_HRTIM_TIME_PWM
#  if !defined(CONFIG_STM32_HRTIM_TIME_PWM_CH1) &&  \
      !defined(CONFIG_STM32_HRTIM_TIME_PWM_CH2)
#    error "HRTIM TIME PWM set but no channel selected"
#  endif
#endif

/* HRTIM clock source configuration */

#ifdef CONFIG_STM32_HRTIM_CLK_FROM_PLL
#  if STM32_SYSCLK_SW == RCC_CFGR_SW_PLL
#    if (STM32_RCC_CFGR_PPRE2 != RCC_CFGR_PPRE2_HCLK) &&  \
        (STM32_RCC_CFGR_PPRE2 != RCC_CFGR_PPRE2_HCLKd2)
#      error "APB2 prescaler factor can not be greater than 2"
#    else
#      define HRTIM_HAVE_CLK_FROM_PLL 1
#      define HRTIM_MAIN_CLOCK 2*STM32_PLL_FREQUENCY
#    endif
#  else
#    error "Clock system must be set to PLL"
#  endif
#else
#  error "Not supported yet: system freezes when no PLL selected."
#  define HRTIM_HAVE_CLK_FROM_APB2 1
#  if STM32_RCC_CFGR_PPRE2 == RCC_CFGR_PPRE2_HCLK
#      define HRTIM_MAIN_CLOCK STM32_PCLK2_FREQUENCY
#  else
#      define HRTIM_MAIN_CLOCK 2*STM32_PCLK2_FREQUENCY
#  endif
#endif

/* High-resolution equivalent clock */

#define HRTIM_CLOCK (HRTIM_MAIN_CLOCK*32ull)

/* Helpers **************************************************************************/

#define HRTIM_CMP_SET(hrtim, tim, index, cmp)               \
        (hrtim)->hd_ops->cmp_update(hrtim, tim, index, cmp)
#define HRTIM_PER_SET(hrtim, tim, per)                      \
        (hrtim)->hd_ops->per_update(hrtim, tim, per)
#define HRTIM_REP_SET(hrtim, tim, per)                      \
        (hrtim)->hd_ops->rep_update(hrtim, tim, per)
#define HRTIM_PER_GET(hrtim, tim)                           \
        (hrtim)->hd_ops->per_get(hrtim, tim)
#define HRTIM_FCLK_GET(hrtim, tim)                          \
        (hrtim)->hd_ops->fclk_get(hrtim, tim)
#define HRTIM_IRQ_GET(hrtim, irq)                           \
        (hrtim)->hd_ops->irq_get(hrtim, irq)
#define HRTIM_CAPTURE_GET(hrtim, timer, cap)                \
        (hrtim)->hd_ops->capture_get(hrtim, timer, cap)
#define HRTIM_IRQ_ACK(hrtim, irq, ack)                      \
        (hrtim)->hd_ops->irq_ack(hrtim, irq, ack)
#define HRTIM_SOFT_UPDATE(hrtim, timer)                     \
        (hrtim)->hd_ops->soft_update(hrtim, timer)
#define HRTIM_SOFT_CAPTURE(hrtim, timer, index)             \
        (hrtim)->hd_ops->soft_capture(hrtim, timer, index)
#define HRTIM_SOFT_RESET(hrtim, timer)                      \
        (hrtim)->hd_ops->soft_reset(hrtim, timer)
#define HRTIM_FREQ_SET(hrtim, timer,freq)                   \
        (hrtim)->hd_ops->freq_set(hrtim, timer, freq)
#define HRTIM_OUTPUTS_ENABLE(hrtim, outputs, state)         \
        (hrtim)->hd_ops->outputs_enable(hrtim, outputs, state)
#define HRTIM_OUTPUT_SET_SET(hrtim, output, set)            \
        (hrtim)->hd_ops->output_set_set(hrtim, output, set)
#define HRTIM_OUTPUT_RST_SET(hrtim, output, rst)            \
        (hrtim)->hd_ops->output_rst_set(hrtim, output, rst)
#define HRTIM_BURST_CMP_SET(hrtim, cmp)                     \
        (hrtim)->hd_ops->burst_cmp_set(hrtim, cmp)
#define HRTIM_BURST_PER_SET(hrtim, per)                     \
        (hrtim)->hd_ops->burst_per_set(hrtim, per)
#define HRTIM_BURST_PRE_SET(hrtim, pre)                     \
        (hrtim)->hd_ops->burst_pre_set(hrtim, pre)
#define HRTIM_BURST_ENABLE(hrtim, state)                    \
        (hrtim)->hd_ops->burst_enable(hrtim, state)
#define HRTIM_DEADTIME_UPDATE(hrtim, tim, dt, val)          \
        (hrtim)->hd_ops->deadtime_update(hrtim, tim, dt, val)

#define HRTIM_PER_MAX 0xFFFF
#define HRTIM_CMP_MAX 0xFFFF
#define HRTIM_CPT_MAX 0xFFFF
#define HRTIM_REP_MAX 0xFF

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* HRTIM Timer X index */

enum stm32_hrtim_tim_e
{
  HRTIM_TIMER_MASTER = (1<<0),
#ifdef CONFIG_STM32_HRTIM_TIMA
  HRTIM_TIMER_TIMA   = (1<<1),
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB
  HRTIM_TIMER_TIMB   = (1<<2),
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC
  HRTIM_TIMER_TIMC   = (1<<3),
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD
  HRTIM_TIMER_TIMD   = (1<<4),
#endif
#ifdef CONFIG_STM32_HRTIM_TIME
  HRTIM_TIMER_TIME   = (1<<5),
#endif
  HRTIM_TIMER_COMMON = (1<<6)
};

/* Source which can force the Tx1/Tx2 output to its inactive state */

enum stm32_hrtim_out_rst_e
{
  HRTIM_OUT_RST_NONE      = 0,
  HRTIM_OUT_RST_SOFT      = (1 << 0),
  HRTIM_OUT_RST_RESYNC    = (1 << 1),
  HRTIM_OUT_RST_PER       = (1 << 2),
  HRTIM_OUT_RST_CMP1      = (1 << 3),
  HRTIM_OUT_RST_CMP2      = (1 << 4),
  HRTIM_OUT_RST_CMP3      = (1 << 5),
  HRTIM_OUT_RST_CMP4      = (1 << 6),
  HRTIM_OUT_RST_MSTPER    = (1 << 7),
  HRTIM_OUT_RST_MSTCMP1   = (1 << 8),
  HRTIM_OUT_RST_MSTCMP2   = (1 << 9),
  HRTIM_OUT_RST_MSTCMP3   = (1 << 10),
  HRTIM_OUT_RST_MSTCMP4   = (1 << 11),
  HRTIM_OUT_RST_TIMEVNT1  = (1 << 12),
  HRTIM_OUT_RST_TIMEVNT2  = (1 << 13),
  HRTIM_OUT_RST_TIMEVNT3  = (1 << 14),
  HRTIM_OUT_RST_TIMEVNT4  = (1 << 15),
  HRTIM_OUT_RST_TIMEVNT5  = (1 << 16),
  HRTIM_OUT_RST_TIMEVNT6  = (1 << 17),
  HRTIM_OUT_RST_TIMEVNT7  = (1 << 18),
  HRTIM_OUT_RST_TIMEVNT8  = (1 << 19),
  HRTIM_OUT_RST_TIMEVNT9  = (1 << 20),
  HRTIM_OUT_RST_EXTEVNT1  = (1 << 21),
  HRTIM_OUT_RST_EXTEVNT2  = (1 << 22),
  HRTIM_OUT_RST_EXTEVNT3  = (1 << 23),
  HRTIM_OUT_RST_EXTEVNT4  = (1 << 24),
  HRTIM_OUT_RST_EXTEVNT5  = (1 << 25),
  HRTIM_OUT_RST_EXTEVNT6  = (1 << 26),
  HRTIM_OUT_RST_EXTEVNT7  = (1 << 27),
  HRTIM_OUT_RST_EXTEVNT8  = (1 << 28),
  HRTIM_OUT_RST_EXTEVNT9  = (1 << 29),
  HRTIM_OUT_RST_EXTEVNT10 = (1 << 30),
  HRTIM_OUT_RST_UPDATE    = (1 << 31),
};

/* Source which can force the Tx1/Tx2 output to its active state  */

enum stm32_hrtim_out_set_e
{
  HRTIM_OUT_SET_NONE      = 0,
  HRTIM_OUT_SET_SOFT      = (1 << 0),
  HRTIM_OUT_SET_RESYNC    = (1 << 1),
  HRTIM_OUT_SET_PER       = (1 << 2),
  HRTIM_OUT_SET_CMP1      = (1 << 3),
  HRTIM_OUT_SET_CMP2      = (1 << 4),
  HRTIM_OUT_SET_CMP3      = (1 << 5),
  HRTIM_OUT_SET_CMP4      = (1 << 6),
  HRTIM_OUT_SET_MSTPER    = (1 << 7),
  HRTIM_OUT_SET_MSTCMP1   = (1 << 8),
  HRTIM_OUT_SET_MSTCMP2   = (1 << 9),
  HRTIM_OUT_SET_MSTCMP3   = (1 << 10),
  HRTIM_OUT_SET_MSTCMP4   = (1 << 11),
  HRTIM_OUT_SET_TIMEVNT1  = (1 << 12),
  HRTIM_OUT_SET_TIMEVNT2  = (1 << 13),
  HRTIM_OUT_SET_TIMEVNT3  = (1 << 14),
  HRTIM_OUT_SET_TIMEVNT4  = (1 << 15),
  HRTIM_OUT_SET_TIMEVNT5  = (1 << 16),
  HRTIM_OUT_SET_TIMEVNT6  = (1 << 17),
  HRTIM_OUT_SET_TIMEVNT7  = (1 << 18),
  HRTIM_OUT_SET_TIMEVNT8  = (1 << 19),
  HRTIM_OUT_SET_TIMEVNT9  = (1 << 20),
  HRTIM_OUT_SET_EXTEVNT1  = (1 << 21),
  HRTIM_OUT_SET_EXTEVNT2  = (1 << 22),
  HRTIM_OUT_SET_EXTEVNT3  = (1 << 23),
  HRTIM_OUT_SET_EXTEVNT4  = (1 << 24),
  HRTIM_OUT_SET_EXTEVNT5  = (1 << 25),
  HRTIM_OUT_SET_EXTEVNT6  = (1 << 26),
  HRTIM_OUT_SET_EXTEVNT7  = (1 << 27),
  HRTIM_OUT_SET_EXTEVNT8  = (1 << 28),
  HRTIM_OUT_SET_EXTEVNT9  = (1 << 29),
  HRTIM_OUT_SET_EXTEVNT10 = (1 << 30),
  HRTIM_OUT_SET_UPDATE    = (1 << 31),
};

/* Events that can reset TimerX Counter */

enum stm32_hrtim_tim_rst_e
{
  /* Timer owns events */

  HRTIM_RST_UPDT      = (1<<1),
  HRTIM_RST_CMP4      = (1<<2),
  HRTIM_RST_CMP2      = (1<<3),

  /* Master Timer Events */

  HRTIM_RST_MSTPER    = (1<<4),
  HRTIM_RST_MSTCMP1   = (1<<5),
  HRTIM_RST_MSTCMP2   = (1<<6),
  HRTIM_RST_MSTCMP3   = (1<<7),
  HRTIM_RST_MSTCMP4   = (1<<8),

  /* External Events */

  HRTIM_RST_EXTEVNT1  = (1<<9),
  HRTIM_RST_EXTEVNT2  = (1<<10),
  HRTIM_RST_EXTEVNT3  = (1<<11),
  HRTIM_RST_EXTEVNT4  = (1<<12),
  HRTIM_RST_EXTEVNT5  = (1<<13),
  HRTIM_RST_EXTEVNT6  = (1<<14),
  HRTIM_RST_EXTEVNT7  = (1<<15),
  HRTIM_RST_EXTEVNT8  = (1<<16),
  HRTIM_RST_EXTEVNT9  = (1<<17),
  HRTIM_RST_EXTEVNT10 = (1<<18),

  /* TimerX events */

  HRTIM_RST_TACMP1    = (1<<19),
  HRTIM_RST_TACMP2    = (1<<20),
  HRTIM_RST_TACMP4    = (1<<21),
  HRTIM_RST_TBCMP1    = (1<<22),
  HRTIM_RST_TBCMP2    = (1<<23),
  HRTIM_RST_TBCMP4    = (1<<24),
  HRTIM_RST_TCCMP1    = (1<<25),
  HRTIM_RST_TCCMP2    = (1<<26),
  HRTIM_RST_TCCMP4    = (1<<27),
  HRTIM_RST_TDCMP1    = (1<<28),
  HRTIM_RST_TDCMP2    = (1<<29),
  HRTIM_RST_TDCMP4    = (1<<30),
  HRTIM_RST_TECMP1    = (1<<31),
};

/* This definitions does not fit to the above 32 bit enum */

#define HRTIM_RST_TECMP2 (1ull<<32)
#define HRTIM_RST_TECMP4 (1ull<<33)

/* HRTIM Timer X prescaler */

enum stm32_hrtim_tim_prescaler_e
{
  HRTIM_PRESCALER_1,            /* CKPSC = 0 */
  HRTIM_PRESCALER_2,            /* CKPSC = 1 */
  HRTIM_PRESCALER_4,            /* CKPSC = 2 */
  HRTIM_PRESCALER_8,            /* CKPSC = 3 */
  HRTIM_PRESCALER_16,           /* CKPSC = 4 */
  HRTIM_PRESCALER_32,           /* CKPSC = 5 */
  HRTIM_PRESCALER_64,           /* CKPSC = 6 */
  HRTIM_PRESCALER_128           /* CKPSC = 7 */
};

/* HRTIM Timer Master/Slave mode */

enum stm32_hrtim_mode_e
{
  HRTIM_MODE_PRELOAD = (1 << 0),  /* Preload enable */
  HRTIM_MODE_HALF    = (1 << 1),  /* Half mode */
  HRTIM_MODE_RETRIG  = (1 << 2),  /* Re-triggerable mode */
  HRTIM_MODE_CONT    = (1 << 3),  /* Continuous mode */
};

/* HRTIM Slave Timer auto-delayed mode
 * NOTE: details in STM32F334 Manual
 */

enum stm32_hrtim_autodelayed_e
{
  /* CMP2 auto-delayed mode */

  HRTIM_AUTODELAYED_CMP2_MODE1 = 1, /* DELCMP2 = 01 */
  HRTIM_AUTODELAYED_CMP2_MODE2 = 2, /* DELCMP2 = 10 */
  HRTIM_AUTODELAYED_CMP2_MODE3 = 3, /* DELCMP2 = 11 */

  /* CMP4 auto-delayed mode */

  HRTIM_AUTODELAYED_CMP4_MODE1 = (1 << 2), /* DELCMP4 = 01 */
  HRTIM_AUTODELAYED_CMP4_MODE2 = (2 << 2), /* DELCMP4 = 10 */
  HRTIM_AUTODELAYED_CMP4_MODE3 = (3 << 2), /* DELCMP4 = 11 */
};

/* HRTIM Slave Timer fault sources Lock */

enum stm32_hrtim_tim_fault_lock_e
{
  HRTIM_TIM_FAULT_RW   = 0,         /* Slave Timer fault source are read/write */
  HRTIM_TIM_FAULT_LOCK = (1 << 7)   /* Slave Timer fault source are read only */
};

/* HRTIM Slave Timer Fault configuration */

enum stm32_hrtim_tim_fault_src_e
{
  HRTIM_TIM_FAULT1 = (1 << 0),
  HRTIM_TIM_FAULT2 = (1 << 2),
  HRTIM_TIM_FAULT3 = (1 << 3),
  HRTIM_TIM_FAULT4 = (1 << 4),
  HRTIM_TIM_FAULT5 = (1 << 5)
};

/* HRTIM Fault Source */

enum stm32_hrtim_fault_src_e
{
  HRTIM_FAULT_SRC_PIN      = 0,
  HRTIM_FAULT_SRC_INTERNAL = 1
};

/* HRTIM External Event Source
 * NOTE: according to Table 82 from STM32F334XX Manual.
 */

enum stm32_hrtim_eev_src_e
{
  HRTIM_EEV_SRC_PIN    = 0,
  HRTIM_EEV_SRC_ANALOG = 1,
  HRTIM_EEV_SRC_TRGO   = 2,
  HRTIM_EEV_SRC_ADC    = 3
};

/* HRTIM Fault Polarity */

enum stm32_hrtim_fault_pol_e
{
  HRTIM_FAULT_POL_LOW  = 0,
  HRTIM_FAULT_POL_HIGH = 1
};

/* HRTIM External Event Polarity */

enum stm32_hrtim_eev_pol_e
{
  HRTIM_EEV_POL_HIGH = 0,       /* External Event is active high */
  HRTIM_EEV_POL_LOW  = 1        /* External Event is active low */
};

/* HRTIM External Event sensitivity */

enum stm32_hrtim_eev_sen_e
{
  HRTIM_EEV_SEN_LEVEL   = 0,    /* On active level defined by polarity  */
  HRTIM_EEV_SEN_RISING  = 1,    /* Rising edgne */
  HRTIM_EEV_SEN_FALLING = 2,    /* Falling edge */
  HRTIM_EEV_SEN_BOTH    = 3     /* Both edges */
};

/* External Event Sampling clock division */

enum stm32_hrtim_eev_sampling_e
{
  HRTIM_EEV_SAMPLING_d1 = 0,
  HRTIM_EEV_SAMPLING_d2 = 1,
  HRTIM_EEV_SAMPLING_d4 = 2,
  HRTIM_EEV_SAMPLING_d8 = 3
};

/* HRTIM External Event Mode.
 * NOTE: supported only for EEV1-5.
 */

enum stm32_hrtim_eev_mode_e
{
  HRTIM_EEV_MODE_NORMAL = 0,
  HRTIM_EEV_MODE_FAST   = 1     /* low latency mode */
};

/* External Event filter.
 * NOTE: supported only for EEV6-10.
 */

enum stm32_hrtim_eev_filter_e
{
  HRTIM_EEV_DISABLE    = 0,
  HRTIM_EEV_HRT_N2     = 1,
  HRTIM_EEV_HRT_N4     = 2,
  HRTIM_EEV_HRT_N8     = 3,
  HRTIM_EEV_EEVSd2_N6  = 4,
  HRTIM_EEV_EEVSd2_N8  = 5,
  HRTIM_EEV_EEVSd4_N6  = 6,
  HRTIM_EEV_EEVSd4_N8  = 7,
  HRTIM_EEV_EEVSd8_N6  = 8,
  HRTIM_EEV_EEVSd8_N8  = 9,
  HRTIM_EEV_EEVSd16_N5 = 10,
  HRTIM_EEV_EEVSd16_N6 = 11,
  HRTIM_EEV_EEVSd16_N8 = 12,
  HRTIM_EEV_EEVSd32_N5 = 13,
  HRTIM_EEV_EEVSd32_N6 = 14,
  HRTIM_EEV_EEVSd32_N8 = 15
};

/* Compare register index */

enum stm32_hrtim_cmp_index_e
{
  HRTIM_CMP1,
  HRTIM_CMP2,
  HRTIM_CMP3,
  HRTIM_CMP4
};

/* HRTIM Slave Timer Outputs index */

enum stm32_output_s
{
  HRTIM_OUT_CH1 = (1 << 0),
  HRTIM_OUT_CH2 = (1 << 1)
};

/* HRTIM Slave Timers Outputs */

enum stm32_outputs_e
{
  HRTIM_OUT_TIMA_CH1 = (1 << 0),
  HRTIM_OUT_TIMA_CH2 = (1 << 1),
  HRTIM_OUT_TIMB_CH1 = (1 << 2),
  HRTIM_OUT_TIMB_CH2 = (1 << 3),
  HRTIM_OUT_TIMC_CH1 = (1 << 4),
  HRTIM_OUT_TIMC_CH2 = (1 << 5),
  HRTIM_OUT_TIMD_CH1 = (1 << 6),
  HRTIM_OUT_TIMD_CH2 = (1 << 7),
  HRTIM_OUT_TIME_CH1 = (1 << 8),
  HRTIM_OUT_TIME_CH2 = (1 << 9)
};

/* HRTIM Output polarisation */

enum stm32_output_polarisation_e
{
  HRTIM_OUT_POL_POS = 0,
  HRTIM_OUT_POL_NEG = 1
};

/* HRTIM Deadtime sign */

enum stm32_hrtim_deadtime_sign_e
{
  HRTIM_DT_SIGN_POSITIVE = 0,
  HRTIM_DT_SIGN_NEGATIVE = 1
};

/* HRTIM Deadtime types  */

enum stm32_hrtim_deadtime_edge_e
{
  HRTIM_DT_EDGE_RISING = 0,
  HRTIM_DT_EDGE_FALLING = 1
};

/* HRTIM Deadtime lock */

enum stm32_hrtim_deadtime_lock_e
{
  HRTIM_DT_RW   = 0,
  HRTIM_DT_LOCK = 1
};

/* HRTIM Deadtime prescaler */

enum stm32_hrtim_deadtime_prescaler_e
{
  HRTIM_DEADTIME_PRESCALER_1   = 0,
  HRTIM_DEADTIME_PRESCALER_2   = 1,
  HRTIM_DEADTIME_PRESCALER_4   = 2,
  HRTIM_DEADTIME_PRESCALER_8   = 3,
  HRTIM_DEADTIME_PRESCALER_16  = 4,
  HRTIM_DEADTIME_PRESCALER_32  = 5,
  HRTIM_DEADTIME_PRESCALER_64  = 6,
  HRTIM_DEADTIME_PRESCALER_128 = 7
};

/* Chopper start pulsewidth */

enum stm32_hrtim_chopper_start_e
{
  HRTIM_CHP_START_16,
  HRTIM_CHP_START_32,
  HRTIM_CHP_START_48,
  HRTIM_CHP_START_64,
  HRTIM_CHP_START_80,
  HRTIM_CHP_START_96,
  HRTIM_CHP_START_112,
  HRTIM_CHP_START_128,
  HRTIM_CHP_START_144,
  HRTIM_CHP_START_160,
  HRTIM_CHP_START_176,
  HRTIM_CHP_START_192,
  HRTIM_CHP_START_208,
  HRTIM_CHP_START_224,
  HRTIM_CHP_START_256
};

/* Chopper duty cycle */

enum stm32_hrtim_chopper_duty_e
{
  HRTIM_CHP_DUTY_0,
  HRTIM_CHP_DUTY_1,
  HRTIM_CHP_DUTY_2,
  HRTIM_CHP_DUTY_3,
  HRTIM_CHP_DUTY_4,
  HRTIM_CHP_DUTY_5,
  HRTIM_CHP_DUTY_6,
  HRTIM_CHP_DUTY_7
};

/* Chopper carrier frequency */

enum stm32_hrtim_chopper_freq_e
{
  HRTIM_CHP_FREQ_d16,
  HRTIM_CHP_FREQ_d32,
  HRTIM_CHP_FREQ_d48,
  HRTIM_CHP_FREQ_d64,
  HRTIM_CHP_FREQ_d80,
  HRTIM_CHP_FREQ_d96,
  HRTIM_CHP_FREQ_d112,
  HRTIM_CHP_FREQ_d128,
  HRTIM_CHP_FREQ_d144,
  HRTIM_CHP_FREQ_d160,
  HRTIM_CHP_FREQ_d176,
  HRTIM_CHP_FREQ_d192,
  HRTIM_CHP_FREQ_d208,
  HRTIM_CHP_FREQ_d224,
  HRTIM_CHP_FREQ_d240,
  HRTIM_CHP_FREQ_d256
};

/* HRTIM ADC Trigger 1/3 */

enum stm32_hrtim_adc_trq13_e
{
  HRTIM_ADCTRG13_NONE  = 0,        /* No trigger */
  HRTIM_ADCTRG13_MC1   = (1 << 0), /* Trigger on Master Compare 1 */
  HRTIM_ADCTRG13_MC2   = (1 << 1), /* Trigger on Master Compare 2 */
  HRTIM_ADCTRG13_MC3   = (1 << 2), /* Trigger on Master Compare 3 */
  HRTIM_ADCTRG13_MC4   = (1 << 3), /* Trigger on Master Compare 4 */
  HRTIM_ADCTRG13_MPER  = (1 << 4), /* Trigger on Master Period */

  HRTIM_ADCTRG13_EEV1  = (1 << 5), /* Trigger on External Event 1 */
  HRTIM_ADCTRG13_EEV2  = (1 << 6), /* Trigger on External Event 2 */
  HRTIM_ADCTRG13_EEV3  = (1 << 7), /* Trigger on External Event 3 */
  HRTIM_ADCTRG13_EEV4  = (1 << 8), /* Trigger on External Event 4 */
  HRTIM_ADCTRG13_EEV5  = (1 << 9), /* Trigger on External Event 5 */

  HRTIM_ADCTRG13_AC2   = (1 << 10), /* Trigger on Timer A Compare 2 */
  HRTIM_ADCTRG13_AC3   = (1 << 11), /* Trigger on Timer A Compare 3 */
  HRTIM_ADCTRG13_AC4   = (1 << 12), /* Trigger on Timer A Compare 4 */
  HRTIM_ADCTRG13_APER  = (1 << 13), /* Trigger on Timer A Period */
  HRTIM_ADCTRG13_ARST  = (1 << 14), /* Trigger on Timer A Reset */

  HRTIM_ADCTRG13_BC2   = (1 << 15), /* Trigger on Timer B Compare 2 */
  HRTIM_ADCTRG13_BC3   = (1 << 16), /* Trigger on Timer B Compare 3 */
  HRTIM_ADCTRG13_BC4   = (1 << 17), /* Trigger on Timer B Compare 4 */
  HRTIM_ADCTRG13_BPER  = (1 << 18), /* Trigger on Timer B Period */
  HRTIM_ADCTRG13_BRST  = (1 << 19), /* Trigger on Timer B Reset */

  HRTIM_ADCTRG13_CC2   = (1 << 20), /* Trigger on Timer C Compare 2 */
  HRTIM_ADCTRG13_CC3   = (1 << 21), /* Trigger on Timer C Compare 3 */
  HRTIM_ADCTRG13_CC4   = (1 << 22), /* Trigger on Timer C Compare 4 */
  HRTIM_ADCTRG13_CPER  = (1 << 23), /* Trigger on Timer C Period */

  HRTIM_ADCTRG13_DC2   = (1 << 24), /* Trigger on Timer D Compare 2 */
  HRTIM_ADCTRG13_DC3   = (1 << 25), /* Trigger on Timer D Compare 3 */
  HRTIM_ADCTRG13_DC4   = (1 << 26), /* Trigger on Timer D Compare 4 */
  HRTIM_ADCTRG13_DPER  = (1 << 27), /* Trigger on Timer D Period */

  HRTIM_ADCTRG13_EC2   = (1 << 28), /* Trigger on Timer E Compare 2 */
  HRTIM_ADCTRG13_EC3   = (1 << 29), /* Trigger on Timer E Compare 3 */
  HRTIM_ADCTRG13_EC4   = (1 << 30), /* Trigger on Timer E Compare 4 */
  HRTIM_ADCTRG13_EPER  = (1 << 31), /* Trigger on Timer E Period */
};

/* HRTIM ADC Trigger 2/4 */

enum stm32_hrtim_adc_trq24_e
{
  HRTIM_ADCTRG24_NONE  = 0,        /* No trigger */
  HRTIM_ADCTRG24_MC1   = (1 << 0), /* Trigger on Master Compare 1 */
  HRTIM_ADCTRG24_MC2   = (1 << 1), /* Trigger on Master Compare 2 */
  HRTIM_ADCTRG24_MC3   = (1 << 2), /* Trigger on Master Compare 3 */
  HRTIM_ADCTRG24_MC4   = (1 << 3), /* Trigger on Master Compare 4 */
  HRTIM_ADCTRG24_MPER  = (1 << 4), /* Trigger on Master Period */

  HRTIM_ADCTRG24_EEV6  = (1 << 5), /* Trigger on External Event 6 */
  HRTIM_ADCTRG24_EEV7  = (1 << 6), /* Trigger on External Event 7 */
  HRTIM_ADCTRG24_EEV8  = (1 << 7), /* Trigger on External Event 8 */
  HRTIM_ADCTRG24_EEV9  = (1 << 8), /* Trigger on External Event 9 */
  HRTIM_ADCTRG24_EEV10 = (1 << 9), /* Trigger on External Event 10 */

  HRTIM_ADCTRG24_AC2   = (1 << 10), /* Trigger on Timer A Compare 2 */
  HRTIM_ADCTRG24_AC3   = (1 << 11), /* Trigger on Timer A Compare 3 */
  HRTIM_ADCTRG24_AC4   = (1 << 12), /* Trigger on Timer A Compare 4 */
  HRTIM_ADCTRG24_APER  = (1 << 13), /* Trigger on Timer A Period */

  HRTIM_ADCTRG24_BC2   = (1 << 14), /* Trigger on Timer B Compare 2 */
  HRTIM_ADCTRG24_BC3   = (1 << 15), /* Trigger on Timer B Compare 3 */
  HRTIM_ADCTRG24_BC4   = (1 << 16), /* Trigger on Timer B Compare 4 */
  HRTIM_ADCTRG24_BPER  = (1 << 17), /* Trigger on Timer B Period */

  HRTIM_ADCTRG24_CC2   = (1 << 18),  /* Trigger on Timer C Compare 2 */
  HRTIM_ADCTRG24_CC3   = (1 << 19),  /* Trigger on Timer C Compare 3 */
  HRTIM_ADCTRG24_CC4   = (1 << 20),  /* Trigger on Timer C Compare 4 */
  HRTIM_ADCTRG24_CPER  = (1 << 21),  /* Trigger on Timer C Period */
  HRTIM_ADCTRG24_CRST  = (1 << 22),  /* Trigger on Timer C Reset */

  HRTIM_ADCTRG24_DC2   = (1 << 23),  /* Trigger on Timer D Compare 2 */
  HRTIM_ADCTRG24_DC3   = (1 << 24),  /* Trigger on Timer D Compare 3 */
  HRTIM_ADCTRG24_DC4   = (1 << 25),  /* Trigger on Timer D Compare 4 */
  HRTIM_ADCTRG24_DPER  = (1 << 26),  /* Trigger on Timer D Period */
  HRTIM_ADCTRG24_DRST  = (1 << 27),  /* Trigger on Timer D Reset */

  HRTIM_ADCTRG24_EC2   = (1 << 28),  /* Trigger on Timer E Compare 2 */
  HRTIM_ADCTRG24_EC3   = (1 << 29),  /* Trigger on Timer E Compare 3 */
  HRTIM_ADCTRG24_EC4   = (1 << 30),  /* Trigger on Timer E Compare 4 */
  HRTIM_ADCTRG24_ERST  = (1 << 31),  /* Trigger on Timer E Reset */
};

/* HRTIM DAC synchronization events */

enum stm32_hrtim_dac_e
{
  HRTIM_DAC_TRIG_DIS = 0,
  HRTIM_DAC_TRIG1    = 1,
  HRTIM_DAC_TRIG2    = 2,
  HRTIM_DAC_TRIG3    = 3
};

/* HRTIM Timer update events */

enum stm32_tim_update_e
{
  HRTIM_UPDATE_NONE  = 0,
  HRTIM_UPDATE_MSTU  = (1 << 0),
  HRTIM_UPDATE_TAU   = (1 << 2),
  HRTIM_UPDATE_TBU   = (1 << 3),
  HRTIM_UPDATE_TCU   = (1 << 4),
  HRTIM_UPDATE_TDU   = (1 << 5),
  HRTIM_UPDATE_TEU   = (1 << 6),
  HRTIM_UPDATE_RSTU  = (1 << 7),
  HRTIM_UPDATE_REPU  = (1 << 8),
};

/* HRTIM Master Timer interrupts  */

enum stm32_irq_master_e
{
  HRTIM_IRQ_MCMP1 = (1 << 0),    /* Master Compare 1 Interrupt */
  HRTIM_IRQ_MCMP2 = (1 << 1),    /* Master Compare 2 Interrupt */
  HRTIM_IRQ_MCMP3 = (1 << 2),    /* Master Compare 3 Interrupt */
  HRTIM_IRQ_MCMP4 = (1 << 3),    /* Master Compare 4 Interrupt */
  HRTIM_IRQ_MREP  = (1 << 4),    /* Master Repetition Interrupt */
  HRTIM_IRQ_MSYNC = (1 << 5),    /* Sync Input Interrupt */
  HRTIM_IRQ_MUPD  = (1 << 6)     /* Master Update Interrupt */
};

/* HRTIM Slave Timer interrupts  */

enum stm32_irq_slave_e
{
  HRTIM_IRQ_CMP1   = (1 << 0),  /* Slave Compare 1 Interrupt */
  HRTIM_IRQ_CMP2   = (1 << 1),  /* Slave Compare 2 Interrupt */
  HRTIM_IRQ_CMP3   = (1 << 2),  /* Slave Compare 3 Interrupt */
  HRTIM_IRQ_CMP4   = (1 << 3),  /* Slave Compare 4 Interrupt */
  HRTIM_IRQ_REP    = (1 << 4),  /* Slave Repetition Interrupt */
  HRTIM_IRQ_UPD    = (1 << 6),  /* Slave Update Interrupt */
  HRTIM_IRQ_CPT1   = (1 << 7),  /* Slave Capture 1 Interrupt */
  HRTIM_IRQ_CPT2   = (1 << 8),  /* Slave Capture 2 Interrupt */
  HRTIM_IRQ_SETX1  = (1 << 9),  /* Slave Output 1 Set Interrupt */
  HRTIM_IRQ_RSTX1  = (1 << 10), /* Slave Output 1 Reset Interrupt */
  HRTIM_IRQ_SETX2  = (1 << 11), /* Slave Output 2 Set Interrupt */
  HRTIM_IRQ_RSTX2  = (1 << 12), /* Slave Output 2 Reset Interrupt */
  HRTIM_IRQ_RST    = (1 << 13), /* Slave Reset/roll-over Interrupt */
  HRTIM_IRQ_DLYPRT = (1 << 14)  /* Slave Delayed Protection Interrupt */
};

/* HRTIM Common Interrupts */

enum stm32_irq_cmn_e
{
  HRTIM_IRQ_FLT1   = (1 << 0),  /* Fault 1 Interrupt */
  HRTIM_IRQ_FLT2   = (1 << 1),  /* Fault 2 Interrupt */
  HRTIM_IRQ_FLT3   = (1 << 2),  /* Fault 3 Interrupt */
  HRTIM_IRQ_FLT4   = (1 << 3),  /* Fault 4 Interrupt */
  HRTIM_IRQ_FLT5   = (1 << 4),  /* Fault 5 Interrupt */
  HRTIM_IRQ_SYSFLT = (1 << 5),  /* System Fault Interrupt */
  HRTIM_IRQ_DLLRDY = (1 << 16), /* DLL Ready Interrupt */
  HRTIM_IRQ_BMPER  = (1 << 17)  /* Burst Mode Period Interrupt */
};

/* HRTIM DMA requests */

enum stm32_hrtim_dma_e
{
  HRTIM_DMA_CMP1   = (1 << 0),  /* Common: Compare 1 DMA request */
  HRTIM_DMA_CMP2   = (1 << 1),  /* Common: Compare 2 DMA request */
  HRTIM_DMA_CMP3   = (1 << 2),  /* Common: Compare 3 DMA request */
  HRTIM_DMA_CMP4   = (1 << 3),  /* Common:Compare 4 DMA request */
  HRTIM_DMA_REP    = (1 << 4),  /* Common: Repetition DMA request */
  HRTIM_DMA_SYNC   = (1 << 5),  /* Master: Sync Input DMA request */
  HRTIM_DMA_UPD    = (1 << 6),  /* Common: Update DMA reques */
  HRTIM_DMA_CPT1   = (1 << 7),  /* Slaves: Capture 1 DMA reques */
  HRTIM_DMA_CPT2   = (1 << 8),  /* Slaves: Capture 2 DMA reques */
  HRTIM_DMA_SET1   = (1 << 9),  /* Slaves: Output 1 Set DMA reques */
  HRTIM_DMA_RST1   = (1 << 10), /* Slaves: Output 1 Reset DMA reques */
  HRTIM_DMA_SET2   = (1 << 11), /* Slaves: Output 2 Set DMA reques */
  HRTIM_DMA_RST2   = (1 << 12), /* Slaves: Output 2 Reset DMA reques */
  HRTIM_DMA_RST    = (1 << 13), /* Slaves: Reset DMA reques */
  HRTIM_DMA_DLYPRT = (1 << 14)  /* Slaves: Delayed Protection DMA reques */
};

/* HRTIM Output IDLE state */

enum stm32_hrtim_idle_state
{
  HRTIM_IDLE_INACTIVE = 0,      /* Output inactive during IDLE state */
  HRTIM_IDLE_ACTIVE   = 1       /* Output active during IDLE state */
};

/* HRTIM Burst Mode clock source */

enum stm32_hrtim_burst_source_e
{
  HRTIM_BURST_CLOCK_MASTER = 0,  /* Master timer counter reset/roll-over */
  HRTIM_BURST_CLOCK_TIMA   = 1,  /* Timer A counter reset/roll-over */
  HRTIM_BURST_CLOCK_TIMB   = 2,  /* Timer B counter reset/roll-over */
  HRTIM_BURST_CLOCK_TIMC   = 3,  /* Timer C counter reset/roll-over */
  HRTIM_BURST_CLOCK_TIMD   = 4,  /* Timer D counter reset/roll-over */
  HRTIM_BURST_CLOCK_TIME   = 5,  /* Timer E counter reset/roll-over */
  HRTIM_BURST_CLOCK_EV1    = 6,  /* On-chip Event 1 */
  HRTIM_BURST_CLOCK_EV2    = 7,  /* On-chip Event 2 */
  HRTIM_BURST_CLOCK_EV3    = 8,  /* On-chip Event 3 */
  HRTIM_BURST_CLOCK_EV4    = 9,  /* On-chip Event 4 */
  HRTIM_BURST_CLOCK_HRTIM  = 10  /* Prescaled f_HRTIM clock */
};

/* HRTIM Burst Mode prescaler for fHRTIM clock */

enum stm32_hrtim_burst_precaler_e
{
  HRTIM_BURST_PRESCALER_1     = 0,
  HRTIM_BURST_PRESCALER_2     = 1,
  HRTIM_BURST_PRESCALER_4     = 2,
  HRTIM_BURST_PRESCALER_8     = 3,
  HRTIM_BURST_PRESCALER_16    = 4,
  HRTIM_BURST_PRESCALER_32    = 5,
  HRTIM_BURST_PRESCALER_64    = 6,
  HRTIM_BURST_PRESCALER_128   = 7,
  HRTIM_BURST_PRESCALER_256   = 8,
  HRTIM_BURST_PRESCALER_512   = 9,
  HRTIM_BURST_PRESCALER_1024  = 10,
  HRTIM_BURST_PRESCALER_2048  = 11,
  HRTIM_BURST_PRESCALER_4096  = 12,
  HRTIM_BURST_PRESCALER_8192  = 13,
  HRTIM_BURST_PRESCALER_16384 = 14,
  HRTIM_BURST_PRESCALER_32768 = 15
};

/* HRTIM Burst Mode triggers  */

enum stm32_hrtim_burst_triggers_e
{
  HRTIM_BURST_TRG_MSTRST  = (1 << 1),
  HRTIM_BURST_TRG_MSTREP  = (1 << 2),
  HRTIM_BURST_TRG_MSTCMP1 = (1 << 3),
  HRTIM_BURST_TRG_MSTCMP2 = (1 << 4),
  HRTIM_BURST_TRG_MSTCMP3 = (1 << 5),
  HRTIM_BURST_TRG_MSTCMP4 = (1 << 6),
  HRTIM_BURST_TRG_TARST   = (1 << 7),
  HRTIM_BURST_TRG_TAREP   = (1 << 8),
  HRTIM_BURST_TRG_TACMP1  = (1 << 9),
  HRTIM_BURST_TRG_TACMP2  = (1 << 10),
  HRTIM_BURST_TRG_TBRST   = (1 << 11),
  HRTIM_BURST_TRG_TBREP   = (1 << 12),
  HRTIM_BURST_TRG_TBCMP1  = (1 << 13),
  HRTIM_BURST_TRG_TBCMP2  = (1 << 14),
  HRTIM_BURST_TRG_TCRST   = (1 << 15),
  HRTIM_BURST_TRG_TCREP   = (1 << 16),
  HRTIM_BURST_TRG_TCCMP1  = (1 << 17),
  HRTIM_BURST_TRG_TCCMP2  = (1 << 18),
  HRTIM_BURST_TRG_TDRST   = (1 << 19),
  HRTIM_BURST_TRG_TDREP   = (1 << 20),
  HRTIM_BURST_TRG_TDCMP1  = (1 << 21),
  HRTIM_BURST_TRG_TDCMP2  = (1 << 22),
  HRTIM_BURST_TRG_TERST   = (1 << 23),
  HRTIM_BURST_TRG_TEREP   = (1 << 24),
  HRTIM_BURST_TRG_TECMP1  = (1 << 25),
  HRTIM_BURST_TRG_TECMP2  = (1 << 26),
  HRTIM_BURST_TRG_TAEEV7  = (1 << 27),
  HRTIM_BURST_TRG_TDEEV8  = (1 << 28),
  HRTIM_BURST_TRG_EEV7    = (1 << 29),
  HRTIM_BURST_TRG_EEV8    = (1 << 30),
  HRTIM_BURST_TRG_OCHPEV  = (1 << 31),
};

/* HRTIM Capture triggers */
enum stm32_hrtim_capture_index_e
{
  HRTIM_CAPTURE1 = 0,
  HRTIM_CAPTURE2 = 1
};

/* HRTIM Capture triggers */

enum stm32_hrtim_capture_triggers_e
{
  HRTIM_CAPTURE_TRG_SW     = (1 << 0),
  HRTIM_CAPTURE_TRG_UPD    = (1 << 1),
  HRTIM_CAPTURE_TRG_EXEV1  = (1 << 2),
  HRTIM_CAPTURE_TRG_EXEV2  = (1 << 3),
  HRTIM_CAPTURE_TRG_EXEV3  = (1 << 4),
  HRTIM_CAPTURE_TRG_EXEV4  = (1 << 5),
  HRTIM_CAPTURE_TRG_EXEV5  = (1 << 6),
  HRTIM_CAPTURE_TRG_EXEV6  = (1 << 7),
  HRTIM_CAPTURE_TRG_EXEV7  = (1 << 8),
  HRTIM_CAPTURE_TRG_EXEV8  = (1 << 9),
  HRTIM_CAPTURE_TRG_EXEV9  = (1 << 10),
  HRTIM_CAPTURE_TRG_EXEV10 = (1 << 11),
  HRTIM_CAPTURE_TRG_TA1SET = (1 << 12),
  HRTIM_CAPTURE_TRG_TA1RST = (1 << 13),
  HRTIM_CAPTURE_TRG_TACMP1 = (1 << 14),
  HRTIM_CAPTURE_TRG_TACMP2 = (1 << 15),
  HRTIM_CAPTURE_TRG_TB1SET = (1 << 16),
  HRTIM_CAPTURE_TRG_TB1RST = (1 << 17),
  HRTIM_CAPTURE_TRG_TBCMP1 = (1 << 18),
  HRTIM_CAPTURE_TRG_TBCMP2 = (1 << 19),
  HRTIM_CAPTURE_TRG_TC1SET = (1 << 20),
  HRTIM_CAPTURE_TRG_TC1RST = (1 << 21),
  HRTIM_CAPTURE_TRG_TCCMP1 = (1 << 22),
  HRTIM_CAPTURE_TRG_TCCMP2 = (1 << 23),
  HRTIM_CAPTURE_TRG_TD1SET = (1 << 24),
  HRTIM_CAPTURE_TRG_TD1RST = (1 << 25),
  HRTIM_CAPTURE_TRG_TDCMP1 = (1 << 26),
  HRTIM_CAPTURE_TRG_TDCMP2 = (1 << 27),
  HRTIM_CAPTURE_TRG_TE1SET = (1 << 28),
  HRTIM_CAPTURE_TRG_TE1RST = (1 << 29),
  HRTIM_CAPTURE_TRG_TECMP1 = (1 << 30),
  HRTIM_CAPTURE_TRG_TECMP2 = (1 << 31),
};

/* HRTIM vtable */

struct hrtim_dev_s;
struct stm32_hrtim_ops_s
{
  int      (*cmp_update)(FAR struct hrtim_dev_s *dev, uint8_t timer,
                         uint8_t index, uint16_t cmp);
  int      (*per_update)(FAR struct hrtim_dev_s *dev, uint8_t timer, uint16_t per);
  int      (*rep_update)(FAR struct hrtim_dev_s *dev, uint8_t timer, uint8_t rep);
  uint16_t (*per_get)(FAR struct hrtim_dev_s *dev, uint8_t timer);
  uint16_t (*cmp_get)(FAR struct hrtim_dev_s *dev, uint8_t timer,
                      uint8_t index);
  uint64_t (*fclk_get)(FAR struct hrtim_dev_s *dev, uint8_t timer);
  int      (*soft_update)(FAR struct hrtim_dev_s *dev, uint8_t timer);
  int      (*soft_reset)(FAR struct hrtim_dev_s *dev, uint8_t timer);
  int      (*freq_set)(FAR struct hrtim_dev_s  *hrtim, uint8_t timer,
                                 uint64_t freq);

#ifdef CONFIG_STM32_HRTIM_INTERRUPTS
  int      (*irq_ack)(FAR struct hrtim_dev_s *dev, uint8_t timer, int source);
  uint16_t (*irq_get)(FAR struct hrtim_dev_s *dev, uint8_t timer);
#endif
#ifdef CONFIG_STM32_HRTIM_PWM
  int      (*outputs_enable)(FAR struct hrtim_dev_s *dev, uint16_t outputs,
                             bool state);
  int      (*output_set_set)(FAR struct hrtim_dev_s *dev, uint16_t output,
                              uint32_t set);
  int      (*output_rst_set)(FAR struct hrtim_dev_s *dev, uint16_t output,
                              uint32_t rst);
#endif
#ifdef CONFIG_STM32_HRTIM_BURST
  int      (*burst_enable)(FAR struct hrtim_dev_s *dev, bool state);
  int      (*burst_cmp_set)(FAR struct hrtim_dev_s *dev, uint16_t cmp);
  int      (*burst_per_set)(FAR struct hrtim_dev_s *dev, uint16_t per);
  int      (*burst_pre_set)(FAR struct hrtim_dev_s *dev, uint8_t pre);
  uint16_t (*burst_cmp_get)(FAR struct hrtim_dev_s *dev);
  uint16_t (*burst_per_get)(FAR struct hrtim_dev_s *dev);
  int      (*burst_pre_get)(FAR struct hrtim_dev_s *dev);
#endif
#ifdef CONFIG_STM32_HRTIM_CHOPPER
  int      (*chopper_enable)(FAR struct hrtim_dev_s *dev, uint8_t timer,
                             uint8_t chan, bool state);
#endif
#ifdef CONFIG_STM32_HRTIM_DEADTIME
  int      (*deadtime_update)(FAR struct hrtim_dev_s *dev, uint8_t timer,
                              uint8_t dt, uint16_t value);
  uint16_t (*deadtime_get)(FAR struct hrtim_dev_s *dev, uint8_t timer,
                           uint8_t dt);
#endif
#ifdef CONFIG_STM32_HRTIM_CAPTURE
  uint16_t (*capture_get)(FAR struct hrtim_dev_s *dev, uint8_t timer,
                          uint8_t index);
  int      (*soft_capture)(FAR struct hrtim_dev_s *dev, uint8_t timer,
                           uint8_t index);

#endif
};

/* HRTIM device structure */

struct hrtim_dev_s
{
#ifdef CONFIG_STM32_HRTIM
  /* Fields managed by common upper half HRTIM logic */

  uint8_t hd_ocount; /* The number of times the device has been opened */
  sem_t hd_closesem; /* Locks out new opens while close is in progress */
#endif

  /* Fields provided by lower half HRTIM logic */

  FAR const struct stm32_hrtim_ops_s *hd_ops; /* HRTIM operations */
  FAR void *hd_priv;                          /* Used by the arch-specific logic */
  bool initialized;                           /* true: HRTIM driver has been initialized */
};

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32_hrtiminitialize
 *
 * Description:
 *   Initialize the HRTIM.
 *
 * Input Parameters:
 *  None
 *
 * Returned Value:
 *   Valid HRTIM device structure reference on succcess; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the HRTIM block has enabled,
 *   2. Board-specific logic has already configured
 *
 ****************************************************************************/

FAR struct hrtim_dev_s* stm32_hrtiminitialize(void);

/****************************************************************************
 * Name: hrtim_register
 ****************************************************************************/

#ifndef CONFIG_STM32_HRTIM_DISABLE_CHARDRV
int hrtim_register(FAR const char *path, FAR struct hrtim_dev_s *dev);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* CONFIG_STM32_HRTIM1 */
#endif /* __ARCH_ARM_SRC_STM32_STM32_HRTIM_H */
