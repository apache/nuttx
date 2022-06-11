/****************************************************************************
 * arch/arm/src/stm32/stm32_hrtim.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_hrtim.h"

#if defined(CONFIG_STM32_HRTIM1)

/* Only STM32F33XXX  */

#if defined(CONFIG_STM32_STM32F33XX)

#if defined(CONFIG_STM32_HRTIM_TIMA_PWM) || defined(CONFIG_STM32_HRTIM_TIMA_DAC) || \
    defined(CONFIG_STM32_HRTIM_TIMA_CAP) || defined(CONFIG_STM32_HRTIM_TIMA_IRQ) || \
    defined(CONFIG_STM32_HRTIM_TIMA_DT)  || defined(CONFIG_STM32_HRTIM_TIMA_CHOP)
#  ifndef CONFIG_STM32_HRTIM_TIMA
#    error "CONFIG_STM32_HRTIM_TIMA must be set"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_TIMB_PWM) || defined(CONFIG_STM32_HRTIM_TIMB_DAC) || \
    defined(CONFIG_STM32_HRTIM_TIMB_CAP) || defined(CONFIG_STM32_HRTIM_TIMB_IRQ) || \
    defined(CONFIG_STM32_HRTIM_TIMB_DT)  || defined(CONFIG_STM32_HRTIM_TIMB_CHOP)
#  ifndef CONFIG_STM32_HRTIM_TIMB
#    error "CONFIG_STM32_HRTIM_TIMB must be set"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_TIMC_PWM) || defined(CONFIG_STM32_HRTIM_TIMC_DAC) || \
    defined(CONFIG_STM32_HRTIM_TIMC_CAP) || defined(CONFIG_STM32_HRTIM_TIMC_IRQ) || \
    defined(CONFIG_STM32_HRTIM_TIMC_DT)  || defined(CONFIG_STM32_HRTIM_TIMC_CHOP)
#  ifndef CONFIG_STM32_HRTIM_TIMC
#    error "CONFIG_STM32_HRTIM_TIMC must be set"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_TIMD_PWM) || defined(CONFIG_STM32_HRTIM_TIMD_DAC) || \
    defined(CONFIG_STM32_HRTIM_TIMD_CAP) || defined(CONFIG_STM32_HRTIM_TIMD_IRQ) || \
    defined(CONFIG_STM32_HRTIM_TIMD_DT)  || defined(CONFIG_STM32_HRTIM_TIMD_CHOP)
#  ifndef CONFIG_STM32_HRTIM_TIMD
#    error "CONFIG_STM32_HRTIM_TIMD must be set"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_TIME_PWM) || defined(CONFIG_STM32_HRTIM_TIME_DAC) || \
    defined(CONFIG_STM32_HRTIM_TIME_CAP) || defined(CONFIG_STM32_HRTIM_TIME_IRQ) || \
    defined(CONFIG_STM32_HRTIM_TIME_DT)  || defined(CONFIG_STM32_HRTIM_TIME_CHOP)
#  ifndef CONFIG_STM32_HRTIM_TIME
#    error "CONFIG_STM32_HRTIM_TIME must be set"
#  endif
#endif

#if defined(CONFIG_STM32_HRTIM_PWM)
#if !defined(CONFIG_STM32_HRTIM_TIMA_PWM) && !defined(CONFIG_STM32_HRTIM_TIMB_PWM) && \
    !defined(CONFIG_STM32_HRTIM_TIMC_PWM) && !defined(CONFIG_STM32_HRTIM_TIMD_PWM) && \
    !defined(CONFIG_STM32_HRTIM_TIME_PWM)
#    warning "CONFIG_STM32_HRTIM_PWM enabled but no timer selected"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_DAC)
#if !defined(CONFIG_STM32_HRTIM_MASTER_DAC) && !defined(CONFIG_STM32_HRTIM_TIMA_DAC) && \
    !defined(CONFIG_STM32_HRTIM_TIMB_DAC)   && !defined(CONFIG_STM32_HRTIM_TIMC_DAC) && \
    !defined(CONFIG_STM32_HRTIM_TIMD_DAC)   && !defined(CONFIG_STM32_HRTIM_TIME_DAC)
#    warning "CONFIG_STM32_HRTIM_DAC enabled but no timer selected"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_CAPTURE)
#if !defined(CONFIG_STM32_HRTIM_TIMA_CAP) && !defined(CONFIG_STM32_HRTIM_TIMB_CAP) && \
    !defined(CONFIG_STM32_HRTIM_TIMC_CAP) && !defined(CONFIG_STM32_HRTIM_TIMD_CAP) && \
    !defined(CONFIG_STM32_HRTIM_TIME_CAP)
#    warning "CONFIG_STM32_HRTIM_CAPTURE enabled but no timer selected"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_INTERRUPTS)
#if !defined(CONFIG_STM32_HRTIM_MASTER_IRQ) && !defined(CONFIG_STM32_HRTIM_TIMA_IRQ) && \
    !defined(CONFIG_STM32_HRTIM_TIMB_IRQ) && !defined(CONFIG_STM32_HRTIM_TIMC_IRQ) && \
    !defined(CONFIG_STM32_HRTIM_TIMD_IRQ) && !defined(CONFIG_STM32_HRTIM_TIME_IRQ) && \
    !defined(CONFIG_STM32_HRTIM_COMMON_IRQ)
#    warning "CONFIG_STM32_HRTIM_INTERRUPTS enabled but no timer selected"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_DEADTIME)
#if !defined(CONFIG_STM32_HRTIM_TIMA_DT) && !defined(CONFIG_STM32_HRTIM_TIMB_DT) && \
    !defined(CONFIG_STM32_HRTIM_TIMC_DT) && !defined(CONFIG_STM32_HRTIM_TIMD_DT) && \
    !defined(CONFIG_STM32_HRTIM_TIME_DT)
#    warning "CONFIG_STM32_HRTIM_DEADTIME enabled but no timer selected"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_CHOPPER)
#if !defined(CONFIG_STM32_HRTIM_TIMA_CHOP) && !defined(CONFIG_STM32_HRTIM_TIMB_CHOP) && \
    !defined(CONFIG_STM32_HRTIM_TIMC_CHOP) && !defined(CONFIG_STM32_HRTIM_TIMD_CHOP) && \
    !defined(CONFIG_STM32_HRTIM_TIME_CHOP)
#    warning "CONFIG_STM32_HRTIM_CHOPPER enabled but no timer selected"
#  endif
#endif

#if defined(CONFIG_STM32_HRTIM_TIMA_PWM) || defined(CONFIG_STM32_HRTIM_TIMB_PWM) || \
    defined(CONFIG_STM32_HRTIM_TIMC_PWM) || defined(CONFIG_STM32_HRTIM_TIMD_PWM) || \
    defined(CONFIG_STM32_HRTIM_TIME_PWM)
#  ifndef CONFIG_STM32_HRTIM_PWM
#    error "CONFIG_STM32_HRTIM_PWM must be set"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_MASTER_DAC) || defined(CONFIG_STM32_HRTIM_TIMA_DAC) || \
    defined(CONFIG_STM32_HRTIM_TIMB_DAC)   || defined(CONFIG_STM32_HRTIM_TIMC_DAC) || \
    defined(CONFIG_STM32_HRTIM_TIMD_DAC)   || defined(CONFIG_STM32_HRTIM_TIME_DAC)
#  ifndef CONFIG_STM32_HRTIM_DAC
#    error "CONFIG_STM32_HRTIM_DAC must be set"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_TIMA_CAP) || defined(CONFIG_STM32_HRTIM_TIMB_CAP) || \
    defined(CONFIG_STM32_HRTIM_TIMC_CAP) || defined(CONFIG_STM32_HRTIM_TIMD_CAP) || \
    defined(CONFIG_STM32_HRTIM_TIME_CAP)
#  ifndef CONFIG_STM32_HRTIM_CAPTURE
#    error "CONFIG_STM32_HRTIM_CAPTURE must be set"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_TIMA_IRQ) || defined(CONFIG_STM32_HRTIM_TIMB_IRQ) || \
    defined(CONFIG_STM32_HRTIM_TIMC_IRQ) || defined(CONFIG_STM32_HRTIM_TIMD_IRQ) || \
    defined(CONFIG_STM32_HRTIM_TIME_IRQ)
#  ifndef CONFIG_STM32_HRTIM_INTERRUPTS
#    error "CONFIG_STM32_HRTIM_INTERRUPTS must be set"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_TIMA_DT) || defined(CONFIG_STM32_HRTIM_TIMB_DT) || \
    defined(CONFIG_STM32_HRTIM_TIMC_DT) || defined(CONFIG_STM32_HRTIM_TIMD_DT) || \
    defined(CONFIG_STM32_HRTIM_TIME_DT)
#  ifndef CONFIG_STM32_HRTIM_DEADTIME
#    error "CONFIG_STM32_HRTIM_DEADTIME must be set"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_TIMA_CHOP) || defined(CONFIG_STM32_HRTIM_TIMB_CHOP) || \
    defined(CONFIG_STM32_HRTIM_TIMC_CHOP) || defined(CONFIG_STM32_HRTIM_TIMD_CHOP) || \
    defined(CONFIG_STM32_HRTIM_TIME_CHOP)
#  ifndef CONFIG_STM32_HRTIM_CHOPPER
#    error "CONFIG_STM32_HRTIM_CHOPPER must be set"
#  endif
#endif
#if defined(CONFIG_STM32_HRTIM_TIMA_PSHPLL) || defined(CONFIG_STM32_HRTIM_TIMB_PSHPLL) || \
    defined(CONFIG_STM32_HRTIM_TIMC_PSHPLL) || defined(CONFIG_STM32_HRTIM_TIMD_PSHPLL) || \
    defined(CONFIG_STM32_HRTIM_TIME_PSHPLL)
#  ifndef CONFIG_STM32_HRTIM_PUSHPULL
#    error "CONFIG_STM32_HRTIM_PUSHPULL must be set"
#  endif
#endif

#if defined(CONFIG_STM32_HRTIM_TIMA_DT) && defined(CONFIG_STM32_HRTIM_TIMA_PSHPLL)
#  error "The deadtime cannot be used simultaneously with the push-pull mode"
#endif
#if defined(CONFIG_STM32_HRTIM_TIMB_DT) && defined(CONFIG_STM32_HRTIM_TIMB_PSHPLL)
#  error "The deadtime cannot be used simultaneously with the push-pull mode"
#endif
#if defined(CONFIG_STM32_HRTIM_TIMC_DT) && defined(CONFIG_STM32_HRTIM_TIMC_PSHPLL)
#  error "The deadtime cannot be used simultaneously with the push-pull mode"
#endif
#if defined(CONFIG_STM32_HRTIM_TIMD_DT) && defined(CONFIG_STM32_HRTIM_TIMD_PSHPLL)
#  error "The deadtime cannot be used simultaneously with the push-pull mode"
#endif
#if defined(CONFIG_STM32_HRTIM_TIME_DT) && defined(CONFIG_STM32_HRTIM_TIME_PSHPLL)
#  error "The deadtime cannot be used simultaneously with the push-pull mode"
#endif

#if defined(CONFIG_STM32_HRTIM_ADC1_TRG1) || defined(CONFIG_STM32_HRTIM_ADC1_TRG2) || \
    defined(CONFIG_STM32_HRTIM_ADC1_TRG3) || defined(CONFIG_STM32_HRTIM_ADC1_TRG4) || \
    defined(CONFIG_STM32_HRTIM_ADC2_TRG1) || defined(CONFIG_STM32_HRTIM_ADC2_TRG2) || \
    defined(CONFIG_STM32_HRTIM_ADC2_TRG3) || defined(CONFIG_STM32_HRTIM_ADC2_TRG4)
#  define HRTIM_HAVE_ADC
#endif

#if defined(CONFIG_STM32_HRTIM_ADC1_TRG1) || defined(CONFIG_STM32_HRTIM_ADC2_TRG1)
#  define HRTIM_HAVE_ADC_TRG1
#endif
#if defined(CONFIG_STM32_HRTIM_ADC1_TRG2) || defined(CONFIG_STM32_HRTIM_ADC2_TRG2)
#  define HRTIM_HAVE_ADC_TRG2
#endif
#if defined(CONFIG_STM32_HRTIM_ADC1_TRG3) || defined(CONFIG_STM32_HRTIM_ADC2_TRG3)
#  define HRTIM_HAVE_ADC_TRG3
#endif
#if defined(CONFIG_STM32_HRTIM_ADC1_TRG4) || defined(CONFIG_STM32_HRTIM_ADC2_TRG4)
#  define HRTIM_HAVE_ADC_TRG4
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HRTIM default configuration **********************************************/

#if defined(CONFIG_STM32_HRTIM_MASTER) && !defined(HRTIM_MASTER_MODE)
#  warning "HRTIM_MASTER_MODE is not set. Set the default value 0"
#  define HRTIM_MASTER_MODE 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIMA) && !defined( HRTIM_TIMA_MODE)
#  warning "HRTIM_TIMA_MODE is not set. Set the default value 0"
#  define HRTIM_TIMA_MODE 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIMB) && !defined(HRTIM_TIMB_MODE)
#  warning "HRTIM_TIMB_MODE is not set. Set the default value 0"
#  define HRTIM_TIMB_MODE 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIMC) && !defined(HRTIM_TIMC_MODE)
#  warning "HRTIM_TIMC_MODE is not set. Set the default value 0"
#  define HRTIM_TIMC_MODE 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIMD) && !defined(HRTIM_TIMD_MODE)
#  warning "HRTIM_TIMD_MODE is not set. Set the default value 0"
#  define HRTIM_TIMD_MODE 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIME) && !defined(HRTIM_TIME_MODE)
#  warning "HRTIM_TIME_MODE is not set. Set the default value 0"
#  define HRTIM_TIME_MODE 0
#endif

#if defined(CONFIG_STM32_HRTIM_TIMA) && !defined(HRTIM_TIMA_UPDATE)
#  warning "HRTIM_TIMA_UPDATE is not set. Set the default value 0"
#  define HRTIM_TIMA_UPDATE 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIMB) && !defined(HRTIM_TIMB_UPDATE)
#  warning "HRTIM_TIMB_UPDATE is not set. Set the default value 0"
#  define HRTIM_TIMB_UPDATE 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIMC) && !defined(HRTIM_TIMC_UPDATE)
#  warning "HRTIM_TIMC_UPDATE is not set. Set the default value 0"
#  define HRTIM_TIMC_UPDATE 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIMD) && !defined(HRTIM_TIMD_UPDATE)
#  warning "HRTIM_TIMD_UPDATE is not set. Set the default value 0"
#  define HRTIM_TIMD_UPDATE 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIME) && !defined(HRTIM_TIME_UPDATE)
#  warning "HRTIM_TIME_UPDATE is not set. Set the default value 0"
#  define HRTIM_TIME_UPDATE 0
#endif

#if defined(CONFIG_STM32_HRTIM_TIMA) && !defined( HRTIM_TIMA_RESET)
#  warning "HRTIM_TIMA_RESET is not set. Set the default value 0"
#  define HRTIM_TIMA_RESET 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIMB) && !defined(HRTIM_TIMB_RESET)
#  warning "HRTIM_TIMB_RESET is not set. Set the default value 0"
#  define HRTIM_TIMB_RESET 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIMC) && !defined(HRTIM_TIMC_RESET)
#  warning "HRTIM_TIMC_RESET is not set. Set the default value 0"
#  define HRTIM_TIMC_RESET 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIMD) && !defined(HRTIM_TIMD_RESET)
#  warning "HRTIM_TIMD_RESET is not set. Set the default value 0"
#  define HRTIM_TIMD_RESET 0
#endif
#if defined(CONFIG_STM32_HRTIM_TIME) && !defined(HRTIM_TIME_RESET)
#  warning "HRTIM_TIME_RESET is not set. Set the default value 0"
#  define HRTIM_TIME_RESET 0
#endif

#ifndef HRTIM_IRQ_COMMON
#  define HRTIM_IRQ_COMMON 0
#endif

#if defined(CONFIG_STM32_HRTIM_TIMA) && !defined(HRTIM_TIMA_CH1_POL)
#  define HRTIM_TIMA_CH1_POL HRTIM_OUT_POL_POS
#endif
#if defined(CONFIG_STM32_HRTIM_TIMA) && !defined(HRTIM_TIMA_CH2_POL)
#  define HRTIM_TIMA_CH2_POL HRTIM_OUT_POL_POS
#endif
#if defined(CONFIG_STM32_HRTIM_TIMB) && !defined(HRTIM_TIMB_CH1_POL)
#  define HRTIM_TIMB_CH1_POL HRTIM_OUT_POL_POS
#endif
#if defined(CONFIG_STM32_HRTIM_TIMB) && !defined(HRTIM_TIMB_CH2_POL)
#  define HRTIM_TIMB_CH2_POL HRTIM_OUT_POL_POS
#endif
#if defined(CONFIG_STM32_HRTIM_TIMC) && !defined(HRTIM_TIMC_CH1_POL)
#  define HRTIM_TIMC_CH1_POL HRTIM_OUT_POL_POS
#endif
#if defined(CONFIG_STM32_HRTIM_TIMC) && !defined(HRTIM_TIMC_CH2_POL)
#  define HRTIM_TIMC_CH2_POL HRTIM_OUT_POL_POS
#endif
#if defined(CONFIG_STM32_HRTIM_TIMD) && !defined(HRTIM_TIMD_CH1_POL)
#  define HRTIM_TIMD_CH1_POL HRTIM_OUT_POL_POS
#endif
#if defined(CONFIG_STM32_HRTIM_TIMD) && !defined(HRTIM_TIMD_CH2_POL)
#  define HRTIM_TIMD_CH2_POL HRTIM_OUT_POL_POS
#endif
#if defined(CONFIG_STM32_HRTIM_TIME) && !defined(HRTIM_TIME_CH1_POL)
#  define HRTIM_TIME_CH1_POL HRTIM_OUT_POL_POS
#endif
#if defined(CONFIG_STM32_HRTIM_TIME) && !defined(HRTIM_TIME_CH2_POL)
#  define HRTIM_TIME_CH2_POL HRTIM_OUT_POL_POS
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_STM32_HRTIM_PWM
/* HRTIM Slave Timer Single Output Set/Reset Configuration */

struct stm32_hrtim_timout_s
{
  uint32_t set;                  /* Set events */
  uint32_t rst;                  /* Reset events */
  uint8_t  pol:1;                /* Output polarisation */
};

/* HRTIM Slave Timer Chopper Configuration */

#ifdef CONFIG_STM32_HRTIM_CHOPPER
struct stm32_hrtim_chopper_s
{
  uint16_t start_pulse:4;        /* Chopper start pulsewidth */
  uint16_t freq:4;               /* Chopper carrier frequency value */
  uint16_t duty:3;               /* Chopper duty cycle */
  uint16_t _res:5;               /* Reserved */
};
#endif

/* HRTIM Slave Timer Deadtime Configuration */

#ifdef CONFIG_STM32_HRTIM_DEADTIME
struct stm32_hrtim_deadtime_s
{
  uint8_t  en:1;                 /* Enable deadtime for timer */
  uint8_t  fsign_lock:1;         /* Deadtime falling sing lock */
  uint8_t  rsign_lock:1;         /* Deadtime rising sing lock */
  uint8_t  falling_lock:1;       /* Deadtime falling value lock */
  uint8_t  rising_lock:1;        /* Deadtime rising value lock */
  uint8_t  fsign:1;              /* Deadtime falling sign */
  uint8_t  rsign:1;              /* Deadtime rising sign */
  uint8_t  prescaler:3;          /* Deadtime prescaler */
  uint16_t rising:9;             /* Deadtime rising value */
  uint16_t falling:9;            /* Deadtime falling value */
};
#endif

/* HRTIM Timer Burst Mode Configuration */

struct stm32_hrtim_tim_burst_s
{
  uint8_t ch1_en:1;              /* Enable burst mode operation for CH1 */
  uint8_t ch1_state:1;           /* CH1 IDLE state */
  uint8_t ch2_en:1;              /* Enable burst mode operation for CH2 */
  uint8_t ch2_state:1;           /* CH2 IDLE state */
  uint8_t res:4;
};

/* HRTIM Timer PWM structure */

struct stm32_hrtim_pwm_s
{
  uint8_t pushpull:1;
  uint8_t res:7;
  struct stm32_hrtim_timout_s ch1; /* Channel 1 Set/Reset configuration */
  struct stm32_hrtim_timout_s ch2; /* Channel 2 Set/Reset configuration */

#ifdef CONFIG_STM32_HRTIM_BURST
  struct stm32_hrtim_tim_burst_s burst;
#endif
#ifdef CONFIG_STM32_HRTIM_CHOPPER
  struct stm32_hrtim_chopper_s chp;
#endif
#ifdef CONFIG_STM32_HRTIM_DEADTIME
  struct stm32_hrtim_deadtime_s dt;
#endif
};
#endif

/* HRTIM TIMER Capture structure */

#ifdef CONFIG_STM32_HRTIM_CAPTURE
struct stm32_hrtim_capture_s
{
  uint32_t cap1;                 /* Capture 1 configuration */
  uint32_t cap2;                 /* Capture 2 configuration */
};
#endif

/* Common data structure for Master Timer and Slave Timers */

struct stm32_hrtim_timcmn_s
{
  uint32_t base;                 /* The base address of the timer */
  uint64_t fclk;                 /* The frequency of the peripheral clock
                                  * that drives the timer module.
                                  */
  uint8_t prescaler:3;           /* Prescaler */
  uint8_t mode;                  /* Timer mode */
  uint8_t dac:2;                 /* DAC triggering */
  uint8_t reserved:3;
#ifdef CONFIG_STM32_HRTIM_INTERRUPTS
  uint16_t irq;                  /* interrupts configuration */
#endif
#ifdef CONFIG_STM32_HRTIM_DMA
  uint16_t dma;
#endif
#ifdef CONFIG_STM32_HRTIM_DMABURST
  uint32_t dmaburst;
#endif
};

/* Master Timer and Slave Timers structure */

struct stm32_hrtim_tim_s
{
  struct stm32_hrtim_timcmn_s tim; /* Common Timer data */
  void *priv;                      /* Timer private data */
};

/* Master Timer private data structure */

struct stm32_hrtim_master_priv_s
{
  uint32_t reserved;             /* reserved for future use */
};

/* Slave Timer (A-E) private data structure */

struct stm32_hrtim_slave_priv_s
{
#ifdef CONFIG_STM32_HRTIM_FAULTS
  uint8_t flt;                      /* Faults configuration.
                                     * First five bits are fault sources,
                                     * last bit is lock configuration.
                                     */
#ifdef CONFIG_STM32_HRTIM_AUTODELAYED
  uint8_t auto_delayed;              /* Auto-delayed mode configuration */
#endif
#endif
  uint16_t update;                  /* Update configuration */
  uint64_t reset;                   /* Timer reset events */
#ifdef CONFIG_STM32_HRTIM_PWM
  struct stm32_hrtim_pwm_s pwm;     /* PWM configuration */
#endif
#ifdef CONFIG_STM32_HRTIM_CAPTURE
  struct stm32_hrtim_capture_s cap; /* Capture configuration */
#endif
};

#ifdef CONFIG_STM32_HRTIM_FAULTS
/* Structure describes single HRTIM Fault configuration */

struct stm32_hrtim_fault_cfg_s
{
  uint8_t pol:1;                /* Fault polarity */
  uint8_t src:1;                /* Fault source */
  uint8_t filter:4;             /* Fault filter */
  uint8_t lock:1;               /* Fault lock */
  uint8_t _res:1;               /* Reserved */
};

/* Structure describes HRTIM Faults configuration */

struct stm32_hrtim_faults_s
{
#ifdef CONFIG_STM32_HRTIM_FAULT1
  struct stm32_hrtim_fault_cfg_s flt1;
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT2
  struct stm32_hrtim_fault_cfg_s flt2;
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT3
  struct stm32_hrtim_fault_cfg_s flt3;
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT4
  struct stm32_hrtim_fault_cfg_s flt4;
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT5
  struct stm32_hrtim_fault_cfg_s flt5;
#endif
};
#endif

#ifdef CONFIG_STM32_HRTIM_EVENTS
/* Structure describes single HRTIM External Event configuration */

struct stm32_hrtim_eev_cfg_s
{
  uint8_t filter:4;             /* External Event filter */
  uint8_t src:4;                /* External Event source */
  uint8_t pol:1;                /* External Event polarity */
  uint8_t sen:1;                /* External Event sensitivity */
  uint8_t mode:1;               /* External Event mode */
  uint8_t _res:5;
};

/* Structure describes HRTIM External Events configuration */

struct stm32_hrtim_eev_s
{
#ifdef CONFIG_STM32_HRTIM_EEV1
  struct stm32_hrtim_eev_cfg_s eev1;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV2
  struct stm32_hrtim_eev_cfg_s eev2;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV3
  struct stm32_hrtim_eev_cfg_s eev3;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV4
  struct stm32_hrtim_eev_cfg_s eev4;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV5
  struct stm32_hrtim_eev_cfg_s eev5;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV6
  struct stm32_hrtim_eev_cfg_s eev6;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV7
  struct stm32_hrtim_eev_cfg_s eev7;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV8
  struct stm32_hrtim_eev_cfg_s eev8;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV9
  struct stm32_hrtim_eev_cfg_s eev9;
#endif
#ifdef CONFIG_STM32_HRTIM_EEV10
  struct stm32_hrtim_eev_cfg_s eev10;
#endif
};
#endif

#ifdef HRTIM_HAVE_ADC
/* Structure describes HRTIM ADC triggering configuration */

struct stm32_hrtim_adc_s
{
#ifdef HRTIM_HAVE_ADC_TRG1
  uint32_t trg1;
#endif
#ifdef HRTIM_HAVE_ADC_TRG2
  uint32_t trg2;
#endif
#ifdef HRTIM_HAVE_ADC_TRG3
  uint32_t trg3;
#endif
#ifdef HRTIM_HAVE_ADC_TRG4
  uint32_t trg4;
#endif
};
#endif

/* Structure describes HRTIM Burst mode configuratione */

#ifdef CONFIG_STM32_HRTIM_BURST
struct stm32_hrtim_burst_s
{
  uint8_t clk:4;                /* Burst mode clock source */
  uint8_t presc:4;              /* Prescaler for f_HRTIM clock */
  uint32_t trg;                 /* Burst mode triggers */
};
#endif

/* This structure describes the configuration of HRTIM device */

struct stm32_hrtim_s
{
  uint32_t base;                     /* Base address of HRTIM block */
  struct stm32_hrtim_tim_s *master;  /* Master Timer */
#ifdef CONFIG_STM32_HRTIM_TIMA
  struct stm32_hrtim_tim_s *tima;    /* HRTIM Timer A */
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB
  struct stm32_hrtim_tim_s *timb;    /* HRTIM Timer B */
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC
  struct stm32_hrtim_tim_s *timc;    /* HRTIM Timer C */
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD
  struct stm32_hrtim_tim_s *timd;    /* HRTIM Timer D */
#endif
#ifdef CONFIG_STM32_HRTIM_TIME
  struct stm32_hrtim_tim_s *time;    /* HRTIM Timer E */
#endif
#ifdef CONFIG_STM32_HRTIM_FAULTS
  struct stm32_hrtim_faults_s *flt;  /* Faults configuration */
#endif
#ifdef CONFIG_STM32_HRTIM_EVENTS
  struct stm32_hrtim_eev_s *eev;     /* External Events configuration  */
#endif
#ifdef HRTIM_HAVE_ADC
  struct stm32_hrtim_adc_s *adc;     /* ADC triggering configuration */
#endif
#ifdef CONFIG_STM32_HRTIM_BURST
  struct stm32_hrtim_burst_s *burst; /* Burst mode configuration */
#endif
#ifdef CONFIG_STM32_HRTIM_INTERRUPTS
  uint32_t irq;                      /* Common interrupts configuration */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifndef CONFIG_STM32_HRTIM_DISABLE_CHARDRV

/* HRTIM Driver Methods */

static int stm32_hrtim_open(struct file *filep);
static int stm32_hrtim_close(struct file *filep);
static int stm32_hrtim_ioctl(struct file *filep, int cmd,
                             unsigned long arg);
#endif

/* HRTIM Register access */

static uint32_t hrtim_cmn_getreg(struct stm32_hrtim_s *priv,
                                 uint32_t offset);
static void hrtim_cmn_putreg(struct stm32_hrtim_s *priv, uint32_t offset,
                             uint32_t value);
#ifdef CONFIG_STM32_HRTIM_BURST
static void hrtim_cmn_modifyreg(struct stm32_hrtim_s *priv,
                                uint32_t offset, uint32_t clrbits,
                                uint32_t setbits);
#endif
static void hrtim_tim_putreg(struct stm32_hrtim_s *priv, uint8_t timer,
                             uint32_t offset, uint32_t value);
static void hrtim_tim_modifyreg(struct stm32_hrtim_s *priv,
                                uint8_t timer, uint32_t offset,
                                uint32_t clrbits, uint32_t setbits);

#ifdef CONFIG_DEBUG_TIMER_INFO
static void hrtim_dumpregs(struct stm32_hrtim_s *priv, uint8_t timer,
                           const char *msg);
#else
#  define hrtim_dumpregs(priv, timer, msg)
#endif

/* HRTIM helper */

static uint32_t hrtim_tim_getreg(struct stm32_hrtim_s *priv,
                                 uint8_t timer, uint32_t offset);
static struct stm32_hrtim_tim_s *
  hrtim_tim_get(struct stm32_hrtim_s *priv,
                                                   uint8_t timer);
#if defined(CONFIG_STM32_HRTIM_PWM) || defined(CONFIG_STM32_HRTIM_FAULTS)
static struct stm32_hrtim_slave_priv_s *
         hrtim_slave_get(struct stm32_hrtim_s *priv, uint8_t timer);
#endif
static uint32_t hrtim_base_get(struct stm32_hrtim_s *priv,
                               uint8_t timer);

/* Configuration */

static int hrtim_dll_cal(struct stm32_hrtim_s *priv);
static int hrtim_tim_clock_config(struct stm32_hrtim_s *priv,
                                  uint8_t timer, uint8_t pre);
static int hrtim_tim_clocks_config(struct stm32_hrtim_s *priv);
#if defined(CONFIG_STM32_HRTIM_PWM) || defined(CONFIG_STM32_HRTIM_SYNC)
static int hrtim_gpios_config(struct stm32_hrtim_s *priv);
#endif
#if defined(CONFIG_STM32_HRTIM_CAPTURE)
static int hrtim_capture_config(struct stm32_hrtim_s *priv);
static uint16_t hrtim_capture_get(struct hrtim_dev_s *dev, uint8_t timer,
                                  uint8_t index);
static int hrtim_soft_capture(struct hrtim_dev_s *dev, uint8_t timer,
                              uint8_t index);
#endif
#if defined(CONFIG_STM32_HRTIM_SYNC)
static int hrtim_synch_config(struct stm32_hrtim_s *priv);
#endif
#if defined(CONFIG_STM32_HRTIM_PWM)
static int hrtim_outputs_config(struct stm32_hrtim_s *priv);
static int hrtim_outputs_enable(struct hrtim_dev_s *dev,
                                uint16_t outputs, bool state);
static int hrtim_output_set_set(struct hrtim_dev_s *dev, uint16_t output,
                                uint32_t set);
static int hrtim_output_rst_set(struct hrtim_dev_s *dev, uint16_t output,
                                 uint32_t rst);
#endif
#ifdef HRTIM_HAVE_ADC
static int hrtim_adc_config(struct stm32_hrtim_s *priv);
#endif
#ifdef CONFIG_STM32_HRTIM_DAC
static int hrtim_dac_config(struct stm32_hrtim_s *priv);
#endif
#ifdef CONFIG_STM32_HRTIM_DMA
static int hrtim_dma_cfg(struct stm32_hrtim_s *priv);
static int hrtim_tim_dma_cfg(struct stm32_hrtim_s *priv, uint8_t timer,
                             uint16_t dma);
#endif
#ifdef CONFIG_STM32_HRTIM_DEADTIME
static int hrtim_deadtime_update(struct hrtim_dev_s *dev, uint8_t timer,
                                 uint8_t dt, uint16_t value);
static uint16_t hrtim_deadtime_get(struct hrtim_dev_s *dev,
                                   uint8_t timer, uint8_t dt);
static int hrtim_tim_deadtime_cfg(struct stm32_hrtim_s *priv,
                                  uint8_t timer);
static int hrtim_deadtime_config(struct stm32_hrtim_s *priv);
#endif
#ifdef CONFIG_STM32_HRTIM_CHOPPER
static int hrtim_chopper_enable(struct hrtim_dev_s *dev, uint8_t timer,
                                uint8_t chan, bool state);
static int hrtim_tim_chopper_cfg(struct stm32_hrtim_s *priv,
                                 uint8_t timer);
static int hrtim_chopper_config(struct stm32_hrtim_s *priv);
#endif
#ifdef CONFIG_STM32_HRTIM_BURST
static int hrtim_burst_enable(struct hrtim_dev_s *dev, bool state);
static int hrtim_burst_cmp_update(struct hrtim_dev_s *dev, uint16_t cmp);
static int hrtim_burst_per_update(struct hrtim_dev_s *dev, uint16_t per);
static uint16_t hrtim_burst_cmp_get(struct hrtim_dev_s *dev);
static uint16_t hrtim_burst_per_get(struct hrtim_dev_s *dev);
static int hrtim_burst_pre_update(struct hrtim_dev_s *dev, uint8_t pre);
static int hrtim_burst_pre_get(struct hrtim_dev_s *dev);
static int hrtim_burst_config(struct stm32_hrtim_s *priv);
#endif
#ifdef CONFIG_STM32_HRTIM_FAULTS
static int hrtim_faults_config(struct stm32_hrtim_s *priv);
static int hrtim_flt_cfg(struct stm32_hrtim_s *priv, uint8_t index);
static int hrtim_tim_faults_cfg(struct stm32_hrtim_s *priv,
                                uint8_t timer);
#endif
#ifdef CONFIG_STM32_HRTIM_EVENTS
static int hrtim_events_config(struct stm32_hrtim_s *priv);
static int hrtim_eev_cfg(struct stm32_hrtim_s *priv, uint8_t index);
#endif
#ifdef CONFIG_STM32_HRTIM_INTERRUPTS
static int hrtim_irq_config(struct stm32_hrtim_s *priv);
static uint16_t hrtim_irq_get(struct hrtim_dev_s *dev, uint8_t timer);
static int hrtim_irq_ack(struct hrtim_dev_s *dev, uint8_t timer,
                         int source);
#endif
static int hrtim_cmp_update(struct hrtim_dev_s *dev, uint8_t timer,
                            uint8_t index, uint16_t cmp);
static int hrtim_per_update(struct hrtim_dev_s *dev, uint8_t timer,
                            uint16_t per);
static int hrtim_rep_update(struct hrtim_dev_s *dev, uint8_t timer,
                            uint8_t rep);
static uint16_t hrtim_per_get(struct hrtim_dev_s *dev, uint8_t timer);
static uint16_t hrtim_cmp_get(struct hrtim_dev_s *dev, uint8_t timer,
                              uint8_t index);
static uint64_t hrtim_fclk_get(struct hrtim_dev_s *dev, uint8_t timer);
static int hrtim_soft_update(struct hrtim_dev_s *dev, uint8_t timer);
static int hrtim_soft_reset(struct hrtim_dev_s *dev, uint8_t timer);
static int hrtim_tim_freq_set(struct hrtim_dev_s *dev, uint8_t timer,
                              uint64_t freq);
static int hrtim_tim_enable(struct hrtim_dev_s  *dev, uint8_t timers,
                            bool state);
static int hrtim_tim_reset_set(struct stm32_hrtim_s *priv,
                               uint8_t timer, uint64_t reset);
static int hrtim_reset_config(struct stm32_hrtim_s *priv);
static int hrtim_tim_update_set(struct stm32_hrtim_s *priv,
                                uint8_t timer,
                                uint16_t update);
static int hrtim_update_config(struct stm32_hrtim_s *priv);

static void hrtim_tim_mode_set(struct stm32_hrtim_s *priv, uint8_t timer,
                               uint8_t mode);
static void hrtim_mode_config(struct stm32_hrtim_s *priv);

/* Initialization */

static int stm32_hrtimconfig(struct stm32_hrtim_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_STM32_HRTIM_DISABLE_CHARDRV
static const struct file_operations hrtim_fops =
{
  stm32_hrtim_open,   /* open */
  stm32_hrtim_close,  /* close */
  NULL,               /* read */
  NULL,               /* write */
  NULL,               /* seek */
  stm32_hrtim_ioctl,  /* ioctl */
  NULL                /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL              /* unlink */
#endif
};
#endif /* CONFIG_STM32_HRTIM_DISABLE_CHARDRV */

/* Master Timer data */

static struct stm32_hrtim_tim_s g_master =
{
  .tim =
  {
    .base  = STM32_HRTIM1_MASTER_BASE,

    /* If MASTER is disabled, we need only MASTER base */

#ifdef CONFIG_STM32_HRTIM_MASTER
    .fclk      = HRTIM_CLOCK / (1 << HRTIM_MASTER_PRESCALER),
    .prescaler = HRTIM_MASTER_PRESCALER,
    .mode      = HRTIM_MASTER_MODE,
#  ifdef CONFIG_STM32_HRTIM_MASTER_DAC
    .dac       = HRTIM_MASTER_DAC,
#  endif
#  ifdef CONFIG_STM32_HRTIM_MASTER_IRQ
    .irq       = HRTIM_MASTER_IRQ
#  endif
#  ifdef CONFIG_STM32_HRTIM_MASTER_DMA
    .dma       = HRTIM_MASTER_DMA
#  endif
#endif
  },
  .priv        = NULL,
};

#ifdef CONFIG_STM32_HRTIM_TIMA

/* Timer A private data */

static struct stm32_hrtim_slave_priv_s g_tima_priv =
{
  .update = HRTIM_TIMA_UPDATE,
  .reset  = HRTIM_TIMA_RESET,
#ifdef CONFIG_STM32_HRTIM_TIMA_PWM
  .pwm =
  {
#ifdef CONFIG_STM32_HRTIM_TIMA_PSHPLL
    .pushpull = 1,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMA_PWM_CH1
    .ch1 =
    {
      .set = HRTIM_TIMA_CH1_SET,
      .rst = HRTIM_TIMA_CH1_RST,
      .pol = HRTIM_TIMA_CH1_POL
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMA_PWM_CH2
    .ch2 =
    {
      .set = HRTIM_TIMA_CH2_SET,
      .rst = HRTIM_TIMA_CH2_RST,
      .pol = HRTIM_TIMA_CH2_POL
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMA_BURST
    .burst =
    {
#  ifdef CONFIG_STM32_HRTIM_TIMA_BURST_CH1
      .ch1_en    = 1,
      .ch1_state = HRTIM_TIMA_CH1_IDLE_STATE,
#  else
      .ch1_en    = 0,
#  endif
#  ifdef CONFIG_STM32_HRTIM_TIMA_BURST_CH2
      .ch2_en    = 1,
      .ch2_state = HRTIM_TIMA_CH2_IDLE_STATE
#  else
      .ch2_en    = 0,
#  endif
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMA_CHOP
    .chp =
    {
      .start_pulse = HRTIM_TIMA_CHOP_START,
      .duty        = HRTIM_TIMA_CHOP_DUTY,
      .freq        = HRTIM_TIMA_CHOP_FREQ
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMA_DT
    .dt =
    {
      .en           = 1,
      .fsign_lock   = HRTIM_TIMA_DT_FSLOCK,
      .rsign_lock   = HRTIM_TIMA_DT_RSLOCK,
      .falling_lock = HRTIM_TIMA_DT_FVLOCK,
      .rising_lock  = HRTIM_TIMA_DT_RVLOCK,
      .fsign        = HRTIM_TIMA_DT_FSIGN,
      .rsign        = HRTIM_TIMA_DT_RSIGN,
      .prescaler    = HRTIM_TIMA_DT_PRESCALER
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMA_CAP
  .cap =
  {
    .cap1 = HRTIM_TIMA_CAPTURE1,
    .cap2 = HRTIM_TIMA_CAPTURE2,
  }
#endif
};

/* Timer A data */

static struct stm32_hrtim_tim_s g_tima =
{
  .tim =
  {
    .base      = STM32_HRTIM1_TIMERA_BASE,
    .fclk      = HRTIM_CLOCK / (1 << HRTIM_TIMA_PRESCALER),
    .prescaler = HRTIM_TIMA_PRESCALER,
    .mode      = HRTIM_TIMA_MODE,
#ifdef CONFIG_STM32_HRTIM_TIMA_DAC
    .dac       = HRTIM_TIMA_DAC,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMA_IRQ
    .irq       = HRTIM_TIMA_IRQ,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMA_DMA
    .dma       = HRTIM_TIMA_DMA
#endif
  },
  .priv        = &g_tima_priv
};
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
/* Timer B private data */

static struct stm32_hrtim_slave_priv_s g_timb_priv =
{
  .update = HRTIM_TIMB_UPDATE,
  .reset  = HRTIM_TIMB_RESET,
#ifdef CONFIG_STM32_HRTIM_TIMB_PWM
  .pwm =
  {
#ifdef CONFIG_STM32_HRTIM_TIMB_PSHPLL
    .pushpull = 1,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB_PWM_CH1
    .ch1 =
    {
      .set = HRTIM_TIMB_CH1_SET,
      .rst = HRTIM_TIMB_CH1_RST,
      .pol = HRTIM_TIMB_CH1_POL
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB_PWM_CH2
    .ch2 =
    {
      .set = HRTIM_TIMB_CH2_SET,
      .rst = HRTIM_TIMB_CH2_RST,
      .pol = HRTIM_TIMB_CH2_POL
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB_BURST
    .burst =
    {
#  ifdef CONFIG_STM32_HRTIM_TIMB_BURST_CH1
      .ch1_en    = 1,
      .ch1_state = HRTIM_TIMB_CH1_IDLE_STATE,
#  else
      .ch1_en    = 0,
#  endif
#  ifdef CONFIG_STM32_HRTIM_TIMB_BURST_CH2
      .ch2_en    = 1,
      .ch2_state = HRTIM_TIMB_CH2_IDLE_STATE
#  else
      .ch2_en    = 0,
#  endif
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB_CHOP
    .chp =
    {
      .start_pulse = HRTIM_TIMB_CHOP_START,
      .duty        = HRTIM_TIMB_CHOP_DUTY,
      .freq        = HRTIM_TIMB_CHOP_FREQ
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB_DT
    .dt =
    {
      .en           = 1,
      .fsign_lock   = HRTIM_TIMB_DT_FSLOCK,
      .rsign_lock   = HRTIM_TIMB_DT_RSLOCK,
      .falling_lock = HRTIM_TIMB_DT_FVLOCK,
      .rising_lock  = HRTIM_TIMB_DT_RVLOCK,
      .fsign        = HRTIM_TIMB_DT_FSIGN,
      .rsign        = HRTIM_TIMB_DT_RSIGN,
      .prescaler    = HRTIM_TIMB_DT_PRESCALER
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB_CAP
  .cap =
  {
    .cap1 = HRTIM_TIMB_CAPTURE1,
    .cap2 = HRTIM_TIMB_CAPTURE2,
  }
#endif
};

/* Timer B data */

static struct stm32_hrtim_tim_s g_timb =
{
  .tim =
  {
    .base      = STM32_HRTIM1_TIMERB_BASE,
    .fclk      = HRTIM_CLOCK / (1 << HRTIM_TIMB_PRESCALER),
    .prescaler = HRTIM_TIMB_PRESCALER,
    .mode      = HRTIM_TIMB_MODE,
#ifdef CONFIG_STM32_HRTIM_TIMB_DAC
    .dac       = HRTIM_TIMB_DAC,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB_IRQ
    .irq       = HRTIM_TIMB_IRQ,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB_DMA
    .dma       = HRTIM_TIMB_DMA
#endif
  },
  .priv        = &g_timb_priv
};
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
/* Timer C private data */

static struct stm32_hrtim_slave_priv_s g_timc_priv =
{
  .update = HRTIM_TIMC_UPDATE,
  .reset  = HRTIM_TIMC_RESET,
#ifdef CONFIG_STM32_HRTIM_TIMC_PWM
  .pwm =
  {
#ifdef CONFIG_STM32_HRTIM_TIMC_PSHPLL
    .pushpull = 1,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC_PWM_CH1
    .ch1 =
    {
      .set = HRTIM_TIMC_CH1_SET,
      .rst = HRTIM_TIMC_CH1_RST,
      .pol = HRTIM_TIMC_CH1_POL
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC_PWM_CH2
    .ch2 =
    {
      .set = HRTIM_TIMC_CH2_SET,
      .rst = HRTIM_TIMC_CH2_RST,
      .pol = HRTIM_TIMC_CH2_POL
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC_BURST
    .burst =
    {
#  ifdef CONFIG_STM32_HRTIM_TIMC_BURST_CH1
      .ch1_en    = 1,
      .ch1_state = HRTIM_TIMC_CH1_IDLE_STATE,
#  else
      .ch1_en    = 0,
#  endif
#  ifdef CONFIG_STM32_HRTIM_TIMC_BURST_CH2
      .ch2_en    = 1,
      .ch2_state = HRTIM_TIMC_CH2_IDLE_STATE
#  else
      .ch2_en    = 0,
#  endif
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC_CHOP
    .chp =
    {
      .start_pulse = HRTIM_TIMC_CHOP_START,
      .duty        = HRTIM_TIMC_CHOP_DUTY,
      .freq        = HRTIM_TIMC_CHOP_FREQ
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC_DT
    .dt =
    {
      .en           = 1,
      .fsign_lock   = HRTIM_TIMC_DT_FSLOCK,
      .rsign_lock   = HRTIM_TIMC_DT_RSLOCK,
      .falling_lock = HRTIM_TIMC_DT_FVLOCK,
      .rising_lock  = HRTIM_TIMC_DT_RVLOCK,
      .fsign        = HRTIM_TIMC_DT_FSIGN,
      .rsign        = HRTIM_TIMC_DT_RSIGN,
      .prescaler    = HRTIM_TIMC_DT_PRESCALER
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC_CAP
  .cap =
  {
    .cap1 = HRTIM_TIMC_CAPTURE1,
    .cap2 = HRTIM_TIMC_CAPTURE2,
  }
#endif
};

/* Timer C data */

static struct stm32_hrtim_tim_s g_timc =
{
  .tim =
  {
    .base      = STM32_HRTIM1_TIMERC_BASE,
    .fclk      = HRTIM_CLOCK / (1 << HRTIM_TIMC_PRESCALER),
    .prescaler = HRTIM_TIMC_PRESCALER,
    .mode      = HRTIM_TIMC_MODE,
#ifdef CONFIG_STM32_HRTIM_TIMC_DAC
    .dac       = HRTIM_TIMC_DAC,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC_IRQ
    .irq       = HRTIM_TIMC_IRQ,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC_DMA
    .dma       = HRTIM_TIMC_DMA
#endif
  },
  .priv        = &g_timc_priv
};
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
/* Timer D private data */

static struct stm32_hrtim_slave_priv_s g_timd_priv =
{
  .update = HRTIM_TIMD_UPDATE,
  .reset  = HRTIM_TIMD_RESET,
#ifdef CONFIG_STM32_HRTIM_TIMD_PWM
  .pwm =
  {
#ifdef CONFIG_STM32_HRTIM_TIMD_PSHPLL
    .pushpull = 1,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD_PWM_CH1
    .ch1 =
    {
      .set = HRTIM_TIMD_CH1_SET,
      .rst = HRTIM_TIMD_CH1_RST,
      .pol = HRTIM_TIMD_CH1_POL
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD_PWM_CH2
    .ch2 =
    {
      .set = HRTIM_TIMD_CH2_SET,
      .rst = HRTIM_TIMD_CH2_RST,
      .pol = HRTIM_TIMD_CH2_POL
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD_BURST
    .burst =
    {
#  ifdef CONFIG_STM32_HRTIM_TIMD_BURST_CH1
      .ch1_en    = 1,
      .ch1_state = HRTIM_TIMD_CH1_IDLE_STATE,
#  else
      .ch1_en    = 0,
#  endif
#  ifdef CONFIG_STM32_HRTIM_TIMD_BURST_CH2
      .ch2_en    = 1,
      .ch2_state = HRTIM_TIMD_CH2_IDLE_STATE
#  else
      .ch2_en    = 0,
#  endif
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD_CHOP
    .chp =
    {
      .start_pulse = HRTIM_TIMD_CHOP_START,
      .duty        = HRTIM_TIMD_CHOP_DUTY,
      .freq        = HRTIM_TIMD_CHOP_FREQ
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD_DT
    .dt =
    {
      .en           = 1,
      .fsign_lock   = HRTIM_TIMD_DT_FSLOCK,
      .rsign_lock   = HRTIM_TIMD_DT_RSLOCK,
      .falling_lock = HRTIM_TIMD_DT_FVLOCK,
      .rising_lock  = HRTIM_TIMD_DT_RVLOCK,
      .fsign        = HRTIM_TIMD_DT_FSIGN,
      .rsign        = HRTIM_TIMD_DT_RSIGN,
      .prescaler    = HRTIM_TIMD_DT_PRESCALER
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD_CAP
  .cap =
  {
    .cap1 = HRTIM_TIMD_CAPTURE1,
    .cap2 = HRTIM_TIMD_CAPTURE2,
  }
#endif
};

/* Timer D data */

static struct stm32_hrtim_tim_s g_timd =
{
  .tim =
  {
    .base      = STM32_HRTIM1_TIMERD_BASE,
    .fclk      = HRTIM_CLOCK / (1 << HRTIM_TIMD_PRESCALER),
    .prescaler = HRTIM_TIMD_PRESCALER,
    .mode      = HRTIM_TIMD_MODE,
#ifdef CONFIG_STM32_HRTIM_TIMD_DAC
    .dac       = HRTIM_TIMD_DAC,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD_IRQ
    .irq       = HRTIM_TIMD_IRQ,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD_DMA
    .dma       = HRTIM_TIMD_DMA
#endif
  },
  .priv        = &g_timd_priv
};
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
/* Timer E private data */

static struct stm32_hrtim_slave_priv_s g_time_priv =
{
  .update = HRTIM_TIME_UPDATE,
  .reset  = HRTIM_TIME_RESET,
#ifdef CONFIG_STM32_HRTIM_TIME_PWM
  .pwm =
  {
#ifdef CONFIG_STM32_HRTIM_TIME_PSHPLL
    .pushpull = 1,
#endif
#ifdef CONFIG_STM32_HRTIM_TIME_PWM_CH1
    .ch1 =
    {
      .set = HRTIM_TIME_CH1_SET,
      .rst = HRTIM_TIME_CH1_RST,
      .pol = HRTIM_TIME_CH1_POL
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIME_PWM_CH2
    .ch2 =
    {
      .set = HRTIM_TIME_CH2_SET,
      .rst = HRTIM_TIME_CH2_RST,
      .pol = HRTIM_TIME_CH1_POL
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIME_BURST
    .burst =
    {
#  ifdef CONFIG_STM32_HRTIM_TIME_BURST_CH1
      .ch1_en    = 1,
      .ch1_state = HRTIM_TIME_CH1_IDLE_STATE,
#  else
      .ch1_en    = 0,
#  endif
#  ifdef CONFIG_STM32_HRTIM_TIME_BURST_CH2
      .ch2_en    = 1,
      .ch2_state = HRTIM_TIME_CH2_IDLE_STATE
#  else
      .ch2_en    = 0,
#  endif
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIME_CHOP
    .chp =
    {
      .start_pulse = HRTIM_TIME_CHOP_START,
      .duty        = HRTIM_TIME_CHOP_DUTY,
      .freq        = HRTIM_TIME_CHOP_FREQ
    },
#endif
#ifdef CONFIG_STM32_HRTIM_TIME_DT
    .dt =
    {
      .en           = 1,
      .fsign_lock   = HRTIM_TIME_DT_FSLOCK,
      .rsign_lock   = HRTIM_TIME_DT_RSLOCK,
      .falling_lock = HRTIM_TIME_DT_FVLOCK,
      .rising_lock  = HRTIM_TIME_DT_RVLOCK,
      .fsign        = HRTIM_TIME_DT_FSIGN,
      .rsign        = HRTIM_TIME_DT_RSIGN,
      .prescaler    = HRTIM_TIME_DT_PRESCALER
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_HRTIM_TIME_CAP
  .cap =
  {
    .cap1 = HRTIM_TIME_CAPTURE1,
    .cap2 = HRTIM_TIME_CAPTURE2,
  }
#endif
};

/* Timer E data */

static struct stm32_hrtim_tim_s g_time =
{
  .tim =
  {
    .base      = STM32_HRTIM1_TIMERE_BASE,
    .fclk      = HRTIM_CLOCK / (1 << HRTIM_TIME_PRESCALER),
    .prescaler = HRTIM_TIME_PRESCALER,
    .mode      = HRTIM_TIME_MODE,
#ifdef CONFIG_STM32_HRTIM_TIME_DAC
    .dac       = HRTIM_TIME_DAC,
#endif
#ifdef CONFIG_STM32_HRTIM_TIME_IRQ
    .irq       = HRTIM_TIME_IRQ,
#endif
#ifdef CONFIG_STM32_HRTIM_TIME_DMA
    .dma       = HRTIM_TIME_DMA
#endif
  },
  .priv        = &g_time_priv
};
#endif

/* Faults data */

#ifdef CONFIG_STM32_HRTIM_FAULTS
struct stm32_hrtim_faults_s g_flt =
{
#ifdef CONFIG_STM32_HRTIM_FAULT1
  .flt1 =
  {
    .pol    = HRTIM_FAULT1_POL,
    .src    = HRTIM_FAULT1_SRC,
    .filter = HRTIM_FAULT1_FILTER,
    .lock   = HRTIM_FAULT1_LOCK,
  },
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT2
  .flt2 =
  {
    .pol    = HRTIM_FAULT2_POL,
    .src    = HRTIM_FAULT2_SRC,
    .filter = HRTIM_FAULT2_FILTER,
    .lock   = HRTIM_FAULT2_LOCK,
  },
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT3
  .flt3 =
  {
    .pol    = HRTIM_FAULT3_POL,
    .src    = HRTIM_FAULT3_SRC,
    .filter = HRTIM_FAULT3_FILTER,
    .lock   = HRTIM_FAULT3_LOCK,
  },
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT4
  .flt2 =
  {
    .pol    = HRTIM_FAULT4_POL,
    .src    = HRTIM_FAULT4_SRC,
    .filter = HRTIM_FAULT4_FILTER,
    .lock   = HRTIM_FAULT4_LOCK,
  },
#endif
#ifdef CONFIG_STM32_HRTIM_FAULT5
  .flt2 =
  {
    .pol    = HRTIM_FAULT5_POL,
    .src    = HRTIM_FAULT5_SRC,
    .filter = HRTIM_FAULT5_FILTER,
    .lock   = HRTIM_FAULT5_LOCK,
  },
#endif
};
#endif

/* External Events data */

#ifdef CONFIG_STM32_HRTIM_EVENTS
struct stm32_hrtim_eev_s g_eev =
{
#ifdef CONFIG_STM32_HRTIM_EEV1
  .eev1 =
  {
    .filter = HRTIM_EEV1_FILTER,
    .src    = HRTIM_EEV1_SRC,
    .pol    = HRTIM_EEV1_POL,
    .sen    = HRTIM_EEV1_SEN,
    .mode   = HRTIM_EEV1_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV2
  .eev2 =
  {
    .filter = HRTIM_EEV2_FILTER,
    .src    = HRTIM_EEV2_SRC,
    .pol    = HRTIM_EEV2_POL,
    .sen    = HRTIM_EEV2_SEN,
    .mode   = HRTIM_EEV2_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV3
  .eev3 =
  {
    .filter = HRTIM_EEV3_FILTER,
    .src    = HRTIM_EEV3_SRC,
    .pol    = HRTIM_EEV3_POL,
    .sen    = HRTIM_EEV3_SEN,
    .mode   = HRTIM_EEV3_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV4
  .eev4 =
  {
    .filter = HRTIM_EEV4_FILTER,
    .src    = HRTIM_EEV4_SRC,
    .pol    = HRTIM_EEV4_POL,
    .sen    = HRTIM_EEV4_SEN,
    .mode   = HRTIM_EEV4_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV5
  .eev5 =
  {
    .filter = HRTIM_EEV5_FILTER,
    .src    = HRTIM_EEV5_SRC,
    .pol    = HRTIM_EEV5_POL,
    .sen    = HRTIM_EEV5_SEN,
    .mode   = HRTIM_EEV5_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV6
  .eev6 =
  {
    .filter = HRTIM_EEV6_FILTER,
    .src    = HRTIM_EEV6_SRC,
    .pol    = HRTIM_EEV6_POL,
    .sen    = HRTIM_EEV6_SEN,
    .mode   = HRTIM_EEV6_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV7
  .eev7 =
  {
    .filter = HRTIM_EEV7_FILTER,
    .src    = HRTIM_EEV7_SRC,
    .pol    = HRTIM_EEV7_POL,
    .sen    = HRTIM_EEV7_SEN,
    .mode   = HRTIM_EEV7_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV8
  .eev8 =
  {
    .filter = HRTIM_EEV8_FILTER,
    .src    = HRTIM_EEV8_SRC,
    .pol    = HRTIM_EEV8_POL,
    .sen    = HRTIM_EEV8_SEN,
    .mode   = HRTIM_EEV8_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV9
  .eev9 =
  {
    .filter = HRTIM_EEV9_FILTER,
    .src    = HRTIM_EEV9_SRC,
    .pol    = HRTIM_EEV9_POL,
    .sen    = HRTIM_EEV9_SEN,
    .mode   = HRTIM_EEV9_MODE,
  }
#endif
#ifdef CONFIG_STM32_HRTIM_EEV10
  .eev10 =
  {
    .filter = HRTIM_EEV10_FILTER,
    .src    = HRTIM_EEV10_SRC,
    .pol    = HRTIM_EEV10_POL,
    .sen    = HRTIM_EEV10_SEN,
    .mode   = HRTIM_EEV10_MODE,
  }
#endif
};
#endif

/* ADC triggering data */

#ifdef HRTIM_HAVE_ADC
struct stm32_hrtim_adc_s g_adc =
{
#ifdef HRTIM_HAVE_ADC_TRG1
  .trg1 = HRTIM_ADC_TRG1,
#endif
#ifdef HRTIM_HAVE_ADC_TRG2
  .trg2 = HRTIM_ADC_TRG2,
#endif
#ifdef HRTIM_HAVE_ADC_TRG3
  .trg3 = HRTIM_ADC_TRG3,
#endif
#ifdef HRTIM_HAVE_ADC_TRG4
  .trg4 = HRTIM_ADC_TRG4
#endif
};
#endif

/* Burst mode data */

#ifdef CONFIG_STM32_HRTIM_BURST
struct stm32_hrtim_burst_s g_burst =
{
  .clk   = HRTIM_BURST_CLOCK,
  .presc = HRTIM_BURST_PRESCALER,
  .trg   = HRTIM_BURST_TRIGGERS
};
#endif

/* HRTIM1 private data */

static struct stm32_hrtim_s g_hrtim1priv =
{
  .master   = &g_master,
  .base     = STM32_HRTIM1_BASE,
#ifdef CONFIG_STM32_HRTIM_TIMA
  .tima     = &g_tima,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB
  .timb     = &g_timb,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC
  .timc     = &g_timc,
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD
  .timd     = &g_timd,
#endif
#ifdef CONFIG_STM32_HRTIM_TIME
  .time     = &g_time,
#endif
#ifdef CONFIG_STM32_HRTIM_FAULTS
  .flt      = &g_flt,
#endif
#ifdef CONFIG_STM32_HRTIM_EVENTS
  .eev      = &g_eev,
#endif
#ifdef HRTIM_HAVE_ADC
  .adc      = &g_adc,
#endif
#ifdef CONFIG_STM32_HRTIM_BURST
  .burst      = &g_burst,
#endif
#ifdef CONFIG_STM32_HRTIM_COMMON_IRQ
  .irq      = HRTIM_IRQ_COMMON,
#endif
};

/* HRTIM interface */

static const struct stm32_hrtim_ops_s g_hrtim1ops =
{
  .cmp_update     = hrtim_cmp_update,
  .per_update     = hrtim_per_update,
  .rep_update     = hrtim_rep_update,
  .per_get        = hrtim_per_get,
  .cmp_get        = hrtim_cmp_get,
  .fclk_get       = hrtim_fclk_get,
  .soft_update    = hrtim_soft_update,
  .soft_reset     = hrtim_soft_reset,
  .freq_set       = hrtim_tim_freq_set,
  .tim_enable     = hrtim_tim_enable,
#ifdef CONFIG_STM32_HRTIM_INTERRUPTS
  .irq_ack        = hrtim_irq_ack,
  .irq_get        = hrtim_irq_get,
#endif
#ifdef CONFIG_STM32_HRTIM_PWM
  .outputs_enable = hrtim_outputs_enable,
  .output_rst_set = hrtim_output_rst_set,
  .output_set_set = hrtim_output_set_set,
#endif
#ifdef CONFIG_STM32_HRTIM_BURST
  .burst_enable  = hrtim_burst_enable,
  .burst_cmp_set = hrtim_burst_cmp_update,
  .burst_per_set = hrtim_burst_per_update,
  .burst_pre_set = hrtim_burst_pre_update,
  .burst_cmp_get = hrtim_burst_cmp_get,
  .burst_per_get = hrtim_burst_per_get,
  .burst_pre_get = hrtim_burst_pre_get,
#endif
#ifdef CONFIG_STM32_HRTIM_CHOPPER
  .chopper_enable = hrtim_chopper_enable,
#endif
#ifdef CONFIG_STM32_HRTIM_DEADTIME
  .deadtime_update = hrtim_deadtime_update,
  .deadtime_get    = hrtim_deadtime_get,
#endif
#ifdef CONFIG_STM32_HRTIM_CAPTURE
  .capture_get   = hrtim_capture_get,
  .soft_capture  = hrtim_soft_capture,
#endif
};

/* HRTIM device structure */

struct hrtim_dev_s g_hrtim1dev =
{
  .hd_ops   = &g_hrtim1ops,
  .hd_priv  = &g_hrtim1priv,
  .initialized = false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_STM32_HRTIM_DISABLE_CHARDRV

/****************************************************************************
 * Name: stm32_hrtim_open
 *
 * Description:
 *   This function is called whenever the HRTIM device is opened.
 *
 ****************************************************************************/

static int stm32_hrtim_open(struct file *filep)
{
#warning "stm32_hrtim_open: missing logic"
  return OK;
}

/****************************************************************************
 * Name: stm32_hrtim_close
 *
 * Description:
 *   This function is called when the HRTIM device is closed.
 *
 ****************************************************************************/

static int stm32_hrtim_close(struct file *filep)
{
#warning "smt32_hrtim_close: missing logic"
  return OK;
}

/****************************************************************************
 * Name: stm32_hrtim_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the HRTIM work is done.
 *
 ****************************************************************************/

static int stm32_hrtim_ioctl(struct file *filep, int cmd,
                             unsigned long arg)
{
  struct inode *inode = filep->f_inode;
  struct hrtim_dev_s  *dev;
  struct stm32_hrtim_s *hrtim;
  int ret;

  tmrinfo("cmd: %d arg: %ld\n", cmd, arg);
  dev = inode->i_private;
  DEBUGASSERT(dev != NULL);
  hrtim = dev->hd_priv;

  UNUSED(hrtim);

#warning "smt32_hrtim_ioctl: missing logic"

  /* Handle HRTIM ioctl commands */

  switch (cmd)
    {
      default:
        {
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

#endif /* CONFIG_STM32_HRTIM_DISABLE_CHARDRV */

/****************************************************************************
 * Name: hrtim_cmn_getreg
 *
 * Description:
 *   Read the value of an HRTIM register.
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t hrtim_cmn_getreg(struct stm32_hrtim_s *priv,
                                 uint32_t offset)
{
  return getreg32(priv->base + STM32_HRTIM_CMN_OFFSET + offset);
}

/****************************************************************************
 * Name: hrtim_cmn_putreg
 *
 * Description:
 *   Write a value to an HRTIM register.
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hrtim_cmn_putreg(struct stm32_hrtim_s *priv, uint32_t offset,
                             uint32_t value)
{
  putreg32(value, priv->base + STM32_HRTIM_CMN_OFFSET + offset);
}

/****************************************************************************
 * Name: hrtim__modifyreg
 *
 * Description:
 *   Modify the value of an HRTIM register (not atomic).
 *
 * Input Parameters:
 *   priv    - A reference to the HRTIM block
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_HRTIM_BURST
static void hrtim_cmn_modifyreg(struct stm32_hrtim_s *priv,
                                uint32_t offset, uint32_t clrbits,
                                uint32_t setbits)
{
  hrtim_cmn_putreg(priv, offset,
                  (hrtim_cmn_getreg(priv, offset) & ~clrbits) | setbits);
}
#endif

/****************************************************************************
 * Name: hrtim_tim_get
 *
 * Description:
 *   Get Timer data structure for given HRTIM Timer index
 *
 * Input Parameters:
 *   priv    - A reference to the HRTIM block
 *   timer   - An HRTIM Timer index to get
 *
 * Returned Value:
 *   Pointer to timer structure on success, NULL on failure
 *
 ****************************************************************************/

static struct stm32_hrtim_tim_s *
hrtim_tim_get(struct stm32_hrtim_s *priv, uint8_t timer)
{
  struct stm32_hrtim_tim_s *tim;

  switch (timer)
    {
      case HRTIM_TIMER_MASTER:
        {
          tim = priv->master;
          break;
        }

#ifdef CONFIG_STM32_HRTIM_TIMA
      case HRTIM_TIMER_TIMA:
        {
          tim = priv->tima;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
      case HRTIM_TIMER_TIMB:
        {
          tim = priv->timb;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
      case HRTIM_TIMER_TIMC:
        {
          tim = priv->timc;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
      case HRTIM_TIMER_TIMD:
        {
          tim = priv->timd;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
      case HRTIM_TIMER_TIME:
        {
          tim = priv->time;
          break;
        }
#endif

      default:
        {
          tmrerr("ERROR: No such timer index: %d\n", timer);
          tim = NULL;
        }
    }

  return tim;
}

/****************************************************************************
 * Name: hrtim_slave_get
 *
 * Description:
 *   Get Slave private data structure for given HRTIM Timer index
 *
 * Input Parameters:
 *   priv    - A reference to the HRTIM block
 *   timer   - An HRTIM Slave Timer index to get
 *
 * Returned Value:
 *   Pointer to slave structure  success, NULL on failure
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_HRTIM_PWM) || defined(CONFIG_STM32_HRTIM_FAULTS)
static struct stm32_hrtim_slave_priv_s *
hrtim_slave_get(struct stm32_hrtim_s *priv, uint8_t timer)
{
  struct stm32_hrtim_tim_s *tim;
  struct stm32_hrtim_slave_priv_s *slave;

  /* Sanity checking */

  if (timer == HRTIM_TIMER_MASTER || timer == HRTIM_TIMER_COMMON)
    {
      slave = NULL;
      goto errout;
    }

  /* Get Timer data structure */

  tim = hrtim_tim_get(priv, timer);
  if (tim == NULL)
    {
      slave = NULL;
      goto errout;
    }

  /* Get Slave Timer data */

  slave = (struct stm32_hrtim_slave_priv_s *)tim->priv;

errout:
  return slave;
}
#endif

/****************************************************************************
 * Name: hrtim_base_get
 *
 * Description:
 *   Get base address offset for given HRTIM Timer index
 *
 * Input Parameters:
 *   priv    - A reference to the HRTIM block
 *   timer   - An HRTIM Timer index to get
 *
 * Returned Value:
 *   Base address offset for given Timer index
 *
 ****************************************************************************/

static uint32_t hrtim_base_get(struct stm32_hrtim_s *priv, uint8_t timer)
{
  struct stm32_hrtim_tim_s *tim;
  uint32_t base = 0;

  tim = hrtim_tim_get(priv, timer);
  if (tim == NULL)
    {
      base = 0;
      goto errout;
    }

  base = tim->tim.base;

errout:
  return base;
}

/****************************************************************************
 * Name: hrtim_tim_getreg
 *
 * Description:
 *   Read the value of an HRTIM Timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   tim    - An HRTIM timer index
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t hrtim_tim_getreg(struct stm32_hrtim_s *priv,
                                 uint8_t timer, uint32_t offset)
{
  uint32_t base = 0;

  base = hrtim_base_get(priv, timer);
  if (base < 0)
    {
      return 0;
    }

  return getreg32(base + offset);
}

/****************************************************************************
 * Name: hrtim_tim_putreg
 *
 * Description:
 *   Write a value to an HRTIM Timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   timer  - An HRTIM Timer index
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hrtim_tim_putreg(struct stm32_hrtim_s *priv, uint8_t timer,
                             uint32_t offset, uint32_t value)
{
  uint32_t base = 0;

  base = hrtim_base_get(priv, timer);
  if (base > 0)
    {
      putreg32(value, base + offset);
    }
}

/****************************************************************************
 * Name: hrtim_tim_modifyreg
 *
 * Description:
 *   Modify the value of an HRTIM Timer register (not atomic).
 *
 * Input Parameters:
 *   priv    - A reference to the HRTIM block
 *   timer   - An HRTIM Timer index
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hrtim_tim_modifyreg(struct stm32_hrtim_s *priv,
                                uint8_t timer, uint32_t offset,
                                uint32_t clrbits, uint32_t setbits)
{
  hrtim_tim_putreg(priv, timer, offset,
                   (hrtim_tim_getreg(priv, timer, offset) & ~clrbits) |
                   setbits);
}

#ifdef CONFIG_DEBUG_TIMER_INFO
static void hrtim_dumpregs(struct stm32_hrtim_s *priv, uint8_t timer,
                           const char *msg)
{
  tmrinfo("%s:\n", msg);

  switch (timer)
    {
      case HRTIM_TIMER_MASTER:
        {
          tmrinfo("\tCR:\t0x%08x\tISR:\t0x%08x\tICR:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET),
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_ISR_OFFSET),
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_ICR_OFFSET));

          tmrinfo("\tDIER:\t0x%08x\tCNTR:\t0x%08x\tPER:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_DIER_OFFSET),
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CNTR_OFFSET),
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_PER_OFFSET));

          tmrinfo("\tREP:\t0x%08x\tCMP1:\t0x%08x\tCMP2:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_REPR_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CMP1R_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CMP2R_OFFSET));

          tmrinfo("\tCMP3:\t0x%08x\tCMP4:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CMP3R_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CMP4R_OFFSET));
          break;
        }

#ifdef CONFIG_STM32_HRTIM_TIMA
      case HRTIM_TIMER_TIMA:
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB
      case HRTIM_TIMER_TIMB:
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC
      case HRTIM_TIMER_TIMC:
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD
      case HRTIM_TIMER_TIMD:
#endif
#ifdef CONFIG_STM32_HRTIM_TIME
      case HRTIM_TIMER_TIME:
#endif
        {
          tmrinfo("\tCR:\t0x%08x\tISR:\t0x%08x\tICR:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET),
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_ISR_OFFSET),
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_ICR_OFFSET));

          tmrinfo("\tDIER:\t0x%08x\tCNTR:\t0x%08x\tPER:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_DIER_OFFSET),
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CNTR_OFFSET),
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_PER_OFFSET));

          tmrinfo("\tREP:\t0x%08x\tCMP1:\t0x%08x\tCMP1C:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_REPR_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CMP1R_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CMP1CR_OFFSET));

          tmrinfo("\tCMP2:\t0x%08x\tCMP3:\t0x%08x\tCMP4:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CMP2R_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CMP3R_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CMP4R_OFFSET));

          tmrinfo("\tCPT1:\t0x%08x\tCPT2:\t0x%08x\tDTR:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CPT1R_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CPT2R_OFFSET),
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_DTR_OFFSET));

          tmrinfo("\tSET1:\t0x%08x\tRST1:\t0x%08x\tSET2:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_SET1R_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_RST1R_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_SET2R_OFFSET));

          tmrinfo("\tRST2:\t0x%08x\tEEF1:\t0x%08x\tEEF2:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_RST2R_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_EEFR1_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_EEFR2_OFFSET));

          tmrinfo("\tRSTR:\t0x%08x\tCHPR:\t0x%08x\tCPT1C:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_RSTR_OFFSET),
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CHPR_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CPT1CR_OFFSET));

          tmrinfo("\tCPT2C:\t0x%08x\tOUT:\t0x%08x\tFLT:\t0x%08x\n",
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_CPT2CR_OFFSET),
                  hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_OUTR_OFFSET),
                  hrtim_tim_getreg(priv, timer,
                                   STM32_HRTIM_TIM_FLTR_OFFSET));

          break;
        }

      case HRTIM_TIMER_COMMON:
        {
          tmrinfo("\tCR1:\t0x%08x\tCR2:\t0x%08x\tISR:\t0x%08x\n",
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_CR1_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_CR2_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_ISR_OFFSET));

          tmrinfo("\tICR:\t0x%08x\tIER:\t0x%08x\tOENR:\t0x%08x\n",
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_ICR_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_IER_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_OENR_OFFSET));

          tmrinfo("\tODISR:\t0x%08x\tODSR:\t0x%08x\tBMCR:\t0x%08x\n",
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_ODISR_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_ODSR_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BMCR_OFFSET));

          tmrinfo("\tBMTRG:\t0x%08x\tBMCMPR:\t0x%08x\tBMPER:\t0x%08x\n",
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BMTRGR_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BMCMPR_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BMPER_OFFSET));

          tmrinfo("\tADC1R:\t0x%08x\tADC2R:\t0x%08x\tADC3R:\t0x%08x\n",
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_ADC1R_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_ADC2R_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_ADC3R_OFFSET));

          tmrinfo("\tADC4R:\t0x%08x\tDLLCR:\t0x%08x\tFLTIN1:\t0x%08x\n",
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_ADC4R_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_DLLCR_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_FLTINR1_OFFSET));

          tmrinfo("\tFLTIN2:\t0x%08x\tBDMUPD:\t0x%08x\tBDTAUP:\t0x%08x\n",
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_FLTINR2_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BDMUPDR_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BDTAUPR_OFFSET));

          tmrinfo("\tBDTBUP: 0x%08x\tBDTCUP:\t0x%08x\tBDTDUP:\t0x%08x\n",
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BDTBUPR_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BDTCUPR_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BDTDUPR_OFFSET));

          tmrinfo("\tBDTEUP:\t0x%08x\tBDMAD:\t0x%08x\n",
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BDTEUPR_OFFSET),
                  hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BDMADR_OFFSET));

          break;
        }

      default:
        {
          tmrerr("ERROR: No such timer index: %d\n", timer);
          break;
        }
    }
}
#endif

/****************************************************************************
 * Name: hrtim_dll_cal
 *
 * Description:
 *   Calibrate HRTIM DLL
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_dll_cal(struct stm32_hrtim_s *priv)
{
  uint32_t regval = 0;

#ifdef CONFIG_STM32_HRTIM_PERIODIC_CAL
  /* Configure calibration rate */

  regval |= HRTIM_DLLCR_CAL_RATE;

  /* Enable Periodic calibration */

  regval |= HRTIM_DLLCR_CALEN;

  /* CALEN must not be set simultaneously with CAL bit */

  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_DLLCR_OFFSET, regval);
#endif

  /* DLL Calibration Start */

  regval |= HRTIM_DLLCR_CAL;

  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_DLLCR_OFFSET, regval);

  while ((hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_ISR_OFFSET) &
          HRTIM_ISR_DLLRDY) == 0);

  return OK;
}

/****************************************************************************
 * Name: hrtim_tim_clock_config
 *
 * Description:
 *   Configure HRTIM Timer clock
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *   timer  - An HRTIM Timer index
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_clock_config(struct stm32_hrtim_s *priv,
                                  uint8_t timer, uint8_t pre)
{
  int ret = OK;
  uint32_t regval = 0;

  regval = hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET);

  switch (pre)
    {
      case HRTIM_PRESCALER_1:
        {
          regval |= HRTIM_CMNCR_CKPSC_NODIV;
          break;
        }

      case HRTIM_PRESCALER_2:
        {
          regval |= HRTIM_CMNCR_CKPSC_d2;
          break;
        }

      case HRTIM_PRESCALER_4:
        {
          regval |= HRTIM_CMNCR_CKPSC_d4;
          break;
        }

      case HRTIM_PRESCALER_8:
        {
          regval |= HRTIM_CMNCR_CKPSC_d8;
          break;
        }

      case HRTIM_PRESCALER_16:
        {
          regval |= HRTIM_CMNCR_CKPSC_d16;
          break;
        }

      case HRTIM_PRESCALER_32:
        {
          regval |= HRTIM_CMNCR_CKPSC_d32;
          break;
        }

      case HRTIM_PRESCALER_64:
        {
          regval |= HRTIM_CMNCR_CKPSC_d64;
          break;
        }

      case HRTIM_PRESCALER_128:
        {
          regval |= HRTIM_CMNCR_CKPSC_d128;
          break;
        }

      default:
        {
          tmrerr("ERROR: invalid prescaler value %d for timer %d\n", timer,
                   pre);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Write prescaler configuration */

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_tim_clocks_config
 *
 * Description:
 *   Configure HRTIM Timers Clocks
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_clocks_config(struct stm32_hrtim_s *priv)
{
  int ret = OK;

  /* Configure Master Timer clock */

#ifdef CONFIG_STM32_HRTIM_MASTER
  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_MASTER,
                               HRTIM_MASTER_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure Timer A clock */

#ifdef CONFIG_STM32_HRTIM_TIMA
  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_TIMA, HRTIM_TIMA_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure Timer B clock */

#ifdef CONFIG_STM32_HRTIM_TIMB
  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_TIMB, HRTIM_TIMB_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure Timer C clock */

#ifdef CONFIG_STM32_HRTIM_TIMC
  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_TIMC, HRTIM_TIMC_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure Timer D clock */

#ifdef CONFIG_STM32_HRTIM_TIMD
  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_TIMD, HRTIM_TIMD_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure Timer E clock */

#ifdef CONFIG_STM32_HRTIM_TIME
  ret = hrtim_tim_clock_config(priv, HRTIM_TIMER_TIME, HRTIM_TIME_PRESCALER);
  if (ret < 0)
    {
      goto errout;
    }
#endif

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_gpios_config
 *
 * Description:
 *   Configure HRTIM GPIO
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_HRTIM_PWM) || defined(CONFIG_STM32_HRTIM_SYNC)
static int hrtim_gpios_config(struct stm32_hrtim_s *priv)
{
#ifdef CONFIG_STM32_HRTIM_EVENTS
  struct stm32_hrtim_eev_s *eev = priv->eev;
#endif
#ifdef CONFIG_STM32_HRTIM_FAULTS
  struct stm32_hrtim_faults_s *flt = priv->flt;
#endif

  /* Configure Timer A Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMA_PWM_CH1
  stm32_configgpio(GPIO_HRTIM1_CHA1);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMA_PWM_CH2
  stm32_configgpio(GPIO_HRTIM1_CHA2);
#endif

  /* Configure Timer B Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMB_PWM_CH1
  stm32_configgpio(GPIO_HRTIM1_CHB1);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB_PWM_CH2
  stm32_configgpio(GPIO_HRTIM1_CHB2);
#endif

  /* Configure Timer C Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMC_PWM_CH1
  stm32_configgpio(GPIO_HRTIM1_CHC1);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC_PWM_CH2
  stm32_configgpio(GPIO_HRTIM1_CHC2);
#endif

  /* Configure Timer D Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMD_PWM_CH1
  stm32_configgpio(GPIO_HRTIM1_CHD1);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD_PWM_CH2
  stm32_configgpio(GPIO_HRTIM1_CHD2);
#endif

  /* Configure Timer E Outputs */

#ifdef CONFIG_STM32_HRTIM_TIME_PWM_CH1
  stm32_configgpio(GPIO_HRTIM1_CHE1);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME_PWM_CH2
  stm32_configgpio(GPIO_HRTIM1_CHE2);
#endif

  /* Configure SCOUT */

#ifdef CONFIG_STM32_HRTIM_SCOUT
  stm32_configgpio(GPIO_HRTIM1_SCOUT);
#endif

  /* Configure SCIN */

#ifdef CONFIG_STM32_HRTIM_SCIN
  stm32_configgpio(GPIO_HRTIM1_SCIN);
#endif

  /* Configure Faults Inputs */

#ifdef CONFIG_STM32_HRTIM_FAULT1
  if (flt->flt1.src == HRTIM_FAULT_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_FLT1);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT2
  if (flt->flt2.src == HRTIM_FAULT_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_FLT2);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT3
  if (flt->flt3.src == HRTIM_FAULT_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_FLT3);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT4
  if (flt->flt4.src == HRTIM_FAULT_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_FLT4);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT5
  if (flt->flt5.src == HRTIM_FAULT_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_FLT5);
    }
#endif

  /* Configure External Events Inputs */

#ifdef CONFIG_STM32_HRTIM_EEV1
  if (eev->eev1.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV1);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV2
  if (eev->eev2.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV2);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV3
  if (eev->eev3.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV3);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV4
  if (eev->eev4.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV4);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV5
  if (eev->eev5.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV5);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV6
  if (eev->eev6.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV6);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV7
  if (eev->eev7.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV7);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV8
  if (eev->eev8.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV8);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV9
  if (eev->eev9.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV9);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_EEV10
  if (eev->eev10.src == HRTIM_EEV_SRC_PIN)
    {
      stm32_configgpio(GPIO_HRTIM1_EEV10);
    }
#endif

  return OK;
}
#endif

#if defined(CONFIG_STM32_HRTIM_CAPTURE)

/****************************************************************************
 * Name: hrtim_tim_capture_cfg
 *
 * Description:
 *  Configure HRTIM Captures
 *
 * Input Parameters:
 *   priv    - A reference to the HRTIM block
 *   timer   - HRTIM Timer index
 *   capture - capture triggers configuration
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_capture_cfg(struct stm32_hrtim_s *priv,
                                 uint8_t timer, uint8_t index,
                                 uint32_t capture)
{
  int ret = OK;
  uint32_t offset = 0;

  /* Sanity checking */

  if (timer == HRTIM_TIMER_MASTER || timer == HRTIM_TIMER_COMMON)
    {
      ret = -EINVAL;
      goto errout;
    }

  switch (index)
    {
      case HRTIM_CAPTURE1:
        {
          offset = STM32_HRTIM_TIM_CPT1CR_OFFSET;
          break;
        }

      case HRTIM_CAPTURE2:
        {
          offset = STM32_HRTIM_TIM_CPT2CR_OFFSET;
          break;
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

  hrtim_tim_putreg(priv, timer, offset, capture);

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_capture_config
 *
 * Description:
 *  Configure HRTIM Captures
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_capture_config(struct stm32_hrtim_s *priv)
{
  struct stm32_hrtim_slave_priv_s *slave;

#ifdef CONFIG_STM32_HRTIM_TIMA_CAP
  slave = (struct stm32_hrtim_slave_priv_s *)priv->tima->priv;
  hrtim_tim_capture_cfg(priv, HRTIM_TIMER_TIMA, HRTIM_CAPTURE1,
                        slave->cap.cap1);
  hrtim_tim_capture_cfg(priv, HRTIM_TIMER_TIMA, HRTIM_CAPTURE2,
                        slave->cap.cap2);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB_CAP
  slave = (struct stm32_hrtim_slave_priv_s *)priv->timb->priv;
  hrtim_tim_capture_cfg(priv, HRTIM_TIMER_TIMB, HRTIM_CAPTURE1,
                        slave->cap.cap1);
  hrtim_tim_capture_cfg(priv, HRTIM_TIMER_TIMB, HRTIM_CAPTURE2,
                        slave->cap.cap2);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC_CAP
  slave = (struct stm32_hrtim_slave_priv_s *)priv->timc->priv;
  hrtim_tim_capture_cfg(priv, HRTIM_TIMER_TIMC, HRTIM_CAPTURE1,
                        slave->cap.cap1);
  hrtim_tim_capture_cfg(priv, HRTIM_TIMER_TIMC, HRTIM_CAPTURE2,
                        slave->cap.cap2);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD_CAP
  slave = (struct stm32_hrtim_slave_priv_s *)priv->timd->priv;
  hrtim_tim_capture_cfg(priv, HRTIM_TIMER_TIMD, HRTIM_CAPTURE1,
                        slave->cap.cap1);
  hrtim_tim_capture_cfg(priv, HRTIM_TIMER_TIMD, HRTIM_CAPTURE2,
                        slave->cap.cap2);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME_CAP
  slave = (struct stm32_hrtim_slave_priv_s *)priv->time->priv;
  hrtim_tim_capture_cfg(priv, HRTIM_TIMER_TIME, HRTIM_CAPTURE1,
                        slave->cap.cap1);
  hrtim_tim_capture_cfg(priv, HRTIM_TIMER_TIME, HRTIM_CAPTURE2,
                        slave->cap.cap2);
#endif

  return OK;
}

/****************************************************************************
 * Name: hrtim_capture_get
 *
 * Description:
 *  Get HRTIM Timer Capture register
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   timer  - HRTIM Timer index
 *   index  - Capture register index
 *
 * Returned Value:
 *   Timer Capture value on success, 0 on failure
 *
 ****************************************************************************/

static uint16_t hrtim_capture_get(struct hrtim_dev_s *dev, uint8_t timer,
                                  uint8_t index)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  uint32_t regval = 0;
  uint32_t offset = 0;

  switch (index)
    {
      case HRTIM_CAPTURE1:
        {
          offset = STM32_HRTIM_TIM_CPT1R_OFFSET;
          break;
        }

      case HRTIM_CAPTURE2:
        {
          offset = STM32_HRTIM_TIM_CPT2R_OFFSET;
          break;
        }

      default:
        {
          regval = 0;
          goto errout;
        }
    }

  regval = (uint16_t)hrtim_tim_getreg(priv, timer, offset);

errout:
  return regval;
}

/****************************************************************************
 * Name: hrtim_soft_capture
 *
 * Description:
 *   HRTIM Timer software capture tirgger.
 *
 * Input Parameters:
 *   dev    - HRTIM device structure
 *   timer  - HRTIM Timer indexes
 *   index  - HRTIM capture index
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_soft_capture(struct hrtim_dev_s *dev, uint8_t timer,
                             uint8_t index)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  uint32_t offset = 0;

  switch (index)
    {
      case HRTIM_CAPTURE1:
        {
          offset = STM32_HRTIM_TIM_CPT1CR_OFFSET;
          break;
        }

      case HRTIM_CAPTURE2:
        {
          offset = STM32_HRTIM_TIM_CPT2CR_OFFSET;
          break;
        }

      default:
        {
          goto errout;
        }
    }

  /* Modify register */

  hrtim_tim_modifyreg(priv, timer, offset, 0, HRTIM_TIMCPT12CR_SWCPT);

errout:
  return OK;
}

#endif

/****************************************************************************
 * Name: hrtim_synch_config
 *
 * Description:
 *   Configure HRTIM Synchronization Input/Output
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_HRTIM_SYNC)
static int hrtim_synch_config(struct stm32_hrtim_s *priv)
{
#warning "hrtim_synch_config: missing logic"
  return OK;
}
#endif

/****************************************************************************
 * Name: hrtim_tim_outputs_config
 *
 * Description:
 *   Configure HRTIM Slave Timer Outputs (CH1 and CH2)
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_HRTIM_PWM)
static int hrtim_tim_outputs_config(struct stm32_hrtim_s *priv,
                                    uint8_t timer)
{
  struct stm32_hrtim_slave_priv_s *slave;
  uint32_t regval = 0;
  int ret = OK;

  /* Get Slave Timer data structure */

  slave = hrtim_slave_get(priv, timer);
  if (slave == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Configure CH1 SET events */

  regval = slave->pwm.ch1.set;
  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_SET1R_OFFSET, regval);

  /* Configure CH1 RESET events */

  regval = slave->pwm.ch1.rst;
  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_RST1R_OFFSET, regval);

  /* Configure CH2 SET events */

  regval = slave->pwm.ch2.set;
  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_SET2R_OFFSET, regval);

  /* Configure CH2 RESET events */

  regval = slave->pwm.ch2.rst;
  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_RST2R_OFFSET, regval);

  /* Now we configure OUT register */

  regval = 0;

#ifdef CONFIG_STM32_HRTIM_BURST
  /* Configure IDLE state for output 1 */

  if (slave->pwm.burst.ch1_en)
    {
      /* Set IDLE mode */

      regval |= HRTIM_TIMOUT_IDLEM1;

      /* Set Idle state */

      regval |= ((slave->pwm.burst.ch1_state & HRTIM_IDLE_ACTIVE) ?
                 HRTIM_TIMOUT_IDLES1 : 0);
    }

  /* Configure IDLE state for output 2 */

  if (slave->pwm.burst.ch2_en)
    {
      /* Set IDLE mode */

      regval |= HRTIM_TIMOUT_IDLEM1;

      /* Set Idle state */

      regval |= ((slave->pwm.burst.ch2_state & HRTIM_IDLE_ACTIVE) ?
                 HRTIM_TIMOUT_IDLES1 : 0);
    }
#endif

#ifdef CONFIG_STM32_HRTIM_DEADTIME
  if (slave->pwm.dt.en == 1)
    {
      /* Set deadtime enable */

      regval |= HRTIM_TIMOUT_DTEN;

      /* TODO: deadtime upon burst mode Idle entry */
    }
#endif

  /* Configure Output 1 polarisation */

  regval |= ((slave->pwm.ch1.pol & HRTIM_OUT_POL_NEG) ?
             HRTIM_TIMOUT_POL1 : 0);

  /* Configure Output 2 polarisation */

  regval |= ((slave->pwm.ch2.pol & HRTIM_OUT_POL_NEG) ?
             HRTIM_TIMOUT_POL2 : 0);

  /* Write HRTIM Slave Timer Output register  */

  hrtim_tim_modifyreg(priv, timer, STM32_HRTIM_TIM_OUTR_OFFSET, 0, regval);

#ifdef CONFIG_STM32_HRTIM_PUSHPULL
  if (slave->pwm.pushpull == 1)
    {
      /* Enable push-pull mode */

      hrtim_tim_modifyreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET, 0,
                          HRTIM_TIMCR_PSHPLL);
    }
#endif

errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: hrtim_outputs_config
 *
 * Description:
 *   Configure HRTIM Outputs
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_HRTIM_PWM)
static int hrtim_outputs_config(struct stm32_hrtim_s *priv)
{
  int ret = OK;

  /* Configure HRTIM TIMER A Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMA_PWM
  ret = hrtim_tim_outputs_config(priv, HRTIM_TIMER_TIMA);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure HRTIM TIMER B Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMB_PWM
  ret = hrtim_tim_outputs_config(priv, HRTIM_TIMER_TIMB);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure HRTIM TIMER C Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMC_PWM
  ret = hrtim_tim_outputs_config(priv, HRTIM_TIMER_TIMC);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure HRTIM TIMER D Outputs */

#ifdef CONFIG_STM32_HRTIM_TIMD_PWM
  ret = hrtim_tim_outputs_config(priv, HRTIM_TIMER_TIMD);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure HRTIM TIMER E Outputs */

#ifdef CONFIG_STM32_HRTIM_TIME_PWM
  ret = hrtim_tim_outputs_config(priv, HRTIM_TIMER_TIME);
  if (ret < 0)
    {
      goto errout;
    }
#endif

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_outputs_enable
 *
 * Description:
 *   Enable/disable HRTIM outputs (bulk operation)
 *
 * Input Parameters:
 *   dev     - HRTIM device structure
 *   outputs - outputs to set
 *   state   - Enable/disable operation
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_outputs_enable(struct hrtim_dev_s *dev,
                                uint16_t outputs, bool state)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  uint32_t offset = 0;

  /* Get register offset */

  if (state == true)
    {
      offset = STM32_HRTIM_CMN_OENR_OFFSET;
    }
  else
    {
      offset = STM32_HRTIM_CMN_ODISR_OFFSET;
    }

  /* Write register */

  hrtim_cmn_putreg(priv, offset, outputs);

  return OK;
}

/****************************************************************************
 * Name: output_tim_index_get
 ****************************************************************************/

static uint8_t output_tim_index_get(uint16_t output)
{
  uint8_t timer = 0;

  switch (output)
    {
#ifdef CONFIG_STM32_HRTIM_TIMA
      case HRTIM_OUT_TIMA_CH1:
      case HRTIM_OUT_TIMA_CH2:
        {
          timer = HRTIM_TIMER_TIMA;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
      case HRTIM_OUT_TIMB_CH1:
      case HRTIM_OUT_TIMB_CH2:
        {
          timer = HRTIM_TIMER_TIMB;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
      case HRTIM_OUT_TIMC_CH1:
      case HRTIM_OUT_TIMC_CH2:
        {
          timer = HRTIM_TIMER_TIMC;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
      case HRTIM_OUT_TIMD_CH1:
      case HRTIM_OUT_TIMD_CH2:
        {
          timer = HRTIM_TIMER_TIMD;
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
      case HRTIM_OUT_TIME_CH1:
      case HRTIM_OUT_TIME_CH2:
        {
          timer = HRTIM_TIMER_TIME;
          break;
        }
#endif

      default:
        {
          timer = 0;
          break;
        }
    }

  return timer;
}

/****************************************************************************
 * Name: output_tim_ch_get
 ****************************************************************************/

static uint8_t output_tim_ch_get(uint16_t output)
{
  uint8_t ch = 0;

  switch (output)
    {
#ifdef CONFIG_STM32_HRTIM_TIMA
      case HRTIM_OUT_TIMA_CH1:
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB
      case HRTIM_OUT_TIMB_CH1:
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC
      case HRTIM_OUT_TIMC_CH1:
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD
      case HRTIM_OUT_TIMD_CH1:
#endif
#ifdef CONFIG_STM32_HRTIM_TIME
      case HRTIM_OUT_TIME_CH1:
#endif
        {
          ch = HRTIM_OUT_CH1;
          break;
        }

#ifdef CONFIG_STM32_HRTIM_TIMA
      case HRTIM_OUT_TIMA_CH2:
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB
      case HRTIM_OUT_TIMB_CH2:
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC
      case HRTIM_OUT_TIMC_CH2:
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD
      case HRTIM_OUT_TIMD_CH2:
#endif
#ifdef CONFIG_STM32_HRTIM_TIME
      case HRTIM_OUT_TIME_CH2:
#endif
        {
          ch = HRTIM_OUT_CH2;
          break;
        }

      default:
        {
          ch = 0;
          break;
        }
    }

  return ch;
}

/****************************************************************************
 * Name: hrtim_output_set_set
 ****************************************************************************/

static int hrtim_output_set_set(struct hrtim_dev_s *dev, uint16_t output,
                                uint32_t set)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  struct stm32_hrtim_slave_priv_s *slave;
  uint8_t timer = 0;
  int ret = OK;

  /* Get timer index from output */

  timer = output_tim_index_get(output);

  /* Get Slave Timer data structure */

  slave = hrtim_slave_get(priv, timer);
  if (slave == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Set new SET value */

  switch (output_tim_ch_get(output))
    {
      case HRTIM_OUT_CH1:
        {
          slave->pwm.ch1.set = set;
          hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_SET1R_OFFSET, set);
          break;
        }

      case HRTIM_OUT_CH2:
        {
          slave->pwm.ch2.set = set;
          hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_SET2R_OFFSET, set);
          break;
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_output_rst_set
 ****************************************************************************/

static int hrtim_output_rst_set(struct hrtim_dev_s *dev, uint16_t output,
                                uint32_t rst)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  struct stm32_hrtim_slave_priv_s *slave;
  uint8_t timer = 0;
  int ret = OK;

  /* Get timer index from output */

  timer = output_tim_index_get(output);

  /* Get Salve Timer data structure */

  slave = hrtim_slave_get(priv, timer);
  if (slave == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Set new RST value */

  switch (output_tim_ch_get(output))
    {
      case HRTIM_OUT_CH1:
        {
          slave->pwm.ch1.rst = rst;
          hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_RST1R_OFFSET, rst);
        }

      case HRTIM_OUT_CH2:
        {
          slave->pwm.ch2.rst = rst;
          hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_RST2R_OFFSET, rst);
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

#endif

/****************************************************************************
 * Name: hrtim_adc_config
 *
 * Description:
 *   Configure HRTIM ADC triggers
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

#ifdef HRTIM_HAVE_ADC
static int hrtim_adc_config(struct stm32_hrtim_s *priv)
{
  /* Configure ADC Trigger 1 */

#ifdef HRTIM_HAVE_ADC_TRG1
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_ADC1R_OFFSET, priv->adc->trg1);
#endif

  /* Configure ADC Trigger 2 */

#ifdef HRTIM_HAVE_ADC_TRG2
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_ADC2R_OFFSET, priv->adc->trg2);
#endif

  /* Configure ADC Trigger 3 */

#ifdef HRTIM_HAVE_ADC_TRG3
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_ADC3R_OFFSET, priv->adc->trg3);
#endif

  /* Configure ADC Trigger 4 */

#ifdef HRTIM_HAVE_ADC_TRG4
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_ADC4R_OFFSET, priv->adc->trg4);
#endif

  return OK;
}
#endif

#ifdef CONFIG_STM32_HRTIM_DAC
/****************************************************************************
 * Name: hrtim_tim_dac_cfg
 *
 * Description:
 *   Configure single HRTIM Timer DAC synchronization event
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *   timer  - Timer index
 *   dac    - DAC synchronisation event configuration
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_dac_cfg(struct stm32_hrtim_s *priv, uint8_t timer,
                             uint8_t dac)
{
  struct stm32_hrtim_tim_s *tim;
  uint32_t regval = 0;

  tim = hrtim_tim_get(priv, timer);

  regval = hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET);

  regval |= (dac << HRTIM_CMNCR_DACSYNC_SHIFT);

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Name: hrtim_dac_config
 *
 * Description:
 *   Configure HRTIM DAC synchronization
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_dac_config(struct stm32_hrtim_s *priv)
{
  struct stm32_hrtim_timcmn_s *tim;

  /* Configure DAC synchronization for Master Timer */

#ifdef CONFIG_STM32_HRTIM_MASTER_DAC
  tim = (struct stm32_hrtim_timcmn_s *)priv->master;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_MASTER, tim->dac);
#endif

  /* Configure DAC synchronization for Timer A */

#ifdef CONFIG_STM32_HRTIM_TIMA_DAC
  tim = (struct stm32_hrtim_timcmn_s *)priv->tima;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_TIMA, tim->dac);
#endif

  /* Configure DAC synchronization for Timer B */

#ifdef CONFIG_STM32_HRTIM_TIMB_DAC
  tim = (struct stm32_hrtim_timcmn_s *)priv->timb;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_TIMB, tim->dac);
#endif

  /* Configure DAC synchronization for Timer C */

#ifdef CONFIG_STM32_HRTIM_TIMC_DAC
  tim = (struct stm32_hrtim_timcmn_s *)priv->timc;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_TIMC, tim->dac);
#endif

  /* Configure DAC synchronization for Timer D */

#ifdef CONFIG_STM32_HRTIM_TIMD_DAC
  tim = (struct stm32_hrtim_timcmn_s *)priv->timd;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_TIMD, tim->dac);
#endif

  /* Configure DAC synchronization for Timer E */

#ifdef CONFIG_STM32_HRTIM_TIME_DAC
  tim = (struct stm32_hrtim_timcmn_s *)priv->time;
  hrtim_tim_dac_cfg(priv, HRTIM_TIMER_TIME, tim->dac);
#endif

  return OK;
}
#endif

#ifdef CONFIG_STM32_HRTIM_DMA
/****************************************************************************
 * Name: hrtim_dma_cfg
 ****************************************************************************/

static int hrtim_tim_dma_cfg(struct stm32_hrtim_s *priv, uint8_t timer,
                             uint16_t dma)
{
  int ret = OK;
  uint32_t regval = 0;

  /* Sanity checking */

  if (timer == HRTIM_TIMER_COMMON)
    {
      ret = -EINVAL;
      goto errout;
    }

  if (timer == HRTIM_TIMER_MASTER)
    {
      /* Master support first 7 DMA requests */

      if (dma > 0x7f)
        {
          tmrerr("ERROR: invalid DMA requests 0x%04X for timer %d\n", dma,
                 timer);
          ret = -EINVAL;
          goto errout;
        }
    }
  else
    {
      if (dma & HRTIM_DMA_SYNC)
        {
          tmrerr("ERROR: timer %d does not support 0x%04X DMA request\n",
                 timer, HRTIM_DMA_SYNC);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* DMA configuration occupies upper half of the DIER register */

  regval = dma << 16;

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_DIER_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_dma_cfg
 ****************************************************************************/

static int hrtim_dma_cfg(struct stm32_hrtim_s *priv)
{
#ifdef CONFIG_STM32_HRTIM_MASTER_DMA
  hrtim_tim_dma_cfg(priv, HRTIM_TIMER_MASTER, priv->master->tim.dma);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMA_DMA
  hrtim_tim_dma_cfg(priv, HRTIM_TIMER_TIMA, priv->tima->tim.dma);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB_DMA
  hrtim_tim_dma_cfg(priv, HRTIM_TIMER_TIMB, priv->timb->tim.dma);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC_DMA
  hrtim_tim_dma_cfg(priv, HRTIM_TIMER_TIMC, priv->timc->tim.dma);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD_DMA
  hrtim_tim_dma_cfg(priv, HRTIM_TIMER_TIMD, priv->timd->tim.dma);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME_DMA
  hrtim_tim_dma_cfg(priv, HRTIM_TIMER_TIME, priv->time->tim.dma);
#endif

  return OK;
}
#endif /* CONFIG_STM32_HRTIM_DAM */

#ifdef CONFIG_STM32_HRTIM_DEADTIME
/****************************************************************************
 * Name: hrtim_deadtime_update
 ****************************************************************************/

static int hrtim_deadtime_update(struct hrtim_dev_s *dev, uint8_t timer,
                                 uint8_t dt, uint16_t value)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  int      ret    = OK;
  uint32_t regval = 0;
  uint32_t shift  = 0;
  uint32_t mask   = 0;

  /* For safety reasons we saturate deadtime value if it exceeds
   * the acceptable range.
   */

  if (value > 0x1ff)
    {
      value = 0x1ff;
    }

  /* Get shift value */

  switch (dt)
    {
      case HRTIM_DT_EDGE_RISING:
        {
          shift = HRTIM_TIMDT_DTR_SHIFT;
          mask  = HRTIM_TIMDT_DTR_MASK;
          break;
        }

      case HRTIM_DT_EDGE_FALLING:
        {
          shift = HRTIM_TIMDT_DTF_SHIFT;
          mask  = HRTIM_TIMDT_DTF_MASK;
          break;
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

  regval = value << shift;

  /* Update register */

  hrtim_tim_modifyreg(priv, timer, STM32_HRTIM_TIM_DTR_OFFSET, mask, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_deadtime_get
 ****************************************************************************/

static uint16_t hrtim_deadtime_get(struct hrtim_dev_s *dev,
                                   uint8_t timer, uint8_t dt)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  uint16_t regval = 0;
  uint32_t shift  = 0;
  uint32_t mask   = 0;

  /* Get shift value */

  switch (dt)
    {
      case HRTIM_DT_EDGE_RISING:
        {
          shift = HRTIM_TIMDT_DTR_SHIFT;
          mask  = HRTIM_TIMDT_DTR_MASK;
          break;
        }

      case HRTIM_DT_EDGE_FALLING:
        {
          shift = HRTIM_TIMDT_DTF_SHIFT;
          mask  = HRTIM_TIMDT_DTF_MASK;
          break;
        }

      default:
        {
          regval = 0;
          goto errout;
        }
    }

  /* Get Deadtime Register */

  regval = hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_DTR_OFFSET);

  /* Get Deadtime value  */

  regval = (regval & mask) >> shift;

errout:
  return regval;
}

/****************************************************************************
 * Name: hrtim_tim_deadtime_cfg
 ****************************************************************************/

static int hrtim_tim_deadtime_cfg(struct stm32_hrtim_s *priv,
                                  uint8_t timer)
{
  struct stm32_hrtim_slave_priv_s *slave;
  uint32_t regval = 0;
  int ret = OK;

  /* Sanity checking */

  if (timer == HRTIM_TIMER_MASTER || timer == HRTIM_TIMER_COMMON)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Get Slave Timer data structure */

  slave = hrtim_slave_get(priv, timer);
  if (slave == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Configure deadtime prescaler  */

  regval |= slave->pwm.dt.prescaler  << HRTIM_TIMDT_DTPRSC_SHIFT;

  /* Configure rising deadtime  */

  regval |= slave->pwm.dt.rising  << HRTIM_TIMDT_DTR_SHIFT;

  /* Configure falling deadtime */

  regval |= slave->pwm.dt.falling << HRTIM_TIMDT_DTF_SHIFT;

  /* Configure falling deadtime sign */

  if (slave->pwm.dt.fsign == HRTIM_DT_SIGN_NEGATIVE)
    {
      regval |= HRTIM_TIMDT_SDTF;
    }

  /* Configure risign deadtime sign */

  if (slave->pwm.dt.rsign == HRTIM_DT_SIGN_NEGATIVE)
    {
      regval |= HRTIM_TIMDT_SDTR;
    }

  /* Configure falling sing lock */

  if (slave->pwm.dt.fsign_lock == HRTIM_DT_LOCK)
    {
      regval |= HRTIM_TIMDT_DTFSLK;
    }

  /* Configure rising sing lock */

  if (slave->pwm.dt.rsign_lock == HRTIM_DT_LOCK)
    {
      regval |= HRTIM_TIMDT_DTRSLK;
    }

  /* Configure rising value lock */

  if (slave->pwm.dt.rising_lock == HRTIM_DT_LOCK)
    {
      regval |= HRTIM_TIMDT_DTRLK;
    }

  /* Configure falling value lock */

  if (slave->pwm.dt.falling_lock == HRTIM_DT_LOCK)
    {
      regval |= HRTIM_TIMDT_DTFLK;
    }

  /* TODO: configure default deadtime values */

  /* Write register */

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_DTR_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_deadtime_config
 ****************************************************************************/

static int hrtim_deadtime_config(struct stm32_hrtim_s *priv)
{
  /* Configure Timer A deadtime */

#ifdef CONFIG_STM32_HRTIM_TIMA_DT
  hrtim_tim_deadtime_cfg(priv, HRTIM_TIMER_TIMA);
#endif

  /* Configure Timer B deadtime */

#ifdef CONFIG_STM32_HRTIM_TIMB_DT
  hrtim_tim_deadtime_cfg(priv, HRTIM_TIMER_TIMB);
#endif

  /* Configure Timer C deadtime */

#ifdef CONFIG_STM32_HRTIM_TIMC_DT
  hrtim_tim_deadtime_cfg(priv, HRTIM_TIMER_TIMC);
#endif

  /* Configure Timer D deadtime */

#ifdef CONFIG_STM32_HRTIM_TIMD_DT
  hrtim_tim_deadtime_cfg(priv, HRTIM_TIMER_TIMD);
#endif

  /* Configure Timer E deadtime */

#ifdef CONFIG_STM32_HRTIM_TIME_DT
  hrtim_tim_deadtime_cfg(priv, HRTIM_TIMER_TIME);
#endif

  return OK;
}
#endif /* CONFIG_STM32_HRTIM_DEADTIME */

#ifdef CONFIG_STM32_HRTIM_CHOPPER
/****************************************************************************
 * Name: hrtim_chopper_enable
 *
 * Description:
 *   Enable/disable HRTIM outputs (bulk operation)
 *
 * Input Parameters:
 *   dev     - HRTIM device structure
 *   timer   - An HRTIM Timer index
 *   chan    - Output channel
 *   state   - Enable/disable operation
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_chopper_enable(struct hrtim_dev_s *dev, uint8_t timer,
                                uint8_t chan, bool state)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  uint32_t val = 0;
  int ret = OK;

  /* Get bit to change */

  switch (chan)
    {
      case HRTIM_OUT_CH1:
        {
          val = HRTIM_TIMOUT_CHP1;
          break;
        }

      case HRTIM_OUT_CH2:
        {
          val = HRTIM_TIMOUT_CHP2;
          break;
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Update register  */

  if (state == true)
    {
      /* Set enable bit */

      hrtim_tim_modifyreg(priv, timer, STM32_HRTIM_TIM_OUTR_OFFSET, 0, val);
    }
  else
    {
      /* Clear enable bit */

      hrtim_tim_modifyreg(priv, timer, STM32_HRTIM_TIM_OUTR_OFFSET, val, 0);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_chopper_cfg
 ****************************************************************************/

static int hrtim_tim_chopper_cfg(struct stm32_hrtim_s *priv,
                                 uint8_t timer)
{
  struct stm32_hrtim_slave_priv_s *slave;

  int ret = OK;
  uint32_t regval = 0;

  /* Sanity checking */

  if (timer == HRTIM_TIMER_MASTER || timer == HRTIM_TIMER_COMMON)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Get Slave Timer data structure */

  slave = hrtim_slave_get(priv, timer);
  if (slave == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Configure start pulsewidth */

  regval |= slave->pwm.chp.start_pulse << HRTIM_TIMCHP_STRTPW_SHIFT;

  /* Configure chopper duty cycle */

  regval |= slave->pwm.chp.duty << HRTIM_TIMCHP_CARDTY_SHIFT;

  /* Configure carrier frequency */

  regval |= slave->pwm.chp.freq << HRTIM_TIMCHP_CARFRQ_SHIFT;

  /* Write register */

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_CHPR_OFFSET, regval);

errout:
  return OK;
}

/****************************************************************************
 * Name: hrtim_chopper_config
 ****************************************************************************/

static int hrtim_chopper_config(struct stm32_hrtim_s *priv)
{
  /* Configure chopper for Timer A */

#ifdef CONFIG_STM32_HRTIM_TIMA_CHOP
  hrtim_tim_chopper_cfg(priv, HRTIM_TIMER_TIMA);
#endif

  /* Configure chopper for Timer B */

#ifdef CONFIG_STM32_HRTIM_TIMB_CHOP
  hrtim_tim_chopper_cfg(priv, HRTIM_TIMER_TIMB);
#endif

  /* Configure chopper for Timer C */

#ifdef CONFIG_STM32_HRTIM_TIMC_CHOP
  hrtim_tim_chopper_cfg(priv, HRTIM_TIMER_TIMC);
#endif

  /* Configure chopper for Timer D */

#ifdef CONFIG_STM32_HRTIM_TIMD_CHOP
  hrtim_tim_chopper_cfg(priv, HRTIM_TIMER_TIMD);
#endif

  /* Configure chopper for Timer E */

#ifdef CONFIG_STM32_HRTIM_TIME_CHOP
  hrtim_tim_chopper_cfg(priv, HRTIM_TIMER_TIME);
#endif

  return OK;
}
#endif

#ifdef CONFIG_STM32_HRTIM_BURST
/****************************************************************************
 * Name: hrtim_burst_enable
 ****************************************************************************/

static int hrtim_burst_enable(struct hrtim_dev_s *dev, bool state)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;

  if (state)
    {
      /* Enable Burst mode */

      hrtim_cmn_modifyreg(priv, STM32_HRTIM_CMN_BMCR_OFFSET, 0,
                          HRTIM_BMCR_BME);

      /* Software start */

      hrtim_cmn_modifyreg(priv, STM32_HRTIM_CMN_BMTRGR_OFFSET, 0,
                          HRTIM_BMTRGR_SW);
    }
  else
    {
      /* Disable Burst mode */

      hrtim_cmn_modifyreg(priv, STM32_HRTIM_CMN_BMCR_OFFSET,
                          HRTIM_BMCR_BME, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: hrtim_burst_cmp_update
 ****************************************************************************/

static int hrtim_burst_cmp_update(struct hrtim_dev_s *dev, uint16_t cmp)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;

  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_BMCMPR_OFFSET, cmp);

  return OK;
}

/****************************************************************************
 * Name: hrtim_burst_per_update
 ****************************************************************************/

static int hrtim_burst_per_update(struct hrtim_dev_s *dev, uint16_t per)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;

  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_BMPER_OFFSET, per);

  return OK;
}

/****************************************************************************
 * Name: hrtim_burst_cmp_get
 ****************************************************************************/

static uint16_t hrtim_burst_cmp_get(struct hrtim_dev_s *dev)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;

  return (uint16_t)hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BMCMPR_OFFSET);
}

/****************************************************************************
 * Name: hrtim_burst_per_get
 ****************************************************************************/

static uint16_t hrtim_burst_per_get(struct hrtim_dev_s *dev)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;

  return (uint16_t)hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_BMPER_OFFSET);
}

/****************************************************************************
 * Name: hrtim_burst_pre_update
 ****************************************************************************/

static int hrtim_burst_pre_update(struct hrtim_dev_s *dev, uint8_t pre)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  int ret = OK;
  uint32_t regval = 0;

  /* Sanity checking */

  if (priv->burst->clk != HRTIM_BURST_CLOCK_HRTIM)
    {
      ret = -EPERM;
      goto errout;
    }

  if (pre > HRTIM_BURST_PRESCALER_32768)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Make sure that Burst mode is disabled */

  hrtim_burst_enable(dev, false);

  /* Change prescaler */

  priv->burst->presc = pre;
  regval = pre << HRTIM_BMCR_BMPRSC_SHIFT;

  hrtim_cmn_modifyreg(priv, STM32_HRTIM_CMN_BMCR_OFFSET,
                      HRTIM_BMCR_BMPRSC_MASK, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_burst_pre_get
 ****************************************************************************/

static int hrtim_burst_pre_get(struct hrtim_dev_s *dev)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  int ret = OK;

  if (priv->burst->clk != HRTIM_BURST_CLOCK_HRTIM)
    {
      ret = -EPERM;
      goto errout;
    }

  ret = priv->burst->presc;

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_burst_config
 ****************************************************************************/

static int hrtim_burst_config(struct stm32_hrtim_s *priv)
{
  struct stm32_hrtim_burst_s *burst = priv->burst;
  uint32_t regval = 0;

  /* Configure triggers */

  regval = burst->trg;

  /* Write triggers register */

  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_BMTRGR_OFFSET, regval);

  /* TODO: timers mode configuration */

  regval  = 0;

  /* Configure burst mode clock source */

  regval |= (burst->clk << HRTIM_BMCR_BMCLK_SHIFT);

  /* Configure burst mode prescaler if f_HRTIM clock */

  if (burst->clk == HRTIM_BURST_CLOCK_HRTIM)
    {
      regval |= (burst->presc << HRTIM_BMCR_BMPRSC_SHIFT);
    }

  /* Set continuous mode */

  regval |= HRTIM_BMCR_BMOM;

  /* Write Burst Mode CR */

  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_BMCR_OFFSET, regval);

  return OK;
}
#endif

#ifdef CONFIG_STM32_HRTIM_FAULTS
/****************************************************************************
 * Name: hrtim_tim_faults_cfg
 *
 * Description:
 *   Configure HRTIM Slave Timer faults sources.
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *   timer  - timer index
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_faults_cfg(struct stm32_hrtim_s *priv,
                                uint8_t timer)
{
  struct stm32_hrtim_slave_priv_s *slave;
  uint32_t regval = 0;
  int ret = OK;

  slave = hrtim_slave_get(priv, timer);
  if (slave == NULL)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Get lock configuration */

  regval = ((slave->flt & HRTIM_TIM_FAULT_LOCK) ? HRTIM_TIMFLT_FLTLCK : 0);

  /* Get sources configuration */

  regval |= slave->flt & 0x1f;

  /* Write register */

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_FLTR_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_faults_config
 *
 * Description:
 *   Configure single HRTIM Fault
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *   index  - Fault index
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_flt_cfg(struct stm32_hrtim_s *priv, uint8_t index)
{
  struct stm32_hrtim_fault_cfg_s *flt;
  int ret = OK;
  uint32_t regval = 0;

  /* Get fault configuration */

  switch (index)
    {
#ifdef CONFIG_STM32_HRTIM_FAULT1
      case 1:
        {
          flt = &priv->flt->flt1;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_FAULT2
      case 2:
        {
          flt = &priv->flt->flt2;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_FAULT3
      case 3:
        {
          flt = &priv->flt->flt3;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_FAULT4
      case 4:
        {
          flt = &priv->flt->flt4;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_FAULT5
      case 5:
        {
          flt = &priv->flt->flt5;
          break;
        }

#endif
      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Configure fault */

  switch (index)
    {
      /* Fault 1-4 Configuration is located in first common fault register */

      case 1:
      case 2:
      case 3:
      case 4:
        {
          regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_FLTINR1_OFFSET);

          /* Configure polarity */

          regval |= (((flt->pol & HRTIM_FAULT_POL_HIGH) ?
                    HRTIM_FLTINR1_FLT1P : 0) << (index - 1) * 8);

          /* Config source */

          regval |= (((flt->src & HRTIM_FAULT_SRC_PIN) ?
                    HRTIM_FLTINR1_FLT1SRC : 0) << (index - 1) * 8);

          /* Config filter */

          regval |= ((flt->filter << HRTIM_FLTINR1_FLT1F_SHIFT) <<
                     (index - 1) * 8);

          /* Fault enable */

          regval |= (HRTIM_FLTINR1_FLT1E << (index - 1) * 8);

          /* Write register */

          hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_FLTINR1_OFFSET, regval);

          break;
        }

        /* Fault 5 configuration is located in second common fault
         * register
         */

      case 5:
        {
          regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_FLTINR2_OFFSET);

          /* Configure polarity */

          regval |= ((flt->pol & HRTIM_FAULT_POL_HIGH) ?
                     HRTIM_FLTINR2_FLT5P : 0);

          /* Config source */

          regval |= ((flt->src & HRTIM_FAULT_SRC_PIN) ?
                     HRTIM_FLTINR2_FLT5SRC : 0);

          /* Config filter */

          regval |= ((flt->filter << HRTIM_FLTINR2_FLT5F_SHIFT));

          /* Fault enable */

          regval |= HRTIM_FLTINR2_FLT5E;

          /* Write register */

          hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_FLTINR2_OFFSET, regval);

          break;
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_faults_config
 *
 * Description:
 *  Configure HRTIM Faults
 *
 * Input Parameters:
 *  priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_faults_config(struct stm32_hrtim_s *priv)
{
  uint32_t regval = 0;

  /* Configure faults */

#ifdef CONFIG_STM32_HRTIM_FAULT1
  hrtim_flt_cfg(priv, 1);
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT2
  hrtim_flt_cfg(priv, 2);
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT3
  hrtim_flt_cfg(priv, 3);
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT4
  hrtim_flt_cfg(priv, 4);
#endif

#ifdef CONFIG_STM32_HRTIM_FAULT5
  hrtim_flt_cfg(priv, 5);
#endif

  /* Configure fault sources in Slave Timers */

#ifdef CONFIG_STM32_HRTIM_TIMA_FLT
  hrtim_tim_faults_cfg(priv, HRTIM_TIMER_TIMA);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB_FLT
  hrtim_tim_faults_cfg(priv, HRTIM_TIMER_TIMA);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC_FLT
  hrtim_tim_faults_cfg(priv, HRTIM_TIMER_TIMA);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD_FLT
  hrtim_tim_faults_cfg(priv, HRTIM_TIMER_TIMA);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME_FLT
  hrtim_tim_faults_cfg(priv, HRTIM_TIMER_TIMA);
#endif

  /* Configure fault sampling clock division */

  regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_FLTINR2_OFFSET);
  regval |= HRTIM_FAULT_SAMPLING << HRTIM_FLTINR1_FLT1F_SHIFT;
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_FLTINR2_OFFSET, regval);

  return OK;
}
#endif

#ifdef CONFIG_STM32_HRTIM_EVENTS
/****************************************************************************
 * Name: hrtim_eev_cfg
 *
 * Description:
 *   Configure single HRTIM External Event
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *   index  - External Event index
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_eev_cfg(struct stm32_hrtim_s *priv, uint8_t index)
{
  struct stm32_hrtim_eev_cfg_s *eev;
  int ret = OK;
  uint32_t regval = 0;

  /* Get External Event configuration */

  switch (index)
    {
#ifdef CONFIG_STM32_HRTIM_EEV1
      case 1:
        {
          eev = &priv->eev->eev1;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_EEV2
      case 2:
        {
          eev = &priv->eev->eev2;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_EEV3
      case 3:
        {
          eev = &priv->eev->eev3;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_EEV4
      case 4:
        {
          eev = &priv->eev->eev4;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_EEV5
      case 5:
        {
          eev = &priv->eev->eev5;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_EEV6
      case 6:
        {
          eev = &priv->eev->eev6;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_EEV7
      case 7:
        {
          eev = &priv->eev->eev7;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_EEV8
      case 8:
        {
          eev = &priv->eev->eev8;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_EEV8
      case 9:
        {
          eev = &priv->eev->eev9;
          break;
        }

#endif
#ifdef CONFIG_STM32_HRTIM_EEV10
      case 10:
        {
          eev = &priv->eev->eev10;
          break;
        }

#endif
      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

  switch (index)
    {
      case 1:
      case 2:
      case 3:
      case 4:
      case 5:
      case 6:
        {
          regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_EECR1_OFFSET);

          /* Configure source */

          regval |= ((eev->src << HRTIM_EECR1_EE1SRC_SHIFT) <<
                     (index - 1) * 6);

          /* Configure polarity */

          regval |= ((eev->pol & HRTIM_FAULT_POL_HIGH ?
                     HRTIM_EECR1_EE1POL : 0) << (index - 1) * 6);

          /* Configure sensitivity */

          regval |= (((eev->sen) << HRTIM_EECR1_EE1SNS_SHIFT) <<
                     (index - 1) * 6);

          /* Configure mode */

          regval |= (((eev->mode & HRTIM_EEV_MODE_FAST) ?
                     HRTIM_EECR1_EE1FAST : 0) << (index - 1) * 6);

          /* Write register */

          hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_EECR1_OFFSET, regval);

          break;
        }

      case 7:
      case 8:
      case 9:
      case 10:
        {
          regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_EECR2_OFFSET);

          /* Configure source */

          regval |= ((eev->src << HRTIM_EECR2_EE6SRC_SHIFT) <<
                     (index - 6) * 6);

          /* Configure polarity */

          regval |= ((eev->pol & HRTIM_FAULT_POL_HIGH ?
                     HRTIM_EECR2_EE6POL : 0) << (index - 6) * 6);

          /* Configure sensitivity */

          regval |= (((eev->sen) << HRTIM_EECR2_EE6SNS_SHIFT) <<
                     (index - 6) * 6);

          /* Configure External Event filter, only EEV6-10 */

          regval |= (((eev->filter) << HRTIM_EECR2_EE6SNS_SHIFT) <<
                     (index - 6) * 6);

          /* Write register */

          hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_EECR2_OFFSET, regval);

          break;
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_events_config
 *
 * Description:
 *   Configure HRTIM External Events
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_events_config(struct stm32_hrtim_s *priv)
{
  uint32_t regval = 0;

  /* Configure Events sources */

#ifdef CONFIG_STM32_HRTIM_EEV1
  hrtim_eev_cfg(priv, 1);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV2
  hrtim_eev_cfg(priv, 2);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV3
  hrtim_eev_cfg(priv, 3);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV4
  hrtim_eev_cfg(priv, 4);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV5
  hrtim_eev_cfg(priv, 5);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV6
  hrtim_eev_cfg(priv, 6);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV7
  hrtim_eev_cfg(priv, 7);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV8
  hrtim_eev_cfg(priv, 8);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV9
  hrtim_eev_cfg(priv, 9);
#endif

#ifdef CONFIG_STM32_HRTIM_EEV10
  hrtim_eev_cfg(priv, 10);
#endif

  /* External Event Sampling clock */

  regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_EECR3_OFFSET);
  regval |= (HRTIM_EEV_SAMPLING << HRTIM_EECR3_EEVSD_SHIFT);
  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_EECR3_OFFSET, regval);

  return OK;
}
#endif /* CONFIG_STM32_HRTIM_FAULTS */

#ifdef CONFIG_STM32_HRTIM_INTERRUPTS

/****************************************************************************
 * Name: hrtim_irq_cfg
 ****************************************************************************/

static int hrtim_irq_cfg(struct stm32_hrtim_s *priv, uint8_t timer,
                         uint16_t irq)
{
  int ret = OK;

  if (timer == HRTIM_TIMER_COMMON)
    {
      hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_IER_OFFSET, irq);
    }
  else
    {
      hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_DIER_OFFSET, irq);
    }

  return ret;
}

/****************************************************************************
 * Name: hrtim_irq_config
 *
 * Description:
 *   Configure HRTIM interrupts
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_irq_config(struct stm32_hrtim_s *priv)
{
#ifdef CONFIG_STM32_HRTIM_MASTER_IRQ
  hrtim_irq_cfg(priv, HRTIM_TIMER_MASTER, priv->master->tim.irq);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMA_IRQ
  hrtim_irq_cfg(priv, HRTIM_TIMER_TIMA, priv->tima->tim.irq);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB_IRQ
  hrtim_irq_cfg(priv, HRTIM_TIMER_TIMB, priv->timb->tim.irq);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB_IRQ
  hrtim_irq_cfg(priv, HRTIM_TIMER_TIMB, priv->timc->tim.irq);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB_IRQ
  hrtim_irq_cfg(priv, HRTIM_TIMER_TIMB, priv->timd->tim.irq);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB_IRQ
  hrtim_irq_cfg(priv, HRTIM_TIMER_TIMB, priv->time->tim.irq);
#endif

#ifdef CONFIG_STM32_HRTIM_COMMON_IRQ
  hrtim_irq_cfg(priv, HRTIM_TIMER_COMMON, priv->irq);
#endif

  return OK;
}

/****************************************************************************
 * Name: hrtim_irq_ack
 ****************************************************************************/

static int hrtim_irq_ack(struct hrtim_dev_s *dev, uint8_t timer,
                         int source)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;

  if (timer == HRTIM_TIMER_COMMON)
    {
      /* Write to the HRTIM common clear interrupt register */

      hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_ICR_OFFSET, source);
    }
  else
    {
      /* Each timer has its own ICR register */

      hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_ICR_OFFSET, source);
    }

  return OK;
}

/****************************************************************************
 * Name: hrtim_irq_get
 ****************************************************************************/

static uint16_t hrtim_irq_get(struct hrtim_dev_s *dev, uint8_t timer)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  uint32_t regval = 0;

  if (timer == HRTIM_TIMER_COMMON)
    {
      /* Get HRTIM common status interrupt register */

      regval = hrtim_cmn_getreg(priv, STM32_HRTIM_CMN_ISR_OFFSET);
    }
  else
    {
      /* Each timer has its own ISR register */

      regval = hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_ISR_OFFSET);
    }

  return (uint16_t)regval;
}
#endif /* CONFIG_STM32_HRTIM_INTERRUPTS */

/****************************************************************************
 * Name: hrtim_tim_mode_set
 *
 * Description:
 *  Set HRTIM Timer mode
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   timer  - HRTIM Timer index
 *   mode   - Timer mode configuration
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hrtim_tim_mode_set(struct stm32_hrtim_s *priv, uint8_t timer,
                               uint8_t mode)
{
  uint32_t regval = 0;

  regval = hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET);

  /* Configure preload */

  if (mode & HRTIM_MODE_PRELOAD)
    {
      regval |= HRTIM_CMNCR_PREEN;
    }

  /* Configure half mode */

  if (mode & HRTIM_MODE_HALF)
    {
      regval |= HRTIM_CMNCR_HALF;
    }

  /* Configure re-triggerable mode */

  if (mode & HRTIM_MODE_RETRIG)
    {
      regval |= HRTIM_CMNCR_RETRIG;
    }

  /* Configure continuous mode */

  if (mode & HRTIM_MODE_CONT)
    {
      regval |= HRTIM_CMNCR_CONT;
    }

  /* Write register */

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET, regval);
}

/****************************************************************************
 * Name: hrtim_mode_config
 *
 * Description:
 *   Configure HRTIM Timers mode
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void hrtim_mode_config(struct stm32_hrtim_s *priv)
{
#ifdef CONFIG_STM32_HRTIM_MASTER
  hrtim_tim_mode_set(priv, HRTIM_TIMER_MASTER, priv->master->tim.mode);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMA
  hrtim_tim_mode_set(priv, HRTIM_TIMER_TIMA, priv->tima->tim.mode);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
  hrtim_tim_mode_set(priv, HRTIM_TIMER_TIMB, priv->timb->tim.mode);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
  hrtim_tim_mode_set(priv, HRTIM_TIMER_TIMC, priv->timc->tim.mode);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
  hrtim_tim_mode_set(priv, HRTIM_TIMER_TIMD, priv->timd->tim.mode);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
  hrtim_tim_mode_set(priv, HRTIM_TIMER_TIME, priv->time->tim.mode);
#endif
}

/****************************************************************************
 * Name: hrtim_cmpcap_mask_get
 *
 * Description:
 *   This function returns not significant bits in counter/capture
 *   registers for given HRTIM Timer index.
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *   timer  - HRTIM Timer index
 *
 * Returned Value:
 *   Not significant bits for counter/capture registers
 *
 ****************************************************************************/

static uint8_t hrtim_cmpcap_mask_get(struct stm32_hrtim_s *priv,
                                     uint8_t timer)
{
  struct stm32_hrtim_tim_s *tim;
  uint8_t mask = 0;

  /* Get Timer data structure */

  tim = hrtim_tim_get(priv, timer);
  if (tim == NULL)
    {
      mask = 0;
      goto errout;
    }

  /* Not significant bits depens on timer prescaler */

  switch (tim->tim.prescaler)
    {
      case HRTIM_PRESCALER_1:
        {
          mask = 0b11111;
          break;
        }

      case HRTIM_PRESCALER_2:
        {
          mask = 0b1111;
          break;
        }

      case HRTIM_PRESCALER_4:
        {
          mask = 0b111;
          break;
        }

      case HRTIM_PRESCALER_8:
        {
          mask = 0b11;
          break;
        }

      case HRTIM_PRESCALER_16:
        {
          mask = 0b1;
          break;
        }

      default:
        {
          mask = 0;
          break;
        }
    }

errout:
  return mask;
}

/****************************************************************************
 * Name: hrtim_cmp_update
 *
 * Description:
 *  Try update HRTIM Timer compare register.
 *
 * Input Parameters:
 *   dev    - HRTIM device structure
 *   timer  - HRTIM Timer index
 *   index  - Compare register timer
 *   cmp    - New compare register value
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_cmp_update(struct hrtim_dev_s *dev, uint8_t timer,
                            uint8_t index, uint16_t cmp)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  int ret = OK;
  uint32_t offset = 0;
  uint8_t mask = 0;

  switch (index)
    {
      case HRTIM_CMP1:
        {
          offset = STM32_HRTIM_TIM_CMP1R_OFFSET;
          break;
        }

      case HRTIM_CMP2:
        {
          offset = STM32_HRTIM_TIM_CMP2R_OFFSET;
          break;
        }

      case HRTIM_CMP3:
        {
          offset = STM32_HRTIM_TIM_CMP3R_OFFSET;
          break;
        }

      case HRTIM_CMP4:
        {
          offset = STM32_HRTIM_TIM_CMP4R_OFFSET;
          break;
        }

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

  /* REVISIT: what should we do if cmp value is not significant ?
   * At this moment we set compare register to the nearest significant value.
   */

  mask = hrtim_cmpcap_mask_get(priv, timer);
  if (cmp <= mask)
    {
      cmp = mask + 1;
    }

  hrtim_tim_putreg(priv, timer, offset, cmp);

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_per_update
 *
 * Description:
 *  Try update HRTIM Timer period register.
 *
 * Input Parameters:
 *   dev    - HRTIM device structure
 *   timer  - HRTIM Timer index
 *   per    - New period register value
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_per_update(struct hrtim_dev_s *dev, uint8_t timer,
                            uint16_t per)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_PER_OFFSET, per);

  return OK;
}

/****************************************************************************
 * Name: hrtim_per_get
 *
 * Description:
 *  Get HRTIM Timer period value
 *
 * Input Parameters:
 *   dev    - HRTIM device structure
 *   timer  - HRTIM Timer index
 *
 * Returned Value:
 *   Timer period value
 *
 ****************************************************************************/

static uint16_t hrtim_per_get(struct hrtim_dev_s *dev, uint8_t timer)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;

  return (uint16_t)hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_PER_OFFSET);
}

/****************************************************************************
 * Name: hrtim_rep_update
 *
 * Description:
 *  Try update HRTIM Timer repetition register.
 *
 * Input Parameters:
 *   dev    - HRTIM device structure
 *   timer  - HRTIM Timer index
 *   rep    - New repetition register value
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_rep_update(struct hrtim_dev_s *dev, uint8_t timer,
                            uint8_t rep)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_REPR_OFFSET, rep);

  return OK;
}

/****************************************************************************
 * Name: hrtim_cmp_update
 *
 * Description:
 *  Get HRTIM Timer compare register
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   timer  - HRTIM Timer index
 *   index  - Compare register timer
 *
 * Returned Value:
 *   Timer compare value
 *
 ****************************************************************************/

static uint16_t hrtim_cmp_get(struct hrtim_dev_s *dev, uint8_t timer,
                         uint8_t index)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  uint16_t cmpx = 0;
  uint32_t offset = 0;

  switch (index)
    {
      case HRTIM_CMP1:
        {
          offset = STM32_HRTIM_TIM_CMP1R_OFFSET;
          break;
        }

      case HRTIM_CMP2:
        {
          offset = STM32_HRTIM_TIM_CMP2R_OFFSET;
          break;
        }

      case HRTIM_CMP3:
        {
          offset = STM32_HRTIM_TIM_CMP3R_OFFSET;
          break;
        }

      case HRTIM_CMP4:
        {
          offset = STM32_HRTIM_TIM_CMP4R_OFFSET;
          break;
        }

      default:
        {
          cmpx = 0;
          goto errout;
        }
    }

  cmpx = (uint16_t)hrtim_tim_getreg(priv, timer, offset);

errout:
  return cmpx;
}

/****************************************************************************
 * Name: hrtim_fclk_get
 *
 * Description:
 *  Get HRTIM Timer clock value
 *
 * Input Parameters:
 *   dev    - HRTIM device structure
 *   timer  - HRTIM Timer index
 *
 * Returned Value:
 *   Timer clock value
 *
 ****************************************************************************/

static uint64_t hrtim_fclk_get(struct hrtim_dev_s *dev, uint8_t timer)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  struct stm32_hrtim_tim_s *tim;
  uint64_t fclk = 0;

  /* Get Timer data structure */

  tim = hrtim_tim_get(priv, timer);
  if (tim == NULL)
    {
      fclk = 0;
      goto errout;
    }

  fclk = tim->tim.fclk;

errout:
  return fclk;
}

/****************************************************************************
 * Name: hrtim_soft_update
 *
 * Description:
 *   HRTIM Timer software update.
 *   This is bulk operation, so we can update many registers at the same
 *   time.
 *
 * Input Parameters:
 *   dev    - HRTIM device structure
 *   timer  - HRTIM Timer indexes
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_soft_update(struct hrtim_dev_s *dev, uint8_t timer)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  uint32_t regval = 0;

  regval |= (timer & HRTIM_TIMER_MASTER ? HRTIM_CR2_MSWU : 0);
#ifdef CONFIG_STM32_HRTIM_TIMA
  regval |= (timer & HRTIM_TIMER_TIMA ? HRTIM_CR2_TASWU : 0);
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB
  regval |= (timer & HRTIM_TIMER_TIMB ? HRTIM_CR2_TBSWU : 0);
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC
  regval |= (timer & HRTIM_TIMER_TIMC ? HRTIM_CR2_TCSWU : 0);
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD
  regval |= (timer & HRTIM_TIMER_TIMD ? HRTIM_CR2_TDSWU : 0);
#endif
#ifdef CONFIG_STM32_HRTIM_TIME
  regval |= (timer & HRTIM_TIMER_TIME ? HRTIM_CR2_TESWU : 0);
#endif

  /* Bits in HRTIM CR2 common register are automatically reset,
   * so we can just write to it.
   */

  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_CR2_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Name: hrtim_soft_reset
 *
 * Description:
 *   HRTIM Timer software reset.
 *   This is bulk operation, so we can update many registers at the same
 *   time.
 *
 * Input Parameters:
 *   dev    - HRTIM device structure
 *   timer  - HRTIM Timer indexes
 *
 * Returned Value:
 *   0 on success; a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_soft_reset(struct hrtim_dev_s *dev, uint8_t timer)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  uint32_t regval = 0;

  regval |= (timer & HRTIM_TIMER_MASTER ? HRTIM_CR2_MRST : 0);
#ifdef CONFIG_STM32_HRTIM_TIMA
  regval |= (timer & HRTIM_TIMER_TIMA ? HRTIM_CR2_TARST : 0);
#endif
#ifdef CONFIG_STM32_HRTIM_TIMB
  regval |= (timer & HRTIM_TIMER_TIMB ? HRTIM_CR2_TBRST : 0);
#endif
#ifdef CONFIG_STM32_HRTIM_TIMC
  regval |= (timer & HRTIM_TIMER_TIMC ? HRTIM_CR2_TCRST : 0);
#endif
#ifdef CONFIG_STM32_HRTIM_TIMD
  regval |= (timer & HRTIM_TIMER_TIMD ? HRTIM_CR2_TDRST : 0);
#endif
#ifdef CONFIG_STM32_HRTIM_TIME
  regval |= (timer & HRTIM_TIMER_TIME ? HRTIM_CR2_TERST : 0);
#endif

  /* Bits in HRTIM CR2 common register are automatically reset,
   * so we can just write to it.
   */

  hrtim_cmn_putreg(priv, STM32_HRTIM_CMN_CR2_OFFSET, regval);

  return OK;
}

/****************************************************************************
 * Name: hrtim_tim_freq_set
 *
 * Description:
 *   Set HRTIM Timer frequency
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_freq_set(struct hrtim_dev_s *dev, uint8_t timer,
                              uint64_t freq)
{
  uint64_t per = 0;
  uint64_t fclk = 0;
  int ret = OK;

  /* Get Timer period value for given frequency */

  fclk = HRTIM_FCLK_GET(dev, timer);
  per = fclk / freq;
  if (per > HRTIM_PER_MAX)
    {
      tmrerr("ERROR: can not achieve timer pwm "
             "freq=%" PRIu64 " if fclk=%" PRIu64 "\n",
             freq, fclk);
      ret = -EINVAL;
      goto errout;
    }

  /* Set Timer period value */

  HRTIM_PER_SET(dev, timer, (uint16_t)per);

errout:
  return ret;
}

/****************************************************************************
 * Name: hrtim_tim_enable
 *
 * Description:
 *   Enable/disable HRTIM timer counter (bulk operation)
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_enable(struct hrtim_dev_s *dev, uint8_t timers,
                            bool state)
{
  struct stm32_hrtim_s *priv = (struct stm32_hrtim_s *)dev->hd_priv;
  uint32_t regval = 0;

  regval |= (timers & HRTIM_TIMERS_MASK) << HRTIM_MCR_TCEN_SHIFT;

  if (state == true)
    {
      /* Set bits */

      hrtim_tim_modifyreg(priv, HRTIM_TIMER_MASTER,
                          STM32_HRTIM_TIM_CR_OFFSET,
                          0, regval);
    }
  else
    {
      /* Clear bits */

      hrtim_tim_modifyreg(priv, HRTIM_TIMER_MASTER,
                          STM32_HRTIM_TIM_CR_OFFSET,
                          regval, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: hrtim_tim_reset_set
 *
 * Description:
 *  Set HRTIM Timer Reset events
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM block
 *   timer  - HRTIM Timer index
 *   reset  - Reset configuration
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int hrtim_tim_reset_set(struct stm32_hrtim_s *priv, uint8_t timer,
                               uint64_t reset)
{
  int ret = OK;
  uint32_t regval = 0;

  /* Sanity checking */

  if (timer == HRTIM_TIMER_MASTER || timer == HRTIM_TIMER_COMMON)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* First 18 bits can be written directly */

  regval |= (reset & 0x3ffff);

  /* TimerX reset events differ for individual timers */

  switch (timer)
    {
#ifdef CONFIG_STM32_HRTIM_TIMA
      case HRTIM_TIMER_TIMA:
        {
          regval |= ((reset & HRTIM_RST_TBCMP1) ?
                     HRTIM_TIMARST_TIMBCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TBCMP2) ?
                     HRTIM_TIMARST_TIMBCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TBCMP4) ?
                     HRTIM_TIMARST_TIMBCMP4 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP1) ?
                     HRTIM_TIMARST_TIMCCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP2) ?
                     HRTIM_TIMARST_TIMCCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP4) ?
                     HRTIM_TIMARST_TIMCCMP4 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP1) ?
                     HRTIM_TIMARST_TIMDCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP2) ?
                     HRTIM_TIMARST_TIMDCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP4) ?
                     HRTIM_TIMARST_TIMDCMP4 : 0);
          regval |= ((reset & HRTIM_RST_TECMP1) ?
                     HRTIM_TIMARST_TIMECMP1 : 0);
          regval |= ((reset & HRTIM_RST_TECMP2) ?
                     HRTIM_TIMARST_TIMECMP2 : 0);
          regval |= ((reset & HRTIM_RST_TECMP4) ?
                     HRTIM_TIMARST_TIMECMP4 : 0);
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
      case HRTIM_TIMER_TIMB:
        {
          regval |= ((reset & HRTIM_RST_TACMP1) ?
                     HRTIM_TIMBRST_TIMACMP1 : 0);
          regval |= ((reset & HRTIM_RST_TACMP2) ?
                     HRTIM_TIMBRST_TIMACMP2 : 0);
          regval |= ((reset & HRTIM_RST_TACMP4) ?
                     HRTIM_TIMBRST_TIMACMP4 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP1) ?
                     HRTIM_TIMBRST_TIMCCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP2) ?
                     HRTIM_TIMBRST_TIMCCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP4) ?
                     HRTIM_TIMBRST_TIMCCMP4 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP1) ?
                     HRTIM_TIMBRST_TIMDCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP2) ?
                     HRTIM_TIMBRST_TIMDCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP4) ?
                     HRTIM_TIMBRST_TIMDCMP4 : 0);
          regval |= ((reset & HRTIM_RST_TECMP1) ?
                     HRTIM_TIMBRST_TIMECMP1 : 0);
          regval |= ((reset & HRTIM_RST_TECMP2) ?
                     HRTIM_TIMBRST_TIMECMP2 : 0);
          regval |= ((reset & HRTIM_RST_TECMP4) ?
                     HRTIM_TIMBRST_TIMECMP4 : 0);
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
      case HRTIM_TIMER_TIMC:
        {
          regval |= ((reset & HRTIM_RST_TACMP1) ?
                     HRTIM_TIMCRST_TIMACMP1 : 0);
          regval |= ((reset & HRTIM_RST_TACMP2) ?
                     HRTIM_TIMCRST_TIMACMP2 : 0);
          regval |= ((reset & HRTIM_RST_TACMP4) ?
                     HRTIM_TIMCRST_TIMACMP4 : 0);
          regval |= ((reset & HRTIM_RST_TBCMP1) ?
                     HRTIM_TIMCRST_TIMBCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TBCMP2) ?
                     HRTIM_TIMCRST_TIMBCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TBCMP4) ?
                     HRTIM_TIMCRST_TIMBCMP4 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP1) ?
                     HRTIM_TIMCRST_TIMDCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP2) ?
                     HRTIM_TIMCRST_TIMDCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP4) ?
                     HRTIM_TIMCRST_TIMDCMP4 : 0);
          regval |= ((reset & HRTIM_RST_TECMP1) ?
                     HRTIM_TIMCRST_TIMECMP1 : 0);
          regval |= ((reset & HRTIM_RST_TECMP2) ?
                     HRTIM_TIMCRST_TIMECMP2 : 0);
          regval |= ((reset & HRTIM_RST_TECMP4) ?
                     HRTIM_TIMCRST_TIMECMP4 : 0);
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
      case HRTIM_TIMER_TIMD:
        {
          regval |= ((reset & HRTIM_RST_TACMP1) ?
                     HRTIM_TIMDRST_TIMACMP1 : 0);
          regval |= ((reset & HRTIM_RST_TACMP2) ?
                     HRTIM_TIMDRST_TIMACMP2 : 0);
          regval |= ((reset & HRTIM_RST_TACMP4) ?
                     HRTIM_TIMDRST_TIMACMP4 : 0);
          regval |= ((reset & HRTIM_RST_TBCMP1) ?
                     HRTIM_TIMDRST_TIMBCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TBCMP2) ?
                     HRTIM_TIMDRST_TIMBCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TBCMP4) ?
                     HRTIM_TIMDRST_TIMBCMP4 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP1) ?
                     HRTIM_TIMDRST_TIMCCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP2) ?
                     HRTIM_TIMDRST_TIMCCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP4) ?
                     HRTIM_TIMDRST_TIMCCMP4 : 0);
          regval |= ((reset & HRTIM_RST_TECMP1) ?
                     HRTIM_TIMDRST_TIMECMP1 : 0);
          regval |= ((reset & HRTIM_RST_TECMP2) ?
                     HRTIM_TIMDRST_TIMECMP2 : 0);
          regval |= ((reset & HRTIM_RST_TECMP4) ?
                     HRTIM_TIMDRST_TIMECMP4 : 0);
          break;
        }
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
      case HRTIM_TIMER_TIME:
        {
          regval |= ((reset & HRTIM_RST_TACMP1) ?
                     HRTIM_TIMERST_TIMACMP1 : 0);
          regval |= ((reset & HRTIM_RST_TACMP2) ?
                     HRTIM_TIMERST_TIMACMP2 : 0);
          regval |= ((reset & HRTIM_RST_TACMP4) ?
                     HRTIM_TIMERST_TIMACMP4 : 0);
          regval |= ((reset & HRTIM_RST_TBCMP1) ?
                     HRTIM_TIMERST_TIMBCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TBCMP2) ?
                     HRTIM_TIMERST_TIMBCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TBCMP4) ?
                     HRTIM_TIMERST_TIMBCMP4 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP1) ?
                     HRTIM_TIMERST_TIMCCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP2) ?
                     HRTIM_TIMERST_TIMCCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TCCMP4) ?
                     HRTIM_TIMERST_TIMCCMP4 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP1) ?
                     HRTIM_TIMERST_TIMDCMP1 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP2) ?
                     HRTIM_TIMERST_TIMDCMP2 : 0);
          regval |= ((reset & HRTIM_RST_TDCMP4) ?
                     HRTIM_TIMERST_TIMDCMP4 : 0);
          break;
        }
#endif

      default:
        {
          ret = -EINVAL;
          goto errout;
        }
    }

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_RSTR_OFFSET, regval);

errout:
  return ret;
}

static int hrtim_reset_config(struct stm32_hrtim_s *priv)
{
  struct stm32_hrtim_slave_priv_s *slave;

#ifdef CONFIG_STM32_HRTIM_TIMA
  slave = (struct stm32_hrtim_slave_priv_s *)priv->tima->priv;
  hrtim_tim_reset_set(priv, HRTIM_TIMER_TIMA, slave->reset);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
  slave = (struct stm32_hrtim_slave_priv_s *)priv->timb->priv;
  hrtim_tim_reset_set(priv, HRTIM_TIMER_TIMB, slave->reset);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
  slave = (struct stm32_hrtim_slave_priv_s *)priv->timc->priv;
  hrtim_tim_reset_set(priv, HRTIM_TIMER_TIMC, slave->reset);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
  slave = (struct stm32_hrtim_slave_priv_s *)priv->timd->priv;
  hrtim_tim_reset_set(priv, HRTIM_TIMER_TIMD, slave->reset);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
  slave = (struct stm32_hrtim_slave_priv_s *)priv->time->priv;
  hrtim_tim_reset_set(priv, HRTIM_TIMER_TIME, slave->reset);
#endif

  return OK;
}

static int hrtim_tim_update_set(struct stm32_hrtim_s *priv,
                                uint8_t timer,
                                uint16_t update)
{
  uint32_t regval = 0;

  regval = hrtim_tim_getreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET);

  /* Configure update events */

  regval |= (update & HRTIM_UPDATE_MSTU ? HRTIM_TIMCR_MSTU : 0);
  regval |= (update & HRTIM_UPDATE_RSTU ? HRTIM_TIMCR_RSTU : 0);
  regval |= (update & HRTIM_UPDATE_REPU ? HRTIM_TIMCR_REPU : 0);

#ifdef CONFIG_STM32_HRTIM_TIMA
  regval |= (update & HRTIM_UPDATE_TAU ? HRTIM_TIMCR_TAU : 0);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
  regval |= (update & HRTIM_UPDATE_TBU ? HRTIM_TIMCR_TBU : 0);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
  regval |= (update & HRTIM_UPDATE_TCU ? HRTIM_TIMCR_TCU : 0);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
  regval |= (update & HRTIM_UPDATE_TDU ? HRTIM_TIMCR_TDU : 0);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
  regval |= (update & HRTIM_UPDATE_TEU ? HRTIM_TIMCR_TEU : 0);
#endif

  /* TODO: Configure update gating */

  /* Write register */

  hrtim_tim_putreg(priv, timer, STM32_HRTIM_TIM_CR_OFFSET, regval);

  return OK;
}

static int hrtim_update_config(struct stm32_hrtim_s *priv)
{
  struct stm32_hrtim_slave_priv_s *slave;

#ifdef CONFIG_STM32_HRTIM_TIMA
  slave = (struct stm32_hrtim_slave_priv_s *)priv->tima->priv;
  hrtim_tim_update_set(priv, HRTIM_TIMER_TIMA, slave->update);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMB
  slave = (struct stm32_hrtim_slave_priv_s *)priv->timb->priv;
  hrtim_tim_update_set(priv, HRTIM_TIMER_TIMB, slave->update);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMC
  slave = (struct stm32_hrtim_slave_priv_s *)priv->timc->priv;
  hrtim_tim_update_set(priv, HRTIM_TIMER_TIMC, slave->update);
#endif

#ifdef CONFIG_STM32_HRTIM_TIMD
  slave = (struct stm32_hrtim_slave_priv_s *)priv->timd->priv;
  hrtim_tim_update_set(priv, HRTIM_TIMER_TIMD, slave->update);
#endif

#ifdef CONFIG_STM32_HRTIM_TIME
  slave = (struct stm32_hrtim_slave_priv_s *)priv->time->priv;
  hrtim_tim_update_set(priv, HRTIM_TIMER_TIME, slave->update);
#endif

  return OK;
}

/****************************************************************************
 * Name: stm32_hrtimconfig
 *
 * Description:
 *   Configure HRTIM
 *
 * Input Parameters:
 *   priv   - A reference to the HRTIM structure
 *
 * Returned Value:
 *   0 on success, a negated errno value on failure
 *
 ****************************************************************************/

static int stm32_hrtimconfig(struct stm32_hrtim_s *priv)
{
  int ret;
  uint32_t regval = 0;

  /* HRTIM DLL calibration */

  ret = hrtim_dll_cal(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM DLL calibration failed!\n");
      goto errout;
    }

  /* Configure Timers Clocks */

  ret = hrtim_tim_clocks_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM timers clock configuration failed!\n");
      goto errout;
    }

  /* Configure Timers reset events */

  hrtim_reset_config(priv);

  /* Configure Timers update events */

  hrtim_update_config(priv);

  /* Configure Timers mode */

  hrtim_mode_config(priv);

  /* Configure auto-delayed mode */

#ifdef CONFIG_STM32_HRTIM_AUTODELAYED
  hrtim_autodelayed_config(priv);
#endif

  /* Configure HRTIM GPIOs */

#if defined(CONFIG_STM32_HRTIM_PWM) || defined(CONFIG_STM32_HRTIM_SYNC)
  ret = hrtim_gpios_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM GPIOs configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure HRTIM capture */

#if defined(CONFIG_STM32_HRTIM_CAPTURE)
  ret = hrtim_capture_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM capture configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure Synchronisation IOs */

#if defined(CONFIG_STM32_HRTIM_SYNC)
  ret = hrtim_synch_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM synchronisation configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure HRTIM outputs deadtime */

#if defined(CONFIG_STM32_HRTIM_DEADTIME)
  ret = hrtim_deadtime_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM deadtime configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure HRTIM outputs GPIOs */

#if defined(CONFIG_STM32_HRTIM_PWM)
  ret = hrtim_outputs_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM outputs configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure ADC triggers */

#ifdef HRTIM_HAVE_ADC
  ret = hrtim_adc_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM ADC configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure DAC synchronization */

#ifdef CONFIG_STM32_HRTIM_DAC
  ret = hrtim_dac_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM ADC configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure Faults */

#ifdef CONFIG_STM32_HRTIM_FAULTS
  ret = hrtim_faults_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM faults configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure External Events */

#ifdef CONFIG_STM32_HRTIM_EVENTS
  ret = hrtim_events_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM EEV configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure interrupts */

#ifdef CONFIG_STM32_HRTIM_INTERRUPTS
  ret = hrtim_irq_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM IRQ configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure DMA */

#ifdef CONFIG_STM32_HRTIM_DMA
  ret = hrtim_dma_cfg(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM DMA configuration failed!\n");
      goto errout;
    }
#endif

  /* Configure burst mode */

#ifdef CONFIG_STM32_HRTIM_BURST
  ret = hrtim_burst_config(priv);
  if (ret != OK)
    {
      tmrerr("ERROR: HRTIM burst mode configuration failed!\n");
      goto errout;
    }
#endif

#ifndef CONFIG_STM32_HRTIM_NO_ENABLE_TIMERS
  /* Enable Master Timer */

#  ifdef CONFIG_STM32_HRTIM_MASTER
  regval |= HRTIM_MCR_MCEN;
#  endif

  /* Enable Slave Timers */

#  ifdef CONFIG_STM32_HRTIM_TIMA
  regval |= HRTIM_MCR_TACEN;
#  endif

#  ifdef CONFIG_STM32_HRTIM_TIMB
  regval |= HRTIM_MCR_TBCEN;
#  endif

#  ifdef CONFIG_STM32_HRTIM_TIMC
  regval |= HRTIM_MCR_TCCEN;
#  endif

#  ifdef CONFIG_STM32_HRTIM_TIMD
  regval |= HRTIM_MCR_TDCEN;
#  endif

#  ifdef CONFIG_STM32_HRTIM_TIME
  regval |= HRTIM_MCR_TECEN;
#  endif

#endif /* CONFIG_STM32_HRTIM_NO_ENABLE_TIMERS */

  /* Write enable bits at once */

  hrtim_tim_modifyreg(priv, HRTIM_TIMER_MASTER, STM32_HRTIM_TIM_CR_OFFSET,
                      0, regval);

  /* Dump registers for Master */

  hrtim_dumpregs(priv, HRTIM_TIMER_MASTER, "Master after configuration");

  /* Dump registers for Timer A */

#ifdef CONFIG_STM32_HRTIM_TIMA
  hrtim_dumpregs(priv, HRTIM_TIMER_TIMA, "Timer A after configuration");
#endif

  /* Dump registers for Timer B */

#ifdef CONFIG_STM32_HRTIM_TIMB
  hrtim_dumpregs(priv, HRTIM_TIMER_TIMB, "Timer B after configuration");
#endif

  /* Dump registers for Timer C */

#ifdef CONFIG_STM32_HRTIM_TIMC
  hrtim_dumpregs(priv, HRTIM_TIMER_TIMC, "Timer C after configuration");
#endif

  /* Dump registers for Timer D */

#ifdef CONFIG_STM32_HRTIM_TIMD
  hrtim_dumpregs(priv, HRTIM_TIMER_TIMD, "Timer D after configuration");
#endif

  /* Dump registers for Timer E */

#ifdef CONFIG_STM32_HRTIM_TIME
  hrtim_dumpregs(priv, HRTIM_TIMER_TIME, "Timer E after configuration");
#endif

  /* Dump common registers */

  hrtim_dumpregs(priv, HRTIM_TIMER_COMMON, "Common after configuration");

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hrtiminitialize
 *
 * Description:
 *   Initialize the HRTIM.
 *
 * Returned Value:
 *   Valid HRTIM device structure reference on success; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the HRTIM block has enabled,
 *   2. Board-specific logic has already configured
 *
 ****************************************************************************/

struct hrtim_dev_s *stm32_hrtiminitialize(void)
{
  struct hrtim_dev_s  *dev;
  struct stm32_hrtim_s *hrtim;
  int ret;

  dev = &g_hrtim1dev;

  hrtim = dev->hd_priv;

  /* configure HRTIM only once */

  if (!dev->initialized)
    {
      ret = stm32_hrtimconfig(hrtim);
      if (ret < 0)
        {
          tmrerr("ERROR: Failed to initialize HRTIM1: %d\n", ret);
          return NULL;
        }

      dev->initialized = true;
    }

  return dev;
}

/****************************************************************************
 * Name: hrtim_register
 ****************************************************************************/

#ifndef CONFIG_STM32_HRTIM_DISABLE_CHARDRV
int hrtim_register(const char *path, struct hrtim_dev_s *dev)
{
  int ret ;

  /* Initialize the HRTIM device structure */

  dev->hd_ocount = 0;

  /* Initialize semaphores */

  nxsem_init(&dev->hd_closesem, 0, 1);

  /* Register the HRTIM character driver */

  ret =  register_driver(path, &hrtim_fops, 0444, dev);
  if (ret < 0)
    {
      nxsem_destroy(&dev->hd_closesem);
    }

  return ret;
}
#endif /* CONFIG_STM32_HRTIM_DISABLE_CHARDRV */

#endif /* CONFIG_STM32_STM32F33XX */
#endif /* CONFIG_STM32_HRTIM1 */
