/****************************************************************************
 * arch/arm/src/stm32/stm32_pwm.c
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
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_pwm.h"
#include "stm32.h"

/* This module then only compiles if there is at least one enabled timer
 * intended for use with the PWM upper half driver.
 *
 * It implements support for both:
 *   1. STM32 TIMER IP version 1 - F0, F1, F2, F37x, F4, L0, L1
 *   2. STM32 TIMER IP version 2 - F3 (no F37x), F7, H7, L4, L4+
 */

#ifdef CONFIG_STM32_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM/Timer Definitions ****************************************************/

/* The following definitions are used to identify the various time types.
 * There are some differences in timer types across STM32 families:
 *   - TIM2 is 16-bit timer for F1, L1 and L0
 *   - TIM5 is 16-bit timer for F1
 */

#define TIMTYPE_BASIC        0  /* Basic timers (no outputs) */
#define TIMTYPE_GENERAL16    1  /* General 16-bit timers (up, down, up/down)*/
#define TIMTYPE_COUNTUP16    2  /* General 16-bit count-up timers */
#define TIMTYPE_COUNTUP16_N  3  /* General 16-bit count-up timers with
                                 * complementary outptus
                                 */
#define TIMTYPE_GENERAL32    4  /* General 32-bit timers (up, down, up/down)*/
#define TIMTYPE_ADVANCED     5  /* Advanced timers */

#define TIMTYPE_TIM1       TIMTYPE_ADVANCED
#if defined(CONFIG_STM32_STM32L15XX) || defined(CONFIG_STM32_STM32F10XX)
#  define TIMTYPE_TIM2     TIMTYPE_GENERAL16
#else
#  define TIMTYPE_TIM2     TIMTYPE_GENERAL32
#endif
#define TIMTYPE_TIM3       TIMTYPE_GENERAL16
#define TIMTYPE_TIM4       TIMTYPE_GENERAL16
#if defined(CONFIG_STM32_STM32F10XX)
#  define TIMTYPE_TIM5     TIMTYPE_GENERAL16
#else
#  define TIMTYPE_TIM5     TIMTYPE_GENERAL32
#endif
#define TIMTYPE_TIM6       TIMTYPE_BASIC
#define TIMTYPE_TIM7       TIMTYPE_BASIC
#define TIMTYPE_TIM8       TIMTYPE_ADVANCED
#define TIMTYPE_TIM9       TIMTYPE_COUNTUP16
#define TIMTYPE_TIM10      TIMTYPE_COUNTUP16
#define TIMTYPE_TIM11      TIMTYPE_COUNTUP16
#define TIMTYPE_TIM12      TIMTYPE_COUNTUP16
#define TIMTYPE_TIM13      TIMTYPE_COUNTUP16
#define TIMTYPE_TIM14      TIMTYPE_COUNTUP16
#define TIMTYPE_TIM15      TIMTYPE_COUNTUP16_N /* Treated as ADVTIM */
#define TIMTYPE_TIM16      TIMTYPE_COUNTUP16_N /* Treated as ADVTIM */
#define TIMTYPE_TIM17      TIMTYPE_COUNTUP16_N /* Treated as ADVTIM */

/* Timer clock source, RCC EN offset, enable bit,
 * RCC RST offset, reset bit to use
 *
 * TODO: simplify this and move somewhere else.
 */

#if defined(CONFIG_STM32_STM32F37XX)
#  define TIMCLK_TIM2      STM32_APB1_TIM2_CLKIN
#  define TIMRCCEN_TIM2    STM32_RCC_APB1ENR
#  define TIMEN_TIM2       RCC_APB1ENR_TIM2EN
#  define TIMRCCRST_TIM2   STM32_RCC_APB1RSTR
#  define TIMRST_TIM2      RCC_APB1RSTR_TIM2RST
#  define TIMCLK_TIM3      STM32_APB1_TIM3_CLKIN
#  define TIMRCCEN_TIM3    STM32_RCC_APB1ENR
#  define TIMEN_TIM3       RCC_APB1ENR_TIM3EN
#  define TIMRCCRST_TIM3   STM32_RCC_APB1RSTR
#  define TIMRST_TIM3      RCC_APB1RSTR_TIM3RST
#  define TIMCLK_TIM4      STM32_APB1_TIM4_CLKIN
#  define TIMRCCEN_TIM4    STM32_RCC_APB1ENR
#  define TIMEN_TIM4       RCC_APB1ENR_TIM4EN
#  define TIMRCCRST_TIM4   STM32_RCC_APB1RSTR
#  define TIMRST_TIM4      RCC_APB1RSTR_TIM4RST
#  define TIMCLK_TIM5      STM32_APB1_TIM5_CLKIN
#  define TIMRCCEN_TIM5    STM32_RCC_APB1ENR
#  define TIMEN_TIM5       RCC_APB1ENR_TIM5EN
#  define TIMRCCRST_TIM5   STM32_RCC_APB1RSTR
#  define TIMRST_TIM5      RCC_APB1RSTR_TIM5RST
#  define TIMCLK_TIM6      STM32_APB1_TIM6_CLKIN
#  define TIMRCCEN_TIM6    STM32_RCC_APB1ENR
#  define TIMEN_TIM6       RCC_APB1ENR_TIM6EN
#  define TIMRCCRST_TIM6   STM32_RCC_APB1RSTR
#  define TIMRST_TIM6      RCC_APB1RSTR_TIM6RST
#  define TIMCLK_TIM7      STM32_APB1_TIM7_CLKIN
#  define TIMRCCEN_TIM7    STM32_RCC_APB1ENR
#  define TIMEN_TIM7       RCC_APB1ENR_TIM7EN
#  define TIMRCCRST_TIM7   STM32_RCC_APB1RSTR
#  define TIMRST_TIM7      RCC_APB1RSTR_TIM7RST
#  define TIMCLK_TIM12     STM32_APB1_TIM12_CLKIN
#  define TIMRCCEN_TIM12   STM32_RCC_APB1ENR
#  define TIMEN_TIM12      RCC_APB1ENR_TIM12EN
#  define TIMRCCRST_TIM12  STM32_RCC_APB1RSTR
#  define TIMRST_TIM12     RCC_APB1RSTR_TIM12RST
#  define TIMCLK_TIM13     STM32_APB1_TIM13_CLKIN
#  define TIMRCCEN_TIM13   STM32_RCC_APB1ENR
#  define TIMEN_TIM13      RCC_APB1ENR_TIM13EN
#  define TIMRCCRST_TIM13  STM32_RCC_APB1RSTR
#  define TIMRST_TIM13     RCC_APB1RSTR_TIM13RST
#  define TIMCLK_TIM14     STM32_APB1_TIM14_CLKIN
#  define TIMRCCEN_TIM14   STM32_RCC_APB1ENR
#  define TIMEN_TIM14      RCC_APB1ENR_TIM14EN
#  define TIMRCCRST_TIM14  STM32_RCC_APB1RSTR
#  define TIMRST_TIM14     RCC_APB1RSTR_TIM14RST
#  define TIMCLK_TIM15     STM32_APB2_TIM15_CLKIN
#  define TIMRCCEN_TIM15   STM32_RCC_APB2ENR
#  define TIMEN_TIM15      RCC_APB2ENR_TIM15EN
#  define TIMRCCRST_TIM15  STM32_RCC_APB2RSTR
#  define TIMRST_TIM15     RCC_APB2RSTR_TIM15RST
#  define TIMCLK_TIM16     STM32_APB2_TIM16_CLKIN
#  define TIMRCCEN_TIM16   STM32_RCC_APB2ENR
#  define TIMEN_TIM16      RCC_APB2ENR_TIM16EN
#  define TIMRCCRST_TIM16  STM32_RCC_APB2RSTR
#  define TIMRST_TIM16     RCC_APB2RSTR_TIM16RST
#  define TIMCLK_TIM17     STM32_APB2_TIM17_CLKIN
#  define TIMRCCEN_TIM17   STM32_RCC_APB2ENR
#  define TIMEN_TIM17      RCC_APB2ENR_TIM17EN
#  define TIMRCCRST_TIM17  STM32_RCC_APB2RSTR
#  define TIMRST_TIM17     RCC_APB2RSTR_TIM17RST
#  define TIMCLK_TIM18     STM32_APB1_TIM18_CLKIN
#  define TIMRCCEN_TIM18   STM32_RCC_APB1ENR
#  define TIMEN_TIM18      RCC_APB1ENR_TIM18EN
#  define TIMRCCRST_TIM18  STM32_RCC_APB1RSTR
#  define TIMRST_TIM18     RCC_APB1RSTR_TIM18RST
#  define TIMCLK_TIM19     STM32_APB2_TIM19_CLKIN
#  define TIMRCCEN_TIM19   STM32_RCC_APB2ENR
#  define TIMEN_TIM19      RCC_APB2ENR_TIM19EN
#  define TIMRCCRST_TIM19  STM32_RCC_APB2RSTR
#  define TIMRST_TIM19     RCC_APB2RSTR_TIM19RST
#elif defined(CONFIG_STM32_STM32G4XXX)
#  define TIMCLK_TIM1      STM32_APB2_TIM1_CLKIN
#  define TIMRCCEN_TIM1    STM32_RCC_APB2ENR
#  define TIMEN_TIM1       RCC_APB2ENR_TIM1EN
#  define TIMRCCRST_TIM1   STM32_RCC_APB2RSTR
#  define TIMRST_TIM1      RCC_APB2RSTR_TIM1RST
#  define TIMCLK_TIM2      STM32_APB1_TIM2_CLKIN
#  define TIMRCCEN_TIM2    STM32_RCC_APB1ENR
#  define TIMEN_TIM2       RCC_APB1ENR_TIM2EN
#  define TIMRCCRST_TIM2   STM32_RCC_APB1RSTR
#  define TIMRST_TIM2      RCC_APB1RSTR_TIM2RST
#  define TIMCLK_TIM3      STM32_APB1_TIM3_CLKIN
#  define TIMRCCEN_TIM3    STM32_RCC_APB1ENR
#  define TIMEN_TIM3       RCC_APB1ENR1_TIM3EN
#  define TIMRCCRST_TIM3   STM32_RCC_APB1RSTR
#  define TIMRST_TIM3      RCC_APB1RSTR_TIM3RST
#  define TIMCLK_TIM4      STM32_APB1_TIM4_CLKIN
#  define TIMRCCEN_TIM4    STM32_RCC_APB1ENR
#  define TIMEN_TIM4       RCC_APB1ENR_TIM4EN
#  define TIMRCCRST_TIM4   STM32_RCC_APB1RSTR
#  define TIMRST_TIM4      RCC_APB1RSTR_TIM4RST
#  define TIMCLK_TIM5      STM32_APB1_TIM5_CLKIN
#  define TIMRCCEN_TIM5    STM32_RCC_APB1ENR
#  define TIMEN_TIM5       RCC_APB1ENR_TIM5EN
#  define TIMRCCRST_TIM5   STM32_RCC_APB1RSTR
#  define TIMRST_TIM5      RCC_APB1RSTR_TIM5RST
#  define TIMCLK_TIM8      STM32_APB2_TIM8_CLKIN
#  define TIMRCCEN_TIM8    STM32_RCC_APB2ENR
#  define TIMEN_TIM8       RCC_APB2ENR_TIM8EN
#  define TIMRCCRST_TIM8   STM32_RCC_APB2RSTR
#  define TIMRST_TIM8      RCC_APB2RSTR_TIM8RST
#  define TIMCLK_TIM15     STM32_APB2_TIM15_CLKIN
#  define TIMRCCEN_TIM15   STM32_RCC_APB2ENR
#  define TIMEN_TIM15      RCC_APB2ENR_TIM15EN
#  define TIMRCCRST_TIM15  STM32_RCC_APB2RSTR
#  define TIMRST_TIM15     RCC_APB2RSTR_TIM15RST
#  define TIMCLK_TIM16     STM32_APB2_TIM16_CLKIN
#  define TIMRCCEN_TIM16   STM32_RCC_APB1ENR
#  define TIMEN_TIM16      RCC_APB2ENR_TIM16EN
#  define TIMRCCRST_TIM16  STM32_RCC_APB1RSTR
#  define TIMRST_TIM16     RCC_APB1RSTR_TIM16RST
#  define TIMCLK_TIM17     STM32_APB2_TIM17_CLKIN
#  define TIMRCCEN_TIM17   STM32_RCC_APB1ENR
#  define TIMEN_TIM17      RCC_APB2ENR_TIM17EN
#  define TIMRCCRST_TIM17  STM32_RCC_APB1RSTR
#  define TIMRST_TIM17     RCC_APB1RSTR_TIM17RST
#  define TIMCLK_TIM20     STM32_APB2_TIM20_CLKIN
#  define TIMRCCEN_TIM20   STM32_RCC_APB2ENR
#  define TIMEN_TIM20      RCC_APB2ENR_TIM20EN
#else
#  define TIMCLK_TIM1      STM32_APB2_TIM1_CLKIN
#  define TIMRCCEN_TIM1    STM32_RCC_APB2ENR
#  define TIMEN_TIM1       RCC_APB2ENR_TIM1EN
#  define TIMRCCRST_TIM1   STM32_RCC_APB2RSTR
#  define TIMRST_TIM1      RCC_APB2RSTR_TIM1RST
#  define TIMCLK_TIM2      STM32_APB1_TIM2_CLKIN
#  define TIMRCCEN_TIM2    STM32_RCC_APB1ENR
#  define TIMEN_TIM2       RCC_APB1ENR_TIM2EN
#  define TIMRCCRST_TIM2   STM32_RCC_APB1RSTR
#  define TIMRST_TIM2      RCC_APB1RSTR_TIM2RST
#  define TIMCLK_TIM3      STM32_APB1_TIM3_CLKIN
#  define TIMRCCEN_TIM3    STM32_RCC_APB1ENR
#  define TIMEN_TIM3       RCC_APB1ENR_TIM3EN
#  define TIMRCCRST_TIM3   STM32_RCC_APB1RSTR
#  define TIMRST_TIM3      RCC_APB1RSTR_TIM3RST
#  define TIMCLK_TIM4      STM32_APB1_TIM4_CLKIN
#  define TIMRCCEN_TIM4    STM32_RCC_APB1ENR
#  define TIMEN_TIM4       RCC_APB1ENR_TIM4EN
#  define TIMRCCRST_TIM4   STM32_RCC_APB1RSTR
#  define TIMRST_TIM4      RCC_APB1RSTR_TIM4RST
#  define TIMCLK_TIM5      STM32_APB1_TIM5_CLKIN
#  define TIMRCCEN_TIM5    STM32_RCC_APB1ENR
#  define TIMEN_TIM5       RCC_APB1ENR_TIM5EN
#  define TIMRCCRST_TIM5   STM32_RCC_APB1RSTR
#  define TIMRST_TIM5      RCC_APB1RSTR_TIM5RST
#  define TIMCLK_TIM8      STM32_APB2_TIM8_CLKIN
#  define TIMRCCEN_TIM8    STM32_RCC_APB2ENR
#  define TIMEN_TIM8       RCC_APB2ENR_TIM8EN
#  define TIMRCCRST_TIM8   STM32_RCC_APB2RSTR
#  define TIMRST_TIM8      RCC_APB2RSTR_TIM8RST
#  define TIMCLK_TIM9      STM32_APB2_TIM9_CLKIN
#  define TIMRCCEN_TIM9    STM32_RCC_APB2ENR
#  define TIMEN_TIM9       RCC_APB2ENR_TIM9EN
#  define TIMRCCRST_TIM9   STM32_RCC_APB2RSTR
#  define TIMRST_TIM9      RCC_APB2RSTR_TIM9RST
#  define TIMCLK_TIM10     STM32_APB2_TIM10_CLKIN
#  define TIMRCCEN_TIM10   STM32_RCC_APB2ENR
#  define TIMEN_TIM10      RCC_APB2ENR_TIM10EN
#  define TIMRCCRST_TIM10  STM32_RCC_APB2RSTR
#  define TIMRST_TIM10     RCC_APB2RSTR_TIM10RST
#  define TIMCLK_TIM11     STM32_APB2_TIM11_CLKIN
#  define TIMRCCEN_TIM11   STM32_RCC_APB2ENR
#  define TIMEN_TIM11      RCC_APB2ENR_TIM11EN
#  define TIMRCCRST_TIM11  STM32_RCC_APB2RSTR
#  define TIMRST_TIM11     RCC_APB2RSTR_TIM11RST
#  define TIMCLK_TIM12     STM32_APB1_TIM12_CLKIN
#  define TIMRCCEN_TIM12   STM32_RCC_APB1ENR
#  define TIMEN_TIM12      RCC_APB1ENR_TIM12EN
#  define TIMRCCRST_TIM12  STM32_RCC_APB1RSTR
#  define TIMRST_TIM12     RCC_APB1RSTR_TIM12RST
#  define TIMCLK_TIM13     STM32_APB1_TIM13_CLKIN
#  define TIMRCCEN_TIM13   STM32_RCC_APB1ENR
#  define TIMEN_TIM13      RCC_APB1ENR_TIM13EN
#  define TIMRCCRST_TIM13  STM32_RCC_APB1RSTR
#  define TIMRST_TIM13     RCC_APB1RSTR_TIM13RST
#  define TIMCLK_TIM14     STM32_APB1_TIM14_CLKIN
#  define TIMRCCEN_TIM14   STM32_RCC_APB1ENR
#  define TIMEN_TIM14      RCC_APB1ENR_TIM14EN
#  define TIMRCCRST_TIM14  STM32_RCC_APB1RSTR
#  define TIMRST_TIM14     RCC_APB1RSTR_TIM14RST
#  define TIMCLK_TIM15     STM32_APB1_TIM15_CLKIN
#  define TIMRCCEN_TIM15   STM32_RCC_APB1ENR
#  define TIMEN_TIM15      RCC_APB1ENR_TIM15EN
#  define TIMRCCRST_TIM15  STM32_RCC_APB1RSTR
#  define TIMRST_TIM15     RCC_APB1RSTR_TIM15RST
#  define TIMCLK_TIM16     STM32_APB1_TIM16_CLKIN
#  define TIMRCCEN_TIM16   STM32_RCC_APB1ENR
#  define TIMEN_TIM16      RCC_APB1ENR_TIM16EN
#  define TIMRCCRST_TIM16  STM32_RCC_APB1RSTR
#  define TIMRST_TIM16     RCC_APB1RSTR_TIM16RST
#  define TIMCLK_TIM17     STM32_APB1_TIM17_CLKIN
#  define TIMRCCEN_TIM17   STM32_RCC_APB1ENR
#  define TIMEN_TIM17      RCC_APB1ENR_TIM71EN
#  define TIMRCCRST_TIM17  STM32_RCC_APB1RSTR
#  define TIMRST_TIM17     RCC_APB1RSTR_TIM17RST
#endif

/* Default GPIO pins state */

#if defined(CONFIG_STM32_STM32F10XX)
#  define PINCFG_DEFAULT (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT)
#elif defined(CONFIG_STM32_STM32F20XX) ||         \
    defined(CONFIG_STM32_STM32F30XX) ||           \
    defined(CONFIG_STM32_STM32F33XX) ||           \
    defined(CONFIG_STM32_STM32F37XX) ||           \
    defined(CONFIG_STM32_STM32F4XXX) ||           \
    defined(CONFIG_STM32_STM32L15XX) ||           \
    defined(CONFIG_STM32_STM32G4XXX)
#  define PINCFG_DEFAULT (GPIO_INPUT | GPIO_FLOAT)
#else
#  error "Unrecognized STM32 chip"
#endif

/* Advanced Timer support
 * NOTE: TIM15-17 are not ADVTIM but they support most of the
 *       ADVTIM functionality.  The main difference is the number of
 *       supported capture/compare.
 */

#if defined(CONFIG_STM32_TIM1_PWM) || defined(CONFIG_STM32_TIM8_PWM) || \
    defined(CONFIG_STM32_TIM15_PWM) || defined(CONFIG_STM32_TIM16_PWM) || \
    defined(CONFIG_STM32_TIM17_PWM)
#  define HAVE_ADVTIM
#else
#  undef HAVE_ADVTIM
#endif

/* Pulsecount support */

#ifdef CONFIG_PWM_PULSECOUNT
#  ifndef HAVE_ADVTIM
#    error "PWM_PULSECOUNT requires HAVE_ADVTIM"
#  endif
#  if defined(CONFIG_STM32_TIM1_PWM) || defined(CONFIG_STM32_TIM8_PWM)
#    define HAVE_PWM_INTERRUPT
#  endif
#endif

/* TRGO/TRGO2 support */

#ifdef CONFIG_STM32_PWM_TRGO
#  define HAVE_TRGO
#endif

/* Break support */

#if defined(CONFIG_STM32_TIM1_BREAK1) || defined(CONFIG_STM32_TIM1_BREAK2) || \
    defined(CONFIG_STM32_TIM8_BREAK1) || defined(CONFIG_STM32_TIM8_BREAK2) || \
    defined(CONFIG_STM32_TIM15_BREAK1) || defined(CONFIG_STM32_TIM16_BREAK1) || \
    defined(CONFIG_STM32_TIM17_BREAK1)
#  defined HAVE_BREAK
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
#  define pwm_dumpgpio(p,m) stm32_dumpgpio(p,m)
#else
#  define pwm_dumpgpio(p,m)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* PWM output configuration */

struct stm32_pwm_out_s
{
  uint8_t  in_use:1;                    /* Output in use */
  uint8_t  pol:1;                       /* Polarity. Default: positive */
  uint8_t  idle:1;                      /* Idle state. Default: inactive */
  uint8_t  _res:5;                      /* Reserved */
  uint32_t pincfg;                      /* Output pin configuration */
};

/* PWM break configuration */

#ifdef HAVE_BREAK
struct stm32_pwm_break_s
{
  uint8_t en1:1;                        /* Break 1 enable */
  uint8_t pol1:1;                       /* Break 1 polarity */
  uint8_t _res:6;                       /* Reserved */
#ifdef HAVE_IP_TIMERS_V2
  uint8_t en2:1;                        /* Break 2 enable */
  uint8_t pol2:1;                       /* Break 2 polarity */
  uint8_t flt2:6;                       /* Break 2 filter */
#endif
};
#endif

/* PWM channel configuration */

struct stm32_pwmchan_s
{
  uint8_t                  channel:4;   /* Timer output channel: {1,..4} */
  uint8_t                  mode:4;      /* PWM channel mode (see stm32_pwm_chanmode_e) */
  struct stm32_pwm_out_s   out1;        /* PWM output configuration */
#ifdef HAVE_BREAK
  struct stm32_pwm_break_s brk;         /* PWM break configuration */
#endif
#ifdef HAVE_PWM_COMPLEMENTARY
  struct stm32_pwm_out_s   out2;        /* PWM complementary output configuration */
#endif
};

/* This structure represents the state of one PWM timer */

struct stm32_pwmtimer_s
{
  const struct pwm_ops_s *ops;      /* PWM operations */
#ifdef CONFIG_STM32_PWM_LL_OPS
  const struct stm32_pwm_ops_s *llops; /* Low-level PWM ops */
#endif
  struct stm32_pwmchan_s *channels; /* Channels configuration */
  uint8_t  timid:5;                 /* Timer ID {1,...,17} */
  uint8_t  chan_num:3;              /* Number of configured channels */
  uint8_t  timtype:3;               /* See the TIMTYPE_* definitions */
  uint8_t  mode:3;                  /* Timer mode (see stm32_pwm_tim_mode_e) */
  uint8_t  lock:2;                  /* TODO: Lock configuration */
  uint8_t  t_dts:3;                 /* Clock division for t_DTS */
  uint8_t  _res:5;                  /* Reserved */
#ifdef HAVE_PWM_COMPLEMENTARY
  uint8_t  deadtime;                /* Dead-time value */
#endif
#ifdef HAVE_TRGO
  uint8_t  trgo;                    /* TRGO configuration:
                                     * 4 LSB = TRGO, 4 MSB = TRGO2
                                     */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  uint8_t  irq;                     /* Timer update IRQ */
  uint8_t  prev;                    /* The previous value of the RCR (pre-loaded) */
  uint8_t  curr;                    /* The current value of the RCR (pre-loaded) */
  uint32_t count;                   /* Remaining pulse count */
#endif
  uint32_t frequency;               /* Current frequency setting */
  uint32_t base;                    /* The base address of the timer */
  uint32_t pclk;                    /* The frequency of the peripheral
                                     * clock that drives the timer module
                                     */
#ifdef CONFIG_PWM_PULSECOUNT
  void *handle;                     /* Handle used for upper-half callback */
#endif
};

/****************************************************************************
 * Static Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t pwm_getreg(struct stm32_pwmtimer_s *priv, int offset);
static void pwm_putreg(struct stm32_pwmtimer_s *priv, int offset,
                       uint32_t value);
static void pwm_modifyreg(struct stm32_pwmtimer_s *priv, uint32_t offset,
                          uint32_t clearbits, uint32_t setbits);

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct pwm_lowerhalf_s *dev,
                         const char *msg);
#else
#  define pwm_dumpregs(priv,msg)
#endif

/* Timer management */

static int pwm_frequency_update(struct pwm_lowerhalf_s *dev,
                                uint32_t frequency);
static int pwm_mode_configure(struct pwm_lowerhalf_s *dev,
                              uint8_t channel, uint32_t mode);
static int pwm_timer_configure(struct stm32_pwmtimer_s *priv);
static int pwm_output_configure(struct stm32_pwmtimer_s *priv,
                                struct stm32_pwmchan_s *chan);
static int pwm_outputs_enable(struct pwm_lowerhalf_s *dev,
                              uint16_t outputs, bool state);
static int pwm_soft_update(struct pwm_lowerhalf_s *dev);
static int pwm_soft_break(struct pwm_lowerhalf_s *dev, bool state);
static int pwm_ccr_update(struct pwm_lowerhalf_s *dev, uint8_t index,
                          uint32_t ccr);
static int pwm_arr_update(struct pwm_lowerhalf_s *dev, uint32_t arr);
static uint32_t pwm_arr_get(struct pwm_lowerhalf_s *dev);
static int pwm_duty_update(struct pwm_lowerhalf_s *dev, uint8_t channel,
                           ub16_t duty);
static int pwm_timer_enable(struct pwm_lowerhalf_s *dev, bool state);

#ifdef HAVE_ADVTIM
static int pwm_break_dt_configure(struct stm32_pwmtimer_s *priv);
#endif
#ifdef HAVE_TRGO
static int pwm_trgo_configure(struct pwm_lowerhalf_s *dev,
                              uint8_t trgo);
#endif
#if defined(HAVE_PWM_COMPLEMENTARY) && defined(CONFIG_STM32_PWM_LL_OPS)
static int pwm_deadtime_update(struct pwm_lowerhalf_s *dev, uint8_t dt);
#endif
#ifdef CONFIG_STM32_PWM_LL_OPS
static uint32_t pwm_ccr_get(struct pwm_lowerhalf_s *dev, uint8_t index);
static uint16_t pwm_rcr_get(struct pwm_lowerhalf_s *dev);
#endif
#ifdef HAVE_ADVTIM
static int pwm_rcr_update(struct pwm_lowerhalf_s *dev, uint16_t rcr);
#endif

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_pulsecount_configure(struct pwm_lowerhalf_s *dev);
#else
static int pwm_configure(struct pwm_lowerhalf_s *dev);
#endif
#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_pulsecount_timer(struct pwm_lowerhalf_s *dev,
                                const struct pwm_info_s *info);
#endif
static int pwm_timer(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);
#ifdef HAVE_PWM_INTERRUPT
static int pwm_interrupt(struct pwm_lowerhalf_s *dev);
#  ifdef CONFIG_STM32_TIM1_PWM
static int pwm_tim1interrupt(int irq, void *context, void *arg);
#  endif
#  ifdef CONFIG_STM32_TIM8_PWM
static int pwm_tim8interrupt(int irq, void *context, void *arg);
#  endif
static uint8_t pwm_pulsecount(uint32_t count);
#endif

/* PWM driver methods */

static int pwm_setup(struct pwm_lowerhalf_s *dev);
static int pwm_shutdown(struct pwm_lowerhalf_s *dev);

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start_pulsecount(struct pwm_lowerhalf_s *dev,
                                const struct pwm_info_s *info,
                                void *handle);
#endif
static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info);

static int pwm_stop(struct pwm_lowerhalf_s *dev);
static int pwm_ioctl(struct pwm_lowerhalf_s *dev,
                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the list of lower half PWM driver methods used by the upper half
 * driver.
 */

static const struct pwm_ops_s g_pwmops =
{
  .setup       = pwm_setup,
  .shutdown    = pwm_shutdown,
#ifdef CONFIG_PWM_PULSECOUNT
  .start       = pwm_start_pulsecount,
#else
  .start       = pwm_start,
#endif
  .stop        = pwm_stop,
  .ioctl       = pwm_ioctl,
};

#ifdef CONFIG_STM32_PWM_LL_OPS
static const struct stm32_pwm_ops_s g_llpwmops =
{
  .configure       = pwm_configure,
  .soft_break      = pwm_soft_break,
  .ccr_update      = pwm_ccr_update,
  .mode_update     = pwm_mode_configure,
  .ccr_get         = pwm_ccr_get,
  .arr_update      = pwm_arr_update,
  .arr_get         = pwm_arr_get,
#ifdef HAVE_ADVTIM
  .rcr_update      = pwm_rcr_update,
#endif
  .rcr_get         = pwm_rcr_get,
#ifdef HAVE_TRGO
  .trgo_set        = pwm_trgo_configure,
#endif
  .outputs_enable  = pwm_outputs_enable,
  .soft_update     = pwm_soft_update,
  .freq_update     = pwm_frequency_update,
  .tim_enable      = pwm_timer_enable,
#  ifdef CONFIG_DEBUG_PWM_INFO
  .dump_regs       = pwm_dumpregs,
#  endif
#  ifdef HAVE_PWM_COMPLEMENTARY
  .dt_update       = pwm_deadtime_update,
#  endif
};
#endif

#ifdef CONFIG_STM32_TIM1_PWM

static struct stm32_pwmchan_s g_pwm1channels[] =
{
  /* TIM1 has 4 channels, 4 complementary */

#ifdef CONFIG_STM32_TIM1_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM1_CH1MODE,
#ifdef HAVE_BREAK
    .brk =
    {
#ifdef CONFIG_STM32_TIM1_BREAK1
      .en1 = 1,
      .pol1 = CONFIG_STM32_TIM1_BRK1POL,
#endif
#ifdef CONFIG_STM32_TIM1_BREAK2
      .en2 = 1,
      .pol2 = CONFIG_STM32_TIM1_BRK2POL,
      .flt2 = CONFIG_STM32_TIM1_BRK2FLT,
#endif
    },
#endif
#ifdef CONFIG_STM32_TIM1_CH1OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_CH1POL,
      .idle    = CONFIG_STM32_TIM1_CH1IDLE,
      .pincfg  = PWM_TIM1_CH1CFG,
    },
#endif
#ifdef CONFIG_STM32_TIM1_CH1NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_CH1NPOL,
      .idle    = CONFIG_STM32_TIM1_CH1NIDLE,
      .pincfg  = PWM_TIM1_CH1NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM1_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32_TIM1_CH2MODE,
#ifdef CONFIG_STM32_TIM1_CH2OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_CH2POL,
      .idle    = CONFIG_STM32_TIM1_CH2IDLE,
      .pincfg  = PWM_TIM1_CH2CFG,
    },
#endif
#ifdef CONFIG_STM32_TIM1_CH2NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_CH2NPOL,
      .idle    = CONFIG_STM32_TIM1_CH2NIDLE,
      .pincfg  = PWM_TIM1_CH2NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM1_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32_TIM1_CH3MODE,
#ifdef CONFIG_STM32_TIM1_CH3OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_CH3POL,
      .idle    = CONFIG_STM32_TIM1_CH3IDLE,
      .pincfg  = PWM_TIM1_CH3CFG,
    },
#endif
#ifdef CONFIG_STM32_TIM1_CH3NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_CH3NPOL,
      .idle    = CONFIG_STM32_TIM1_CH3NIDLE,
      .pincfg  = PWM_TIM1_CH3NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM1_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32_TIM1_CH4MODE,
#ifdef CONFIG_STM32_TIM1_CH4OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_CH4POL,
      .idle    = CONFIG_STM32_TIM1_CH4IDLE,
      .pincfg  = PWM_TIM1_CH4CFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM1_CHANNEL5
  {
    .channel = 5,
    .mode    = CONFIG_STM32_TIM1_CH5MODE,
#ifdef CONFIG_STM32_TIM1_CH5OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_CH5POL,
      .idle    = CONFIG_STM32_TIM1_CH5IDLE,
      .pincfg  = 0,    /* Not available externally */
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM1_CHANNEL6
  {
    .channel = 6,
    .mode    = CONFIG_STM32_TIM1_CH6MODE,
#ifdef CONFIG_STM32_TIM1_CH6OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM1_CH6POL,
      .idle    = CONFIG_STM32_TIM1_CH6IDLE,
      .pincfg  = 0,    /* Not available externally */
    }
#endif
  }
#endif
};

static struct stm32_pwmtimer_s g_pwm1dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 1,
  .chan_num    = PWM_TIM1_NCHANNELS,
  .channels    = g_pwm1channels,
  .timtype     = TIMTYPE_TIM1,
  .mode        = CONFIG_STM32_TIM1_MODE,
  .lock        = CONFIG_STM32_TIM1_LOCK,
  .t_dts       = CONFIG_STM32_TIM1_TDTS,
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = CONFIG_STM32_TIM1_DEADTIME,
#endif
#if defined(HAVE_TRGO) && defined(STM32_TIM1_TRGO)
  .trgo        = STM32_TIM1_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM1UP,
#endif
  .base        = STM32_TIM1_BASE,
  .pclk        = TIMCLK_TIM1,
};
#endif /* CONFIG_STM32_TIM1_PWM */

#ifdef CONFIG_STM32_TIM2_PWM

static struct stm32_pwmchan_s g_pwm2channels[] =
{
  /* TIM2 has 4 channels */

#ifdef CONFIG_STM32_TIM2_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM2_CH1MODE,
#ifdef CONFIG_STM32_TIM2_CH1OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM2_CH1POL,
      .idle    = CONFIG_STM32_TIM2_CH1IDLE,
      .pincfg  = PWM_TIM2_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM2_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32_TIM2_CH2MODE,
#ifdef CONFIG_STM32_TIM2_CH2OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM2_CH2POL,
      .idle    = CONFIG_STM32_TIM2_CH2IDLE,
      .pincfg  = PWM_TIM2_CH2CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM2_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32_TIM2_CH3MODE,
#ifdef CONFIG_STM32_TIM2_CH3OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM2_CH3POL,
      .idle    = CONFIG_STM32_TIM2_CH3IDLE,
      .pincfg  = PWM_TIM2_CH3CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM2_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32_TIM2_CH4MODE,
#ifdef CONFIG_STM32_TIM2_CH4OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM2_CH4POL,
      .idle    = CONFIG_STM32_TIM2_CH4IDLE,
      .pincfg  = PWM_TIM2_CH4CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32_pwmtimer_s g_pwm2dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 2,
  .chan_num    = PWM_TIM2_NCHANNELS,
  .channels    = g_pwm2channels,
  .timtype     = TIMTYPE_TIM2,
  .mode        = CONFIG_STM32_TIM2_MODE,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO) && defined(STM32_TIM2_TRGO)
  .trgo        = STM32_TIM2_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM2,
#endif
  .base        = STM32_TIM2_BASE,
  .pclk        = TIMCLK_TIM2,
};
#endif /* CONFIG_STM32_TIM2_PWM */

#ifdef CONFIG_STM32_TIM3_PWM

static struct stm32_pwmchan_s g_pwm3channels[] =
{
  /* TIM3 has 4 channels */

#ifdef CONFIG_STM32_TIM3_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM3_CH1MODE,
#ifdef CONFIG_STM32_TIM3_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM3_CH1POL,
      .idle    = CONFIG_STM32_TIM3_CH1IDLE,
      .pincfg  = PWM_TIM3_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM3_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32_TIM3_CH2MODE,
#ifdef CONFIG_STM32_TIM3_CH2OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM3_CH2POL,
      .idle    = CONFIG_STM32_TIM3_CH2IDLE,
      .pincfg  = PWM_TIM3_CH2CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM3_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32_TIM3_CH3MODE,
#ifdef CONFIG_STM32_TIM3_CH3OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM3_CH3POL,
      .idle    = CONFIG_STM32_TIM3_CH3IDLE,
      .pincfg  = PWM_TIM3_CH3CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM3_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32_TIM3_CH4MODE,
#ifdef CONFIG_STM32_TIM3_CH4OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM3_CH4POL,
      .idle    = CONFIG_STM32_TIM3_CH4IDLE,
      .pincfg  = PWM_TIM3_CH4CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32_pwmtimer_s g_pwm3dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 3,
  .chan_num    = PWM_TIM3_NCHANNELS,
  .channels    = g_pwm3channels,
  .timtype     = TIMTYPE_TIM3,
  .mode        = CONFIG_STM32_TIM3_MODE,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO) && defined(STM32_TIM3_TRGO)
  .trgo        = STM32_TIM3_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM3,
#endif
  .base        = STM32_TIM3_BASE,
  .pclk        = TIMCLK_TIM3,
};
#endif /* CONFIG_STM32_TIM3_PWM */

#ifdef CONFIG_STM32_TIM4_PWM

static struct stm32_pwmchan_s g_pwm4channels[] =
{
  /* TIM4 has 4 channels */

#ifdef CONFIG_STM32_TIM4_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM4_CH1MODE,
#ifdef CONFIG_STM32_TIM4_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM4_CH1POL,
      .idle    = CONFIG_STM32_TIM4_CH1IDLE,
      .pincfg  = PWM_TIM4_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM4_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32_TIM4_CH2MODE,
#ifdef CONFIG_STM32_TIM4_CH2OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM4_CH2POL,
      .idle    = CONFIG_STM32_TIM4_CH2IDLE,
      .pincfg  = PWM_TIM4_CH2CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM4_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32_TIM4_CH3MODE,
#ifdef CONFIG_STM32_TIM4_CH3OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM4_CH3POL,
      .idle    = CONFIG_STM32_TIM4_CH3IDLE,
      .pincfg  = PWM_TIM4_CH3CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM4_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32_TIM4_CH4MODE,
#ifdef CONFIG_STM32_TIM4_CH4OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM4_CH4POL,
      .idle    = CONFIG_STM32_TIM4_CH4IDLE,
      .pincfg  = PWM_TIM4_CH4CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32_pwmtimer_s g_pwm4dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 4,
  .chan_num    = PWM_TIM4_NCHANNELS,
  .channels    = g_pwm4channels,
  .timtype     = TIMTYPE_TIM4,
  .mode        = CONFIG_STM32_TIM4_MODE,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO) && defined(STM32_TIM4_TRGO)
  .trgo        = STM32_TIM4_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM4,
#endif
  .base        = STM32_TIM4_BASE,
  .pclk        = TIMCLK_TIM4,
};
#endif /* CONFIG_STM32_TIM4_PWM */

#ifdef CONFIG_STM32_TIM5_PWM

static struct stm32_pwmchan_s g_pwm5channels[] =
{
  /* TIM5 has 4 channels */

#ifdef CONFIG_STM32_TIM5_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM5_CH1MODE,
#ifdef CONFIG_STM32_TIM5_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM5_CH1POL,
      .idle    = CONFIG_STM32_TIM5_CH1IDLE,
      .pincfg  = PWM_TIM5_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM5_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32_TIM5_CH2MODE,
#ifdef CONFIG_STM32_TIM5_CH2OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM5_CH2POL,
      .idle    = CONFIG_STM32_TIM5_CH2IDLE,
      .pincfg  = PWM_TIM5_CH2CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM5_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32_TIM5_CH3MODE,
#ifdef CONFIG_STM32_TIM5_CH3OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM5_CH3POL,
      .idle    = CONFIG_STM32_TIM5_CH3IDLE,
      .pincfg  = PWM_TIM5_CH3CFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM5_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32_TIM5_CH4MODE,
#ifdef CONFIG_STM32_TIM5_CH4OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM5_CH4POL,
      .idle    = CONFIG_STM32_TIM5_CH4IDLE,
      .pincfg  = PWM_TIM5_CH4CFG,
    }
#endif
  },
#endif
};

static struct stm32_pwmtimer_s g_pwm5dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 5,
  .chan_num    = PWM_TIM5_NCHANNELS,
  .channels    = g_pwm5channels,
  .timtype     = TIMTYPE_TIM5,
  .mode        = CONFIG_STM32_TIM5_MODE,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO) && defined(STM32_TIM5_TRGO)
  .trgo        = STM32_TIM5_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM5,
#endif
  .base        = STM32_TIM5_BASE,
  .pclk        = TIMCLK_TIM5,
};
#endif /* CONFIG_STM32_TIM5_PWM */

#ifdef CONFIG_STM32_TIM8_PWM

static struct stm32_pwmchan_s g_pwm8channels[] =
{
  /* TIM8 has 4 channels, 4 complementary */

#ifdef CONFIG_STM32_TIM8_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM8_CH1MODE,
#ifdef HAVE_BREAK
    .brk =
    {
#ifdef CONFIG_STM32_TIM8_BREAK1
      .en1 = 1,
      .pol1 = CONFIG_STM32_TIM8_BRK1POL,
#endif
#ifdef CONFIG_STM32_TIM8_BREAK2
      .en2 = 1,
      .pol2 = CONFIG_STM32_TIM8_BRK2POL,
      .flt2 = CONFIG_STM32_TIM8_BRK2FLT,
#endif
    },
#endif
#ifdef CONFIG_STM32_TIM8_CH1OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_CH1POL,
      .idle    = CONFIG_STM32_TIM8_CH1IDLE,
      .pincfg  = PWM_TIM8_CH1CFG,
    },
#endif
#ifdef CONFIG_STM32_TIM8_CH1NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_CH1NPOL,
      .idle    = CONFIG_STM32_TIM8_CH1NIDLE,
      .pincfg  = PWM_TIM8_CH1NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM8_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32_TIM8_CH2MODE,
#ifdef CONFIG_STM32_TIM8_CH2OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_CH2POL,
      .idle    = CONFIG_STM32_TIM8_CH2IDLE,
      .pincfg  = PWM_TIM8_CH2CFG,
    },
#endif
#ifdef CONFIG_STM32_TIM8_CH2NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_CH2NPOL,
      .idle    = CONFIG_STM32_TIM8_CH2NIDLE,
      .pincfg  = PWM_TIM8_CH2NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM8_CHANNEL3
  {
    .channel = 3,
    .mode    = CONFIG_STM32_TIM8_CH3MODE,
#ifdef CONFIG_STM32_TIM8_CH3OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_CH3POL,
      .idle    = CONFIG_STM32_TIM8_CH3IDLE,
      .pincfg  = PWM_TIM8_CH3CFG,
    },
#endif
#ifdef CONFIG_STM32_TIM8_CH3NOUT
    .out2 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_CH3NPOL,
      .idle    = CONFIG_STM32_TIM8_CH3NIDLE,
      .pincfg  = PWM_TIM8_CH3NCFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM8_CHANNEL4
  {
    .channel = 4,
    .mode    = CONFIG_STM32_TIM8_CH4MODE,
#ifdef CONFIG_STM32_TIM8_CH4OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_CH4POL,
      .idle    = CONFIG_STM32_TIM8_CH4IDLE,
      .pincfg  = PWM_TIM8_CH4CFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM8_CHANNEL5
  {
    .channel = 5,
    .mode    = CONFIG_STM32_TIM8_CH5MODE,
#ifdef CONFIG_STM32_TIM8_CH5OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_CH5POL,
      .idle    = CONFIG_STM32_TIM8_CH5IDLE,
      .pincfg  = 0,    /* Not available externally */
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM8_CHANNEL6
  {
    .channel = 6,
    .mode    = CONFIG_STM32_TIM8_CH6MODE,
#ifdef CONFIG_STM32_TIM8_CH6OUT
    .out1 =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM8_CH6POL,
      .idle    = CONFIG_STM32_TIM8_CH6IDLE,
      .pincfg  = 0,    /* Not available externally */
    }
#endif
  }
#endif
};

static struct stm32_pwmtimer_s g_pwm8dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 8,
  .chan_num    = PWM_TIM8_NCHANNELS,
  .channels    = g_pwm8channels,
  .timtype     = TIMTYPE_TIM8,
  .mode        = CONFIG_STM32_TIM8_MODE,
  .lock        = CONFIG_STM32_TIM8_LOCK,
  .t_dts       = CONFIG_STM32_TIM8_TDTS,
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = CONFIG_STM32_TIM8_DEADTIME,
#endif
#if defined(HAVE_TRGO) && defined(STM32_TIM8_TRGO)
  .trgo        = STM32_TIM8_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM8UP,
#endif
  .base        = STM32_TIM8_BASE,
  .pclk        = TIMCLK_TIM8,
};
#endif /* CONFIG_STM32_TIM8_PWM */

#ifdef CONFIG_STM32_TIM9_PWM

static struct stm32_pwmchan_s g_pwm9channels[] =
{
  /* TIM9 has 2 channels */

#ifdef CONFIG_STM32_TIM9_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM9_CH1MODE,
#ifdef CONFIG_STM32_TIM9_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM9_CH1POL,
      .idle    = CONFIG_STM32_TIM9_CH1IDLE,
      .pincfg  = PWM_TIM9_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM9_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32_TIM9_CH2MODE,
#ifdef CONFIG_STM32_TIM9_CH2OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM9_CH2POL,
      .idle    = CONFIG_STM32_TIM9_CH2IDLE,
      .pincfg  = PWM_TIM9_CH2CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32_pwmtimer_s g_pwm9dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 9,
  .chan_num    = PWM_TIM9_NCHANNELS,
  .channels    = g_pwm9channels,
  .timtype     = TIMTYPE_TIM9,
  .mode        = STM32_TIMMODE_COUNTUP,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for TIM9 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM9,
#endif
  .base        = STM32_TIM9_BASE,
  .pclk        = TIMCLK_TIM9,
};
#endif /* CONFIG_STM32_TIM9_PWM */

#ifdef CONFIG_STM32_TIM10_PWM

static struct stm32_pwmchan_s g_pwm10channels[] =
{
  /* TIM10 has 1 channel */

#ifdef CONFIG_STM32_TIM10_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM10_CH1MODE,
#ifdef CONFIG_STM32_TIM10_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM10_CH1POL,
      .idle    = CONFIG_STM32_TIM10_CH1IDLE,
      .pincfg  = PWM_TIM10_CH1CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32_pwmtimer_s g_pwm10dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 10,
  .chan_num    = PWM_TIM10_NCHANNELS,
  .channels    = g_pwm10channels,
  .timtype     = TIMTYPE_TIM10,
  .mode        = STM32_TIMMODE_COUNTUP,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for TIM10 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM10,
#endif
  .base        = STM32_TIM10_BASE,
  .pclk        = TIMCLK_TIM10,
};
#endif /* CONFIG_STM32_TIM10_PWM */

#ifdef CONFIG_STM32_TIM11_PWM

static struct stm32_pwmchan_s g_pwm11channels[] =
{
  /* TIM11 has 1 channel */

#ifdef CONFIG_STM32_TIM11_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM11_CH1MODE,
#ifdef CONFIG_STM32_TIM11_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM11_CH1POL,
      .idle    = CONFIG_STM32_TIM11_CH1IDLE,
      .pincfg  = PWM_TIM11_CH1CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32_pwmtimer_s g_pwm11dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 11,
  .chan_num    = PWM_TIM11_NCHANNELS,
  .channels    = g_pwm11channels,
  .timtype     = TIMTYPE_TIM11,
  .mode        = STM32_TIMMODE_COUNTUP,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for TIM11 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM11,
#endif
  .base        = STM32_TIM11_BASE,
  .pclk        = TIMCLK_TIM11,
};
#endif /* CONFIG_STM32_TIM11_PWM */

#ifdef CONFIG_STM32_TIM12_PWM

static struct stm32_pwmchan_s g_pwm12channels[] =
{
  /* TIM12 has 2 channels */

#ifdef CONFIG_STM32_TIM12_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM12_CH1MODE,
#ifdef CONFIG_STM32_TIM12_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM12_CH1POL,
      .idle    = CONFIG_STM32_TIM12_CH1IDLE,
      .pincfg  = PWM_TIM12_CH1CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
#ifdef CONFIG_STM32_TIM12_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32_TIM12_CH2MODE,
#ifdef CONFIG_STM32_TIM12_CH2OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM12_CH2POL,
      .idle    = CONFIG_STM32_TIM12_CH2IDLE,
      .pincfg  = PWM_TIM12_CH2CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32_pwmtimer_s g_pwm12dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 12,
  .chan_num    = PWM_TIM12_NCHANNELS,
  .channels    = g_pwm12channels,
  .timtype     = TIMTYPE_TIM12,
  .mode        = STM32_TIMMODE_COUNTUP,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for TIM12 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM12,
#endif
  .base        = STM32_TIM12_BASE,
  .pclk        = TIMCLK_TIM12,
};
#endif /* CONFIG_STM32_TIM12_PWM */

#ifdef CONFIG_STM32_TIM13_PWM

static struct stm32_pwmchan_s g_pwm13channels[] =
{
  /* TIM13 has 1 channel */

#ifdef CONFIG_STM32_TIM13_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM13_CH1MODE,
#ifdef CONFIG_STM32_TIM13_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM13_CH1POL,
      .idle    = CONFIG_STM32_TIM13_CH1IDLE,
      .pincfg  = PWM_TIM13_CH1CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32_pwmtimer_s g_pwm13dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 13,
  .chan_num    = PWM_TIM13_NCHANNELS,
  .channels    = g_pwm13channels,
  .timtype     = TIMTYPE_TIM13,
  .mode        = STM32_TIMMODE_COUNTUP,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for TIM13 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM13,
#endif
  .base        = STM32_TIM13_BASE,
  .pclk        = TIMCLK_TIM13,
};
#endif /* CONFIG_STM32_TIM13_PWM */

#ifdef CONFIG_STM32_TIM14_PWM

static struct stm32_pwmchan_s g_pwm14channels[] =
{
  /* TIM14 has 1 channel */

#ifdef CONFIG_STM32_TIM14_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM14_CH1MODE,
#ifdef CONFIG_STM32_TIM14_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM14_CH1POL,
      .idle    = CONFIG_STM32_TIM14_CH1IDLE,
      .pincfg  = PWM_TIM14_CH1CFG,
    }
#endif
    /* No complementary outputs */
  }
#endif
};

static struct stm32_pwmtimer_s g_pwm14dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 14,
  .chan_num    = PWM_TIM14_NCHANNELS,
  .channels    = g_pwm14channels,
  .timtype     = TIMTYPE_TIM14,
  .mode        = STM32_TIMMODE_COUNTUP,
  .lock        = 0,             /* No lock */
  .t_dts       = 0,             /* No t_dts */
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = 0,             /* No deadtime */
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for TIM14 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM14,
#endif
  .base        = STM32_TIM14_BASE,
  .pclk        = TIMCLK_TIM14,
};
#endif /* CONFIG_STM32_TIM14_PWM */

#ifdef CONFIG_STM32_TIM15_PWM

static struct stm32_pwmchan_s g_pwm15channels[] =
{
  /* TIM15 has 2 channels, 1 complementary */

#ifdef CONFIG_STM32_TIM15_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM15_CH1MODE,
#ifdef HAVE_BREAK
    .brk =
    {
#ifdef CONFIG_STM32_TIM15_BREAK1
      .en1 = 1,
      .pol1 = CONFIG_STM32_TIM15_BRK1POL,
#endif
      /* No BREAK2 */
    },
#endif
#ifdef CONFIG_STM32_TIM15_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM15_CH1POL,
      .idle    = CONFIG_STM32_TIM15_CH1IDLE,
      .pincfg  = PWM_TIM15_CH1CFG,
    },
#endif
#ifdef CONFIG_STM32_TIM15_CH1NOUT
    .out2    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM15_CH1NPOL,
      .idle    = CONFIG_STM32_TIM15_CH1NIDLE,
      .pincfg  = PWM_TIM15_CH2CFG,
    }
#endif
  },
#endif
#ifdef CONFIG_STM32_TIM15_CHANNEL2
  {
    .channel = 2,
    .mode    = CONFIG_STM32_TIM15_CH2MODE,
#ifdef CONFIG_STM32_TIM12_CH2OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM15_CH2POL,
      .idle    = CONFIG_STM32_TIM15_CH2IDLE,
      .pincfg  = PWM_TIM15_CH2CFG,
    }
#endif
    /* No complementary outputs */
  },
#endif
};

static struct stm32_pwmtimer_s g_pwm15dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 15,
  .chan_num    = PWM_TIM15_NCHANNELS,
  .channels    = g_pwm15channels,
  .timtype     = TIMTYPE_TIM15,
  .mode        = STM32_TIMMODE_COUNTUP,
  .lock        = CONFIG_STM32_TIM15_LOCK,
  .t_dts       = CONFIG_STM32_TIM15_TDTS,
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = CONFIG_STM32_TIM15_DEADTIME,
#endif
#if defined(HAVE_TRGO) && defined(STM32_TIM15_TRGO)
  .trgo        = STM32_TIM15_TRGO,
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM15,
#endif
  .base        = STM32_TIM15_BASE,
  .pclk        = TIMCLK_TIM15,
};
#endif /* CONFIG_STM32_TIM15_PWM */

#ifdef CONFIG_STM32_TIM16_PWM

static struct stm32_pwmchan_s g_pwm16channels[] =
{
  /* TIM16 has 1 channel, 1 complementary */

#ifdef CONFIG_STM32_TIM16_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM16_CH1MODE,
#ifdef HAVE_BREAK
    .brk =
    {
#ifdef CONFIG_STM32_TIM16_BREAK1
      .en1 = 1,
      .pol1 = CONFIG_STM32_TIM16_BRK1POL,
#endif
      /* No BREAK2 */
    },
#endif
#ifdef CONFIG_STM32_TIM16_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM16_CH1POL,
      .idle    = CONFIG_STM32_TIM16_CH1IDLE,
      .pincfg  = PWM_TIM16_CH1CFG,
    },
#endif
#ifdef CONFIG_STM32_TIM16_CH1NOUT
    .out2    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM16_CH1NPOL,
      .idle    = CONFIG_STM32_TIM16_CH1NIDLE,
      .pincfg  = PWM_TIM16_CH2CFG,
    }
#endif
  },
#endif
};

static struct stm32_pwmtimer_s g_pwm16dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 16,
  .chan_num    = PWM_TIM16_NCHANNELS,
  .channels    = g_pwm16channels,
  .timtype     = TIMTYPE_TIM16,
  .mode        = STM32_TIMMODE_COUNTUP,
  .lock        = CONFIG_STM32_TIM16_LOCK,
  .t_dts       = CONFIG_STM32_TIM16_TDTS,
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = CONFIG_STM32_TIM16_DEADTIME,
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for TIM16 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM16,
#endif
  .base        = STM32_TIM16_BASE,
  .pclk        = TIMCLK_TIM16,
};
#endif /* CONFIG_STM32_TIM16_PWM */

#ifdef CONFIG_STM32_TIM17_PWM

static struct stm32_pwmchan_s g_pwm17channels[] =
{
  /* TIM17 has 1 channel, 1 complementary */

#ifdef CONFIG_STM32_TIM17_CHANNEL1
  {
    .channel = 1,
    .mode    = CONFIG_STM32_TIM17_CH1MODE,
#ifdef HAVE_BREAK
    .brk =
    {
#ifdef CONFIG_STM32_TIM17_BREAK1
      .en1 = 1,
      .pol1 = CONFIG_STM32_TIM17_BRK1POL,
#endif
      /* No BREAK2 */
    },
#endif
#ifdef CONFIG_STM32_TIM17_CH1OUT
    .out1    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM17_CH1POL,
      .idle    = CONFIG_STM32_TIM17_CH1IDLE,
      .pincfg  = PWM_TIM17_CH1CFG,
    },
#endif
#ifdef CONFIG_STM32_TIM17_CH1NOUT
    .out2    =
    {
      .in_use  = 1,
      .pol     = CONFIG_STM32_TIM17_CH1NPOL,
      .idle    = CONFIG_STM32_TIM17_CH1NIDLE,
      .pincfg  = PWM_TIM17_CH2CFG,
    }
#endif
  },
#endif
};

static struct stm32_pwmtimer_s g_pwm17dev =
{
  .ops         = &g_pwmops,
#ifdef CONFIG_STM32_PWM_LL_OPS
  .llops       = &g_llpwmops,
#endif
  .timid       = 17,
  .chan_num    = PWM_TIM17_NCHANNELS,
  .channels    = g_pwm17channels,
  .timtype     = TIMTYPE_TIM17,
  .mode        = STM32_TIMMODE_COUNTUP,
  .lock        = CONFIG_STM32_TIM17_LOCK,
  .t_dts       = CONFIG_STM32_TIM17_TDTS,
#ifdef HAVE_PWM_COMPLEMENTARY
  .deadtime    = CONFIG_STM32_TIM17_DEADTIME,
#endif
#if defined(HAVE_TRGO)
  .trgo        = 0,             /* TRGO not supported for TIM17 */
#endif
#ifdef CONFIG_PWM_PULSECOUNT
  .irq         = STM32_IRQ_TIM17,
#endif
  .base        = STM32_TIM17_BASE,
  .pclk        = TIMCLK_TIM17,
};
#endif /* CONFIG_STM32_TIM17_PWM */

/* TODO: support for TIM19,20,21,22 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pwm_reg_is_32bit
 ****************************************************************************/

static bool pwm_reg_is_32bit(uint8_t timtype, uint32_t offset)
{
  bool ret = false;

  if (timtype == TIMTYPE_GENERAL32)
    {
      if (offset == STM32_GTIM_CNT_OFFSET ||
          offset == STM32_GTIM_ARR_OFFSET ||
          offset == STM32_GTIM_CCR1_OFFSET ||
          offset == STM32_GTIM_CCR2_OFFSET ||
          offset == STM32_GTIM_CCR3_OFFSET ||
          offset == STM32_GTIM_CCR4_OFFSET)
        {
          ret = true;
        }
    }
#ifdef HAVE_IP_TIMERS_V2
  else if (timtype == TIMTYPE_ADVANCED)
    {
      if (offset == STM32_ATIM_CR2_OFFSET ||
          offset == STM32_ATIM_CCMR1_OFFSET ||
          offset == STM32_ATIM_CCMR2_OFFSET ||
          offset == STM32_ATIM_CCER_OFFSET ||
          offset == STM32_ATIM_BDTR_OFFSET ||
          offset == STM32_ATIM_CCMR3_OFFSET ||
          offset == STM32_ATIM_CCR5_OFFSET)
        {
          ret = true;
        }
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: pwm_getreg
 *
 * Description:
 *   Read the value of an PWM timer register
 *
 * Input Parameters:
 *   priv   - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t pwm_getreg(struct stm32_pwmtimer_s *priv, int offset)
{
  uint32_t retval = 0;

  if (pwm_reg_is_32bit(priv->timtype, offset) == true)
    {
      /* 32-bit register */

      retval = getreg32(priv->base + offset);
    }
  else
    {
      /* 16-bit register */

      retval = getreg16(priv->base + offset);
    }

  /* Return 32-bit value */

  return retval;
}

/****************************************************************************
 * Name: pwm_putreg
 *
 * Description:
 *   Read the value of an PWM timer register
 *
 * Input Parameters:
 *   priv   - A reference to the PWM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_putreg(struct stm32_pwmtimer_s *priv, int offset,
                       uint32_t value)
{
  if (pwm_reg_is_32bit(priv->timtype, offset) == true)
    {
      /* 32-bit register */

      putreg32(value, priv->base + offset);
    }
  else
    {
      /* 16-bit register */

      putreg16((uint16_t)value, priv->base + offset);
    }
}

/****************************************************************************
 * Name: pwm_modifyreg
 *
 * Description:
 *   Modify PWM register (32-bit or 16-bit)
 *
 * Input Parameters:
 *   priv    - A reference to the PWM block status
 *   offset  - The offset to the register to read
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void pwm_modifyreg(struct stm32_pwmtimer_s *priv, uint32_t offset,
                            uint32_t clearbits, uint32_t setbits)
{
  if (pwm_reg_is_32bit(priv->timtype, offset) == true)
    {
      /* 32-bit register */

      modifyreg32(priv->base + offset, clearbits, setbits);
    }
  else
    {
      /* 16-bit register */

      modifyreg16(priv->base + offset, (uint16_t)clearbits,
                  (uint16_t)setbits);
    }
}

/****************************************************************************
 * Name: pwm_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_PWM_INFO
static void pwm_dumpregs(struct pwm_lowerhalf_s *dev, const char *msg)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  pwminfo("%s:\n", msg);
  if (priv->timid == 16 || priv->timid == 17)
    {
      pwminfo("  CR1: %04x CR2:  %04x DIER:  %04x\n",
              pwm_getreg(priv, STM32_GTIM_CR1_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CR2_OFFSET),
              pwm_getreg(priv, STM32_GTIM_DIER_OFFSET));
    }
  else
    {
      pwminfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
              pwm_getreg(priv, STM32_GTIM_CR1_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CR2_OFFSET),
              pwm_getreg(priv, STM32_GTIM_SMCR_OFFSET),
              pwm_getreg(priv, STM32_GTIM_DIER_OFFSET));
    }

  if (priv->timid >= 15 && priv->timid <= 17)
    {
      pwminfo("   SR: %04x EGR:  %04x CCMR1: %04x\n",
              pwm_getreg(priv, STM32_GTIM_SR_OFFSET),
              pwm_getreg(priv, STM32_GTIM_EGR_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCMR1_OFFSET));
    }
  else
    {
      pwminfo("   SR: %04x EGR:  %04x CCMR1: %04x CCMR2: %04x\n",
              pwm_getreg(priv, STM32_GTIM_SR_OFFSET),
              pwm_getreg(priv, STM32_GTIM_EGR_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCMR1_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCMR2_OFFSET));
    }

  /* REVISIT: CNT and ARR may be 32-bits wide */

  pwminfo(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
          pwm_getreg(priv, STM32_GTIM_CCER_OFFSET),
          pwm_getreg(priv, STM32_GTIM_CNT_OFFSET),
          pwm_getreg(priv, STM32_GTIM_PSC_OFFSET),
          pwm_getreg(priv, STM32_GTIM_ARR_OFFSET));

  if (priv->timid == 1 || priv->timid == 8 ||
      (priv->timid >= 15 && priv->timid <= 17))
    {
      pwminfo("  RCR: %04x BDTR: %04x\n",
          pwm_getreg(priv, STM32_ATIM_RCR_OFFSET),
          pwm_getreg(priv, STM32_ATIM_BDTR_OFFSET));
    }

  /* REVISIT: CCR1-CCR4 may be 32-bits wide */

  if (priv->timid == 16 || priv->timid == 17)
    {
      pwminfo(" CCR1: %04x\n",
              pwm_getreg(priv, STM32_GTIM_CCR1_OFFSET));
    }
  else if (priv->timid == 15)
    {
      pwminfo(" CCR1: %04x CCR2: %04x\n",
              pwm_getreg(priv, STM32_GTIM_CCR1_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCR2_OFFSET));
    }
  else
    {
      pwminfo(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
              pwm_getreg(priv, STM32_GTIM_CCR1_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCR2_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCR3_OFFSET),
              pwm_getreg(priv, STM32_GTIM_CCR4_OFFSET));
    }

  pwminfo("  DCR: %04x DMAR: %04x\n",
      pwm_getreg(priv, STM32_GTIM_DCR_OFFSET),
      pwm_getreg(priv, STM32_GTIM_DMAR_OFFSET));

#ifdef HAVE_IP_TIMERS_V2
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      pwminfo("  CCMR3: %04x CCR5: %04x CCR6: %04x\n",
              pwm_getreg(priv, STM32_ATIM_CCMR3_OFFSET),
              pwm_getreg(priv, STM32_ATIM_CCR5_OFFSET),
              pwm_getreg(priv, STM32_ATIM_CCR6_OFFSET));
    }
#endif
}
#endif

/****************************************************************************
 * Name: pwm_ccr_update
 ****************************************************************************/

static int pwm_ccr_update(struct pwm_lowerhalf_s *dev, uint8_t index,
                          uint32_t ccr)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t offset = 0;

  /* Only ADV timers have CC5 and CC6 */

#ifdef HAVE_IP_TIMERS_V2
  if (priv->timtype != TIMTYPE_ADVANCED && (index == 5 || index == 6))
    {
      pwmerr("ERROR: No such CCR: %u\n", index);
      return -EINVAL;
    }
#endif

  /* REVISIT: start index from 0? */

  switch (index)
    {
      case STM32_PWM_CHAN1:
        {
          offset = STM32_GTIM_CCR1_OFFSET;
          break;
        }

      case STM32_PWM_CHAN2:
        {
          offset = STM32_GTIM_CCR2_OFFSET;
          break;
        }

      case STM32_PWM_CHAN3:
        {
          offset = STM32_GTIM_CCR3_OFFSET;
          break;
        }

      case STM32_PWM_CHAN4:
        {
          offset = STM32_GTIM_CCR4_OFFSET;
          break;
        }

#ifdef HAVE_IP_TIMERS_V2
      case STM32_PWM_CHAN5:
        {
          offset = STM32_ATIM_CCR5_OFFSET;
          break;
        }

      case STM32_PWM_CHAN6:
        {
          offset = STM32_ATIM_CCR6_OFFSET;
          break;
        }
#endif

      default:
        {
          pwmerr("ERROR: No such CCR: %u\n", index);
          return -EINVAL;
        }
    }

  /* Update CCR register */

  pwm_putreg(priv, offset, ccr);

  return OK;
}

/****************************************************************************
 * Name: pwm_ccr_get
 ****************************************************************************/

#ifdef CONFIG_STM32_PWM_LL_OPS
static uint32_t pwm_ccr_get(struct pwm_lowerhalf_s *dev, uint8_t index)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t offset = 0;

  switch (index)
    {
      case STM32_PWM_CHAN1:
        {
          offset = STM32_GTIM_CCR1_OFFSET;
          break;
        }

      case STM32_PWM_CHAN2:
        {
          offset = STM32_GTIM_CCR2_OFFSET;
          break;
        }

      case STM32_PWM_CHAN3:
        {
          offset = STM32_GTIM_CCR3_OFFSET;
          break;
        }

      case STM32_PWM_CHAN4:
        {
          offset = STM32_GTIM_CCR4_OFFSET;
          break;
        }

#ifdef HAVE_IP_TIMERS_V2
      case STM32_PWM_CHAN5:
        {
          offset = STM32_ATIM_CCR5_OFFSET;
          break;
        }

      case STM32_PWM_CHAN6:
        {
          offset = STM32_ATIM_CCR6_OFFSET;
          break;
        }
#endif

      default:
        {
          pwmerr("ERROR: No such CCR: %u\n", index);
          return -EINVAL;
        }
    }

  /* Return CCR register */

  return pwm_getreg(priv, offset);
}
#endif /* CONFIG_STM32_PWM_LL_OPS */

/****************************************************************************
 * Name: pwm_arr_update
 ****************************************************************************/

static int pwm_arr_update(struct pwm_lowerhalf_s *dev, uint32_t arr)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  /* Update ARR register */

  pwm_putreg(priv, STM32_GTIM_ARR_OFFSET, arr);

  return OK;
}

/****************************************************************************
 * Name: pwm_arr_get
 ****************************************************************************/

static uint32_t pwm_arr_get(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  return pwm_getreg(priv, STM32_GTIM_ARR_OFFSET);
}

#ifdef HAVE_ADVTIM
/****************************************************************************
 * Name: pwm_rcr_update
 ****************************************************************************/

static int pwm_rcr_update(struct pwm_lowerhalf_s *dev, uint16_t rcr)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  /* Update RCR register */

  pwm_putreg(priv, STM32_ATIM_RCR_OFFSET, rcr);

  return OK;
}
#endif

#ifdef CONFIG_STM32_PWM_LL_OPS
/****************************************************************************
 * Name: pwm_rcr_get
 ****************************************************************************/

static uint16_t pwm_rcr_get(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  return pwm_getreg(priv, STM32_ATIM_RCR_OFFSET);
}
#endif

/****************************************************************************
 * Name: pwm_duty_update
 *
 * Description:
 *   Try to change only channel duty
 *
 * Input Parameters:
 *   dev     - A reference to the lower half PWM driver state structure
 *   channel - Channel to by updated
 *   duty    - New duty
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_duty_update(struct pwm_lowerhalf_s *dev, uint8_t channel,
                           ub16_t duty)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t reload = 0;
  uint32_t ccr    = 0;

  /* We don't want compilation warnings if no DEBUGASSERT */

  UNUSED(priv);

  DEBUGASSERT(priv != NULL);

  pwminfo("TIM%u channel: %u duty: %08" PRIx32 "\n",
          priv->timid, channel, duty);

#ifndef CONFIG_PWM_MULTICHAN
  DEBUGASSERT(channel == priv->channels[0].channel);
  DEBUGASSERT(duty >= 0 && duty < uitoub16(100));
#endif

  /* Get the reload values */

  reload = pwm_arr_get(dev);

  /* Duty cycle:
   *
   * duty cycle = ccr / reload (fractional value)
   */

  ccr = b16toi(duty * reload + b16HALF);

  pwminfo("ccr: %" PRIu32 "\n", ccr);

  /* Write corresponding CCR register */

  pwm_ccr_update(dev, channel, ccr);

  return OK;
}

/****************************************************************************
 * Name: pwm_timer_enable
 ****************************************************************************/

static int pwm_timer_enable(struct pwm_lowerhalf_s *dev, bool state)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  if (state == true)
    {
      /* Enable timer counter */

      pwm_modifyreg(priv, STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);
    }
  else
    {
      /* Disable timer counter */

      pwm_modifyreg(priv, STM32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_frequency_update
 *
 * Description:
 *   Update a PWM timer frequency
 *
 ****************************************************************************/

static int pwm_frequency_update(struct pwm_lowerhalf_s *dev,
                                uint32_t frequency)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t reload    = 0;
  uint32_t timclk    = 0;
  uint32_t prescaler = 0;

  /* Calculate optimal values for the timer prescaler and for the timer
   * reload register. If 'frequency' is the desired frequency, then
   *
   *   reload = timclk / frequency
   *   timclk = pclk / presc
   *
   * Or,
   *
   *   reload = pclk / presc / frequency
   *
   * There are many solutions to this, but the best solution will be the one
   * that has the largest reload value and the smallest prescaler value.
   * That is the solution that should give us the most accuracy in the timer
   * control.  Subject to:
   *
   *   0 <= presc  <= 65536
   *   1 <= reload <= 65535
   *
   * So presc = pclk / 65535 / frequency would be optimal.
   *
   * Example:
   *
   *  pclk      = 42 MHz
   *  frequency = 100 Hz
   *
   *  prescaler = 42,000,000 / 65,535 / 100
   *            = 6.4 (or 7 -- taking the ceiling always)
   *  timclk    = 42,000,000 / 7
   *            = 6,000,000
   *  reload    = 6,000,000 / 100
   *            = 60,000
   */

  prescaler = (priv->pclk / frequency + 65534) / 65535;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > 65536)
    {
      prescaler = 65536;
    }

  timclk = priv->pclk / prescaler;

  reload = timclk / frequency;
  if (reload < 2)
    {
      reload = 1;
    }
  else if (reload > 65535)
    {
      reload = 65535;
    }
  else
    {
      reload--;
    }

  pwminfo("TIM%u PCLK: %" PRIu32" frequency: %" PRIu32
          " TIMCLK: %" PRIu32 " "
          "prescaler: %" PRIu32 " reload: %" PRIu32 "\n",
          priv->timid, priv->pclk, frequency, timclk, prescaler, reload);

  /* Set the reload and prescaler values */

  pwm_arr_update(dev, reload);
  pwm_putreg(priv, STM32_GTIM_PSC_OFFSET, (uint16_t)(prescaler - 1));

  return OK;
}

/****************************************************************************
 * Name: pwm_timer_configure
 *
 * Description:
 *   Initial configuration for PWM timer
 *
 ****************************************************************************/

static int pwm_timer_configure(struct stm32_pwmtimer_s *priv)
{
  uint16_t cr1 = 0;
  int      ret = OK;

  /* Set up the timer CR1 register:
   *
   * 1,8   CKD[1:0] ARPE CMS[1:0] DIR OPM URS UDIS CEN
   * 2-5   CKD[1:0] ARPE CMS      DIR OPM URS UDIS CEN
   * 6-7            ARPE              OPM URS UDIS CEN
   * 9-14  CKD[1:0] ARPE                  URS UDIS CEN
   * 15-17 CKD[1:0] ARPE              OPM URS UDIS CEN
   */

  cr1 = pwm_getreg(priv, STM32_GTIM_CR1_OFFSET);

  /* Set the counter mode for the advanced timers (1,8) and most general
   * purpose timers (all 2-5, but not 9-17), i.e., all but TIMTYPE_COUNTUP16
   * and TIMTYPE_BASIC
   */

  if (priv->timtype != TIMTYPE_BASIC && priv->timtype != TIMTYPE_COUNTUP16)
    {
      /* Select the Counter Mode:
       *
       * GTIM_CR1_EDGE: The counter counts up or down depending on the
       *   direction bit (DIR).
       * GTIM_CR1_CENTER1, GTIM_CR1_CENTER2, GTIM_CR1_CENTER3: The counter
       *   counts up then down.
       * GTIM_CR1_DIR: 0: count up, 1: count down
       */

      cr1 &= ~(GTIM_CR1_DIR | GTIM_CR1_CMS_MASK);

      switch (priv->mode)
        {
          case STM32_TIMMODE_COUNTUP:
            {
              cr1 |= GTIM_CR1_EDGE;
              break;
            }

          case STM32_TIMMODE_COUNTDOWN:
            {
              cr1 |= GTIM_CR1_EDGE | GTIM_CR1_DIR;
              break;
            }

          case STM32_TIMMODE_CENTER1:
            {
              cr1 |= GTIM_CR1_CENTER1;
              break;
            }

          case STM32_TIMMODE_CENTER2:
            {
              cr1 |= GTIM_CR1_CENTER2;
              break;
            }

          case STM32_TIMMODE_CENTER3:
            {
              cr1 |= GTIM_CR1_CENTER3;
              break;
            }

          default:
            {
              pwmerr("ERROR: No such timer mode: %u\n",
                     (unsigned int)priv->mode);
              ret = -EINVAL;
              goto errout;
            }
        }
    }

  /* Enable ARR Preload
   * TODO: this should be configurable
   */

  cr1 |= GTIM_CR1_ARPE;

  /* Write CR1 */

  pwm_putreg(priv, STM32_GTIM_CR1_OFFSET, cr1);

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_mode_configure
 *
 * Description:
 *   Configure a PWM mode for given channel
 *
 ****************************************************************************/

static int pwm_mode_configure(struct pwm_lowerhalf_s *dev,
                              uint8_t channel, uint32_t mode)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t chanmode = 0;
  uint32_t ocmode   = 0;
  uint32_t ccmr     = 0;
  uint32_t offset   = 0;
  int      ret      = OK;
#ifdef HAVE_IP_TIMERS_V2
  bool     ocmbit   = false;
#endif

#ifdef HAVE_IP_TIMERS_V2
  /* Only advanced timers have channels 5-6 */

  if (channel > 4 && priv->timtype != TIMTYPE_ADVANCED)
    {
      pwmerr("ERROR: No such channel: %u\n", channel);
      ret = -EINVAL;
      goto errout;
    }
#endif

  /* Get channel mode
   * TODO: configurable preload for CCxR
   */

  switch (mode)
    {
      case STM32_CHANMODE_FRZN:
        {
          chanmode = GTIM_CCMR_MODE_FRZN;
          break;
        }

      case STM32_CHANMODE_CHACT:
        {
          chanmode = GTIM_CCMR_MODE_CHACT;
          break;
        }

      case STM32_CHANMODE_CHINACT:
        {
          chanmode = GTIM_CCMR_MODE_CHINACT;
          break;
        }

      case STM32_CHANMODE_OCREFTOG:
        {
          chanmode = GTIM_CCMR_MODE_OCREFTOG;
          break;
        }

      case STM32_CHANMODE_OCREFLO:
        {
          chanmode = GTIM_CCMR_MODE_OCREFLO;
          break;
        }

      case STM32_CHANMODE_OCREFHI:
        {
          chanmode = GTIM_CCMR_MODE_OCREFHI;
          break;
        }

      case STM32_CHANMODE_PWM1:
        {
          chanmode = GTIM_CCMR_MODE_PWM1;
          break;
        }

      case STM32_CHANMODE_PWM2:
        {
          chanmode = GTIM_CCMR_MODE_PWM2;
          break;
        }

#ifdef HAVE_IP_TIMERS_V2
      case STM32_CHANMODE_COMBINED1:
        {
          chanmode = ATIM_CCMR_MODE_COMBINED1;
          ocmbit   = true;
          break;
        }

      case STM32_CHANMODE_COMBINED2:
        {
          chanmode = ATIM_CCMR_MODE_COMBINED2;
          ocmbit   = true;
          break;
        }

      case STM32_CHANMODE_ASYMMETRIC1:
        {
          chanmode = ATIM_CCMR_MODE_ASYMMETRIC1;
          ocmbit   = true;
          break;
        }

      case STM32_CHANMODE_ASYMMETRIC2:
        {
          chanmode = ATIM_CCMR_MODE_ASYMMETRIC2;
          ocmbit   = true;
          break;
        }
#endif

      default:
        {
          pwmerr("ERROR: No such mode: %u\n", (unsigned int)mode);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Get CCMR offset */

  switch (channel)
    {
      case STM32_PWM_CHAN1:
      case STM32_PWM_CHAN2:
        {
          offset = STM32_GTIM_CCMR1_OFFSET;
          break;
        }

      case STM32_PWM_CHAN3:
      case STM32_PWM_CHAN4:
        {
          offset = STM32_GTIM_CCMR2_OFFSET;
          break;
        }

#ifdef HAVE_IP_TIMERS_V2
      case STM32_PWM_CHAN5:
      case STM32_PWM_CHAN6:
        {
          offset = STM32_ATIM_CCMR3_OFFSET;
          break;
        }
#endif

      default:
        {
          pwmerr("ERROR: No such channel: %u\n", channel);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Get current registers */

  ccmr = pwm_getreg(priv, offset);

  /* PWM mode configuration.
   * NOTE: The CCMRx registers are identical if the channels are outputs.
   */

  switch (channel)
    {
      /* Configure channel 1/3/5 */

      case STM32_PWM_CHAN1:
      case STM32_PWM_CHAN3:
#ifdef HAVE_IP_TIMERS_V2
      case STM32_PWM_CHAN5:
#endif
        {
          /* Reset current channel 1/3/5 mode configuration */

          ccmr &= ~(ATIM_CCMR1_CC1S_MASK | ATIM_CCMR1_OC1M_MASK |
                     ATIM_CCMR1_OC1PE);

          /* Configure CC1/3/5 as output */

          ocmode |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR1_CC1S_SHIFT);

          /* Configure Compare 1/3/5 mode */

          ocmode |= (chanmode << ATIM_CCMR1_OC1M_SHIFT);

          /* Enable CCR1/3/5 preload */

          ocmode |= ATIM_CCMR1_OC1PE;

#ifdef HAVE_IP_TIMERS_V2
          /* Reset current OC bit */

          ccmr &= ~(ATIM_CCMR1_OC1M);

          /* Set an additional OC1/3/5M bit */

          if (ocmbit)
            {
              ocmode |= ATIM_CCMR1_OC1M;
            }
#endif
          break;
        }

      /* Configure channel 2/4/6 */

      case STM32_PWM_CHAN2:
      case STM32_PWM_CHAN4:
#ifdef HAVE_IP_TIMERS_V2
      case STM32_PWM_CHAN6:
#endif
        {
          /* Reset current channel 2/4/6 mode configuration */

          ccmr &= ~(ATIM_CCMR1_CC2S_MASK | ATIM_CCMR1_OC2M_MASK |
                     ATIM_CCMR1_OC2PE);

          /* Configure CC2/4/6 as output */

          ocmode |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR1_CC2S_SHIFT);

          /* Configure Compare 2/4/6 mode */

          ocmode |= (chanmode << ATIM_CCMR1_OC2M_SHIFT);

          /* Enable CCR2/4/6 preload */

          ocmode |= ATIM_CCMR1_OC2PE;

#ifdef HAVE_IP_TIMERS_V2
          /* Reset current OC bit */

          ccmr &= ~(ATIM_CCMR1_OC2M);

          /* Set an additioneal OC2/4/6M bit */

          if (ocmbit)
            {
              ocmode |= ATIM_CCMR1_OC2M;
            }
#endif
          break;
        }
    }

  /* Set the selected output compare mode */

  ccmr |= ocmode;

  /* Write CCMRx registers */

  pwm_putreg(priv, offset, ccmr);

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_output_configure
 *
 * Description:
 *   Configure PWM output for given channel
 *
 ****************************************************************************/

static int pwm_output_configure(struct stm32_pwmtimer_s *priv,
                                struct stm32_pwmchan_s *chan)
{
  uint32_t cr2  = 0;
  uint32_t ccer = 0;
  uint8_t  channel = 0;

  /* Get channel */

  channel = chan->channel;

  /* Get current registers state */

  cr2  = pwm_getreg(priv, STM32_GTIM_CR2_OFFSET);
  ccer = pwm_getreg(priv, STM32_GTIM_CCER_OFFSET);

  /* | OISx/OISxN  | IDLE | for ADVANCED and COUNTUP16 | CR2 register
   * | CCxP/CCxNP  | POL  | all PWM timers             | CCER register
   */

  /* Configure output polarity (all PWM timers) */

  if (chan->out1.pol == STM32_POL_NEG)
    {
      ccer |= (GTIM_CCER_CC1P << ((channel - 1) * 4));
    }
  else
    {
      ccer &= ~(GTIM_CCER_CC1P << ((channel - 1) * 4));
    }

#ifdef HAVE_ADVTIM
  if (priv->timtype == TIMTYPE_ADVANCED ||
      priv->timtype == TIMTYPE_COUNTUP16_N)
    {
      /* Configure output IDLE State */

      if (chan->out1.idle == STM32_IDLE_ACTIVE)
        {
          cr2 |= (ATIM_CR2_OIS1 << ((channel - 1) * 2));
        }
      else
        {
          cr2 &= ~(ATIM_CR2_OIS1 << ((channel - 1) * 2));
        }

#ifdef HAVE_PWM_COMPLEMENTARY
      /* Configure complementary output IDLE state */

      if (chan->out2.idle == STM32_IDLE_ACTIVE)
        {
          cr2 |= (ATIM_CR2_OIS1N << ((channel - 1) * 2));
        }
      else
        {
          cr2 &= ~(ATIM_CR2_OIS1N << ((channel - 1) * 2));
        }

      /* Configure complementary output polarity */

      if (chan->out2.pol == STM32_POL_NEG)
        {
          ccer |= (ATIM_CCER_CC1NP << ((channel - 1) * 4));
        }
      else
        {
          ccer &= ~(ATIM_CCER_CC1NP << ((channel - 1) * 4));
        }
#endif /* HAVE_PWM_COMPLEMENTARY */

#ifdef HAVE_IP_TIMERS_V2
      /* TODO: OIS5 and OIS6 */

      cr2 &= ~(ATIM_CR2_OIS5 | ATIM_CR2_OIS6);

      /* TODO: CC5P and CC6P */

      ccer &= ~(ATIM_CCER_CC5P | ATIM_CCER_CC6P);
#endif /* HAVE_IP_TIMERS_V2 */
    }
#ifdef HAVE_GTIM_CCXNP
  else
#endif /* HAVE_GTIM_CCXNP */
#endif /* HAVE_ADVTIM */
#ifdef HAVE_GTIM_CCXNP
    {
      /* CCxNP must be cleared if not ADVANCED timer.
       *
       * REVISIT: not all families have CCxNP bits for GTIM,
       *          which causes an ugly condition above
       */

      ccer &= ~(GTIM_CCER_CC1NP << ((channel - 1) * 4));
    }
#endif /* HAVE_GTIM_CCXNP */

  /* Write registers */

  pwm_modifyreg(priv, STM32_GTIM_CR2_OFFSET, 0, cr2);
  pwm_modifyreg(priv, STM32_GTIM_CCER_OFFSET, 0, ccer);

  return OK;
}

/****************************************************************************
 * Name: pwm_outputs_enable
 *
 * Description:
 *   Enable/disable given timer PWM outputs.
 *
 *   NOTE: This is bulk operation - we can enable/disable many outputs
 *   at one time
 *
 * Input Parameters:
 *   dev     - A reference to the lower half PWM driver state structure
 *   outputs - outputs to set (look at enum stm32_chan_e in stm32_pwm.h)
 *   state   - Enable/disable operation
 *
 ****************************************************************************/

static int pwm_outputs_enable(struct pwm_lowerhalf_s *dev,
                              uint16_t outputs, bool state)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t ccer   = 0;
  uint32_t regval = 0;

  /* Get curren register state */

  ccer = pwm_getreg(priv, STM32_GTIM_CCER_OFFSET);

  /* Get outputs configuration */

  regval |= ((outputs & STM32_PWM_OUT1)  ? GTIM_CCER_CC1E  : 0);
  regval |= ((outputs & STM32_PWM_OUT1N) ? ATIM_CCER_CC1NE : 0);
  regval |= ((outputs & STM32_PWM_OUT2)  ? GTIM_CCER_CC2E  : 0);
  regval |= ((outputs & STM32_PWM_OUT2N) ? ATIM_CCER_CC2NE : 0);
  regval |= ((outputs & STM32_PWM_OUT3)  ? GTIM_CCER_CC3E  : 0);
  regval |= ((outputs & STM32_PWM_OUT3N) ? ATIM_CCER_CC3NE : 0);
  regval |= ((outputs & STM32_PWM_OUT4)  ? GTIM_CCER_CC4E  : 0);

  /* NOTE: CC4N doesn't exist, but some docs show configuration bits for it */

#ifdef HAVE_IP_TIMERS_V2
  regval |= ((outputs & STM32_PWM_OUT5)  ? ATIM_CCER_CC5E  : 0);
  regval |= ((outputs & STM32_PWM_OUT6)  ? ATIM_CCER_CC6E  : 0);
#endif

  if (state == true)
    {
      /* Enable outpus - set bits */

      ccer |= regval;
    }
  else
    {
      /* Disable outputs - reset bits */

      ccer &= ~regval;
    }

  /* Write register */

  pwm_putreg(priv, STM32_GTIM_CCER_OFFSET, ccer);

  return OK;
}

#if defined(HAVE_PWM_COMPLEMENTARY) && defined(CONFIG_STM32_PWM_LL_OPS)

/****************************************************************************
 * Name: pwm_deadtime_update
 ****************************************************************************/

static int pwm_deadtime_update(struct pwm_lowerhalf_s *dev, uint8_t dt)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t bdtr = 0;
  int      ret  = OK;

  /* Check if locked */

  if (priv->lock > 0)
    {
      ret = -EACCES;
      goto errout;
    }

  /* Get current register state */

  bdtr = pwm_getreg(priv, STM32_ATIM_BDTR_OFFSET);

  /* TODO: check if BDTR not locked */

  /* Update deadtime */

  bdtr &= ~(ATIM_BDTR_DTG_MASK);
  bdtr |= (dt << ATIM_BDTR_DTG_SHIFT);

  /* Write BDTR register */

  pwm_putreg(priv, STM32_ATIM_BDTR_OFFSET, bdtr);

errout:
  return ret;
}
#endif

#ifdef HAVE_TRGO
/****************************************************************************
 * Name: pwm_trgo_configure
 *
 * Description:
 *   Confiugre an output synchronisation event for PWM timer (TRGO/TRGO2)
 *
 ****************************************************************************/

static int pwm_trgo_configure(struct pwm_lowerhalf_s *dev,
                              uint8_t trgo)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t cr2 = 0;

  /* Configure TRGO (4 LSB in trgo) */

  cr2 |= (((trgo >> 0) & 0x0f) << ATIM_CR2_MMS_SHIFT) & ATIM_CR2_MMS_MASK;

#ifdef HAVE_IP_TIMERS_V2
  /* Configure TRGO2 (4 MSB in trgo) */

  cr2 |= (((trgo >> 4) & 0x0f) << ATIM_CR2_MMS2_SHIFT) & ATIM_CR2_MMS2_MASK;
#endif

  /* Write register */

  pwm_modifyreg(priv, STM32_GTIM_CR2_OFFSET, 0, cr2);

  return OK;
}
#endif

/****************************************************************************
 * Name: pwm_soft_update
 *
 * Description:
 *   Generate an software update event
 *
 ****************************************************************************/

static int pwm_soft_update(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  pwm_putreg(priv, STM32_GTIM_EGR_OFFSET, GTIM_EGR_UG);

  return OK;
}

/****************************************************************************
 * Name: pwm_soft_break
 *
 * Description:
 *   Generate an software break event
 *
 *   Outputs are enabled if state is false.
 *   Outputs are disabled if state is true.
 *
 *   NOTE: only timers with complementary outputs have BDTR register and
 *         support software break.
 *
 ****************************************************************************/

static int pwm_soft_break(struct pwm_lowerhalf_s *dev, bool state)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  if (state == true)
    {
      /* Reset MOE bit */

      pwm_modifyreg(priv, STM32_ATIM_BDTR_OFFSET, ATIM_BDTR_MOE, 0);
    }
  else
    {
      /* Set MOE bit */

      pwm_modifyreg(priv, STM32_ATIM_BDTR_OFFSET, 0, ATIM_BDTR_MOE);
    }

  return OK;
}

/****************************************************************************
 * Name: pwm_outputs_from_channels
 *
 * Description:
 *   Get enabled outputs configuration from the PWM timer state
 *
 ****************************************************************************/

static uint16_t pwm_outputs_from_channels(struct stm32_pwmtimer_s *priv)
{
  uint16_t outputs = 0;
  uint8_t  channel = 0;
  uint8_t  i       = 0;

  for (i = 0; i < priv->chan_num; i += 1)
    {
      /* Get channel */

      channel = priv->channels[i].channel;

      /* Set outputs if channel configured */

      if (channel != 0)
        {
          /* Enable output if confiugred */

          if (priv->channels[i].out1.in_use == 1)
            {
              outputs |= (STM32_PWM_OUT1 << ((channel - 1) * 2));
            }

#ifdef HAVE_PWM_COMPLEMENTARY
          /* Enable complementary output if configured */

          if (priv->channels[i].out2.in_use == 1)
            {
              outputs |= (STM32_PWM_OUT1N << ((channel - 1) * 2));
            }
#endif
        }
    }

  return outputs;
}

#ifdef HAVE_ADVTIM

/****************************************************************************
 * Name: pwm_break_dt_configure
 *
 * Description:
 *   Configure break and deadtime
 *
 * NOTE: we have to configure all BDTR registers at once due to possible
 *       lock configuration
 *
 ****************************************************************************/

static int pwm_break_dt_configure(struct stm32_pwmtimer_s *priv)
{
  uint32_t bdtr = 0;

  /* Set the clock division to zero for all (but the basic timers, but there
   * should be no basic timers in this context
   */

  pwm_modifyreg(priv, STM32_GTIM_CR1_OFFSET, GTIM_CR1_CKD_MASK,
                priv->t_dts << GTIM_CR1_CKD_SHIFT);

#ifdef HAVE_PWM_COMPLEMENTARY
  /* Initialize deadtime */

  bdtr |= (priv->deadtime << ATIM_BDTR_DTG_SHIFT);
#endif

#ifdef HAVE_BREAK
  /* Configure Break 1 */

  if (priv->brk.en1 == 1)
    {
      /* Enable Break 1 */

      bdtr |= ATIM_BDTR_BKE;

      /* Set Break 1 polarity */

      bdtr |= (priv->brk.pol1 == STM32_POL_NEG ? ATIM_BDTR_BKP : 0);
    }

#ifdef HAVE_IP_TIMERS_V2
  /* Configure Break 1 */

  if (priv->brk.en2 == 1)
    {
      /* Enable Break 2 */

      bdtr |= ATIM_BDTR_BK2E;

      /* Set Break 2 polarity */

      bdtr |= (priv->brk.pol2 == STM32_POL_NEG ? ATIM_BDTR_BK2P : 0);

      /* Configure BRK2 filter */

      bdtr |= (priv->brk.flt2 << ATIM_BDTR_BK2F_SHIFT);
    }
#endif /* HAVE_IP_TIMERS_V2 */
#endif /* HAVE_BREAK */

  /* Clear the OSSI and OSSR bits in the BDTR register.
   *
   * REVISIT: this should be configurable
   */

  bdtr &= ~(ATIM_BDTR_OSSI | ATIM_BDTR_OSSR);

  /* Configure lock */

  bdtr |= priv->lock << ATIM_BDTR_LOCK_SHIFT;

  /* Write BDTR register at once */

  pwm_putreg(priv, STM32_ATIM_BDTR_OFFSET, bdtr);

  return OK;
}
#endif

#ifdef CONFIG_PWM_PULSECOUNT

/****************************************************************************
 * Name: pwm_pulsecount_configure
 *
 * Description:
 *   Configure PWM timer in PULSECOUNT mode
 *
 ****************************************************************************/

static int pwm_pulsecount_configure(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint16_t outputs = 0;
  uint8_t j        = 0;
  int     ret      = OK;

  /* NOTE: leave timer counter disabled and all outputs disabled! */

  /* Disable the timer until we get it configured */

  pwm_timer_enable(dev, false);

  /* Get configured outputs */

  outputs = pwm_outputs_from_channels(priv);

  /* REVISIT: Disable outputs */

  ret = pwm_outputs_enable(dev, outputs, false);
  if (ret < 0)
    {
      goto errout;
    }

  /* Initial timer configuration */

  ret = pwm_timer_configure(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure break and deadtime register */

  ret = pwm_break_dt_configure(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Disable software break (enable outputs) */

  ret = pwm_soft_break(dev, false);
  if (ret < 0)
    {
      goto errout;
    }

#ifdef HAVE_TRGO
  /* Configure TRGO/TRGO2 */

  ret = pwm_trgo_configure(dev, priv->trgo);
  if (ret < 0)
    {
      goto errout;
    }
#endif

  /* Configure timer channels */

  for (j = 0; j < priv->chan_num; j++)
    {
      /* Skip channel if not in use */

      if (priv->channels[j].channel != 0)
        {
          /* Update PWM mode */

          pwm_mode_configure(dev, priv->channels[j].channel,
                             priv->channels[j].mode);

          /* PWM outputs configuration */

          pwm_output_configure(priv, &priv->channels[j]);
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_pulsecount_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * TODO: PWM_PULSECOUNT should be configurable for each timer instance
 * TODO: PULSECOUNT doesn't work with MULTICHAN at this moment
 *
 ****************************************************************************/

static int pwm_pulsecount_timer(struct pwm_lowerhalf_s *dev,
                                const struct pwm_info_s *info)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  ub16_t    duty    = 0;
  uint8_t   channel = 0;
  uint16_t  outputs = 0;
  int       ret     = OK;

  /* If we got here it means that timer instance support pulsecount mode! */

  DEBUGASSERT(priv != NULL && info != NULL);

  pwminfo("TIM%u channel: %u frequency: %" PRIx32 " duty: %08" PRIx32
          " count: %" PRIx32 "\n",
          priv->timid, priv->channels[0].channel, info->frequency,
          info->duty, info->count);

  DEBUGASSERT(info->frequency > 0);

  /* Channel specific setup */

  duty    = info->duty;
  channel = priv->channels[0].channel;

  /* Disable all interrupts and DMA requests, clear all pending status */

  pwm_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  pwm_putreg(priv, STM32_GTIM_SR_OFFSET, 0);

  /* Set timer frequency */

  ret = pwm_frequency_update(dev, info->frequency);
  if (ret < 0)
    {
      goto errout;
    }

  /* Update duty cycle */

  ret = pwm_duty_update(dev, channel, duty);
  if (ret < 0)
    {
      goto errout;
    }

  /* If a non-zero repetition count has been selected, then set the
   * repetition counter to the count-1 (pwm_pulsecount_start() has already
   * assured us that the count value is within range).
   */

  if (info->count > 0)
    {
      /* Save the remaining count and the number of counts that will have
       * elapsed on the first interrupt.
       */

      /* If the first interrupt occurs at the end end of the first
       * repetition count, then the count will be the same as the RCR
       * value.
       */

      priv->prev  = pwm_pulsecount(info->count);
      pwm_rcr_update(dev, priv->prev - 1);

      /* Generate an update event to reload the prescaler.  This should
       * preload the RCR into active repetition counter.
       */

      pwm_soft_update(dev);

      /* Now set the value of the RCR that will be loaded on the next
       * update event.
       */

      priv->count = info->count;
      priv->curr  = pwm_pulsecount(info->count - priv->prev);
      pwm_rcr_update(dev, priv->curr - 1);
    }

  /* Otherwise, just clear the repetition counter */

  else
    {
      /* Set the repetition counter to zero */

      pwm_rcr_update(dev, 0);

      /* Generate an update event to reload the prescaler */

      pwm_soft_update(dev);
    }

  /* Get configured outputs */

  outputs = pwm_outputs_from_channels(priv);

  /* Enable output */

  ret = pwm_outputs_enable(dev, outputs, true);
  if (ret < 0)
    {
      goto errout;
    }

  /* Setup update interrupt.  If info->count is > 0, then we can be
   * assured that pwm_pulsecount_start() has already verified: (1) that this
   * is an advanced timer, and that (2) the repetition count is within range.
   */

  if (info->count > 0)
    {
      /* Clear all pending interrupts and enable the update interrupt. */

      pwm_putreg(priv, STM32_GTIM_SR_OFFSET, 0);
      pwm_putreg(priv, STM32_GTIM_DIER_OFFSET, GTIM_DIER_UIE);

      /* Enable the timer */

      pwm_timer_enable(dev, true);

      /* And enable timer interrupts at the NVIC */

      up_enable_irq(priv->irq);
    }

  pwm_dumpregs(dev, "After starting");

errout:
  return ret;
}

#endif /* CONFIG_PWM_PULSECOUNT */

/****************************************************************************
 * Name: pwm_configure
 *
 * Description:
 *   Configure PWM timer in normal mode (no PULSECOUNT)
 *
 ****************************************************************************/

static int pwm_configure(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint16_t outputs = 0;
  uint8_t j        = 0;
  int     ret      = OK;

  /* NOTE: leave timer counter disabled and all outputs disabled! */

  /* Get configured outputs */

  outputs = pwm_outputs_from_channels(priv);

  /* Disable outputs */

  ret = pwm_outputs_enable(dev, outputs, false);
  if (ret < 0)
    {
      goto errout;
    }

  /* Disable the timer until we get it configured */

  pwm_timer_enable(dev, false);

  /* Initial timer configuration */

  ret = pwm_timer_configure(priv);
  if (ret < 0)
    {
      goto errout;
    }

  /* Some special setup for advanced timers */

#ifdef HAVE_ADVTIM
  if (priv->timtype == TIMTYPE_ADVANCED ||
      priv->timtype == TIMTYPE_COUNTUP16_N)
    {
      /* Configure break and deadtime register */

      ret = pwm_break_dt_configure(priv);
      if (ret < 0)
        {
          goto errout;
        }

#ifdef HAVE_TRGO
      /* Configure TRGO/TRGO2 */

      ret = pwm_trgo_configure(dev, priv->trgo);
      if (ret < 0)
        {
          goto errout;
        }
#endif
    }
#endif

  /* Configure timer channels */

  for (j = 0; j < priv->chan_num; j++)
    {
      /* Skip channel if not in use */

      if (priv->channels[j].channel != 0)
        {
          /* Update PWM mode */

          ret = pwm_mode_configure(dev, priv->channels[j].channel,
                                   priv->channels[j].mode);
          if (ret < 0)
            {
              goto errout;
            }

          /* PWM outputs configuration */

          ret = pwm_output_configure(priv, &priv->channels[j]);
          if (ret < 0)
            {
              goto errout;
            }
        }
    }

  /* Disable software break at the end of the outputs configuration (enablei
   * outputs).
   *
   * NOTE: Only timers with complementary outputs have BDTR register and
   *       support software break.
   */

  if (priv->timtype == TIMTYPE_ADVANCED ||
      priv->timtype == TIMTYPE_COUNTUP16_N)
    {
      ret = pwm_soft_break(dev, false);
      if (ret < 0)
        {
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_duty_channels_update
 *
 * Description:
 *   Update duty cycle for given channels
 *
 ****************************************************************************/

static int pwm_duty_channels_update(struct pwm_lowerhalf_s *dev,
                                    const struct pwm_info_s *info)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint8_t   channel = 0;
  ub16_t    duty    = 0;
  int       ret     = OK;
#ifdef CONFIG_PWM_MULTICHAN
  int       i       = 0;
  int       j       = 0;
#endif

#ifdef CONFIG_PWM_MULTICHAN
  for (i = 0; i < CONFIG_PWM_NCHANNELS; i++)
#endif
    {
#ifdef CONFIG_PWM_MULTICHAN
      /* Break the loop if all following channels are not configured */

      if (info->channels[i].channel == -1)
        {
          break;
        }

      duty    = info->channels[i].duty;
      channel = info->channels[i].channel;

      /* A value of zero means to skip this channel */

      if (channel != 0)
        {
          /* Find the channel */

          for (j = 0; j < priv->chan_num; j++)
            {
              if (priv->channels[j].channel == channel)
                {
                  break;
                }
            }

          /* Check range */

          if (j >= priv->chan_num)
            {
              pwmerr("ERROR: No such channel: %u\n", channel);
              ret = -EINVAL;
              goto errout;
            }
#else
          duty = info->duty;
          channel = priv->channels[0].channel;
#endif

          /* Update duty cycle */

          ret = pwm_duty_update(dev, channel, duty);
          if (ret < 0)
            {
              goto errout;
            }
#ifdef CONFIG_PWM_MULTICHAN
        }
#endif
    }

errout:
  return OK;
}

/****************************************************************************
 * Name: pwm_timer
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_timer(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint16_t outputs = 0;
  int      ret     = OK;

  DEBUGASSERT(priv != NULL && info != NULL);

#if defined(CONFIG_PWM_MULTICHAN)
  pwminfo("TIM%u frequency: %" PRIu32 "\n",
          priv->timid, info->frequency);
#else
  pwminfo("TIM%u channel: %u frequency: %" PRIu32 " duty: %08" PRIx32 "\n",
          priv->timid, priv->channels[0].channel,
          info->frequency, info->duty);
#endif

  DEBUGASSERT(info->frequency > 0);
#ifndef CONFIG_PWM_MULTICHAN
  DEBUGASSERT(info->duty >= 0 && info->duty < uitoub16(100));
#endif

  /* TODO: what if we have pwm running and we want disable some channels ? */

  /* Set timer frequency */

  ret = pwm_frequency_update(dev, info->frequency);
  if (ret < 0)
    {
      goto errout;
    }

  /* Channel specific configuration */

  ret = pwm_duty_channels_update(dev, info);
  if (ret < 0)
    {
      goto errout;
    }

  /* Set the advanced timer's repetition counter */

#ifdef HAVE_ADVTIM
  if (priv->timtype == TIMTYPE_ADVANCED ||
      priv->timtype == TIMTYPE_COUNTUP16_N)
    {
      /* If a non-zero repetition count has been selected, then set the
       * repetition counter to the count-1 (pwm_start() has already
       * assured us that the count value is within range).
       */

      /* Set the repetition counter to zero */

      pwm_rcr_update(dev, 0);

      /* Generate an update event to reload the prescaler */

      pwm_soft_update(dev);
    }
  else
#endif
    {
      /* Generate an update event to reload the prescaler (all timers) */

      pwm_soft_update(dev);
    }

  /* Get configured outputs */

  outputs = pwm_outputs_from_channels(priv);

  /* Enable outputs */

  ret = pwm_outputs_enable(dev, outputs, true);
  if (ret < 0)
    {
      goto errout;
    }

  /* Just enable the timer, leaving all interrupts disabled */

  pwm_timer_enable(dev, true);

  pwm_dumpregs(dev, "After starting");

errout:
  return ret;
}

#ifdef HAVE_PWM_INTERRUPT

/****************************************************************************
 * Name: pwm_interrupt
 *
 * Description:
 *   Handle timer interrupts.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_interrupt(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint16_t regval;

  /* Verify that this is an update interrupt.  Nothing else is expected. */

  regval = pwm_getreg(priv, STM32_ATIM_SR_OFFSET);
  DEBUGASSERT((regval & ATIM_SR_UIF) != 0);

  /* Clear the UIF interrupt bit */

  pwm_putreg(priv, STM32_ATIM_SR_OFFSET, (regval & ~ATIM_SR_UIF));

  /* Calculate the new count by subtracting the number of pulses
   * since the last interrupt.
   */

  if (priv->count <= priv->prev)
    {
      /* We are finished.  Turn off the master output to stop the output as
       * quickly as possible.
       */

      pwm_soft_break(dev, true);

      /* Disable first interrupts, stop and reset the timer */

      pwm_stop(dev);

      /* Then perform the callback into the upper half driver */

      pwm_expired(priv->handle);

      priv->handle = NULL;
      priv->count  = 0;
      priv->prev   = 0;
      priv->curr   = 0;
    }
  else
    {
      /* Decrement the count of pulses remaining using the number of
       * pulses generated since the last interrupt.
       */

      priv->count -= priv->prev;

      /* Set up the next RCR.  Set 'prev' to the value of the RCR that
       * was loaded when the update occurred (just before this interrupt)
       * and set 'curr' to the current value of the RCR register (which
       * will bet loaded on the next update event).
       */

      priv->prev = priv->curr;
      priv->curr = pwm_pulsecount(priv->count - priv->prev);
      pwm_rcr_update(dev, priv->curr - 1);
    }

  /* Now all of the time critical stuff is done so we can do some debug
   * output.
   */

  pwminfo("Update interrupt SR: %04x prev: %u curr: %u count: %" PRIx32 "\n",
          regval, priv->prev, priv->curr, priv->count);

  return OK;
}

/****************************************************************************
 * Name: pwm_tim1/8interrupt
 *
 * Description:
 *   Handle timer 1 and 8 interrupts.
 *
 * Input Parameters:
 *   Standard NuttX interrupt inputs
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_TIM1_PWM
static int pwm_tim1interrupt(int irq, void *context, void *arg)
{
  return pwm_interrupt((struct pwm_lowerhalf_s *)&g_pwm1dev);
}
#endif /* CONFIG_STM32_TIM1_PWM */

#ifdef CONFIG_STM32_TIM8_PWM
static int pwm_tim8interrupt(int irq, void *context, void *arg)
{
  return pwm_interrupt((struct pwm_lowerhalf_s *)&g_pwm8dev);
}
#endif /* CONFIG_STM32_TIM8_PWM */

/****************************************************************************
 * Name: pwm_pulsecount
 *
 * Description:
 *   Pick an optimal pulse count to program the RCR.
 *
 * Input Parameters:
 *   count - The total count remaining
 *
 * Returned Value:
 *   The recommended pulse count
 *
 ****************************************************************************/

static uint8_t pwm_pulsecount(uint32_t count)
{
  /* REVISIT: RCR_REP_MAX for GTIM or ATIM ? */

  /* The the remaining pulse count is less than or equal to the maximum, the
   * just return the count.
   */

  if (count <= ATIM_RCR_REP_MAX)
    {
      return (uint8_t)count;
    }

  /* Otherwise, we have to be careful.  We do not want a small number of
   * counts at the end because we might have trouble responding fast enough.
   * If the remaining count is less than 150% of the maximum, then return
   * half of the maximum.  In this case the final sequence will be between 64
   * and 128.
   */

  else if (count < (3 * ATIM_RCR_REP_MAX / 2))
    {
      return (uint8_t)((ATIM_RCR_REP_MAX + 1) >> 1);
    }

  /* Otherwise, return the maximum.  The final count will be 64 or more */

  else
    {
      return (uint8_t)ATIM_RCR_REP_MAX;
    }
}
#endif /* HAVE_PWM_INTERRUPT */

/****************************************************************************
 * Name: pwm_set_apb_clock
 *
 * Description:
 *   Enable or disable APB clock for the timer peripheral
 *
 * Input Parameters:
 *   priv - A reference to the PWM block status
 *   on   - Enable clock if 'on' is 'true' and disable if 'false'
 *
 ****************************************************************************/

static int pwm_set_apb_clock(struct stm32_pwmtimer_s *priv, bool on)
{
  uint32_t en_bit  = 0;
  uint32_t regaddr = 0;
  int      ret     = OK;

  pwminfo("timer %d clock enable: %d\n", priv->timid, on ? 1 : 0);

  /* Determine which timer to configure */

  switch (priv->timid)
    {
#ifdef CONFIG_STM32_TIM1_PWM
      case 1:
        {
          regaddr = TIMRCCEN_TIM1;
          en_bit  = TIMEN_TIM1;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM2_PWM
      case 2:
        {
          regaddr = TIMRCCEN_TIM2;
          en_bit  = TIMEN_TIM2;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM3_PWM
      case 3:
        {
          regaddr = TIMRCCEN_TIM3;
          en_bit  = TIMEN_TIM3;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM4_PWM
      case 4:
        {
          regaddr = TIMRCCEN_TIM4;
          en_bit  = TIMEN_TIM4;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM5_PWM
      case 5:
        {
          regaddr = TIMRCCEN_TIM5;
          en_bit  = TIMEN_TIM5;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM8_PWM
      case 8:
        {
          regaddr = TIMRCCEN_TIM8;
          en_bit  = TIMEN_TIM8;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM9_PWM
      case 9:
        {
          regaddr = TIMRCCEN_TIM9;
          en_bit  = TIMEN_TIM9;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM10_PWM
      case 10:
        {
          regaddr = TIMRCCEN_TIM10;
          en_bit  = TIMEN_TIM10;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM11_PWM
      case 11:
        {
          regaddr = TIMRCCEN_TIM11;
          en_bit  = TIMEN_TIM11;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM12_PWM
      case 12:
        {
          regaddr = TIMRCCEN_TIM12;
          en_bit  = TIMEN_TIM12;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM13_PWM
      case 13:
        {
          regaddr = TIMRCCEN_TIM13;
          en_bit  = TIMEN_TIM13;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM14_PWM
      case 14:
        {
          regaddr = TIMRCCEN_TIM14;
          en_bit  = TIMEN_TIM14;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM15_PWM
      case 15:
        {
          regaddr = TIMRCCEN_TIM15;
          en_bit  = TIMEN_TIM15;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM16_PWM
      case 16:
        {
          regaddr = TIMRCCEN_TIM16;
          en_bit  = TIMEN_TIM16;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM17_PWM
      case 17:
        {
          regaddr = TIMRCCEN_TIM17;
          en_bit  = TIMEN_TIM17;
          break;
        }
#endif

      default:
        {
          pwmerr("ERROR: No such timer configured %d\n", priv->timid);
          ret = -EINVAL;
          goto errout;
        }
    }

  /* Enable/disable APB 1/2 clock for timer */

  pwminfo("RCC_APBxENR base: %08" PRIx32 "  bits: %04" PRIx32 "\n",
          regaddr, en_bit);

  if (on)
    {
      modifyreg32(regaddr, 0, en_bit);
    }
  else
    {
      modifyreg32(regaddr, en_bit, 0);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_setup
 *
 * Description:
 *   This method is called when the driver is opened.  The lower half driver
 *   should configure and initialize the device so that it is ready for use.
 *   It should not, however, output pulses until the start method is called.
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   APB1 or 2 clocking for the GPIOs has already been configured by the RCC
 *   logic at power up.
 *
 ****************************************************************************/

static int pwm_setup(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t pincfg = 0;
  int      ret    = OK;
  int      i      = 0;

  pwminfo("TIM%u\n", priv->timid);

  /* Enable APB1/2 clocking for timer. */

  ret = pwm_set_apb_clock(priv, true);
  if (ret < 0)
    {
      goto errout;
    }

  pwm_dumpregs(dev, "Initially");

  /* Configure the PWM output pins, but do not start the timer yet */

  for (i = 0; i < priv->chan_num; i++)
    {
      if (priv->channels[i].out1.in_use == 1)
        {
          /* Do not configure the pin if pincfg is not specified.
           * This prevents overwriting the PA0 configuration if the
           * channel is used internally.
           */

          pincfg = priv->channels[i].out1.pincfg;
          if (pincfg != 0)
            {
              pwminfo("pincfg: %08" PRIx32 "\n", pincfg);

              stm32_configgpio(pincfg);
              pwm_dumpgpio(pincfg, "PWM setup");
            }
        }

#ifdef HAVE_PWM_COMPLEMENTARY
      if (priv->channels[i].out2.in_use == 1)
        {
          pincfg = priv->channels[i].out2.pincfg;

          /* Do not configure the pin if pincfg is not specified.
           * This prevents overwriting the PA0 configuration if the
           * channel is used internally.
           */

          if (pincfg != 0)
            {
              pwminfo("pincfg: %08" PRIx32 "\n", pincfg);

              stm32_configgpio(pincfg);
              pwm_dumpgpio(pincfg, "PWM setup");
            }
        }
#endif
    }

  /* Configure PWM timer with the selected configuration.
   *
   * NOTE: We configure PWM here during setup, but leave timer with disabled
   *       counter, disabled outputs, not configured frequency and duty cycle
   */

#ifdef CONFIG_PWM_PULSECOUNT
  if (priv->timtype == TIMTYPE_ADVANCED)
    {
      ret = pwm_pulsecount_configure(dev);
    }
  else
#endif
    {
      ret = pwm_configure(dev);
    }

  if (ret < 0)
    {
      pwmerr("failed to configure PWM %d\n", priv->timid);
      ret = ERROR;
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_shutdown
 *
 * Description:
 *   This method is called when the driver is closed.  The lower half driver
 *   stop pulsed output, free any resources, disable the timer hardware, and
 *   put the system into the lowest possible power usage state
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_shutdown(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  uint32_t pincfg = 0;
  int      i      = 0;
  int      ret    = OK;

  pwminfo("TIM%u\n", priv->timid);

  /* Make sure that the output has been stopped */

  pwm_stop(dev);

  /* Disable APB1/2 clocking for timer. */

  ret = pwm_set_apb_clock(priv, false);
  if (ret < 0)
    {
      goto errout;
    }

  /* Then put the GPIO pins back to the default state */

  for (i = 0; i < priv->chan_num; i++)
    {
      pincfg = priv->channels[i].out1.pincfg;
      if (pincfg != 0)
        {
          pwminfo("pincfg: %08" PRIx32 "\n", pincfg);

          pincfg &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
          pincfg |= PINCFG_DEFAULT;

          stm32_configgpio(pincfg);
        }

#ifdef HAVE_PWM_COMPLEMENTARY
      pincfg = priv->channels[i].out2.pincfg;
      if (pincfg != 0)
        {
          pwminfo("pincfg: %08" PRIx32 "\n", pincfg);

          pincfg &= (GPIO_PORT_MASK | GPIO_PIN_MASK);
          pincfg |= PINCFG_DEFAULT;

          stm32_configgpio(pincfg);
        }
#endif
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: pwm_start
 *
 * Description:
 *   (Re-)initialize the timer resources and start the pulsed output
 *
 * Input Parameters:
 *   dev  - A reference to the lower half PWM driver state structure
 *   info - A reference to the characteristics of the pulsed output
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

#ifdef CONFIG_PWM_PULSECOUNT
static int pwm_start_pulsecount(struct pwm_lowerhalf_s *dev,
                                const struct pwm_info_s *info,
                                void *handle)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  /* Generate an indefinite number of pulses */

  if (info->count == 0)
    {
      return pwm_start(dev, info);
    }

  /* Check if a pulsecount has been selected */

  if (info->count > 0)
    {
      /* Only the advanced timers (TIM1,8 can support the pulse counting)
       * REVISIT: verify if TIMTYPE_COUNTUP16_N works with it
       */

      if (priv->timtype != TIMTYPE_ADVANCED)
        {
          pwmerr("ERROR: TIM%u cannot support pulse count: %" PRIx32 "\n",
                 priv->timid, info->count);
          return -EPERM;
        }
    }

  /* Save the handle */

  priv->handle = handle;

  /* Start the time */

  return pwm_pulsecount_timer(dev, info);
}
#endif /* CONFIG_PWM_PULSECOUNT */

static int pwm_start(struct pwm_lowerhalf_s *dev,
                     const struct pwm_info_s *info)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  int ret = OK;

  /* if frequency has not changed we just update duty */

  if (info->frequency == priv->frequency)
    {
#ifdef CONFIG_PWM_MULTICHAN
      int i;

      for (i = 0; ret == OK && i < CONFIG_PWM_NCHANNELS; i++)
        {
          /* Break the loop if all following channels are not configured */

          if (info->channels[i].channel == -1)
            {
              break;
            }

          /* Set output if channel configured */

          if (info->channels[i].channel != 0)
            {
              ret = pwm_duty_update(dev, info->channels[i].channel,
                                    info->channels[i].duty);
            }
        }
#else
      ret = pwm_duty_update(dev, priv->channels[0].channel, info->duty);
#endif /* CONFIG_PWM_MULTICHAN */
    }
  else
    {
      ret = pwm_timer(dev, info);

      /* Save current frequency */

      if (ret == OK)
        {
          priv->frequency = info->frequency;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: pwm_stop
 *
 * Description:
 *   Stop the pulsed output and reset the timer resources
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 * Assumptions:
 *   This function is called to stop the pulsed output at anytime.  This
 *   method is also called from the timer interrupt handler when a repetition
 *   count expires... automatically stopping the timer.
 *
 ****************************************************************************/

static int pwm_stop(struct pwm_lowerhalf_s *dev)
{
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;
  irqstate_t flags    = 0;
  uint16_t   outputs  = 0;
  int        ret = OK;

  pwminfo("TIM%u\n", priv->timid);

  /* Disable interrupts momentary to stop any ongoing timer processing and
   * to prevent any concurrent access to the reset register.
   */

  flags = enter_critical_section();

  /* Stopped so frequency is zero */

  priv->frequency = 0;

  /* Disable further interrupts and stop the timer */

  pwm_putreg(priv, STM32_GTIM_DIER_OFFSET, 0);
  pwm_putreg(priv, STM32_GTIM_SR_OFFSET, 0);

  /* Disable the timer and timer outputs */

  pwm_timer_enable(dev, false);
  outputs = pwm_outputs_from_channels(priv);
  ret = pwm_outputs_enable(dev, outputs, false);

  leave_critical_section(flags);

  pwm_dumpregs(dev, "After stop");

  return ret;
}

/****************************************************************************
 * Name: pwm_ioctl
 *
 * Description:
 *   Lower-half logic may support platform-specific ioctl commands
 *
 * Input Parameters:
 *   dev - A reference to the lower half PWM driver state structure
 *   cmd - The ioctl command
 *   arg - The argument accompanying the ioctl command
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure
 *
 ****************************************************************************/

static int pwm_ioctl(struct pwm_lowerhalf_s *dev, int cmd,
                     unsigned long arg)
{
#ifdef CONFIG_DEBUG_PWM_INFO
  struct stm32_pwmtimer_s *priv = (struct stm32_pwmtimer_s *)dev;

  /* There are no platform-specific ioctl commands */

  pwminfo("TIM%u\n", priv->timid);
#endif
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwminitialize
 *
 * Description:
 *   Initialize one timer for use with the upper_level PWM driver.
 *
 * Input Parameters:
 *   timer - A number identifying the timer use.  The number of valid timer
 *     IDs varies with the STM32 MCU and MCU family but is somewhere in
 *     the range of {1,..,17}.
 *
 * Returned Value:
 *   On success, a pointer to the STM32 lower half PWM driver is returned.
 *   NULL is returned on any failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *stm32_pwminitialize(int timer)
{
  struct stm32_pwmtimer_s *lower = NULL;

  pwminfo("TIM%u\n", timer);

  switch (timer)
    {
#ifdef CONFIG_STM32_TIM1_PWM
      case 1:
        {
          lower = &g_pwm1dev;

          /* Attach but disable the TIM1 update interrupt */

#ifdef CONFIG_PWM_PULSECOUNT
          irq_attach(lower->irq, pwm_tim1interrupt, NULL);
          up_disable_irq(lower->irq);
#endif
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM2_PWM
      case 2:
        {
          lower = &g_pwm2dev;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM3_PWM
      case 3:
        {
          lower = &g_pwm3dev;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM4_PWM
      case 4:
        {
          lower = &g_pwm4dev;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM5_PWM
      case 5:
        {
          lower = &g_pwm5dev;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM8_PWM
      case 8:
        {
          lower = &g_pwm8dev;

          /* Attach but disable the TIM8 update interrupt */

#ifdef CONFIG_PWM_PULSECOUNT
          irq_attach(lower->irq, pwm_tim8interrupt, NULL);
          up_disable_irq(lower->irq);
#endif
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM9_PWM
      case 9:
        {
          lower = &g_pwm9dev;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM10_PWM
      case 10:
        {
          lower = &g_pwm10dev;
          break;
        }

#endif

#ifdef CONFIG_STM32_TIM11_PWM
      case 11:
        {
          lower = &g_pwm11dev;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM12_PWM
      case 12:
        {
          lower = &g_pwm12dev;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM13_PWM
      case 13:
        {
          lower = &g_pwm13dev;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM14_PWM
      case 14:
        {
          lower = &g_pwm14dev;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM15_PWM
      case 15:
        {
          lower = &g_pwm15dev;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM16_PWM
      case 16:
        {
          lower = &g_pwm16dev;
          break;
        }
#endif

#ifdef CONFIG_STM32_TIM17_PWM
      case 17:
        {
          lower = &g_pwm17dev;
          break;
        }
#endif

      default:
        {
          pwmerr("ERROR: No such timer configured %d\n", timer);
          lower = NULL;
          goto errout;
        }
    }

errout:
  return (struct pwm_lowerhalf_s *)lower;
}

#endif /* CONFIG_STM32_PWM */
