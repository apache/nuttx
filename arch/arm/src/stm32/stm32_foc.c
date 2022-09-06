/****************************************************************************
 * arch/arm/src/stm32/stm32_foc.c
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

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "stm32_pwm.h"
#include "stm32_adc.h"
#include "stm32_dma.h"

#include <arch/irq.h>
#include <arch/chip/chip.h>

#include "stm32_foc.h"

#include "hardware/stm32_dbgmcu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Verify peripheral configuration ******************************************/

/* This is the lower-half implementation for the STM32 FOC devices.
 *
 * We currently support a current sensing topology with two and three shunts.
 * Configuration with a single-shunt is not supported at the moment and will
 * require additional current reconstruction logic.
 *
 * A single FOC device uses one advanced timer to generate a center-aligned
 * PWM which control phase switches bridge. Phase currents must be sampled
 * at vector 0 (all low-side switches are on and the current flows through
 * current sensors).
 *
 * This implementation uses one ADC per controller and we only use injected
 * conversion to sample currents. ADC regular conversion is not used
 * and can be used to other tasks with the help of DMA transfer.
 * For ADC regular conversion, only DMA transfer is possible since ADC
 * interrupt handler is reserved for the FOC only.
 *
 * The ADC conversion trigger is configurable. Available options are:
 *   1. TRGO events generated on update event
 *   2. CCR4 events
 *
 * There are some differences in implementation depending on the peripherals
 * supported by the chip.
 *
 * There is no differences according to TIMER IPv1 and IPv2 affecting
 * this implementation.
 *
 * For STM32 ADC cores there are some dissimilarities that had to be taken
 * into account. It is:
 *
 *   1. For ADC IPv1 (F2, F4, F7, L1)
 *     - all ADC instances coupled in single block
 *     - single entry point for ADC123 interrupts
 *
 *   2. For ADC IPv1 basic (F1, F37x)
 *     - ADC instances are no coupled in blocks
 *     - common interrupts for ADC1 and ADC2
 *
 *   3. For ADC IPv2 (F3 (without F37x), H7, L4, L4+, G4)
 *     - ADC grouped in slave-master configuration (ADC12, ADC34)
 *
 * This code will not work for chips with ADC IPv2 basic (F0, L0, G0).
 * For these, only regular channels are available and we cannot use injected
 * conversion.
 *
 * Currently, up to two FOC instances are supported.
 */

/* Verify system configuration **********************************************/

/* This is not for ADC IPv2 basic */

#if defined(CONFIG_STM32_HAVE_IP_ADC_V2) && defined(HAVE_BASIC_ADC)
#  error Not supported ADC IP core
#endif

/* Multi instances support tested only on IP_ADC_V1 */

#if CONFIG_MOTOR_FOC_INST > 1
#  if defined(CONFIG_STM32_HAVE_IP_ADC_V2)
#    error Not tested yet
#  endif
#endif

/* PWM lower-half ops and ADC lower-half ops must be enabled */

#ifndef CONFIG_STM32_PWM_LL_OPS
#  error PWM low-level operations interface must be enabled
#endif
#ifndef CONFIG_STM32_ADC_LL_OPS
#  error ADC low-level operations interface must be enabled
#endif

/* We don't want start conversion during ADC setup */

#ifndef CONFIG_STM32_ADC_NO_STARTUP_CONV
#  error ADC startup conversion must be disabled
#endif

/* We need interface to change ADC sample-time */

#ifndef CONFIG_STM32_ADC_CHANGE_SAMPLETIME
#  error ADC sample-time configuration interface must be enabled
#endif

/* Debug register for PWM timers */

#if defined(CONFIG_STM32_HAVE_IP_DBGMCU_V2) || \
    defined(CONFIG_STM32_HAVE_IP_DBGMCU_V3)
#  define FOC_PWM_FZ_REG    (STM32_DBGMCU_APB2_FZ)
#elif defined(CONFIG_STM32_HAVE_IP_DBGMCU_V1)
#  define FOC_PWM_FZ_REG    (STM32_DBGMCU_CR)
#endif

/* FOC0 always use TIMER1 for PWM */

#ifdef CONFIG_STM32_FOC_FOC0
#  define FOC0_PWM           (1)
#  define FOC0_PWM_NCHANNELS (PWM_TIM1_NCHANNELS)
#  define FOC0_PWM_BASE      (STM32_TIM1_BASE)
#  if defined(CONFIG_STM32_HAVE_IP_DBGMCU_V2) ||  \
      defined(CONFIG_STM32_HAVE_IP_DBGMCU_V3)
#    define FOC0_PWM_FZ_BIT    (DBGMCU_APB2_TIM1STOP)
#  elif defined(CONFIG_STM32_HAVE_IP_DBGMCU_V1)
#    define FOC0_PWM_FZ_BIT    (DBGMCU_CR_TIM1STOP)
#  endif
#  if CONFIG_STM32_TIM1_MODE != 2
#    error TIM1 must be configured in center-aligned mode 1
#  endif
#endif  /* CONFIG_STM32_FOC_FOC0 */

/* FOC1 always use TIMER8 for PWM */

#ifdef CONFIG_STM32_FOC_FOC1
#  define FOC1_PWM           (8)
#  define FOC1_PWM_NCHANNELS (PWM_TIM8_NCHANNELS)
#  define FOC1_PWM_BASE      (STM32_TIM8_BASE)
#  if defined(CONFIG_STM32_HAVE_IP_DBGMCU_V2) ||  \
      defined(CONFIG_STM32_HAVE_IP_DBGMCU_V3)
#    define FOC1_PWM_FZ_BIT    (DBGMCU_APB2_TIM8STOP)
#  elif defined(CONFIG_STM32_HAVE_IP_DBGMCU_V1)
#    define FOC1_PWM_FZ_BIT    (DBGMCU_CR_TIM8STOP)
#  endif
#  if CONFIG_STM32_TIM8_MODE != 2
#    error TIM8 must be configured in center-aligned mode 1
#  endif
#endif

/* The maximum supported number of phases depends on the ADC trigger */

#if defined(CONFIG_STM32_FOC_ADC_CCR4)
#  if CONFIG_MOTOR_FOC_PHASES > 3
#    error max 3 phases supported
#  endif
#elif defined(CONFIG_STM32_FOC_ADC_TRGO)
#  if CONFIG_MOTOR_FOC_PHASES > 4
#    error max 4 phases supported
#  endif
#else
#  error
#endif

/* Tested only for 3-phase devices */

#if CONFIG_MOTOR_FOC_PHASES != 3
#  error Tested only for 3-phase devices
#endif

/* Only one ADC trigger must be selected */

#if defined(CONFIG_STM32_FOC_ADC_CCR4) && defined(CONFIG_STM32_FOC_ADC_TRGO)
#  error Invalid ADC trigger configuration
#endif

/* Phase currents can only be sampled when all low-side switches are off.
 * This is only valid for the V0 vector in the SVM.
 *
 * For PWM mode 1:
 *   V7 for CNTR = 0
 *   V0 for CNTR = ARR
 *
 * For PWM mode 2:
 *   V7 for CNTR = ARR
 *   V0 for CNTR = 0
 */

#if defined(CONFIG_STM32_FOC_ADC_CCR4)

/* FOC ADC trigger on CCR4 **************************************************/

/* PWM channels configuration:
 *   - n channels for phases PWM (CCR1, CCR2, CCR3)
 *   - 1 channel for ADC injection sequence trigger (CCR4)
 */

#  if defined(CONFIG_STM32_FOC_FOC0)
#    if FOC0_PWM_NCHANNELS != (CONFIG_MOTOR_FOC_PHASES + 1)
#      error Invalid channels configuration
#    endif
#  endif
#  if defined(CONFIG_STM32_FOC_FOC1)
#    if FOC1_PWM_NCHANNELS != (CONFIG_MOTOR_FOC_PHASES + 1)
#      error Invalid channels configuration
#    endif
#  endif

/* Generalize JEXTSEL bits for CCR4 trigger.
 *
 * ADC trigger event on PWM timer CCR4 (rising edge).
 *
 * This implementation uses PWM mode 1 so:
 *   TIMx CCR4 = (ARR - trigger_offset)
 */

#  if defined(CONFIG_STM32_HAVE_IP_ADC_V2)
#    ifdef CONFIG_STM32_FOC_USE_TIM1
#      define ADC_JEXTSEL_T1CC4 (ADC12_JSQR_JEXTSEL_T1CC4)
#    endif
#    ifdef CONFIG_STM32_FOC_USE_TIM8
#      define ADC_JEXTSEL_T8CC4 (ADC12_JSQR_JEXTSEL_T8CC4)
#    endif
#  elif defined(CONFIG_STM32_HAVE_IP_ADC_V1)
#    ifdef CONFIG_STM32_FOC_USE_TIM1
#      define ADC_JEXTSEL_T1CC4  (ADC_CR2_JEXTSEL_T1CC4)
#    endif
#    ifdef CONFIG_STM32_FOC_USE_TIM8
#      define ADC_JEXTSEL_T8CC4  (ADC_CR2_JEXTSEL_T8CC4)
#    endif
#  else
#    error Not supported
#  endif

/* ADC trigger offset - must be greater than 0! */

#  define ADC_TRIGGER_OFFSET (1)

#  ifdef CONFIG_STM32_FOC_FOC0
#    define FOC0_ADC_JEXTSEL  (ADC_JEXTSEL_T1CC4)
#  endif
#  ifdef CONFIG_STM32_FOC_FOC1
#    define FOC1_ADC_JEXTSEL  (ADC_JEXTSEL_T8CC4)
#  endif

#elif defined(CONFIG_STM32_FOC_ADC_TRGO)

/* FOC ADC trigger on TRGO **************************************************/

/* PWM TRGO support must be enabled */

#  ifndef CONFIG_STM32_PWM_TRGO
#    error PWM TRGO support must be enabled
#  endif

/* TRGO on update event = ATIM_CR2_MMS_UPDATE (2) */

#  define FOC_PWM_TRGO (2)

/* PWM channels configuration:
 *   - n channels for phases PWM (CCR1, CCR2, CCR3, CCR4)
 */

#  if defined(CONFIG_STM32_FOC_FOC0)
#    if FOC0_PWM_NCHANNELS != (CONFIG_MOTOR_FOC_PHASES)
#      error Invalid channels configuration
#    endif
#  endif
#  if defined(CONFIG_STM32_FOC_FOC1)
#    if FOC1_PWM_NCHANNELS != (CONFIG_MOTOR_FOC_PHASES)
#      error Invalid channels configuration
#    endif
#  endif

/* Generalize JEXTSEL bits for TRGO trigger.
 *
 * ADC trigger event on PWM timer TRGO (rising edge).
 *
 * This implementation uses PWM mode 1 so:
 *   TIMx TRGO = (ARR)
 */

#  if defined(CONFIG_STM32_HAVE_IP_ADC_V2)
#    ifdef CONFIG_STM32_FOC_USE_TIM1
#      define ADC_JEXTSEL_T1TRGO (ADC12_JSQR_JEXTSEL_T1TRGO)
#    endif
#    ifdef CONFIG_STM32_FOC_USE_TIM8
#      define ADC_JEXTSEL_T8TRGO (ADC12_JSQR_JEXTSEL_T8TRGO)
#    endif
#  elif defined(CONFIG_STM32_HAVE_IP_ADC_V1)
#    ifdef CONFIG_STM32_FOC_USE_TIM1
#      define ADC_JEXTSEL_T1TRGO  (ADC_CR2_JEXTSEL_T1TRGO)
#    endif
#    ifdef CONFIG_STM32_FOC_USE_TIM8
#      error TIM8 and TRGO trigger not supported for ADC IPv1
#    endif
#  else
#    error Not supported
#  endif

#  ifdef CONFIG_STM32_FOC_FOC0
#    define FOC0_ADC_JEXTSEL  (ADC_JEXTSEL_T1TRGO)
#  endif
#  ifdef CONFIG_STM32_FOC_FOC1
#    define FOC1_ADC_JEXTSEL  (ADC_JEXTSEL_T8TRGO)
#  endif

#else

/* No trigger selected ******************************************************/

#  error Invalid FOC ADC trigger
#endif

/* Phase current samples for FOC0 */

#ifdef CONFIG_STM32_FOC_FOC0
#  ifdef CONFIG_STM32_FOC_FOC0_ADC1
#    define FOC0_ADC 1
#  endif
#  ifdef CONFIG_STM32_FOC_FOC0_ADC2
#    define FOC0_ADC 2
#  endif
#  ifdef CONFIG_STM32_FOC_FOC0_ADC3
#    define FOC0_ADC 3
#  endif
#  ifdef CONFIG_STM32_FOC_FOC0_ADC4
#    define FOC0_ADC 4
#  endif
#endif

/* Phase current samples for FOC1 */

#ifdef CONFIG_STM32_FOC_FOC1
#  ifdef CONFIG_STM32_FOC_FOC1_ADC1
#    define FOC1_ADC 1
#  endif
#  ifdef CONFIG_STM32_FOC_FOC1_ADC2
#    define FOC1_ADC 2
#  endif
#  ifdef CONFIG_STM32_FOC_FOC1_ADC3
#    define FOC1_ADC 3
#  endif
#  ifdef CONFIG_STM32_FOC_FOC1_ADC4
#    define FOC1_ADC 4
#  endif
#endif

/* The number of required injected channels */

#ifdef CONFIG_STM32_FOC_G4_ADCCHAN0_WORKAROUND
#  define FOC_ADC_INJ_CHAN_REQUIRED (CONFIG_MOTOR_FOC_SHUNTS + 1)
#else
#  define FOC_ADC_INJ_CHAN_REQUIRED (CONFIG_MOTOR_FOC_SHUNTS)
#endif

/* Validate ADC configuration:
 *   1. ADC must be supported by chip,
 *   2. ADC support for injected channels must be enabled,
 *   3. ADC software trigger starts only regular conversion.
 */

#ifdef CONFIG_STM32_FOC_USE_ADC1
#  ifndef CONFIG_STM32_ADC1
#    error ADC1 not supported !
#  endif
#  ifndef ADC1_HAVE_JEXTCFG
#    error ADC1 must support JEXTCFG
#  endif
#  if CONFIG_STM32_ADC1_ANIOC_TRIGGER != 1
#    error CONFIG_STM32_ADC1_ANIOC_TRIGGER must be 1
#  endif
#  if CONFIG_STM32_ADC1_INJECTED_CHAN != FOC_ADC_INJ_CHAN_REQUIRED
#    error Invalid configuration for ADC1 injected channels
#  endif
#endif
#ifdef CONFIG_STM32_FOC_USE_ADC2
#  ifndef CONFIG_STM32_ADC2
#    error ADC2 not supported !
#  endif
#  ifndef ADC2_HAVE_JEXTCFG
#    error ADC2 must support JEXTCFG
#  endif
#  if CONFIG_STM32_ADC2_ANIOC_TRIGGER != 1
#    error CONFIG_STM32_ADC2_ANIOC_TRIGGER must be 1
#  endif
#  if CONFIG_STM32_ADC2_INJECTED_CHAN != FOC_ADC_INJ_CHAN_REQUIRED
#    error Invalid configuration for ADC2 injected channels
#  endif
#endif
#ifdef CONFIG_STM32_FOC_USE_ADC3
#  ifndef CONFIG_STM32_ADC3
#    error ADC3 not supported !
#  endif
#  ifndef ADC3_HAVE_JEXTCFG
#    error ADC3 must support JEXTCFG
#  endif
#  if CONFIG_STM32_ADC3_ANIOC_TRIGGER != 1
#    error CONFIG_STM32_ADC3_ANIOC_TRIGGER must be 1
#  endif
#  if CONFIG_STM32_ADC3_INJECTED_CHAN != FOC_ADC_INJ_CHAN_REQUIRED
#    error Invalid configuration for ADC3 injected channels
#  endif
#endif
#ifdef CONFIG_STM32_FOC_USE_ADC4
#  ifndef CONFIG_STM32_ADC4
#    error ADC4 not supported !
#  endif
#  ifndef ADC4_HAVE_JEXTCFG
#    error ADC4 must support JEXTCFG
#  endif
#  if CONFIG_STM32_ADC4_ANIOC_TRIGGER != 1
#    error CONFIG_STM32_ADC4_ANIOC_TRIGGER must be 1
#  endif
#  if CONFIG_STM32_ADC4_INJECTED_CHAN != FOC_ADC_INJ_CHAN_REQUIRED
#    error Invalid configuration for ADC4 injected channels
#  endif
#endif

/* Max 3 shunts supported if STM32G4 ADC CHAN0 workaround enabled */

#ifdef CONFIG_STM32_FOC_G4_ADCCHAN0_WORKAROUND
#  if CONFIG_MOTOR_FOC_SHUNTS > 3
#    error
#  endif
#endif

/* Combine JEXTSEL with JEXTEN default */

#ifdef CONFIG_STM32_FOC_FOC0
#  define FOC0_ADC_JEXT (ADC_JEXTREG_JEXTEN_DEFAULT | FOC0_ADC_JEXTSEL)
#endif
#ifdef CONFIG_STM32_FOC_FOC1
#  define FOC1_ADC_JEXT (ADC_JEXTREG_JEXTEN_DEFAULT | FOC1_ADC_JEXTSEL)
#endif

/* Generalize ADC interrupt flags */

#if defined(CONFIG_STM32_HAVE_IP_ADC_V2)
#  define FOC_ADC_ISR_FOC ADC_ISR_JEOS
#  define FOC_ADC_IER_FOC ADC_IER_JEOS
#  define FOC_ADC_ISR_OVR ADC_INT_OVR
#elif defined(CONFIG_STM32_HAVE_IP_ADC_V1)
#  define FOC_ADC_ISR_FOC ADC_ISR_JEOC
#  define FOC_ADC_IER_FOC ADC_IER_JEOC
#  define FOC_ADC_ISR_OVR ADC_SR_OVR
#else
#  error Not supported
#endif

/* We have 3 possible ADC IRQ configuration */

#if defined(STM32_IRQ_ADC1)

/* Only ADC1 supported */

#  define STM32_IRQ_ADC1_FOC  STM32_IRQ_ADC1

#elif defined(STM32_IRQ_ADC12)

/* ADC1 + ADC2 interrupt */

#  define STM32_IRQ_ADC1_FOC  STM32_IRQ_ADC12
#  define STM32_IRQ_ADC2_FOC  STM32_IRQ_ADC12

/* ADC3 + ADC4 interrupt */

#  define STM32_IRQ_ADC3_FOC  STM32_IRQ_ADC34
#  define STM32_IRQ_ADC4_FOC  STM32_IRQ_ADC34

#elif defined(STM32_IRQ_ADC)

/* ADC1 + ADC2 + ADC3 interrupt */

#  define STM32_IRQ_ADC1_FOC  STM32_IRQ_ADC
#  define STM32_IRQ_ADC2_FOC  STM32_IRQ_ADC
#  define STM32_IRQ_ADC3_FOC  STM32_IRQ_ADC
#endif

/* ADC common ***************************************************************/

/* Common for ADCv1 */

#if defined(CONFIG_STM32_HAVE_IP_ADC_V1) && !defined(HAVE_BASIC_ADC)
#  define FOC_ADC_HAVE_CMN (1)
#  ifdef CONFIG_STM32_FOC_USE_ADC1
#    define FOC_ADC1_CMN (&g_stm32_foc_adccmn123)
#  endif
#  ifdef CONFIG_STM32_FOC_USE_ADC2
#    define FOC_ADC2_CMN (&g_stm32_foc_adccmn123)
#  endif
#  ifdef CONFIG_STM32_FOC_USE_ADC3
#    define FOC_ADC3_CMN (&g_stm32_foc_adccmn123)
#  endif
#endif

/* Common for ADCv1 basic */

#if defined(CONFIG_STM32_HAVE_IP_ADC_V1) && defined(HAVE_BASIC_ADC)
#  undef FOC_ADC_HAVE_CMN
#  ifdef CONFIG_STM32_FOC_USE_ADC1
#    define FOC_ADC1_CMN (0)
#  endif
#  ifdef CONFIG_STM32_FOC_USE_ADC2
#    define FOC_ADC2_CMN (0)
#  endif
#  ifdef CONFIG_STM32_FOC_USE_ADC3
#    define FOC_ADC3_CMN (0)
#  endif
#endif

/* Common for ADCv2 */

#ifdef CONFIG_STM32_HAVE_IP_ADC_V2
#  define FOC_ADC_HAVE_CMN (1)
#  ifdef CONFIG_STM32_FOC_USE_ADC1
#    define FOC_ADC1_CMN (&g_stm32_foc_adccmn12)
#  endif
#  ifdef CONFIG_STM32_FOC_USE_ADC2
#    define FOC_ADC2_CMN (&g_stm32_foc_adccmn12)
#  endif
#  ifdef CONFIG_STM32_FOC_USE_ADC3
#    define FOC_ADC3_CMN (&g_stm32_foc_adccmn34)
#  endif
#  ifdef CONFIG_STM32_FOC_USE_ADC4
#    define FOC_ADC4_CMN (&g_stm32_foc_adccmn34)
#  endif
#endif

/* FOC ADC configuration ****************************************************/

#ifdef CONFIG_STM32_FOC_FOC0
#  ifdef CONFIG_STM32_FOC_FOC0_ADC1
#    define FOC0_ADC_IRQ STM32_IRQ_ADC1_FOC
#    define FOC0_ADC_CMN FOC_ADC1_CMN
#  endif
#  ifdef CONFIG_STM32_FOC_FOC0_ADC2
#    define FOC0_ADC_IRQ STM32_IRQ_ADC2_FOC
#    define FOC0_ADC_CMN FOC_ADC2_CMN
#  endif
#  ifdef CONFIG_STM32_FOC_FOC0_ADC3
#    define FOC0_ADC_IRQ STM32_IRQ_ADC3_FOC
#    define FOC0_ADC_CMN FOC_ADC3_CMN
#  endif
#  ifdef CONFIG_STM32_FOC_FOC0_ADC4
#    define FOC0_ADC_IRQ STM32_IRQ_ADC4_FOC
#    define FOC0_ADC_CMN FOC_ADC4_CMN
#  endif
#endif

#ifdef CONFIG_STM32_FOC_FOC1
#  ifdef CONFIG_STM32_FOC_FOC1_ADC1
#    define FOC1_ADC_IRQ STM32_IRQ_ADC1_FOC
#    define FOC1_ADC_CMN FOC_ADC1_CMN
#  endif
#  ifdef CONFIG_STM32_FOC_FOC1_ADC2
#    define FOC1_ADC_IRQ STM32_IRQ_ADC2_FOC
#    define FOC1_ADC_CMN FOC_ADC2_CMN
#  endif
#  ifdef CONFIG_STM32_FOC_FOC1_ADC3
#    define FOC1_ADC_IRQ STM32_IRQ_ADC3_FOC
#    define FOC1_ADC_CMN FOC_ADC3_CMN
#  endif
#  ifdef CONFIG_STM32_FOC_FOC1_ADC4
#    define FOC1_ADC_IRQ STM32_IRQ_ADC4_FOC
#    define FOC1_ADC_CMN FOC_ADC4_CMN
#  endif
#endif

/* Helper macros ************************************************************/

/* Get arch-specific FOC private part */

#define STM32_FOC_PRIV_FROM_DEV_GET(d)              \
  ((struct stm32_foc_priv_s *)(d)->lower->data)

/* Get board-specific FOC data */

#define STM32_FOC_BOARD_FROM_DEV_GET(d)         \
  ((STM32_FOC_PRIV_FROM_DEV_GET(d))->board)

/* Get arch-specific FOC devices */

#define STM32_FOC_DEV_FROM_DEV_GET(d)           \
  ((STM32_FOC_PRIV_FROM_DEV_GET(d))->dev)

/* Get PWM device */

#define PWM_FROM_FOC_DEV_GET(d) (STM32_FOC_DEV_FROM_DEV_GET(d)->pwm)

/* Get ADC device */

#define ADC_FROM_FOC_DEV_GET(d) (STM32_FOC_DEV_FROM_DEV_GET(d)->adc)

/* Define PWM all outputs */

#ifdef CONFIG_STM32_FOC_HAS_PWM_COMPLEMENTARY
#  define PMW_OUTPUTS_ALL_COMP (STM32_PWM_OUT1N|  \
                                STM32_PWM_OUT2N|  \
                                STM32_PWM_OUT3N)
#else
#  define PMW_OUTPUTS_ALL_COMP (0)
#endif

#if defined(CONFIG_STM32_FOC_ADC_CCR4) || (CONFIG_MOTOR_FOC_PHASES > 3)
#  define PMW_OUTPUTS_ALL_OUT4 (STM32_PWM_OUT4)
#else
#  define PMW_OUTPUTS_ALL_OUT4 (0)
#endif

#define PWM_OUTPUTS_ALL (STM32_PWM_OUT1|        \
                         STM32_PWM_OUT2|        \
                         STM32_PWM_OUT3|        \
                         PMW_OUTPUTS_ALL_COMP|  \
                         PMW_OUTPUTS_ALL_OUT4)

/* Enable all PWM outputs at once (include CHAN4 for ADC trigger) */

#define PWM_ALL_OUTPUTS_ENABLE(pwm, state)          \
  PWM_OUTPUTS_ENABLE(pwm, PWM_OUTPUTS_ALL, state);

/* Enable/disable ADC interrupts (FOC worker loop) */

#define STM32_ADC_ENABLEINT(adc)  STM32_ADC_INT_ENABLE(adc, FOC_ADC_IER_FOC)
#define STM32_ADC_DISABLEINT(adc) STM32_ADC_INT_DISABLE(adc, FOC_ADC_IER_FOC)

/* ADC calibration samples */

#define CAL_SAMPLES        (5000)

/* ADC calibration frequency */

#define CAL_FREQ           (10000)

/* Define PWM modes to control H-bridge.
 *
 * Any H-bridge specific configuration can be done with PWM_CHxPOL
 * and PWM_CHxIDLE configuration options
 */

#define PWM_MODE_FOC       STM32_CHANMODE_PWM1
#define PWM_MODE_ADC_TRG   STM32_CHANMODE_PWM1
#define PWM_MODE_HSLO_LSHI STM32_CHANMODE_OCREFHI
#define PWM_MODE_HSHI_LSLO STM32_CHANMODE_OCREFLO
#define PWM_MODE_HIZ       STM32_CHANMODE_FRZN

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* STM32 FOC devices.
 * This structure gathers all low level drivers required by FOC device.
 */

struct stm32_foc_dev_s
{
  uint8_t                     pwm_inst; /* PWM timer instance */
  uint8_t                     adc_inst; /* ADC timer instance */
  uint32_t                    pwm_base; /* PWM timer base */
  uint32_t                    adc_irq;  /* ADC irq */
  uint32_t                    jextval;  /* JEXT configuration */

  struct stm32_pwm_dev_s *pwm;      /* PWM device reference */
  struct adc_dev_s       *adc_dev;  /* ADC device reference */
  struct stm32_adc_dev_s *adc;      /* STM32 ADC device reference */

  /* Interrupt handler for FOC device */

  int (*adc_isr)(struct foc_dev_s *dev);
};

/* STM32 FOC common data */

struct stm32_foc_adccmn_s
{
  uint8_t       cntr; /* ADC common counter */
  mutex_t       lock; /* Lock data */
};

/* STM32 FOC volatile data */

struct stm32_foc_data_s
{
  foc_current_t curr[CONFIG_MOTOR_FOC_PHASES];        /* Current */
  uint8_t       notifier_div;                         /* FOC notifier prescaler */
  uint32_t      adc_freq;                             /* ADC interrupts frequency */
  uint32_t      per;                                  /* PWM timer period (ARR) */
  uint32_t      adcint_cntr;                          /* ADC interrupt counter */
  uint32_t      curr_offset[CONFIG_MOTOR_FOC_SHUNTS]; /* ADC current offset */
  int16_t       curr_raw[CONFIG_MOTOR_FOC_SHUNTS];    /* ADC current RAW */
};

/* STM32 FOC private */

struct stm32_foc_priv_s
{
  /* Volatile data */

  struct stm32_foc_data_s data;

  /* ADC calbration done */

  sem_t cal_done_sem;

  /* STM32 FOC devices */

  struct stm32_foc_dev_s *dev;

  /* Board-specific data */

  struct stm32_foc_board_s *board;

  /* Upper-half FOC controller callbacks */

  const struct foc_callbacks_s *cb;

#ifdef FOC_ADC_HAVE_CMN
  /* Common data */

  struct stm32_foc_adccmn_s *adc_cmn;
#endif
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* FOC lower-half operations */

static int stm32_foc_configure(struct foc_dev_s *dev,
                               struct foc_cfg_s *cfg);
static int stm32_foc_setup(struct foc_dev_s *dev);
static int stm32_foc_shutdown(struct foc_dev_s *dev);
static int stm32_foc_start(struct foc_dev_s *dev, bool state);
static int stm32_foc_pwm_duty_set(struct foc_dev_s *dev,
                                  foc_duty_t *duty);
static int stm32_foc_ioctl(struct foc_dev_s *dev, int cmd,
                           unsigned long arg);
static int stm32_foc_bind(struct foc_dev_s *dev,
                          struct foc_callbacks_s *cb);
static int stm32_foc_fault_clear(struct foc_dev_s *dev);
#ifdef CONFIG_MOTOR_FOC_TRACE
int stm32_foc_trace_init(struct foc_dev_s *dev);
void stm32_foc_trace(struct foc_dev_s *dev, int type, bool state);
#endif

/* ADC handlers */

static int stm32_foc_adc_handler(int irq, void *context, void *arg);
static int stm32_foc_adc_calibration_handler(struct foc_dev_s *dev);
static int stm32_foc_worker_handler(struct foc_dev_s *dev);

/* Helpers */

static void stm32_foc_curr_get(struct foc_dev_s *dev,
                               int16_t *curr, int shunts);
static int stm32_foc_notifier_cfg(struct foc_dev_s *dev, uint32_t freq);
static int stm32_foc_pwm_cfg(struct foc_dev_s *dev, uint32_t freq);
static int stm32_foc_adc_cfg(struct foc_dev_s *dev);
static int stm32_foc_pwm_start(struct foc_dev_s *dev, bool state);
static int stm32_foc_adc_start(struct foc_dev_s *dev, bool state);
static int stm32_foc_calibration_start(struct foc_dev_s *dev);
static int stm32_foc_pwm_freq_set(struct foc_dev_s *dev, uint32_t freq);

#if defined(CONFIG_STM32_FOC_ADC_CCR4)
static void stm32_foc_adc_ccr4_trg_set(struct foc_dev_s *dev,
                                       uint32_t offset);
#elif defined(CONFIG_STM32_FOC_ADC_TRGO)
static void stm32_foc_adc_trgo_trg_set(struct foc_dev_s *dev,
                                       uint8_t rcr);
#else
#  error Invalid FOC ADC trigger
#endif

static void stm32_foc_hw_config_get(struct foc_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef FOC_ADC_HAVE_CMN
#  ifdef CONFIG_STM32_HAVE_IP_ADC_V1
/* Common for ADC123 */

static struct stm32_foc_adccmn_s g_stm32_foc_adccmn123 =
{
  .cntr = 0,
  .lock = NXMUTEX_INITIALIZER,
};
#  endif  /* CONFIG_STM32_HAVE_IP_ADC_V1 */

#  ifdef CONFIG_STM32_HAVE_IP_ADC_V2
#    if defined(CONFIG_STM32_HAVE_ADC1) || defined(CONFIG_STM32_HAVE_ADC2)
/* Common for ADC12 */

static struct stm32_foc_adccmn_s g_stm32_foc_adccmn12 =
{
  .cntr = 0,
  .lock = NXMUTEX_INITIALIZER,
};
#    endif  /* CONFIG_STM32_HAVE_ADC1 || CONFIG_STM32_HAVE_ADC2 */
#    if defined(CONFIG_STM32_HAVE_ADC3) || defined(CONFIG_STM32_HAVE_ADC4)
/* Common for ADC34 */

static struct stm32_foc_adccmn_s g_stm32_foc_adccmn34 =
{
  .cntr = 0,
  .lock = NXMUTEX_INITIALIZER,
};
#    endif  /* CONFIG_STM32_HAVE_ADC3 || CONFIG_STM32_HAVE_ADC4 */
#  endif    /* CONFIG_STM32_HAVE_IP_ADC_V2 */
#endif  /* FOC_ADC_HAVE_CMN */

/* STM32 specific FOC data */

static struct stm32_foc_dev_s  g_stm32_foc_dev[CONFIG_MOTOR_FOC_INST];
static struct stm32_foc_priv_s g_stm32_foc_priv[CONFIG_MOTOR_FOC_INST];

/* STM32 specific FOC ops */

static struct foc_lower_ops_s g_stm32_foc_ops =
{
  .configure      = stm32_foc_configure,
  .setup          = stm32_foc_setup,
  .shutdown       = stm32_foc_shutdown,
  .start          = stm32_foc_start,
  .pwm_duty_set   = stm32_foc_pwm_duty_set,
  .ioctl          = stm32_foc_ioctl,
  .bind           = stm32_foc_bind,
  .fault_clear    = stm32_foc_fault_clear,
#ifdef CONFIG_MOTOR_FOC_TRACE
  .trace          = stm32_foc_trace
#endif
};

/* FOC lower-half */

static struct foc_lower_s g_stm32_foc_lower[CONFIG_MOTOR_FOC_INST];

/* FOC upper-half device data */

static struct foc_dev_s g_foc_dev[CONFIG_MOTOR_FOC_INST];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if (CONFIG_MOTOR_FOC_INST > 1)

/****************************************************************************
 * Name: stm32_foc_sync_all
 *
 * Description:
 *   Synchronise all FOC PWM timers
 *
 ****************************************************************************/

void stm32_foc_sync_all(void)
{
  struct foc_dev_s       *dev     = NULL;
  struct stm32_foc_dev_s *foc_dev = NULL;
  uint32_t                egr_reg[CONFIG_MOTOR_FOC_INST];
  int                     i       = 0;

  /* Get registers to write */

  for (i = 0; i < CONFIG_MOTOR_FOC_INST; i += 1)
    {
      /* Get FOC device */

      dev = &g_foc_dev[i];

      /* Get FOC lower half devices */

      foc_dev = STM32_FOC_DEV_FROM_DEV_GET(dev);

      /* Store EGR register address */

      egr_reg[i] = foc_dev->pwm_base + STM32_GTIM_EGR_OFFSET;
    }

  /* Write all registers at once */

  for (i = 0; i < CONFIG_MOTOR_FOC_INST; i += 1)
    {
      /* Force update event to reset CNTR */

      putreg32(GTIM_EGR_UG, egr_reg[i]);
    }
}
#endif

/****************************************************************************
 * Name: stm32_foc_pwm_cfg
 *
 * Description:
 *   PWM configuration for the FOC device
 *
 ****************************************************************************/

static int stm32_foc_pwm_cfg(struct foc_dev_s *dev, uint32_t freq)
{
  struct stm32_foc_board_s *board = STM32_FOC_BOARD_FROM_DEV_GET(dev);
  struct stm32_pwm_dev_s   *pwm   = PWM_FROM_FOC_DEV_GET(dev);
  int                       ret   = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(board);
  DEBUGASSERT(pwm);
  DEBUGASSERT(freq > 0);

  /* Set phases PWM frequency */

  ret = stm32_foc_pwm_freq_set(dev, freq);
  if (ret < 0)
    {
      goto errout;
    }

#ifdef CONFIG_STM32_FOC_HAS_PWM_COMPLEMENTARY
  /* Configure deadtime */

  PWM_DT_UPDATE(pwm, (uint8_t)board->data->pwm_dt);
#else
  UNUSED(board);
#endif

  /* Configure PWM mode for PWM outputs */

  PWM_MODE_UPDATE(pwm, STM32_PWM_CHAN1, PWM_MODE_FOC);
  PWM_MODE_UPDATE(pwm, STM32_PWM_CHAN2, PWM_MODE_FOC);
#if CONFIG_MOTOR_FOC_PHASES > 2
  PWM_MODE_UPDATE(pwm, STM32_PWM_CHAN3, PWM_MODE_FOC);
#endif
#if CONFIG_MOTOR_FOC_PHASES > 3
  PWM_MODE_UPDATE(pwm, STM32_PWM_CHAN4, PWM_MODE_FOC);
#endif

  /* Dump PWM regs */

  PWM_DUMP_REGS(pwm, NULL);

errout:
  return ret;
}

/****************************************************************************
 * Name: stm32_foc_pwm_freq_set
 *
 * Description:
 *   Configure the PWM frequency for the FOC device
 *
 ****************************************************************************/

static int stm32_foc_pwm_freq_set(struct foc_dev_s *dev, uint32_t freq)
{
  struct stm32_foc_priv_s *priv = STM32_FOC_PRIV_FROM_DEV_GET(dev);
  struct stm32_pwm_dev_s  *pwm  = PWM_FROM_FOC_DEV_GET(dev);
  int                      ret  = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);
  DEBUGASSERT(pwm);
  DEBUGASSERT(freq > 0);

  /* Update the PWM frequency.
   * IMPORTANT: must be x2 as the PWM is in center-aligned mode.
   */

  ret = PWM_FREQ_UPDATE(pwm, (freq * 2));
  if (ret < 0)
    {
      goto errout;
    }

  /* Store the PWM period to improve some future calculations */

  priv->data.per = PWM_ARR_GET(pwm);

errout:
  return ret;
}

/****************************************************************************
 * Name: stm32_foc_start
 *
 * Description:
 *   Start or stop the FOC lower-half operations
 *
 ****************************************************************************/

static int stm32_foc_start(struct foc_dev_s *dev, bool state)
{
  int ret = OK;

  DEBUGASSERT(dev);

  /* Start PWM */

  ret = stm32_foc_pwm_start(dev, state);
  if (ret < 0)
    {
      mtrerr("stm32_foc_pwm_start failed %d\n", ret);
      goto errout;
    }

  /* Start ADC */

  ret = stm32_foc_adc_start(dev, state);
  if (ret < 0)
    {
      mtrerr("stm32_foc_adc_start failed %d\n", ret);
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: stm32_foc_pwm_start
 *
 * Description:
 *   Start or stop PWM
 *
 ****************************************************************************/

static int stm32_foc_pwm_start(struct foc_dev_s *dev, bool state)
{
  struct stm32_foc_board_s *board = STM32_FOC_BOARD_FROM_DEV_GET(dev);
  struct stm32_pwm_dev_s   *pwm   = PWM_FROM_FOC_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(board);
  DEBUGASSERT(pwm);

  /* Configure outputs state */

  PWM_ALL_OUTPUTS_ENABLE(pwm, state);

  /* Call board-specific logic */

  board->ops->pwm_start(dev, state);

  return OK;
}

/****************************************************************************
 * Name: stm32_foc_adc_start
 *
 * Description:
 *   Start or stop ADC
 *
 ****************************************************************************/

static int stm32_foc_adc_start(struct foc_dev_s *dev, bool state)
{
  struct stm32_foc_dev_s *foc_dev = STM32_FOC_DEV_FROM_DEV_GET(dev);
  struct stm32_adc_dev_s *adc     = ADC_FROM_FOC_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(foc_dev);
  DEBUGASSERT(adc);

  if (state == false)
    {
      /* Disable ADC interrupts */

      STM32_ADC_DISABLEINT(adc);

      /* Disable ADC injected conversion */

      STM32_ADC_INJ_STARTCONV(adc, false);

      /* Reset ADC injected trigger */

      STM32_ADC_JEXTCFG_SET(adc, foc_dev->jextval);
    }
  else
    {
      /* Configure ADC injected trigger */

      STM32_ADC_JEXTCFG_SET(adc, foc_dev->jextval);

      /* Enable ADC interrupts */

      STM32_ADC_ENABLEINT(adc);

      /* Enable ADC injected conversion */

      STM32_ADC_INJ_STARTCONV(adc, true);
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_foc_adc_cfg
 *
 * Description:
 *   Configure ADC for FOC worker
 *
 ****************************************************************************/

static int stm32_foc_adc_cfg(struct foc_dev_s *dev)
{
  struct stm32_foc_dev_s *foc_dev = STM32_FOC_DEV_FROM_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(foc_dev);

  /* Set ADC interrupt handler to FOC worker */

  foc_dev->adc_isr = stm32_foc_worker_handler;

  return OK;
}

#if defined(CONFIG_STM32_FOC_ADC_CCR4)

/****************************************************************************
 * Name: stm32_foc_adc_ccr4_trg_set
 *
 * Description:
 *   Configure ADC CCR4 trigger for FOC controller
 *
 ****************************************************************************/

static void stm32_foc_adc_ccr4_trg_set(struct foc_dev_s *dev,
                                       uint32_t offset)
{
  struct stm32_pwm_dev_s *pwm = PWM_FROM_FOC_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(pwm);
  DEBUGASSERT(offset > 0);

  /* Configure PWM mode for ADC trigger
   * NOTE:
   *   For PWM mode 1 we have V7 when CRR=0 and V0 when CRR = ARR
   *   For PWM mode 2 we have V7 when CRR=ARR and V0 when CRR = 0
   */

  PWM_MODE_UPDATE(pwm, STM32_PWM_CHAN4, PWM_MODE_ADC_TRG);

  /* Set CCR4 */

  PWM_CCR_UPDATE(pwm, STM32_PWM_CHAN4, offset);
}

#elif defined(CONFIG_STM32_FOC_ADC_TRGO)

/****************************************************************************
 * Name: stm32_foc_adc_trgo_trg_set
 *
 * Description:
 *   Configure ADC TRGO trigger for FOC controller
 *
 ****************************************************************************/

static void stm32_foc_adc_trgo_trg_set(struct foc_dev_s *dev,
                                       uint8_t rcr)
{
  struct stm32_pwm_dev_s *pwm = PWM_FROM_FOC_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(pwm);

  /* We want TRGO on update events only if overflow (ARR):
   *   1. RCR must be configured when timer is enabled
   *   2. RCR must be odd value
   */

  if (rcr % 2 == 0)
    {
      rcr -= 1;
    }

  /* Configure RCR */

  PWM_RCR_UPDATE(pwm, rcr);

  /* Configure TRGO */

  PWM_TRGO_SET(pwm, FOC_PWM_TRGO);
}
#else
#  error Invalid FOC ADC trigger
#endif

/****************************************************************************
 * Name: stm32_foc_configure
 *
 * Description:
 *   Arch-specific FOC device configuration
 *
 ****************************************************************************/

static int stm32_foc_configure(struct foc_dev_s *dev,
                               struct foc_cfg_s *cfg)
{
  struct stm32_foc_priv_s *priv = STM32_FOC_PRIV_FROM_DEV_GET(dev);
  int                      ret  = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cfg);
  DEBUGASSERT(priv);
  DEBUGASSERT(cfg->pwm_freq > 0);
  DEBUGASSERT(cfg->notifier_freq > 0);

  /* Configure ADC */

  ret = stm32_foc_adc_cfg(dev);
  if (ret < 0)
    {
      mtrerr("stm32_foc_adc_cfg failed %d\n", ret);
      goto errout;
    }

  /* Configure PWM */

  ret = stm32_foc_pwm_cfg(dev, cfg->pwm_freq);
  if (ret < 0)
    {
      mtrerr("stm32_foc_pwm_cfg failed %d\n", ret);
      goto errout;
    }

  /* Configure FOC notifier */

  ret = stm32_foc_notifier_cfg(dev, cfg->notifier_freq);
  if (ret < 0)
    {
      mtrerr("stm32_foc_notifier_cfg failed %d\n", ret);
      goto errout;
    }

  /* Configure ADC trigger - must be after PWM frequency set */

  DEBUGASSERT(priv->data.per != 0);

#if defined(CONFIG_STM32_FOC_ADC_CCR4)
  stm32_foc_adc_ccr4_trg_set(dev, (priv->data.per - ADC_TRIGGER_OFFSET));
#elif defined(CONFIG_STM32_FOC_ADC_TRGO)
  stm32_foc_adc_trgo_trg_set(dev, (dev->cfg.pwm_freq /
                                   priv->data.adc_freq) * 2);
#else
#  error Invalid FOC ADC trigger
#endif

  /* Reset ADC interrupts counter */

  priv->data.adcint_cntr = 0;

  /* REVISIT: synchronise instances if TRGO trigger selected */

#if (CONFIG_MOTOR_FOC_INST > 1)
#  if defined(CONFIG_STM32_FOC_ADC_TRGO)
#    error stm32_foc_sync_all breaks TRGO event on V0 vector
#  endif

  /* Sync all FOC PWM timers instances.
   * IMPORTANT: This must be done after PWM frequency update !
   */

  stm32_foc_sync_all();
#endif

errout:
  return ret;
}

/****************************************************************************
 * Name: stm32_foc_setup
 *
 * Description:
 *   Arch-specific FOC device setup
 *
 ****************************************************************************/

static int stm32_foc_setup(struct foc_dev_s *dev)
{
  struct stm32_foc_dev_s   *foc_dev = STM32_FOC_DEV_FROM_DEV_GET(dev);
  struct stm32_foc_board_s *board   = STM32_FOC_BOARD_FROM_DEV_GET(dev);
  struct stm32_foc_priv_s  *priv    = STM32_FOC_PRIV_FROM_DEV_GET(dev);
  struct stm32_adc_dev_s   *adc     = ADC_FROM_FOC_DEV_GET(dev);
  struct adc_sample_time_s  stime;
  int                       ret     = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(foc_dev);
  DEBUGASSERT(board);
  DEBUGASSERT(priv);
  DEBUGASSERT(adc);
#ifdef FOC_ADC_HAVE_CMN
  DEBUGASSERT(priv->adc_cmn);
#endif

  /* Call board-specific setup - must be done before TIM enable */

  ret = board->ops->setup(dev);
  if (ret < 0)
    {
      mtrerr("board->setup failed %d\n", ret);
      goto errout;
    }

  /* Setup ADC */

  STM32_ADC_SETUP(foc_dev->adc);

#ifdef FOC_ADC_HAVE_CMN
  /* Lock ADC common data */

  ret = nxmutex_lock(&priv->adc_cmn->lock);
  if (ret < 0)
    {
      goto errout;
    }

  /* Only if first device */

  if (priv->adc_cmn->cntr == 0)
    {
      /* Enable ADC interrupts */

      up_enable_irq(foc_dev->adc_irq);
    }

  /* Increase counter */

  priv->adc_cmn->cntr += 1;

  /* Unlock ADC common data */

  nxmutex_unlock(&priv->adc_cmn->lock);
#endif

  /* Setup PWM */

  PWM_SETUP(foc_dev->pwm);
  PWM_TIM_ENABLE(foc_dev->pwm, true);

  /* Stop ADC and PWM */

  stm32_foc_pwm_start(dev, false);
  stm32_foc_adc_start(dev, false);

  /* Reset ADC handler */

  foc_dev->adc_isr = NULL;

  /* Configure sample times for ADC channels */

  memset(&stime, 0, sizeof(struct adc_sample_time_s));

  stime.channels_nbr = board->data->adc_cfg->nchan;
  stime.channel      = board->data->adc_cfg->stime;

  STM32_ADC_SAMPLETIME_SET(adc, &stime);
  STM32_ADC_SAMPLETIME_WRITE(adc);

  /* Set the priority of the ADC interrupt vector */

  ret = up_prioritize_irq(foc_dev->adc_irq, NVIC_SYSH_PRIORITY_DEFAULT);
  if (ret < 0)
    {
      mtrerr("up_prioritize_irq failed: %d\n", ret);
      goto errout;
    }

  /* Attach the ADC interrupt handler */

  ret = irq_attach(foc_dev->adc_irq, stm32_foc_adc_handler, NULL);
  if (ret < 0)
    {
      mtrerr("irq_attach failed: %d\n", ret);
      goto errout;
    }

  /* Get HW configuration */

  stm32_foc_hw_config_get(dev);

#ifdef CONFIG_MOTOR_FOC_TRACE
  /* Initialize trace interface */

  ret = stm32_foc_trace_init(dev);
  if (ret < 0)
    {
      mtrerr("stm32_foc_trace_init failed %d\n", ret);
      goto errout;
    }
#endif

  /* Start hardware calibration */

  ret = stm32_foc_calibration_start(dev);
  if (ret < 0)
    {
      mtrerr("stm32_foc_calibration_start failed %d\n", ret);
      goto errout;
    }

  /* Dump ADC regs */

  STM32_ADC_DUMP_REGS(adc);

errout:
  return ret;
}

/****************************************************************************
 * Name: stm32_foc_shutdown
 *
 * Description:
 *   Arch-specific FOC device shutdown
 *
 ****************************************************************************/

static int stm32_foc_shutdown(struct foc_dev_s *dev)
{
  struct stm32_foc_dev_s   *foc_dev = STM32_FOC_DEV_FROM_DEV_GET(dev);
  struct stm32_foc_board_s *board   = STM32_FOC_BOARD_FROM_DEV_GET(dev);
  struct stm32_foc_priv_s  *priv    = STM32_FOC_PRIV_FROM_DEV_GET(dev);
  int                       ret     = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(foc_dev);
  DEBUGASSERT(board);
  DEBUGASSERT(priv);

  /* Disable PWM */

  PWM_TIM_ENABLE(foc_dev->pwm, false);
  PWM_SHUTDOWN(foc_dev->pwm);

  /* Reset ADC interrupt handler */

  foc_dev->adc_isr = NULL;

  /* Deinitialize ADC */

  STM32_ADC_SHUTDOWN(foc_dev->adc);

#ifdef FOC_ADC_HAVE_CMN
  /* Lock ADC common data */

  ret = nxmutex_lock(&priv->adc_cmn->lock);
  if (ret < 0)
    {
      goto errout;
    }

  /* Decrease counter */

  priv->adc_cmn->cntr -= 1;

  /* Deinitialize ADC only if last device */

  if (priv->adc_cmn->cntr == 0)
#endif
    {
      /* Disable ADC interrupts */

      up_disable_irq(foc_dev->adc_irq);
    }

#ifdef FOC_ADC_HAVE_CMN
  /* Unlock ADC common data */

  nxmutex_unlock(&priv->adc_cmn->lock);
#endif

  /* Call board-specific shutdown */

  board->ops->shutdown(dev);

  /* Reset STM32 FOC volatile data */

  memset(&priv->data, 0, sizeof(struct stm32_foc_data_s));

#ifdef FOC_ADC_HAVE_CMN
errout:
#endif
  return ret;
}

/****************************************************************************
 * Name: stm32_foc_ioctl
 *
 * Description:
 *   Arch-specific FOC device IOCTL
 *
 ****************************************************************************/

static int stm32_foc_ioctl(struct foc_dev_s *dev, int cmd,
                           unsigned long arg)
{
  return -1;
}

/****************************************************************************
 * Name: stm32_foc_calibration_handler
 *
 * Description:
 *   ADC interrupt handler for FOC calibration
 *
 ****************************************************************************/

static int stm32_foc_adc_calibration_handler(struct foc_dev_s *dev)
{
  struct stm32_foc_priv_s *priv = STM32_FOC_PRIV_FROM_DEV_GET(dev);
  int                      i    = 0;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);

  if (priv->data.adcint_cntr < CAL_SAMPLES)
    {
      /* Get raw current samples */

      stm32_foc_curr_get(dev, priv->data.curr_raw, CONFIG_MOTOR_FOC_SHUNTS);

      /* Get sum */

      for (i = 0; i < CONFIG_MOTOR_FOC_SHUNTS; i += 1)
        {
          priv->data.curr_offset[i] += priv->data.curr_raw[i];
        }
    }

  else if (priv->data.adcint_cntr == CAL_SAMPLES)
    {
      /* Get average offset */

      for (i = 0; i < CONFIG_MOTOR_FOC_SHUNTS; i += 1)
        {
          priv->data.curr_offset[i] =
            (priv->data.curr_offset[i] / CAL_SAMPLES);
        }

      /* Post semaphore that calibration is done */

      nxsem_post(&priv->cal_done_sem);
    }
  else
    {
      /* Calibration completed */
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_foc_adc_handler
 *
 * Description:
 *   ADC interrupt handler
 *
 ****************************************************************************/

static int stm32_foc_adc_handler(int irq, void *context, void *arg)
{
  struct foc_dev_s         *dev     = NULL;
  struct stm32_foc_priv_s  *priv    = NULL;
#ifdef CONFIG_MOTOR_FOC_TRACE
  struct stm32_foc_board_s *board   = NULL;
#endif
  struct stm32_adc_dev_s   *adc     = NULL;
  struct stm32_foc_dev_s   *foc_dev = NULL;
  uint32_t                  pending = 0;
  int                       ret     = OK;
  int                       i       = 0;

  UNUSED(irq);
  UNUSED(context);
  UNUSED(arg);

  /* Loop through all FOC instances to prevent context switching if
   * all instances are synchronized.
   */

  for (i = 0; i < CONFIG_MOTOR_FOC_INST; i += 1)
    {
      /* Reset pointer to a device */

      dev = NULL;

      /* Get ADC device associated with FOC device */

      adc = ADC_FROM_FOC_DEV_GET(&g_foc_dev[i]);
      DEBUGASSERT(adc);

      /* Get ADC pending interrupts */

      pending = STM32_ADC_INT_GET(adc);

      /* Only if end of injected sequence */

      if (pending & FOC_ADC_ISR_FOC)
        {
          /* Found device with penidng ADC interrupt */

          dev = &g_foc_dev[i];
        }

      /* Handle pending interrupt for device */

      if (dev != NULL)
        {
          priv = STM32_FOC_PRIV_FROM_DEV_GET(dev);
          DEBUGASSERT(priv);

          foc_dev = STM32_FOC_DEV_FROM_DEV_GET(dev);
          DEBUGASSERT(foc_dev);

#ifdef CONFIG_MOTOR_FOC_TRACE
          board = STM32_FOC_BOARD_FROM_DEV_GET(dev);
          DEBUGASSERT(board);

          board->ops->trace(dev, FOC_TRACE_LOWER, true);
#endif
          /* Clear pending */

          STM32_ADC_INT_ACK(adc, pending);

          /* Call interrupt handler if registered */

          if (foc_dev->adc_isr != NULL)
            {
              ret = foc_dev->adc_isr(dev);
              if (ret < 0)
                {
                  DEBUGPANIC();
                }
            }

          /* Increase interrupt counter */

          priv->data.adcint_cntr += 1;

#ifdef CONFIG_MOTOR_FOC_TRACE
          board->ops->trace(dev, FOC_TRACE_LOWER, false);
#endif
        }
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_foc_worker_handler
 *
 * Description:
 *   Handle ADC conversion and do FOC device work.
 *
 ****************************************************************************/

static int stm32_foc_worker_handler(struct foc_dev_s *dev)
{
  struct stm32_foc_priv_s  *priv  = STM32_FOC_PRIV_FROM_DEV_GET(dev);
  struct stm32_foc_board_s *board = STM32_FOC_BOARD_FROM_DEV_GET(dev);
  struct stm32_adc_dev_s   *adc   = ADC_FROM_FOC_DEV_GET(dev);
  int                       ret   = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);
  DEBUGASSERT(adc);
  DEBUGASSERT(board);
  DEBUGASSERT(priv->cb);
  DEBUGASSERT(priv->cb->notifier);

  if (priv->data.adcint_cntr % priv->data.notifier_div == 0)
    {
      /* Get raw current samples */

      stm32_foc_curr_get(dev, priv->data.curr_raw, CONFIG_MOTOR_FOC_SHUNTS);

      /* Get phase currents */

      ret = board->ops->current_get(dev,
                                    priv->data.curr_raw,
                                    priv->data.curr);

      /* Call upper-half worker callback */

      priv->cb->notifier(dev, priv->data.curr);
    }

  return ret;
}

/****************************************************************************
 * Name: stm32_foc_calibration_start
 *
 * Description:
 *   Start FOC hardware calibration (ADC offsets)
 *
 ****************************************************************************/

static int stm32_foc_calibration_start(struct foc_dev_s *dev)
{
  struct stm32_foc_dev_s   *foc_dev = STM32_FOC_DEV_FROM_DEV_GET(dev);
  struct stm32_foc_priv_s  *priv    = STM32_FOC_PRIV_FROM_DEV_GET(dev);
  struct stm32_foc_board_s *board   = STM32_FOC_BOARD_FROM_DEV_GET(dev);
  struct stm32_pwm_dev_s   *pwm     = PWM_FROM_FOC_DEV_GET(dev);
  struct stm32_adc_dev_s   *adc     = ADC_FROM_FOC_DEV_GET(dev);
  uint8_t                   i       = 0;
  uint8_t                   ch      = 0;
  int                       ret     = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(foc_dev);
  DEBUGASSERT(priv);
  DEBUGASSERT(board);
  DEBUGASSERT(pwm);
  DEBUGASSERT(adc);

  /* Call board-specific */

  board->ops->calibration(dev, true);

  /* Force high side transistors to low state and
   * low side tranisstors to high state
   */

  PWM_MODE_UPDATE(pwm, STM32_PWM_CHAN1, PWM_MODE_HSLO_LSHI);
  PWM_MODE_UPDATE(pwm, STM32_PWM_CHAN2, PWM_MODE_HSLO_LSHI);
#if CONFIG_MOTOR_FOC_PHASES > 2
  PWM_MODE_UPDATE(pwm, STM32_PWM_CHAN3, PWM_MODE_HSLO_LSHI);
#endif
#if CONFIG_MOTOR_FOC_PHASES > 3
  PWM_MODE_UPDATE(pwm, STM32_PWM_CHAN4, PWM_MODE_HSLO_LSHI);
#endif

  /* Set PWM to trigger ADC */

  ret = stm32_foc_pwm_freq_set(dev, CAL_FREQ);
  if (ret < 0)
    {
      goto errout;
    }

  /* Configure ADC interrupt handler to calibration */

  foc_dev->adc_isr = stm32_foc_adc_calibration_handler;

  /* Configure ADC trigger - must be after PWM frequency set */

  DEBUGASSERT(priv->data.per != 0);

#if defined(CONFIG_STM32_FOC_ADC_CCR4)
  stm32_foc_adc_ccr4_trg_set(dev, (priv->data.per - ADC_TRIGGER_OFFSET));
#elif defined(CONFIG_STM32_FOC_ADC_TRGO)
  stm32_foc_adc_trgo_trg_set(dev, 1);
#else
#  error Invalid FOC ADC trigger
#endif

  /* Reset ADC interrupts counter */

  priv->data.adcint_cntr = 0;

  /* Start ADC and PWM */

  stm32_foc_adc_start(dev, true);
  stm32_foc_pwm_start(dev, true);

  /* Wait for calibration done semaphore
   * All work is done in adc_calibration_handler
   */

  ret = nxsem_wait_uninterruptible(&priv->cal_done_sem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Stop ADC and PWM */

  stm32_foc_pwm_start(dev, false);
  stm32_foc_adc_start(dev, false);

  /* Reset ADC interrupt handler */

  foc_dev->adc_isr = NULL;

  /* Clear last ADC data */

  for (i = 0; i < CONFIG_MOTOR_FOC_SHUNTS; i += 1)
    {
      priv->data.curr_raw[i] = 0;
    }

  /* Set ADC hardware offset for current channels (only injected channels) */

  for (i = 0; i < CONFIG_MOTOR_FOC_SHUNTS; i += 1)
    {
      /* Get channel */

      ch = board->data->adc_cfg->chan[board->data->adc_cfg->regch + i];

      /* Write offset */

      STM32_ADC_OFFSET_SET(adc, ch, i, priv->data.curr_offset[i]);
    }

  mtrinfo("ADC offset calibration - DONE!\n");

errout:

  /* Call board-specific */

  board->ops->calibration(dev, false);

  /* Reset ADC interrupts counter */

  priv->data.adcint_cntr = 0;

  return ret;
}

/****************************************************************************
 * Name: stm32_foc_pwm_duty_set
 *
 * Description:
 *   Set the 3-phase PWM duty cycle
 *
 ****************************************************************************/

static int stm32_foc_pwm_duty_set(struct foc_dev_s *dev,
                                  foc_duty_t *duty)
{
  struct stm32_foc_priv_s *priv    = STM32_FOC_PRIV_FROM_DEV_GET(dev);
  struct stm32_foc_dev_s  *foc_dev = STM32_FOC_DEV_FROM_DEV_GET(dev);
  uint16_t                 ccr[CONFIG_MOTOR_FOC_PHASES];

  DEBUGASSERT(dev);
  DEBUGASSERT(duty);
  DEBUGASSERT(priv);
  DEBUGASSERT(foc_dev);
  DEBUGASSERT(priv->data.per != 0);

  /* Get the CCR for a given duty cycle */

  DEBUGASSERT(duty[0] >= 0);
  DEBUGASSERT(duty[1] >= 0);
#if CONFIG_MOTOR_FOC_PHASES > 2
  DEBUGASSERT(duty[2] >= 0);
#endif
#if CONFIG_MOTOR_FOC_PHASES > 3
  DEBUGASSERT(duty[3] >= 0);
#endif

  ccr[0] = (uint16_t)b16toi(b16muli(duty[0], priv->data.per));
  ccr[1] = (uint16_t)b16toi(b16muli(duty[1], priv->data.per));
#if CONFIG_MOTOR_FOC_PHASES > 2
  ccr[2] = (uint16_t)b16toi(b16muli(duty[2], priv->data.per));
#endif
#if CONFIG_MOTOR_FOC_PHASES > 3
  ccr[3] = (uint16_t)b16toi(b16muli(duty[3], priv->data.per));
#endif

  /* Write directly to timer registers.
   * We are not using the PWM_CCR_UPDATE interface as it is too slow
   */

  putreg32(ccr[0], (foc_dev->pwm_base + STM32_GTIM_CCR1_OFFSET));
  putreg32(ccr[1], (foc_dev->pwm_base + STM32_GTIM_CCR2_OFFSET));
#if CONFIG_MOTOR_FOC_PHASES > 2
  putreg32(ccr[2], (foc_dev->pwm_base + STM32_GTIM_CCR3_OFFSET));
#endif
#if CONFIG_MOTOR_FOC_PHASES > 3
  putreg32(ccr[3], (foc_dev->pwm_base + STM32_GTIM_CCR4_OFFSET));
#endif

  return OK;
}

/****************************************************************************
 * Name: stm32_foc_hw_config_get
 *
 * Description:
 *   Get HW configuration for FOC device
 *
 ****************************************************************************/

static void stm32_foc_hw_config_get(struct foc_dev_s *dev)
{
  struct stm32_foc_board_s *board = STM32_FOC_BOARD_FROM_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(board);

  /* Get data from board configuration */

  dev->info.hw_cfg.pwm_dt_ns = board->data->pwm_dt_ns;
  dev->info.hw_cfg.pwm_max   = board->data->duty_max;
}

/****************************************************************************
 * Name: stm32_foc_curr_get
 *
 * Description:
 *   Get current samples from ADC
 *
 ****************************************************************************/

static void stm32_foc_curr_get(struct foc_dev_s *dev,
                               int16_t *curr, int shunts)
{
  struct stm32_foc_priv_s *priv = STM32_FOC_PRIV_FROM_DEV_GET(dev);
  struct stm32_adc_dev_s  *adc  = ADC_FROM_FOC_DEV_GET(dev);
  int                      i    = 0;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);
  DEBUGASSERT(adc);
  DEBUGASSERT(curr);

  for (i = 0; i < shunts; i += 1)
    {
      /* Get raw current samples.
       * We have ADC offset enabled for injected channels so this
       * gives us signed values.
       * NOTE: ADC value is 11 bits + sign.
       */

#ifdef CONFIG_STM32_FOC_G4_ADCCHAN0_WORKAROUND
      /* Ignore first channel */

      curr[i] = (int16_t)STM32_ADC_INJDATA_GET(adc, (i + 1));
#else
      curr[i] = (int16_t)STM32_ADC_INJDATA_GET(adc, i);
#endif
    }
}

/****************************************************************************
 * Name: stm32_foc_notifier_cfg
 *
 * Description:
 *   Configure FOC notifier
 *
 ****************************************************************************/

static int stm32_foc_notifier_cfg(struct foc_dev_s *dev, uint32_t freq)
{
  struct stm32_foc_priv_s *priv = STM32_FOC_PRIV_FROM_DEV_GET(dev);
  int                      ret  = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv);
  DEBUGASSERT(freq > 0);
  DEBUGASSERT(dev->cfg.pwm_freq > 0);

  /* Validate input:
   *   1. must be fraction of PWM frequency
   */

  if (dev->cfg.pwm_freq % freq != 0)
    {
      ret = -EINVAL;
      goto errout;
    }

#if defined(CONFIG_STM32_FOC_ADC_CCR4)
  /* ADC interrupts frequency is PWM frequency */

  priv->data.adc_freq = dev->cfg.pwm_freq;

  /* Get worker divider */

  priv->data.notifier_div = (dev->cfg.pwm_freq / freq);

#elif defined(CONFIG_STM32_FOC_ADC_TRGO)
  /* Call work on every ADC interrupt */

  priv->data.notifier_div = 1;

  /* ADC interrupts frequency is notifier frequency */

  priv->data.adc_freq = freq;
#else
#  error Invalid FOC ADC trigger
#endif

errout:
  return ret;
}

/****************************************************************************
 * Name: stm32_foc_bind
 *
 * Description:
 *   Bind lower-half FOC device with upper-half FOC logic
 *
 ****************************************************************************/

static int stm32_foc_bind(struct foc_dev_s *dev,
                          struct foc_callbacks_s *cb)
{
  struct stm32_foc_priv_s *priv = STM32_FOC_PRIV_FROM_DEV_GET(dev);
  int                      ret  = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cb);
  DEBUGASSERT(priv);

  /* Validate callbacks */

  DEBUGASSERT(cb->notifier);

  /* Bind upper-half FOC device callbacks */

  priv->cb = cb;
  return ret;
}

/****************************************************************************
 * Name: stm32_foc_fault_clear
 *
 * Description:
 *   Arch-specific fault clear
 *
 ****************************************************************************/

static int stm32_foc_fault_clear(struct foc_dev_s *dev)
{
  struct stm32_foc_board_s *board = STM32_FOC_BOARD_FROM_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(board);

  return board->ops->fault_clear(dev);
}

#ifdef CONFIG_MOTOR_FOC_TRACE

/****************************************************************************
 * Name: stm32_foc_trace
 *
 * Description:
 *   Arch-specific trace initialization
 *
 ****************************************************************************/

int stm32_foc_trace_init(struct foc_dev_s *dev)
{
  struct stm32_foc_board_s *board = STM32_FOC_BOARD_FROM_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(board);

  /* Call board-specific logic */

  return board->ops->trace_init(dev);
}

/****************************************************************************
 * Name: stm32_foc_trace
 *
 * Description:
 *   Arch-specific trace
 *
 ****************************************************************************/

void stm32_foc_trace(struct foc_dev_s *dev, int type, bool state)
{
  struct stm32_foc_board_s *board = STM32_FOC_BOARD_FROM_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(board);

  /* Call board-specific logic */

  board->ops->trace(dev, type, state);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_foc_initialize
 *
 * Description:
 *   Initialize the FOC lower-half.
 *
 * Input Parameters:
 *   inst  - FOC instance number
 *   board - FOC board-specific data
 *
 * Returned Value:
 *   Valid lower-half FOC controller structure reference on success;
 *   NULL on failure
 *
 ****************************************************************************/

struct foc_dev_s *
stm32_foc_initialize(int inst, struct stm32_foc_board_s *board)
{
  struct foc_dev_s          *dev       = NULL;
  struct stm32_foc_adc_s    *adc_cfg   = NULL;
  struct foc_lower_s        *foc_lower = NULL;
  struct stm32_foc_dev_s    *foc_dev   = NULL;
  struct stm32_foc_priv_s   *foc_priv  = NULL;
#ifdef FOC_ADC_HAVE_CMN
  struct stm32_foc_adccmn_s *adc_cmn   = NULL;
#endif
  uint32_t                   adc_irq   = 0;
  uint32_t                   pwm_base  = 0;
  uint32_t                   jextval   = 0;
  uint8_t                    pwm_inst  = 0;
  uint8_t                    adc_inst  = 0;
  uint32_t                   pwmfzbit  = 0;
  int                        i         = 0;
#ifdef CONFIG_STM32_FOC_G4_ADCCHAN0_WORKAROUND
  uint8_t                   *adc_chan  = NULL;
  uint8_t                    adc_nchan = 0;
#endif

  DEBUGASSERT(board != NULL);
  DEBUGASSERT(board->ops != NULL);
  DEBUGASSERT(board->data != NULL);

  /* Assert board-specific ops */

  DEBUGASSERT(board->ops->setup);
  DEBUGASSERT(board->ops->shutdown);
  DEBUGASSERT(board->ops->calibration);
  DEBUGASSERT(board->ops->fault_clear);
  DEBUGASSERT(board->ops->pwm_start);
  DEBUGASSERT(board->ops->current_get);
#ifdef CONFIG_MOTOR_FOC_TRACE
  DEBUGASSERT(board->ops->trace_init);
  DEBUGASSERT(board->ops->trace);
#endif

  /* Get ADC configuration from board data */

  adc_cfg = board->data->adc_cfg;
  DEBUGASSERT(adc_cfg);

  /* Get FOC instance configuration */

  switch (inst)
    {
#ifdef CONFIG_STM32_FOC_FOC0
      case 0:
        {
          pwm_inst = FOC0_PWM;
          adc_inst = FOC0_ADC;
          adc_irq  = FOC0_ADC_IRQ;
          pwm_base = FOC0_PWM_BASE;
          jextval  = FOC0_ADC_JEXT;
          pwmfzbit = FOC0_PWM_FZ_BIT;
#ifdef FOC_ADC_HAVE_CMN
          adc_cmn  = FOC0_ADC_CMN;
#endif
          break;
        }
#endif

#ifdef CONFIG_STM32_FOC_FOC1
      case 1:
        {
          pwm_inst = FOC1_PWM;
          adc_inst = FOC1_ADC;
          adc_irq  = FOC1_ADC_IRQ;
          pwm_base = FOC1_PWM_BASE;
          jextval  = FOC1_ADC_JEXT;
          pwmfzbit = FOC1_PWM_FZ_BIT;
#ifdef FOC_ADC_HAVE_CMN
          adc_cmn  = FOC1_ADC_CMN;
#endif
          break;
        }
#endif

      default:
        {
          mtrerr("Unsupported STM32 FOC instance %d\n", inst);
          set_errno(EINVAL);
          goto errout;
        }
    }

  /* Get STM32 FOC lower-half */

  foc_lower = &g_stm32_foc_lower[inst];

  /* Connect STM32 FOC private data with ops and data */

  foc_lower->data  = &g_stm32_foc_priv[inst];
  foc_lower->ops   = &g_stm32_foc_ops;
  foc_priv         = foc_lower->data;

  /* Reset STM32 FOC private data */

  memset(foc_lower->data, 0, sizeof(struct stm32_foc_priv_s));

  /* Connect STM32 FOC devices */

  foc_priv->dev = &g_stm32_foc_dev[inst];

  /* Connect board data */

  foc_priv->board = board;

#ifdef FOC_ADC_HAVE_CMN
  /* Connect ADC common data */

  foc_priv->adc_cmn = adc_cmn;
#endif

  /* Get arch-specific device */

  foc_dev = (struct stm32_foc_dev_s *)foc_priv->dev;
  DEBUGASSERT(foc_dev);

  /* Store STM32 FOC devices data */

  foc_dev->adc_inst = adc_inst;
  foc_dev->pwm_inst = pwm_inst;
  foc_dev->pwm_base = pwm_base;
  foc_dev->jextval  = jextval;
  foc_dev->adc_irq  = adc_irq;

  /* Get the advanced timer PWM interface */

  foc_dev->pwm = (struct stm32_pwm_dev_s *)stm32_pwminitialize(pwm_inst);
  if (foc_dev->pwm == NULL)
    {
      mtrerr("Failed to get PWM%d interface\n", pwm_inst);
      set_errno(EINVAL);
      goto errout;
    }

  /* Configure pins as analog inputs for the selected channels */

  DEBUGASSERT(adc_cfg != NULL);
  DEBUGASSERT(adc_cfg->pins != NULL);
  DEBUGASSERT(adc_cfg->chan != NULL);

  for (i = 0; i < adc_cfg->nchan; i++)
    {
      stm32_configgpio(adc_cfg->pins[i]);
    }

  /* Make sure that we are using the appropriate ADC interface */

  if (adc_inst != adc_cfg->intf)
    {
      mtrerr("Configuration doesn't match %d, %d\n",
             adc_inst, adc_cfg->intf);
      set_errno(EINVAL);
      goto errout;
    }

  /* STM32G4 ADC channel 0 unwanted conversion workaround */

#ifdef CONFIG_STM32_FOC_G4_ADCCHAN0_WORKAROUND
  /* Add one dummy channel to conversion */

  adc_nchan = (adc_cfg->nchan + 1);

  /* Allocate memory for the extended list of channels */

  adc_chan = zalloc(adc_nchan);
  if (adc_chan == NULL)
    {
      goto errout;
    }

  /* Copy regular channels first */

  for (i = 0; i < adc_cfg->regch; i += 1)
    {
      adc_chan[i] = adc_cfg->chan[i];
    }

  /* Add dummy channel at the beginning of injected channels */

  adc_chan[adc_cfg->regch] = 0;

  /* Copy injected channels */

  for (i = (adc_cfg->regch + 1); i < adc_nchan; i += 1)
    {
      adc_chan[i] = adc_cfg->chan[i - 1];
    }

#endif  /* CONFIG_STM32_FOC_G4_ADCCHAN0_WORKAROUND */

  /* Get the ADC interface */

#ifdef CONFIG_STM32_FOC_G4_ADCCHAN0_WORKAROUND
  foc_dev->adc_dev = stm32_adcinitialize(adc_inst,
                                         adc_chan,
                                         adc_nchan);

  free(adc_chan);
#else
  foc_dev->adc_dev = stm32_adcinitialize(adc_inst,
                                         adc_cfg->chan,
                                         adc_cfg->nchan);
#endif

  if (foc_dev->adc_dev == NULL)
    {
      mtrerr("Failed to get ADC%d interface\n", adc_cfg->intf);
      set_errno(EINVAL);
      goto errout;
    }

  /* Get ADC private part */

  foc_dev->adc = (struct stm32_adc_dev_s *)foc_dev->adc_dev->ad_priv;

  /* Froze timer and reset outputs when core is halted.
   * TODO: move this to stm32_pwm.c and configure from Kconfig
   */

  modifyreg32(FOC_PWM_FZ_REG, 0, pwmfzbit);

  /* Initialize calibration semaphore */

  nxsem_init(&foc_priv->cal_done_sem, 0, 0);

  /* Get FOC device */

  dev = &g_foc_dev[inst];

  /* Connect the lower-half device with the upper-half device */

  dev->lower = (void *)foc_lower;

  /* Return upper-half driver instance */

  return dev;

errout:
  return NULL;
}

/****************************************************************************
 * Name: stm32_foc_adcget
 *
 * Description:
 *   Get a handler for ADC device associated with a given FOC device.
 *
 *   The FOC lower-half logic uses only injected ADC channels for operations.
 *   We are using a custom ADC interrupt logic that cannot handle
 *   additional regular channels conversion. This limitation can be overcome
 *   with the DMA transfer.
 *   With this function we can get a handler to the ADC device and use it
 *   to register a standard ADC character device.
 *
 * Input Parameters:
 *   lower - a pointer to the uperr-half FOC device
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *stm32_foc_adcget(struct foc_dev_s *dev)
{
  struct stm32_foc_dev_s *foc_dev = STM32_FOC_DEV_FROM_DEV_GET(dev);

  DEBUGASSERT(dev);
  DEBUGASSERT(foc_dev);

  /* Return STM32 ADC device */

  return foc_dev->adc_dev;
}
