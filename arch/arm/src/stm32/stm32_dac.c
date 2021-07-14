/****************************************************************************
 * arch/arm/src/stm32/stm32_dac.c
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
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/analog/dac.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "stm32.h"
#include "stm32_dac.h"
#include "stm32_rcc.h"
#include "stm32_dma.h"
#include "stm32_syscfg.h"

#ifdef CONFIG_DAC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RCC reset ****************************************************************/

#if defined(HAVE_IP_DAC_V1)
#  define STM32_RCC_RSTR     STM32_RCC_APB1RSTR
#  define RCC_RSTR_DAC1RST   RCC_APB1RSTR_DAC1RST
#  define RCC_RSTR_DAC2RST   RCC_APB1RSTR_DAC2RST
#elif defined(HAVE_IP_DAC_V2)
#  define STM32_RCC_RSTR     STM32_RCC_AHB2RSTR
#  define RCC_RSTR_DAC1RST   RCC_AHB2RSTR_DAC1RST
#  define RCC_RSTR_DAC2RST   RCC_AHB2RSTR_DAC2RST
#  define RCC_RSTR_DAC3RST   RCC_AHB2RSTR_DAC3RST
#  define RCC_RSTR_DAC4RST   RCC_AHB2RSTR_DAC4RST
#endif

/* Configuration ************************************************************/

/* Up to 2 DAC interfaces for up to 3 channels are supported
 *
 * NOTE: STM32_NDAC tells how many channels chip supports.
 *       ST is not consistent in the naming of DAC interfaces, so we
 *       introduce our own naming convention. We distinguish DAC1 and DAC2
 *       only if the chip has two separate areas in memory map to support DAC
 *       channels.
 */

#if STM32_NDAC < 3
#  warning
#  undef CONFIG_STM32_DAC2CH1
#  undef CONFIG_STM32_DAC2CH1_DMA
#  undef CONFIG_STM32_DAC2CH1_TIMER
#  undef CONFIG_STM32_DAC2CH1_TIMER_FREQUENCY
#endif

#if STM32_NDAC < 2
#  warning
#  undef CONFIG_STM32_DAC1CH2
#  undef CONFIG_STM32_DAC1CH2_DMA
#  undef CONFIG_STM32_DAC1CH2_TIMER
#  undef CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY
#endif

#if STM32_NDAC < 1
#  warning
#  undef CONFIG_STM32_DAC1CH1
#  undef CONFIG_STM32_DAC1CH1_DMA
#  undef CONFIG_STM32_DAC1CH1_TIMER
#  undef CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY
#endif

#if defined(CONFIG_STM32_DAC1) || defined(CONFIG_STM32_DAC2)

/* Sanity checking */

#ifdef CONFIG_STM32_DAC1
#  if !defined(CONFIG_STM32_DAC1CH1) && !defined(CONFIG_STM32_DAC1CH2)
#    error "DAC1 enabled but no channel was selected"
#  endif
#endif

#ifdef CONFIG_STM32_DAC2
#  if !defined(CONFIG_STM32_DAC2CH1)
#    error "DAC2 enabled but no channel was selected"
#  endif
#endif

/* DMA configuration. */

#if defined(CONFIG_STM32_DAC1CH1_DMA) || defined(CONFIG_STM32_DAC1CH2_DMA) || \
    defined(CONFIG_STM32_DAC2CH1_DMA)
# if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX)
#   ifndef CONFIG_STM32_DMA2
#     warning "STM32 F1/F3 DAC DMA support requires CONFIG_STM32_DMA2"
#     undef CONFIG_STM32_DAC1CH1_DMA
#     undef CONFIG_STM32_DAC1CH2_DMA
#     undef CONFIG_STM32_DAC2CH1_DMA
#   endif
# elif defined(CONFIG_STM32_STM32F33XX)
#   ifndef CONFIG_STM32_DMA1
#     warning "STM32 F334 DAC DMA support requires CONFIG_STM32_DMA1"
#     undef CONFIG_STM32_DAC1CH1_DMA
#     undef CONFIG_STM32_DAC1CH2_DMA
#     undef CONFIG_STM32_DAC2CH1_DMA
#   endif
# elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#   ifndef CONFIG_STM32_DMA1
#     warning "STM32 F4 DAC DMA support requires CONFIG_STM32_DMA1"
#     undef CONFIG_STM32_DAC1CH1_DMA
#     undef CONFIG_STM32_DAC1CH2_DMA
#     undef CONFIG_STM32_DAC2CH1_DMA
#   endif
# else
#   warning "No DAC DMA information for this STM32 family"
#   undef CONFIG_STM32_DAC1CH1_DMA
#   undef CONFIG_STM32_DAC1CH2_DMA
#   undef CONFIG_STM32_DAC2CH1_DMA
# endif
#endif

#if defined(CONFIG_STM32_DAC1CH1_HRTIM_TRG1) || defined(CONFIG_STM32_DAC1CH1_HRTIM_TRG2)
#  define DAC1CH1_HRTIM
#endif
#if defined(CONFIG_STM32_DAC1CH2_HRTIM_TRG1) || defined(CONFIG_STM32_DAC1CH2_HRTIM_TRG2)
#  define DAC1CH2_HRTIM
#endif
#if defined(CONFIG_STM32_DAC2CH1_HRTIM_TRG3)
#  define DAC2CH1_HRTIM
#endif

/* If DMA is selected, then a timer and output frequency must also be
 * provided to support the DMA transfer.  The DMA transfer could be
 * supported by and EXTI trigger, but this feature is not currently
 * supported by the driver.
 */

#if defined(CONFIG_STM32_DAC1CH1_DMA) && !defined(DAC1CH1_HRTIM) && \
    !defined(CONFIG_STM32_DAC1CH1_DMA_EXTERNAL)
#  if !defined(CONFIG_STM32_DAC1CH1_TIMER)
#    warning "A timer number must be specified in CONFIG_STM32_DAC1CH1_TIMER"
#    undef CONFIG_STM32_DAC1CH1_DMA
#    undef CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY
#  elif !defined(CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY)
#    warning "A timer frequency must be specified in CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY"
#    undef CONFIG_STM32_DAC1CH1_DMA
#    undef CONFIG_STM32_DAC1CH1_TIMER
#  endif
#endif

#if defined(CONFIG_STM32_DAC1CH2_DMA) && !defined(DAC1CH2_HRTIM) && \
    !defined(CONFIG_STM32_DAC1CH2_DMA_EXTERNAL)
#  if !defined(CONFIG_STM32_DAC1CH2_TIMER)
#    warning "A timer number must be specified in CONFIG_STM32_DAC1CH2_TIMER"
#    undef CONFIG_STM32_DAC1CH2_DMA
#    undef CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY
#  elif !defined(CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY)
#    warning "A timer frequency must be specified in CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY"
#    undef CONFIG_STM32_DAC1CH2_DMA
#    undef CONFIG_STM32_DAC1CH2_TIMER
#  endif
#endif

#if defined(CONFIG_STM32_DAC2CH1_DMA) && !defined(DAC2CH1_HRTIM) && \
    !defined(CONFIG_STM32_DAC2CH1_DMA_EXTERNAL)
#  if !defined(CONFIG_STM32_DAC2CH1_TIMER)
#    warning "A timer number must be specified in CONFIG_STM32_DAC2CH1_TIMER"
#    undef CONFIG_STM32_DAC2CH1_DMA
#    undef CONFIG_STM32_DAC2CH1_TIMER_FREQUENCY
#  elif !defined(CONFIG_STM32_DAC2CH1_TIMER_FREQUENCY)
#    warning "A timer frequency must be specified in CONFIG_STM32_DAC2CH1_TIMER_FREQUENCY"
#    undef CONFIG_STM32_DAC2CH1_DMA
#    undef CONFIG_STM32_DAC2CH1_TIMER
#  endif
#endif

/* DMA **********************************************************************/

/* DMA channels and interface values differ for the F1 and F4 families */

#undef HAVE_DMA
#if defined(CONFIG_STM32_DAC1CH1_DMA) || defined(CONFIG_STM32_DAC1CH2_DMA) || \
    defined(CONFIG_STM32_DAC2CH1_DMA)
# if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F30XX) || \
     defined(CONFIG_STM32_STM32F33XX)
#  define HAVE_DMA        1
#  define DAC_DMA         2
#  if defined(CONFIG_STM32_DAC1CH1) && !defined(CONFIG_STM32_DAC1CH1_DMA_EXTERNAL)
#    define DAC1CH1_DMA_CHAN   DMACHAN_DAC1_CH1
#  endif
#  if defined(CONFIG_STM32_DAC1CH2) && !defined(CONFIG_STM32_DAC1CH2_DMA_EXTERNAL)
#    define DAC1CH2_DMA_CHAN   DMACHAN_DAC1_CH2
#  endif
#  if defined(CONFIG_STM32_DAC2CH1) && !defined(CONFIG_STM32_DAC2CH1_DMA_EXTERNAL)
#    define DAC2CH1_DMA_CHAN   DMACHAN_DAC2_CH1
#  endif
# elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define HAVE_DMA        1
#  define DAC_DMA         1
#  if defined(CONFIG_STM32_DAC1CH1) && !defined(CONFIG_STM32_DAC1CH1_DMA_EXTERNAL)
#    define DAC1CH1_DMA_CHAN   DMAMAP_DAC1
#  endif
#  if defined(CONFIG_STM32_DAC1CH2) && !defined(CONFIG_STM32_DAC1CH2_DMA_EXTERNAL)
#    define DAC1CH2_DMA_CHAN   DMAMAP_DAC1
#  endif
#  if defined(CONFIG_STM32_DAC2CH1) && !defined(CONFIG_STM32_DAC2CH1_DMA_EXTERNAL)
#    define DAC2CH1_DMA_CHAN   DMAMAP_DAC2
#  endif
# endif
#endif

/* Timer configuration.  The STM32 supports 8 different trigger for DAC
 * output:
 *
 * TSEL SOURCE                  DEVICES
 * ---- ----------------------- -------------------------------------
 * 000  Timer 6 TRGO event      ALL
 * 001  Timer 3 TRGO event      STM32 F1 Connectivity Line and STM32 F3
 *      Timer 8 TRGO event      Other STM32 F1 and all STM32 F4
 * 010  Timer 7 TRGO event      ALL
 * 011  Timer 5 TRGO event      ALL
 *      Timer 15 TRGO event     STM32 F3
 *      HRTIM1_DACTRG1 event    STM32F33XX (DAC1 only)
 * 100  Timer 2 TRGO event      ALL
 * 101  Timer 4 TRGO event      ALL
 *      HRTIM1_DACTRG2 event    STM32F33XX (DAC1 only)
 *      HRTIM1_DACTRG3 event    STM32F33XX (DAC2 only)
 * 110  EXTI line9              ALL
 * 111  SWTRIG Software control ALL
 *
 * This driver does not support the EXTI trigger.
 */

/* DMA transfer from DMA buffer to DAC register can also be triggered by an
 * external to the DAC block events. In this case, the DAC trigger (TEN bit)
 * must be reset and board configuration must provide DACxCHy_DMA_CHAN.
 */

#undef NEED_TIM6
#undef NEED_TIM3
#undef NEED_TIM8
#undef NEED_TIM7
#undef NEED_TIM5
#undef NEED_TIM2
#undef NEED_TIM4

#ifdef CONFIG_STM32_DAC1CH1_DMA
#  if defined(CONFIG_STM32_DAC1CH1_DMA_EXTERNAL)
#  elif defined(CONFIG_STM32_DAC1CH1_HRTIM_TRG1)
#    ifndef CONFIG_STM32_HRTIM_DAC
#      error  "CONFIG_STM32_HRTIM_DAC required for DAC1CH1"
#    endif
#    define DAC1CH1_TSEL_VALUE           DAC_CR_TSEL_HRT1TRG1
#  elif defined(CONFIG_STM32_DAC1CH1_HRTIM_TRG2)
#    ifndef CONFIG_STM32_HRTIM_DAC
#      error  "CONFIG_STM32_HRTIM_DAC required for DAC1CH2"
#    endif
#    define DAC1CH1_TSEL_VALUE           DAC_CR_TSEL_HRT1TRG2
#  elif CONFIG_STM32_DAC1CH1_TIMER == 6
#    ifndef CONFIG_STM32_TIM6_DAC
#      error "CONFIG_STM32_TIM6_DAC required for DAC1CH1"
#    endif
#    define NEED_TIM6
#    define DAC1CH1_TSEL_VALUE           DAC_CR_TSEL_TIM6
#    define DAC1CH1_TIMER_BASE           STM32_TIM6_BASE
#    define DAC1CH1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1CH1_TIMER == 3 && defined(CONFIG_STM32_CONNECTIVITYLINE)
#    ifndef CONFIG_STM32_TIM3_DAC
#      error "CONFIG_STM32_TIM3_DAC required for DAC1CH1"
#    endif
#    define NEED_TIM3
#    define DAC1CH1_TSEL_VALUE           DAC_CR_TSEL_TIM3
#    define DAC1CH1_TIMER_BASE           STM32_TIM3_BASE
#    define DAC1CH1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1CH1_TIMER == 8 && !defined(CONFIG_STM32_CONNECTIVITYLINE)
#    ifndef CONFIG_STM32_TIM8_DAC
#      error "CONFIG_STM32_TIM8_DAC required for DAC1CH1"
#    endif
#    define NEED_TIM8
#    define DAC1CH1_TSEL_VALUE           DAC_CR_TSEL_TIM8
#    define DAC1CH1_TIMER_BASE           STM32_TIM8_BASE
#    define DAC1CH1_TIMER_PCLK_FREQUENCY STM32_PCLK2_FREQUENCY
#  elif CONFIG_STM32_DAC1CH1_TIMER == 7
#    ifndef CONFIG_STM32_TIM7_DAC
#      error "CONFIG_STM32_TIM7_DAC required for DAC1CH1"
#    endif
#    define NEED_TIM7
#    define DAC1CH1_TSEL_VALUE DAC_CR_TSEL_TIM7
#    define DAC1CH1_TIMER_BASE STM32_TIM7_BASE
#  elif CONFIG_STM32_DAC1CH1_TIMER == 5
#    ifndef CONFIG_STM32_TIM5_DAC
#      error "CONFIG_STM32_TIM5_DAC required for DAC1CH1"
#    endif
#    define NEED_TIM5
#    define DAC1CH1_TSEL_VALUE           DAC_CR_TSEL_TIM5
#    define DAC1CH1_TIMER_BASE           STM32_TIM5_BASE
#    define DAC1CH1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1CH1_TIMER == 2
#    ifndef CONFIG_STM32_TIM2_DAC
#      error "CONFIG_STM32_TIM2_DAC required for DAC1CH1"
#    endif
#    define NEED_TIM2
#    define DAC1CH1_TSEL_VALUE           DAC_CR_TSEL_TIM2
#    define DAC1CH1_TIMER_BASE           STM32_TIM2_BASE
#    define DAC1CH1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1CH1_TIMER == 4
#    ifndef CONFIG_STM32_TIM4_DAC
#      error "CONFIG_STM32_TIM4_DAC required for DAC1CH1"
#    endif
#    define NEED_TIM4
#    define DAC1CH1_TSEL_VALUE           DAC_CR_TSEL_TIM4
#    define DAC1CH1_TIMER_BASE           STM32_TIM4_BASE
#    define DAC1CH1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  else
#    error "Unsupported CONFIG_STM32_DAC1CH1_TIMER"
#  endif
#else
#  define DAC1CH1_TSEL_VALUE DAC_CR_TSEL_SW
#endif

#if defined(NEED_TIM2) || defined(NEED_TIM3) || defined(NEED_TIM4) || \
    defined(NEED_TIM5) || defined(NEED_TIM6) || defined(NEED_TIM7) || \
    defined(NEED_TIM8)
#  define HAVE_TIMER
#endif

#ifdef CONFIG_STM32_DAC1CH2_DMA
#  if defined(CONFIG_STM32_DAC1CH2_DMA_EXTERNAL)
#  elif defined(CONFIG_STM32_DAC1CH2_HRTIM_TRG1)
#    ifndef CONFIG_STM32_HRTIM_DAC
#      error  "CONFIG_STM32_HRTIM_DAC required for DAC1CH2"
#    endif
#    define DAC1CH2_TSEL_VALUE           DAC_CR_TSEL_HRT1TRG1
#  elif defined(CONFIG_STM32_DAC1CH2_HRTIM_TRG2)
#    ifndef CONFIG_STM32_HRTIM_DAC
#      error  "CONFIG_STM32_HRTIM_DAC required for DAC1CH2"
#    endif
#    define DAC1CH2_TSEL_VALUE           DAC_CR_TSEL_HRT1TRG2
#  elif CONFIG_STM32_DAC1CH2_TIMER == 6
#    ifndef CONFIG_STM32_TIM6_DAC
#      error "CONFIG_STM32_TIM6_DAC required for DAC1CH2"
#    endif
#    define DAC1CH2_TSEL_VALUE           DAC_CR_TSEL_TIM6
#    define DAC1CH2_TIMER_BASE           STM32_TIM6_BASE
#    define DAC1CH2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1CH2_TIMER == 3 && defined(CONFIG_STM32_CONNECTIVITYLINE)
#    ifndef CONFIG_STM32_TIM3_DAC
#      error "CONFIG_STM32_TIM3_DAC required for DAC1CH2"
#    endif
#    define DAC1CH2_TSEL_VALUE           DAC_CR_TSEL_TIM3
#    define DAC1CH2_TIMER_BASE           STM32_TIM3_BASE
#    define DAC1CH2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1CH2_TIMER == 8 && !defined(CONFIG_STM32_CONNECTIVITYLINE)
#    ifndef CONFIG_STM32_TIM8_DAC
#      error "CONFIG_STM32_TIM8_DAC required for DAC1CH2"
#    endif
#    define DAC1CH2_TSEL_VALUE           DAC_CR_TSEL_TIM8
#    define DAC1CH2_TIMER_BASE           STM32_TIM8_BASE
#    define DAC1CH2_TIMER_PCLK_FREQUENCY STM32_PCLK2_FREQUENCY
#  elif CONFIG_STM32_DAC1CH2_TIMER == 7
#    ifndef CONFIG_STM32_TIM7_DAC
#      error "CONFIG_STM32_TIM7_DAC required for DAC1CH2"
#    endif
#    define DAC1CH2_TSEL_VALUE           DAC_CR_TSEL_TIM7
#    define DAC1CH2_TIMER_BASE           STM32_TIM7_BASE
#    define DAC1CH2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1CH2_TIMER == 5
#    ifndef CONFIG_STM32_TIM5_DAC
#      error "CONFIG_STM32_TIM5_DAC required for DAC1CH2"
#    endif
#    define DAC1CH2_TSEL_VALUE           DAC_CR_TSEL_TIM5
#    define DAC1CH2_TIMER_BASE           STM32_TIM5_BASE
#    define DAC1CH2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1CH2_TIMER == 2
#    ifndef CONFIG_STM32_TIM2_DAC
#      error "CONFIG_STM32_TIM2_DAC required for DAC1CH2"
#    endif
#    define DAC1CH2_TSEL_VALUE           DAC_CR_TSEL_TIM2
#    define DAC1CH2_TIMER_BASE           STM32_TIM2_BASE
#    define DAC1CH2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC1CH2_TIMER == 4
#    ifndef CONFIG_STM32_TIM4_DAC
#      error "CONFIG_STM32_TIM4_DAC required for DAC1CH2"
#    endif
#    define DAC1CH2_TSEL_VALUE           DAC_CR_TSEL_TIM4
#    define DAC1CH2_TIMER_BASE           STM32_TIM4_BASE
#    define DAC1CH2_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  else
#    error "Unsupported CONFIG_STM32_DAC1CH2_TIMER"
#  endif
#else
#  define DAC1CH2_TSEL_VALUE DAC_CR_TSEL_SW
#endif

#ifdef CONFIG_STM32_DAC2CH1_DMA
#  if defined(CONFIG_STM32_DAC2CH1_DMA_EXTERNAL)
#  elif defined(CONFIG_STM32_DAC2CH1_HRTIM_TRG3)
#    ifndef CONFIG_STM32_HRTIM_DAC
#      error  "CONFIG_STM32_HRTIM_DAC required for DAC2CH1"
#    endif
#    define DAC2CH1_TSEL_VALUE           DAC_CR_TSEL_HRT1TRG3
#  elif CONFIG_STM32_DAC2CH1_TIMER == 6
#    ifndef CONFIG_STM32_TIM6_DAC
#      error "CONFIG_STM32_TIM6_DAC required for DAC2CH1"
#    endif
#    define DAC2CH1_TSEL_VALUE           DAC_CR_TSEL_TIM6
#    define DAC2CH1_TIMER_BASE           STM32_TIM6_BASE
#    define DAC2CH1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC2CH1_TIMER == 3 && defined(CONFIG_STM32_CONNECTIVITYLINE)
#    ifndef CONFIG_STM32_TIM3_DAC
#      error "CONFIG_STM32_TIM3_DAC required for DAC2CH1"
#    endif
#    define DAC2CH1_TSEL_VALUE           DAC_CR_TSEL_TIM3
#    define DAC2CH1_TIMER_BASE           STM32_TIM3_BASE
#    define DAC2CH1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC2CH1_TIMER == 8 && !defined(CONFIG_STM32_CONNECTIVITYLINE)
#    ifndef CONFIG_STM32_TIM8_DAC
#      error "CONFIG_STM32_TIM8_DAC required for DAC2CH1"
#    endif
#    define DAC2CH1_TSEL_VALUE           DAC_CR_TSEL_TIM8
#    define DAC2CH1_TIMER_BASE           STM32_TIM8_BASE
#    define DAC2CH1_TIMER_PCLK_FREQUENCY STM32_PCLK2_FREQUENCY
#  elif CONFIG_STM32_DAC2CH1_TIMER == 7
#    ifndef CONFIG_STM32_TIM7_DAC
#      error "CONFIG_STM32_TIM7_DAC required for DAC2CH1"
#    endif
#    define DAC2CH1_TSEL_VALUE           DAC_CR_TSEL_TIM7
#    define DAC2CH1_TIMER_BASE           STM32_TIM7_BASE
#    define DAC2CH1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC2CH1_TIMER == 5
#    ifndef CONFIG_STM32_TIM5_DAC
#      error "CONFIG_STM32_TIM5_DAC required for DAC2CH1"
#    endif
#    define DAC2CH1_TSEL_VALUE           DAC_CR_TSEL_TIM5
#    define DAC2CH1_TIMER_BASE           STM32_TIM5_BASE
#    define DAC2CH1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC2CH1_TIMER == 2
#    ifndef CONFIG_STM32_TIM2_DAC
#      error "CONFIG_STM32_TIM2_DAC required for DAC2CH1"
#    endif
#    define DAC2CH1_TSEL_VALUE           DAC_CR_TSEL_TIM2
#    define DAC2CH1_TIMER_BASE           STM32_TIM2_BASE
#    define DAC2CH1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  elif CONFIG_STM32_DAC2CH1_TIMER == 4
#    ifndef CONFIG_STM32_TIM4_DAC
#      error "CONFIG_STM32_TIM4_DAC required for DAC2CH1"
#    endif
#    define DAC2CH1_TSEL_VALUE           DAC_CR_TSEL_TIM4
#    define DAC2CH1_TIMER_BASE           STM32_TIM4_BASE
#    define DAC2CH1_TIMER_PCLK_FREQUENCY STM32_PCLK1_FREQUENCY
#  else
#    error "Unsupported CONFIG_STM32_DAC2CH1_TIMER"
#  endif
#else
#  define DAC2CH1_TSEL_VALUE DAC_CR_TSEL_SW
#endif

/* We need index which describes when HRTIM is selected as trigger.
 * It will be used to skip timer configuration where needed.
 */

#define TIM_INDEX_HRTIM 255

#if defined(DAC1CH1_HRTIM) || defined(DAC1CH2_HRTIM) || defined(DAC2CH1_HRTIM)
#  define HAVE_HRTIM
#endif

/* DMA buffers default size */

#if !defined(CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE) && defined(CONFIG_STM32_DAC1CH1_DMA)
#  error "DAC1CH1 buffer size must be provided"
#endif
#if !defined(CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE) && defined(CONFIG_STM32_DAC1CH2_DMA)
#  error "DAC1CH2 buffer size must be provided"
#endif
#if !defined(CONFIG_STM32_DAC2CH1_DMA_BUFFER_SIZE) && defined(CONFIG_STM32_DAC2CH1_DMA)
#  error "DAC2CH1 buffer size must be provided"
#endif

/* Calculate timer divider values based upon DACn_TIMER_PCLK_FREQUENCY and
 * CONFIG_STM32_DACn_TIMER_FREQUENCY.
 */

#warning "Missing Logic"

/* DMA stream/channel configuration */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define DAC_DMA_CONTROL_WORD (DMA_SCR_MSIZE_16BITS | \
                                DMA_SCR_PSIZE_16BITS | \
                                DMA_SCR_MINC | \
                                DMA_SCR_CIRC | \
                                DMA_SCR_DIR_M2P)
#else
#  define DAC_DMA_CONTROL_WORD (DMA_CCR_MSIZE_16BITS | \
                                DMA_CCR_PSIZE_16BITS | \
                                DMA_CCR_MINC | \
                                DMA_CCR_CIRC | \
                                DMA_CCR_DIR)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure represents the internal state of the single STM32 DAC
 * block
 */

struct stm32_dac_s
{
  uint8_t    init   : 1; /* True, the DAC block has been initialized */
};

/* This structure represents the internal state of one STM32 DAC channel */

struct stm32_chan_s
{
  uint8_t    inuse  : 1; /* True, the driver is in use and not available */
#ifdef HAVE_DMA
  uint8_t    hasdma : 1; /* True, this channel supports DMA */
  uint8_t    text   : 1; /* True, DMA triggering from external source */
  uint8_t    timer;      /* Timer number 2-8 */
#endif
  uint8_t    intf;       /* DAC zero-based interface number (0 or 1) */
  uint32_t   pin;        /* Pin configuration */
  uint32_t   dro;        /* Data output register */
  uint32_t   cr;         /* Control register */
  uint32_t   tsel;       /* CR trigger select value */
#ifdef HAVE_IP_DAC_V2
  uint32_t   sr;         /* Status register */
  uint32_t   mcr;        /* Mode Control register */
#endif
#ifdef HAVE_DMA
  uint16_t   dmachan;    /* DMA channel needed by this DAC */
  uint16_t   buffer_len; /* DMA buffer length */
  DMA_HANDLE dma;        /* Allocated DMA channel */
#  ifdef HAVE_TIMER
  uint32_t   tbase;      /* Timer base address */
  uint32_t   tfrequency; /* Timer frequency */
#  endif
  uint16_t   *dmabuffer; /* DMA transfer buffer */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* DAC Register access */

#ifdef HAVE_TIMER
static uint32_t tim_getreg(FAR struct stm32_chan_s *chan, int offset);
static void     tim_putreg(FAR struct stm32_chan_s *chan, int offset,
                           uint32_t value);
static void     tim_modifyreg(FAR struct stm32_chan_s *chan, int offset,
                              uint32_t clearbits, uint32_t setbits);
#endif

/* Interrupt handler */

#if 0 /* defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX) */
static int  dac_interrupt(int irq, FAR void *context, FAR void *arg);
#endif

/* DAC methods */

static void dac_reset(FAR struct dac_dev_s *dev);
static int  dac_setup(FAR struct dac_dev_s *dev);
static void dac_shutdown(FAR struct dac_dev_s *dev);
static void dac_txint(FAR struct dac_dev_s *dev, bool enable);
static int  dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg);
static int  dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg);

/* Initialization */

#ifdef HAVE_DMA
#  ifdef HAVE_TIMER
static int  dac_timinit(FAR struct stm32_chan_s *chan);
#  endif
static int  dma_remap(FAR struct stm32_chan_s *chan);
static void dma_bufferinit(FAR struct stm32_chan_s *chan, uint16_t *buffer,
                           uint16_t len);
#endif
static int  dac_chaninit(FAR struct stm32_chan_s *chan);
static int  dac_blockinit(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct dac_ops_s g_dacops =
{
  .ao_reset    = dac_reset,
  .ao_setup    = dac_setup,
  .ao_shutdown = dac_shutdown,
  .ao_txint    = dac_txint,
  .ao_send     = dac_send,
  .ao_ioctl    = dac_ioctl,
};

#ifdef CONFIG_STM32_DAC1
#ifdef CONFIG_STM32_DAC1CH1
/* Channel 1: DAC1 channel 1 */

#ifdef CONFIG_STM32_DAC1CH1_DMA
uint16_t   dac1ch1_buffer[CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE];
#endif

static struct stm32_chan_s g_dac1ch1priv =
{
  .intf       = 0,
  .pin        = GPIO_DAC1_OUT1,
  .dro        = STM32_DAC1_DHR12R1,
  .cr         = STM32_DAC1_CR,
#ifdef HAVE_IP_DAC_V2
  .sr         = STM32_DAC1_SR,
  .mcr        = STM32_DAC1_MCR,
#endif
#ifdef CONFIG_STM32_DAC1CH1_DMA
  .hasdma     = 1,
  .dmachan    = DAC1CH1_DMA_CHAN,
  .buffer_len = CONFIG_STM32_DAC1CH1_DMA_BUFFER_SIZE,
  .dmabuffer  = dac1ch1_buffer,
#  ifdef CONFIG_STM32_DAC1CH1_DMA_EXTERNAL
  .text       = 1,
#  else
  .text       = 0,
  .tsel       = DAC1CH1_TSEL_VALUE,
#    ifdef DAC1CH1_HRTIM
  .timer      = TIM_INDEX_HRTIM,
#    else
  .timer      = CONFIG_STM32_DAC1CH1_TIMER,
  .tbase      = DAC1CH1_TIMER_BASE,
  .tfrequency = CONFIG_STM32_DAC1CH1_TIMER_FREQUENCY,
#    endif
#  endif
#endif
};

static struct dac_dev_s g_dac1ch1dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1ch1priv,
};
#endif /* CONFIG_STM32_DAC1CH1 */

#ifdef CONFIG_STM32_DAC1CH2
/* Channel 2: DAC1 channel 2 */

#ifdef CONFIG_STM32_DAC1CH2_DMA
uint16_t   dac1ch2_buffer[CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE];
#endif

static struct stm32_chan_s g_dac1ch2priv =
{
  .intf       = 1,
  .pin        = GPIO_DAC1_OUT2,
  .dro        = STM32_DAC1_DHR12R2,
  .cr         = STM32_DAC1_CR,
#ifdef HAVE_IP_DAC_V2
  .sr         = STM32_DAC1_SR,
  .mcr        = STM32_DAC1_MCR,
#endif
#ifdef CONFIG_STM32_DAC1CH2_DMA
  .hasdma     = 1,
  .dmachan    = DAC1CH2_DMA_CHAN,
  .buffer_len = CONFIG_STM32_DAC1CH2_DMA_BUFFER_SIZE,
  .dmabuffer  = dac1ch2_buffer,
#  ifdef CONFIG_STM32_DAC1CH2_DMA_EXTERNAL
  .text       = 1,
#  else
  .text       = 0,
  .tsel       = DAC1CH2_TSEL_VALUE,
#    ifdef DAC1CH2_HRTIM
  .timer      = TIM_INDEX_HRTIM,
#    else
  .timer      = CONFIG_STM32_DAC1CH2_TIMER,
  .tbase      = DAC1CH2_TIMER_BASE,
  .tfrequency = CONFIG_STM32_DAC1CH2_TIMER_FREQUENCY,
#    endif
#  endif
#endif
};

static struct dac_dev_s g_dac1ch2dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac1ch2priv,
};
#endif /* CONFIG_STM32_DAC1CH2 */

#endif /* CONFIG_STM32_DAC1 */

#ifdef CONFIG_STM32_DAC2
#ifdef CONFIG_STM32_DAC2CH1
/* Channel 3: DAC2 channel 1 */

#ifdef CONFIG_STM32_DAC2CH1_DMA
uint16_t   dac2ch1_buffer[CONFIG_STM32_DAC2CH1_DMA_BUFFER_SIZE];
#endif

static struct stm32_chan_s g_dac2ch1priv =
{
  .intf       = 2,
  .pin        = GPIO_DAC2_OUT1,
  .dro        = STM32_DAC2_DHR12R1,
  .cr         = STM32_DAC2_CR,
#ifdef HAVE_IP_DAC_V2
  .sr         = STM32_DAC2_SR,
  .mcr        = STM32_DAC2_MCR,
#endif
#ifdef CONFIG_STM32_DAC2CH1_DMA
  .hasdma     = 1,
  .dmachan    = DAC2CH1_DMA_CHAN,
  .buffer_len = CONFIG_STM32_DAC2CH1_DMA_BUFFER_SIZE,
  .dmabuffer  = dac2ch1_buffer,
#  ifdef CONFIG_STM32_DAC2CH1_DMA_EXTERNAL
  .text       = 1,
#  else
  .text       = 0,
  .tsel       = DAC2CH1_TSEL_VALUE,
#    ifdef DAC2CH1_HRTIM
  .timer      = TIM_INDEX_HRTIM,
#    else
  .timer      = CONFIG_STM32_DAC2CH1_TIMER,
  .tbase      = DAC2CH1_TIMER_BASE,
  .tfrequency = CONFIG_STM32_DAC2CH1_TIMER_FREQUENCY,
#    endif
#  endif
#endif
};

static struct dac_dev_s g_dac2ch1dev =
{
  .ad_ops  = &g_dacops,
  .ad_priv = &g_dac2ch1priv,
};
#endif /* CONFIG_STM32_DAC2CH1 */
#endif /* CONFIG_STM32_DAC2 */

static struct stm32_dac_s g_dacblock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dac_modify_cr
 *
 * Description:
 *   Modify the contents of the DAC control register.
 *
 * Input Parameters:
 *   priv - Driver state instance
 *   clearbits - Bits in the control register to be cleared
 *   setbits - Bits in the control register to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_dac_modify_cr(FAR struct stm32_chan_s *chan,
                                       uint32_t clearbits, uint32_t setbits)
{
  unsigned int shift;

  /* DAC1 channels 1 and 2 share the STM32_DAC[1]_CR control register.  DAC2
   * channel 1 (and perhaps channel 2) uses the STM32_DAC2_CR control
   * register.  In either case, bit 0 of the interface number provides the
   * correct shift.
   *
   *   Bit 0 = 0: Shift = 0
   *   Bit 0 = 1: Shift = 16
   */

  shift = (chan->intf & 1) << 4;
  modifyreg32(chan->cr, clearbits << shift, setbits << shift);
}

#ifdef HAVE_TIMER

/****************************************************************************
 * Name: tim_getreg
 *
 * Description:
 *   Read the value of an DMA timer register.
 *
 * Input Parameters:
 *   chan - A reference to the DAC block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t tim_getreg(FAR struct stm32_chan_s *chan, int offset)
{
  return getreg32(chan->tbase + offset);
}

/****************************************************************************
 * Name: tim_putreg
 *
 * Description:
 *   Read the value of an DMA timer register.
 *
 * Input Parameters:
 *   chan - A reference to the DAC block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tim_putreg(FAR struct stm32_chan_s *chan, int offset,
                       uint32_t value)
{
  putreg32(value, chan->tbase + offset);
}

/****************************************************************************
 * Name: tim_modifyreg
 *
 * Description:
 *   Modify the value of an DMA timer register.
 *
 * Input Parameters:
 *   priv - Driver state instance
 *   offset - The timer register offset
 *   clearbits - Bits in the control register to be cleared
 *   setbits - Bits in the control register to be set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void tim_modifyreg(FAR struct stm32_chan_s *chan, int offset,
                          uint32_t clearbits, uint32_t setbits)
{
  modifyreg32(chan->tbase + offset, clearbits, setbits);
}
#endif /* HAVE_TIMER */

/****************************************************************************
 * Name: dac_interrupt
 *
 * Description:
 *   DAC interrupt handler.  The STM32 F4 family supports a only a DAC
 *   underrun interrupt.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   OK
 *
 ****************************************************************************/

#if 0 /* defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX) */
static int dac_interrupt(int irq, FAR void *context, FAR void *arg)
{
#warning "Missing logic"
  return OK;
}
#endif

/****************************************************************************
 * Name: dac_reset
 *
 * Description:
 *   Reset the DAC channel.  Called early to initialize the hardware. This
 *   is called, before dac_setup() and on error conditions.
 *
 *   NOTE:  DAC reset will reset both DAC channels!
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_reset(FAR struct dac_dev_s *dev)
{
  irqstate_t flags;

  /* Reset only the selected DAC channel; the other DAC channel must remain
   * functional.
   */

  flags   = enter_critical_section();

#warning "Missing logic"

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: dac_setup
 *
 * Description:
 *   Configure the DAC. This method is called the first time that the DAC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching DAC interrupts.
 *   Interrupts are all disabled upon return.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_setup(FAR struct dac_dev_s *dev)
{
#warning "Missing logic"
  return OK;
}

/****************************************************************************
 * Name: dac_shutdown
 *
 * Description:
 *   Disable the DAC.  This method is called when the DAC device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_shutdown(FAR struct dac_dev_s *dev)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: dac_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dac_txint(FAR struct dac_dev_s *dev, bool enable)
{
#warning "Missing logic"
}

/****************************************************************************
 * Name: dac_dmatxcallback
 *
 * Description:
 *   DMA callback function.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef HAVE_DMA
static void dac_dmatxcallback(DMA_HANDLE handle, uint8_t isr, FAR void *arg)
{
}
#endif

/****************************************************************************
 * Name: dac_send
 *
 * Description:
 *   Set the DAC output.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_send(FAR struct dac_dev_s *dev, FAR struct dac_msg_s *msg)
{
  FAR struct stm32_chan_s *chan = dev->ad_priv;

  /* Enable DAC Channel */

#if defined(CONFIG_STM32_STM32F33XX)
  /* For STM32F33XX we have to set BOFF/OUTEN bit for DAC1CH2 and DAC2CH1
   * REVISIT: what if we connect DAC internally with comparator ?
   */

  if (chan->intf > 0)
    {
      stm32_dac_modify_cr(chan, 0, DAC_CR_EN | DAC_CR_BOFF);
    }
  else
#endif
    {
      stm32_dac_modify_cr(chan, 0, DAC_CR_EN);
    }

#if defined(HAVE_IP_DAC_V2)
  /* Check channelx ready status bit */

  uint32_t regval;
  uint32_t dac = (chan->intf >> 1);
  do
    {
      regval = getreg32(chan->sr);
    }
  while (!(regval & DAC_SR_DACRDY(dac + 1)));
#endif

#ifdef HAVE_DMA
  if (chan->hasdma)
    {
      /* Configure the DMA stream/channel.
       *
       * - Channel number
       * - Peripheral address
       * - Direction: Memory to peripheral
       * - Disable peripheral address increment
       * - Enable memory address increment
       * - Peripheral data size: half word
       * - Mode: circular???
       * - Priority: ?
       * - FIFO mode: disable
       * - FIFO threshold: half full
       * - Memory Burst: single
       * - Peripheral Burst: single
       */

      stm32_dmasetup(chan->dma, chan->dro, (uint32_t)chan->dmabuffer,
                     chan->buffer_len, DAC_DMA_CONTROL_WORD);

      /* Enable DMA */

      stm32_dmastart(chan->dma, dac_dmatxcallback, chan, false);

      /* Enable DMA for DAC Channel */

      stm32_dac_modify_cr(chan, 0, DAC_CR_DMAEN);
    }
  else
#endif
    {
      /* Non-DMA transfer */

#if defined(HAVE_IP_DAC_V1)
      putreg16(msg->am_data, chan->dro);
#else
      putreg32(msg->am_data, chan->dro);
#endif
      dac_txdone(dev);
    }

  /* Reset counters (generate an update). Only when timer is not HRTIM */

#ifdef HAVE_TIMER
  if (chan->timer != TIM_INDEX_HRTIM)
    {
      tim_modifyreg(chan, STM32_GTIM_EGR_OFFSET, 0, GTIM_EGR_UG);
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: dac_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_ioctl(FAR struct dac_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct stm32_chan_s *chan = dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
#ifdef HAVE_DMA
      case IO_DMABUFFER_INIT:
        {
          uint16_t *buffer = (uint16_t *)arg;

          /* The caller is responsible for providing buffer with
           * suitable length equal to CONFIG_STM32_DACxCHy_DMA_BUFFER_SIZE
           */

          dma_bufferinit(chan, buffer, chan->buffer_len * sizeof(buffer));
          break;
        }
#endif

      default:
        {
          aerr("ERROR: Unknown cmd: %d\n", cmd);
          ret = -ENOTTY;
          break;
        }
    }

  UNUSED(chan);
  return ret;
}

#ifdef HAVE_DMA

/****************************************************************************
 * Name: dma_bufferinit
 ****************************************************************************/

static void dma_bufferinit(FAR struct stm32_chan_s *chan, uint16_t *buffer,
                           uint16_t len)
{
  memcpy(chan->dmabuffer, buffer, len);
}

/****************************************************************************
 * Name: dma_remap
 ****************************************************************************/

static int dma_remap(FAR struct stm32_chan_s *chan)
{
#if defined(CONFIG_STM32_STM32F33XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined(CONFIG_STM32_STM32F37XX)
  uint32_t regval = 0;

  switch (chan->intf)
    {
      case 0:
        {
          /* Remap DMA1CH3 to DAC1CH1 */

          regval |= SYSCFG_CFGR1_DAC1CH1_DMARMP;

         /* Remap DAC trigger for STM32F33XX if needed */

#  ifdef CONFIG_STM32_STM32F33XX
#    if defined(CONFIG_STM32_DAC1CH1_HRTIM_TRG1)
          modifyreg32(STM32_SYSCFG_CFGR3, 0, SYSCFG_CFGR3_DAC1_TRIG3_RMP);
#    elif defined(CONFIG_STM32_DAC1CH1_HRTIM_TRG2)
          modifyreg32(STM32_SYSCFG_CFGR3, 0, SYSCFG_CFGR3_DAC1_TRIG5_RMP);
#    endif
#  endif
          break;
        }

      case 1:
        {
          /* Remap DMA1CH4 to DAC1CH2 */

          regval |= SYSCFG_CFGR1_DAC1CH2_DMARMP;

          /* Remap DAC trigger for STM32F33XX if needed */

#  ifdef CONFIG_STM32_STM32F33XX
#    if defined(CONFIG_STM32_DAC1CH2_HRTIM_TRG1)
          modifyreg32(STM32_SYSCFG_CFGR3, 0, SYSCFG_CFGR3_DAC1_TRIG3_RMP);
#    elif defined(CONFIG_STM32_DAC1CH2_HRTIM_TRG2)
          modifyreg32(STM32_SYSCFG_CFGR3, 0, SYSCFG_CFGR3_DAC1_TRIG5_RMP);
#    endif
#  endif
          break;
        }

      case 2:
        {
          /* Remap DMA1CH5 to DAC2CH1 */

          regval |= SYSCFG_CFGR1_DAC2CH1_DMARMP;
          break;
        }

      default:
        {
          return -EINVAL;
        }
    }

  modifyreg32(STM32_SYSCFG_BASE, 0, regval);

#endif

  return OK;
}

/****************************************************************************
 * Name: dac_timinit
 *
 * Description:
 *   Initialize the timer that drivers the DAC DMA for this channel using
 *   the pre-calculated timer divider definitions.
 *
 * Input Parameters:
 *   chan - A reference to the DAC channel state data
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef HAVE_TIMER
static int dac_timinit(FAR struct stm32_chan_s *chan)
{
  uint32_t pclk;
  uint32_t prescaler;
  uint32_t timclk;
  uint32_t reload;
  uint32_t regaddr;
  uint32_t setbits;

  /* Configure the time base: Timer period, prescaler, clock division,
   * counter mode (up).
   */

  /* Enable the timer.  At most, two of the following cases (pulse the
   * default) will be enabled
   */

  regaddr = STM32_RCC_APB1ENR;

  switch (chan->timer)
    {
#ifdef NEED_TIM2
      case 2:
        setbits = RCC_APB1ENR_TIM2EN;
        pclk    = BOARD_TIM2_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM3
      case 3:
        setbits = RCC_APB1ENR_TIM3EN;
        pclk    = BOARD_TIM3_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM4
      case 4:
        setbits = RCC_APB1ENR_TIM4EN;
        pclk    = BOARD_TIM4_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM5
      case 5:
        setbits = RCC_APB1ENR_TIM5EN;
        pclk    = BOARD_TIM5_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM6
      case 6:
        setbits = RCC_APB1ENR_TIM6EN;
        pclk    = BOARD_TIM6_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM7
      case 7:
        setbits = RCC_APB1ENR_TIM7EN;
        pclk    = BOARD_TIM7_FREQUENCY;
        break;
#endif
#ifdef NEED_TIM8
      case 8:
        regaddr = STM32_RCC_APB2ENR;
        setbits = RCC_APB2ENR_TIM8EN;
        pclk    = BOARD_TIM8_FREQUENCY;
        break;
#endif
      default:
        aerr("ERROR: Could not enable timer\n");
        return -EINVAL;
    }

  /* Enable the timer. */

  modifyreg32(regaddr, 0, setbits);

  /* Calculate optimal values for the timer prescaler and for the timer
   * reload register.  If 'frequency' is the desired frequency, then
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

  prescaler = (pclk / chan->tfrequency + 65534) / 65535;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > 65536)
    {
      prescaler = 65536;
    }

  timclk = pclk / prescaler;

  reload = timclk / chan->tfrequency;
  if (reload < 1)
    {
      reload = 1;
    }
  else if (reload > 65535)
    {
      reload = 65535;
    }

  /* Set the reload and prescaler values */

  tim_putreg(chan, STM32_GTIM_ARR_OFFSET, (uint16_t)reload);
  tim_putreg(chan, STM32_GTIM_PSC_OFFSET, (uint16_t)(prescaler - 1));

  /* Count mode up, auto reload */

  tim_modifyreg(chan, STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_ARPE);

  /* Selection TRGO selection: update */

  tim_modifyreg(chan, STM32_GTIM_CR2_OFFSET, GTIM_CR2_MMS_MASK,
                GTIM_CR2_MMS_UPDATE);

  /* Update DMA request enable ???? */
#if 0
  tim_modifyreg(chan, STM32_GTIM_DIER_OFFSET, 0, GTIM_DIER_UDE);
#endif

  /* Enable the counter */

  tim_modifyreg(chan, STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);
  return OK;
}
#endif
#endif /* HAVE_DMA */

/****************************************************************************
 * Name: dac_chaninit
 *
 * Description:
 *   Initialize the DAC channel.
 *
 * Input Parameters:
 *   chan - A reference to the DAC channel state data
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_chaninit(FAR struct stm32_chan_s *chan)
{
  uint16_t clearbits;
  uint16_t setbits;
#ifdef HAVE_TIMER
  int ret;
#endif

  /* Is the selected channel already in-use? */

  if (chan->inuse)
    {
      /* Yes.. then return EBUSY */

      return -EBUSY;
    }

  /* Configure the DAC output pin:
   *
   * DAC -" Once the DAC channelx is enabled, the corresponding GPIO pin
   * (PA4 or PA5) is automatically connected to the analog converter output
   * (DAC_OUTx). In order to avoid parasitic consumption, the PA4 or PA5 pin
   * should first be configured to analog (AIN)".
   */

  stm32_configgpio(chan->pin);

  /* DAC channel configuration:
   *
   * - Set the trigger selection based upon the configuration.
   * - Set wave generation == None.
   * - Enable the output buffer.
   */

  /* Disable before change */

  stm32_dac_modify_cr(chan, DAC_CR_EN, 0);

  clearbits = DAC_CR_TSEL_MASK |
              DAC_CR_MAMP_MASK |
              DAC_CR_WAVE_MASK;
#if defined (HAVE_IP_DAC_V1)
  clearbits |= DAC_CR_BOFF;
#endif

  setbits =
      chan->tsel |           /* Set trigger source (SW or timer TRGO event) */
      DAC_CR_MAMP_AMP1 |     /* Set waveform characteristics */
      DAC_CR_WAVE_DISABLED;  /* Set wave generation disabled */
#if defined (HAVE_IP_DAC_V1)
  setbits |= DAC_CR_BOFF_EN; /* Enable output buffer */
#endif

  stm32_dac_modify_cr(chan, clearbits, setbits);

#if defined(HAVE_IP_DAC_V2)
  /* High frequency interface mode selection */

  uint32_t regval;
  if (STM32_SYSCLK_FREQUENCY > 160000000)
    {
      regval = DAC_MCR_HFSEL_AHB_160MHz;
    }
  else if (STM32_SYSCLK_FREQUENCY > 80000000)
    {
      regval = DAC_MCR_HFSEL_AHB_80MHz;
    }
  else
    {
      regval = DAC_MCR_HFSEL_DISABLED;
    }

  putreg32(regval, STM32_DAC1_MCR);
#endif

#ifdef HAVE_DMA
  /* Determine if DMA is supported by this channel */

  if (chan->hasdma)
    {
      /* Remap DMA request if necessary */

      dma_remap(chan);

      /* DAC trigger enable if not external triggering */

      if (!chan->text)
        {
          stm32_dac_modify_cr(chan, 0, DAC_CR_TEN);
        }

      /* Allocate a DMA channel */

      chan->dma = stm32_dmachannel(chan->dmachan);
      if (!chan->dma)
        {
          aerr("ERROR: Failed to allocate a DMA channel\n");
          return -EBUSY;
        }

      /* Configure the timer that supports the DMA operation
       * Do nothing if HRTIM is selected as trigger.
       * All necessary configuration is done in the HRTIM driver.
       */

#ifdef HAVE_TIMER
      if (chan->timer != TIM_INDEX_HRTIM)
        {
          ret = dac_timinit(chan);
          if (ret < 0)
            {
              aerr("ERROR: Failed to initialize the DMA timer: %d\n", ret);
              return ret;
            }
        }
#endif
    }
#endif

  /* Mark the DAC channel "in-use" */

  chan->inuse = 1;
  return OK;
}

/****************************************************************************
 * Name: dac_blockinit
 *
 * Description:
 *   Initialize the DAC block.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int dac_blockinit(void)
{
  irqstate_t flags;
  uint32_t regval;

  /* Has the DAC block already been initialized? */

  if (g_dacblock.init)
    {
      /* Yes.. then return success  We only have to do this once */

      return OK;
    }

  /* Put the entire DAC block in reset state */

  flags   = enter_critical_section();
  regval  = getreg32(STM32_RCC_RSTR);
#ifdef CONFIG_STM32_DAC1
  regval |= RCC_RSTR_DAC1RST;
#endif
#ifdef CONFIG_STM32_DAC2
  regval |= RCC_RSTR_DAC2RST;
#endif
  putreg32(regval, STM32_RCC_RSTR);

  /* Take the DAC out of reset state */

#ifdef CONFIG_STM32_DAC1
  regval &= ~RCC_RSTR_DAC1RST;
#endif
#ifdef CONFIG_STM32_DAC2
  regval &= ~RCC_RSTR_DAC2RST;
#endif
  putreg32(regval, STM32_RCC_RSTR);
  leave_critical_section(flags);

  /* Mark the DAC block as initialized */

  g_dacblock.init = 1;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dacinitialize
 *
 * Description:
 *   Initialize the DAC.
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *
 * Returned Value:
 *   Valid DAC device structure reference on success; a NULL on failure.
 *
 * Assumptions:
 *   1. Clock to the DAC block has enabled,
 *   2. Board-specific logic has already configured
 *
 ****************************************************************************/

FAR struct dac_dev_s *stm32_dacinitialize(int intf)
{
  FAR struct dac_dev_s    *dev;
  FAR struct stm32_chan_s *chan;
  int ret;

#ifdef CONFIG_STM32_DAC1CH1
  if (intf == 1)
    {
      ainfo("DAC1-1 Selected\n");
      dev = &g_dac1ch1dev;
    }
  else
#endif /* CONFIG_STM32_DAC1CH1 */
#ifdef CONFIG_STM32_DAC1CH2
  if (intf == 2)
    {
      ainfo("DAC1-2 Selected\n");
      dev = &g_dac1ch2dev;
    }
  else
#endif /* CONFIG_STM32_DAC1CH2 */
#ifdef CONFIG_STM32_DAC2CH1
  if (intf == 3)
    {
      ainfo("DAC2-1 Selected\n");
      dev = &g_dac2ch1dev;
    }
  else
#endif /* CONFIG_STM32_DAC2CH1 */
    {
      aerr("ERROR: No such DAC interface: %d\n", intf);
      return NULL;
    }

  /* Make sure that the DAC block has been initialized */

  ret = dac_blockinit();
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize the DAC block: %d\n", ret);
      return NULL;
    }

  /* Configure the selected DAC channel */

  chan = dev->ad_priv;
  ret  = dac_chaninit(chan);
  if (ret < 0)
    {
      aerr("ERROR: Failed to initialize DAC channel %d: %d\n", intf, ret);
      return NULL;
    }

  return dev;
}

#endif /* CONFIG_STM32_DAC1 || CONFIG_STM32_DAC2 */
#endif /* CONFIG_DAC */
