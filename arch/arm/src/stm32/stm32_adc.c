/****************************************************************************
 * arch/arm/src/stm32/stm32_adc.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *            Mateusz Szafoni <raiden00@railab.me>
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "stm32.h"
#include "stm32_dma.h"
#include "stm32_adc.h"

/* The STM32 ADC lower-half driver functionality overview:
 *   - one lower-half driver for all STM32 ADC IP cores,
 *   - general lower-half logic for the NuttX upper-half ADC driver,
 *   - lower-half ADC driver can be used not only with the upper-half ADC
 *     driver, but also in the lower-half logic for special-case custom
 *     drivers (eg. power-control, custom sensors),
 *   - ADC can be used in time-critical operations (eg. control loop for
 *     converters or motor drivers) therefore it is necessary to support the
 *     high performance, zero latency ADC interrupts,
 *   - ADC triggering from different sources (EXTSEL and JEXTSEL),
 *   - regular sequence conversion (supported in upper-half ADC driver)
 *   - injected sequence conversion (not supported in upper-half ADC driver)
 */

/* STM32 ADC "lower-half" support must be enabled */

#ifdef CONFIG_STM32_ADC

/* Some ADC peripheral must be enabled */

#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2) || \
    defined(CONFIG_STM32_ADC3) || defined(CONFIG_STM32_ADC4)

/* This implementation is for the STM32 ADC IP version 1 and 2 */

#if !defined(HAVE_IP_ADC_V1) && !defined(HAVE_IP_ADC_V2)
#  error "STM32 ADC IP version not specified"
#endif

/* Supported ADC modes:
 *   - SW triggering with/without DMA transfer
 *   - TIM triggering with/without DMA transfer
 *   - external triggering with/without DMA transfer
 *
 * (tested with ADC example app from NuttX apps repo).
 */

/* At the moment there is no proper implementation for timers external
 * trigger in STM32L15XX may be added later
 */

#if defined(ADC_HAVE_TIMER) && defined(CONFIG_STM32_STM32L15XX)
#  warning "There is no proper implementation for TIMER TRIGGERS at the moment"
#endif

/* If ADC use HSI as clock-source and HSI is not used for PLL and system
 * clock, then we can control it directly from ADC driver.
 */

#if defined(HAVE_ADC_CLOCK_HSI) &&                                  \
    (STM32_CFGR_PLLSRC != 0 || STM32_SYSCLK_SW != RCC_CFGR_SW_HSI)
#  define HAVE_HSI_CONTROL
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RCC reset ****************************************************************/

#if defined(HAVE_IP_ADC_V1)
#  ifdef HAVE_BASIC_ADC
#    define STM32_RCC_RSTR     STM32_RCC_APB2RSTR
#    define RCC_RSTR_ADC1RST   RCC_APB2RSTR_ADC1RST
#    define RCC_RSTR_ADC2RST   RCC_APB2RSTR_ADC2RST
#    define RCC_RSTR_ADC3RST   RCC_APB2RSTR_ADC3RST
#  else
#    define STM32_RCC_RSTR     STM32_RCC_APB2RSTR
#    define RCC_RSTR_ADC123RST RCC_APB2RSTR_ADCRST
#  endif
#elif defined(HAVE_IP_ADC_V2)
#  ifdef STM32_RCC_AHB2RSTR_OFFSET
#    define STM32_RCC_RSTR       STM32_RCC_AHB2RSTR
#    define RCC_RSTR_ADC12RST    RCC_AHB2RSTR_ADC12RST
#    define RCC_RSTR_ADC34RST    RCC_AHB2RSTR_ADC345RST
#  else
#    define STM32_RCC_RSTR       STM32_RCC_AHBRSTR
#    define RCC_RSTR_ADC12RST    RCC_AHBRSTR_ADC12RST
#    define RCC_RSTR_ADC34RST    RCC_AHBRSTR_ADC34RST
#  endif
#endif

/* ADC Channels/DMA *********************************************************/

/* DMA values differs according to STM32 DMA IP core version */

#if defined(HAVE_IP_DMA_V2)
#  define ADC_DMA_CONTROL_WORD (DMA_SCR_MSIZE_16BITS | \
                                DMA_SCR_PSIZE_16BITS | \
                                DMA_SCR_MINC | \
                                DMA_SCR_CIRC | \
                                DMA_SCR_DIR_P2M)
#elif  defined(HAVE_IP_DMA_V1)
#  define ADC_DMA_CONTROL_WORD (DMA_CCR_MSIZE_16BITS | \
                                DMA_CCR_PSIZE_16BITS | \
                                DMA_CCR_MINC | \
                                DMA_CCR_CIRC)
#endif

/* Sample time default configuration
 *
 * REVISIT: simplify this, use adc_sampletime_write() function.
 * REVISIT: default SMPR configurable from Kconfig
 */

#if defined(CONFIG_STM32_STM32F10XX)
#  define ADC_SMPR_DEFAULT    ADC_SMPR_55p5
#  define ADC_SMPR1_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR1_SMP10_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP11_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP12_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP13_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP14_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP15_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP16_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP17_SHIFT))
#  define ADC_SMPR2_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR2_SMP0_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP1_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP2_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP3_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP4_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP5_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP6_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP7_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP8_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP9_SHIFT))
#elif defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX)
#  if defined(ADC_HAVE_DMA) || (CONFIG_STM32_ADC_MAX_SAMPLES == 1)
#    define ADC_SMPR_DEFAULT    ADC_SMPR_61p5
#  else /* Slow down sampling frequency */
#    define ADC_SMPR_DEFAULT    ADC_SMPR_601p5
#  endif
#  define ADC_SMPR1_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR1_SMP1_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP2_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP3_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP4_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP5_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP6_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP7_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP8_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP9_SHIFT))
#  define ADC_SMPR2_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR2_SMP10_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP11_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP12_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP13_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP14_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP15_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP16_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP17_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP18_SHIFT))
#elif defined(CONFIG_STM32_STM32G4XXX)
#  if defined(ADC_HAVE_DMA) || (CONFIG_STM32_ADC_MAX_SAMPLES == 1)
#    define ADC_SMPR_DEFAULT    ADC_SMPR_47p5
#  else /* Slow down sampling frequency */
#    define ADC_SMPR_DEFAULT    ADC_SMPR_640p5
#  endif
#  define ADC_SMPR1_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR1_SMP0_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP1_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP2_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP3_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP4_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP5_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP6_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP7_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP8_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP9_SHIFT))
#  define ADC_SMPR2_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR2_SMP10_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP11_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP12_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP13_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP14_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP15_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP16_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP17_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP18_SHIFT))
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F37XX) || \
    defined(CONFIG_STM32_STM32F4XXX)
#  if defined(CONFIG_STM32_STM32F37XX)
#    define ADC_SMPR_DEFAULT    ADC_SMPR_239p5 /* TODO choose 1p5? */
#  else
#    define ADC_SMPR_DEFAULT    ADC_SMPR_112
#  endif
#  define ADC_SMPR1_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR1_SMP10_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP11_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP12_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP13_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP14_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP15_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP16_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP17_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP18_SHIFT))
#  define ADC_SMPR2_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR2_SMP0_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP1_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP2_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP3_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP4_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP5_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP6_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP7_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP8_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP9_SHIFT))
#elif defined(CONFIG_STM32_STM32L15XX)
#  define ADC_SMPR_DEFAULT    ADC_SMPR_384
#  define ADC_SMPR1_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR1_SMP20_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP21_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP22_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP23_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP24_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP25_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP26_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP27_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP28_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP29_SHIFT))
#  define ADC_SMPR2_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR2_SMP10_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP11_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP12_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP13_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP14_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP15_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP16_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP17_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP18_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP19_SHIFT))
#  define ADC_SMPR3_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR3_SMP0_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR3_SMP1_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR3_SMP2_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR3_SMP3_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR3_SMP4_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR3_SMP5_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR3_SMP6_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR3_SMP7_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR3_SMP8_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR3_SMP9_SHIFT))
#  define ADC_SMPR0_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR0_SMP30_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR0_SMP31_SHIFT))
#endif

/* Number of channels per ADC:
 *   - F0, L0             - 19, but single SMP for all channels
 *   - F1                 - 18
 *   - F2,F3,F4,F7,L4,L4+ - 19
 *   - H7                 - 20
 *   - L1                 - 32
 *
 * NOTE: this value can be obtained from SMPRx register description
 *       (ST manual)
 */

#if defined(CONFIG_STM32_STM32F10XX)
#  define ADC_CHANNELS_NUMBER 18
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F30XX) || \
      defined(CONFIG_STM32_STM32F33XX) || defined(CONFIG_STM32_STM32F4XXX) || \
      defined(CONFIG_STM32_STM32G4XXX)
#  define ADC_CHANNELS_NUMBER 19
#elif defined(CONFIG_STM32_STM32L15XX)
#  define ADC_CHANNELS_NUMBER 32
#else
#  error "Not supported"
#endif

/* ADC resolution. Not supported for basic STM32 ADC IPv1 */

#ifndef CONFIG_STM32_HAVE_IP_ADC_V1_BASIC
#  define HAVE_ADC_RESOLUTION
#else
#  undef HAVE_ADC_RESOLUTION
#endif

/* ADC have common registers for all cores except basic ADC IPv1 (F1, F37x) */

#ifdef CONFIG_STM32_HAVE_IP_ADC_V1_BASIC
#  undef HAVE_ADC_CMN_REGS
#else
#  define HAVE_ADC_CMN_REGS
#endif
#if defined(HAVE_ADC_CMN_REGS) && STM32_NADC > 1
#  define HAVE_ADC_CMN_DATA
#else
#  undef HAVE_ADC_CMN_DATA
#endif

/* Max 4 injected channels */

#define ADC_INJ_MAX_SAMPLES   4

/* ADC DMA configuration bit support */

#ifndef CONFIG_STM32_HAVE_IP_ADC_V1_BASIC
#  define ADC_HAVE_DMACFG 1
#else
#  undef ADC_HAVE_DMACFG
#endif

/* ADC scan mode support - only for ADCv1 */

#ifdef CONFIG_STM32_HAVE_IP_ADC_V1
#  define ADC_HAVE_SCAN 1
#  ifndef CONFIG_STM32_ADC1_SCAN
#    define CONFIG_STM32_ADC1_SCAN 0
#  endif
#  ifndef CONFIG_STM32_ADC2_SCAN
#    define CONFIG_STM32_ADC2_SCAN 0
#  endif
#  ifndef CONFIG_STM32_ADC3_SCAN
#    define CONFIG_STM32_ADC3_SCAN 0
#  endif
#else
#  undef ADC_HAVE_SCAN
#endif

/* We have to support ADC callbacks if default ADC interrupts or
 * DMA transfer are enabled
 */

#if !defined(CONFIG_STM32_ADC_NOIRQ) || defined(ADC_HAVE_DMA)
#  define ADC_HAVE_CB
#else
#  undef ADC_HAVE_CB
#endif

/* ADC software trigger configuration */

#define ANIOC_TRIGGER_REGULAR  (1 << 0)
#define ANIOC_TRIGGER_INJECTED (1 << 1)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Data common to all ADC instances */

#ifdef HAVE_ADC_CMN_DATA
struct adccmn_data_s
{
  uint8_t refcount; /* How many ADC instances are currently in use */
  sem_t   lock;     /* Exclusive access to common ADC data */
};
#endif

/* This structure describes the state of one ADC block
 * REVISIT: save some space with bit fields.
 */

struct stm32_dev_s
{
#ifdef CONFIG_STM32_ADC_LL_OPS
  FAR const struct stm32_adc_ops_s *llops; /* Low-level ADC ops */
  FAR struct adc_dev_s             *dev;   /* Upper-half ADC reference */
#endif
#ifdef ADC_HAVE_CB
  FAR const struct adc_callback_s *cb;
  uint8_t irq;               /* Interrupt generated by this ADC block */
#endif
#ifdef HAVE_ADC_CMN_DATA
  struct adccmn_data_s *cmn; /* Common ADC data */
#endif
  uint8_t rnchannels;        /* Number of regular channels */
  uint8_t cr_channels;       /* Number of configured regular channels */
#ifdef ADC_HAVE_INJECTED
  uint8_t cj_channels;       /* Number of configured injected channels */
#endif
  uint8_t intf;              /* ADC interface number */
  uint8_t initialized;       /* ADC interface initialization counter */
  uint8_t current;           /* Current ADC channel being converted */
  uint8_t anioc_trg;         /* ANIOC_TRIGGER configuration */
#ifdef HAVE_ADC_RESOLUTION
  uint8_t resolution;        /* ADC resolution (0-3) */
#endif
#ifdef ADC_HAVE_DMA
  uint8_t dmachan;           /* DMA channel needed by this ADC */
#  ifdef ADC_HAVE_DMACFG
  uint8_t dmacfg;            /* DMA channel configuration, only for ADC IPv2 */
#  endif
  bool    hasdma;            /* True: This channel supports DMA */
#endif
#ifdef ADC_HAVE_SCAN
  bool    scan;              /* True: Scan mode */
#endif
#ifdef CONFIG_STM32_ADC_CHANGE_SAMPLETIME
  /* Sample time selection. These bits must be written only when ADON=0.
   * REVISIT: this takes too much space. We need only 3 bits per channel.
   */

  uint8_t sample_rate[ADC_CHANNELS_NUMBER];
  uint8_t adc_channels;      /* ADC channels number */
#endif
#ifdef ADC_HAVE_TIMER
  uint8_t trigger;           /* Timer trigger channel: 0=CC1, 1=CC2, 2=CC3,
                              * 3=CC4, 4=TRGO */
#endif
  xcpt_t   isr;              /* Interrupt handler for this ADC block */
  uint32_t base;             /* Base address of registers unique to this ADC
                              * block */
#ifdef ADC_HAVE_EXTCFG
  uint32_t extcfg;           /* External event configuration for regular group */
#endif
#ifdef ADC_HAVE_JEXTCFG
  uint32_t jextcfg;          /* External event configuration for injected group */
#endif
#ifdef ADC_HAVE_TIMER
  uint32_t tbase;            /* Base address of timer used by this ADC block */
  uint32_t pclck;            /* The PCLK frequency that drives this timer */
  uint32_t freq;             /* The desired frequency of conversions */
#endif
#ifdef ADC_HAVE_DMA
  DMA_HANDLE dma;            /* Allocated DMA channel */

  /* DMA transfer buffer */

  uint16_t r_dmabuffer[CONFIG_STM32_ADC_MAX_SAMPLES];
#endif

  /* List of selected ADC channels to sample */

  uint8_t  r_chanlist[CONFIG_STM32_ADC_MAX_SAMPLES];

#ifdef ADC_HAVE_INJECTED
  /* List of selected ADC injected channels to sample */

  uint8_t  j_chanlist[ADC_INJ_MAX_SAMPLES];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC Register access */

#ifndef HAVE_BASIC_ADC
static void stm32_modifyreg32(unsigned int addr, uint32_t clrbits,
                              uint32_t setbits);
#endif
static uint32_t adc_getreg(FAR struct stm32_dev_s *priv, int offset);
static void adc_putreg(FAR struct stm32_dev_s *priv, int offset,
                       uint32_t value);
static void adc_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits);
#ifdef HAVE_ADC_CMN_REGS
static uint32_t adccmn_base_get(FAR struct stm32_dev_s *priv);
static void adccmn_modifyreg(FAR struct stm32_dev_s *priv, uint32_t offset,
                             uint32_t clrbits, uint32_t setbits);
static uint32_t adccmn_getreg(FAR struct stm32_dev_s *priv, uint32_t offset);
#endif
#ifdef ADC_HAVE_TIMER
static uint16_t tim_getreg(FAR struct stm32_dev_s *priv, int offset);
static void tim_putreg(FAR struct stm32_dev_s *priv, int offset,
                       uint16_t value);
static void tim_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                          uint16_t clrbits, uint16_t setbits);
static void tim_dumpregs(FAR struct stm32_dev_s *priv, FAR const char *msg);
#endif

#ifdef HAVE_ADC_CMN_DATA
static int  adccmn_lock(FAR struct stm32_dev_s *priv, bool lock);
#endif

static void adc_rccreset(FAR struct stm32_dev_s *priv, bool reset);

/* ADC Interrupt Handler */

#ifndef CONFIG_STM32_ADC_NOIRQ
static int  adc_interrupt(FAR struct adc_dev_s *dev);
#  if defined(STM32_IRQ_ADC1) && defined(CONFIG_STM32_ADC1)
static int  adc1_interrupt(int irq, FAR void *context, FAR void *arg);
#  endif
#  if defined(STM32_IRQ_ADC12) && (defined(CONFIG_STM32_ADC1) || \
                                 defined(CONFIG_STM32_ADC2))
static int  adc12_interrupt(int irq, FAR void *context, FAR void *arg);
#  endif
#  if (defined(STM32_IRQ_ADC3) && defined(CONFIG_STM32_ADC3))
static int  adc3_interrupt(int irq, FAR void *context, FAR void *arg);
#  endif
#  if defined(STM32_IRQ_ADC4) && defined(CONFIG_STM32_ADC4)
static int  adc4_interrupt(int irq, FAR void *context, FAR void *arg);
#  endif
#  if defined(STM32_IRQ_ADC)
static int  adc123_interrupt(int irq, FAR void *context, FAR void *arg);
#  endif
#endif /* CONFIG_STM32_ADC_NOIRQ */

/* ADC Driver Methods */

static int  adc_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void adc_reset(FAR struct adc_dev_s *dev);
static int  adc_setup(FAR struct adc_dev_s *dev);
static void adc_shutdown(FAR struct adc_dev_s *dev);
static void adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);
static void adc_enable(FAR struct stm32_dev_s *priv, bool enable);

static uint32_t adc_sqrbits(FAR struct stm32_dev_s *priv, int first,
                            int last, int offset);
static int  adc_set_ch(FAR struct adc_dev_s *dev, uint8_t ch);

static int  adc_ioc_change_ints(FAR struct adc_dev_s *dev, int cmd,
                                bool arg);

#ifdef HAVE_ADC_RESOLUTION
static int  adc_resolution_set(FAR struct adc_dev_s *dev, uint8_t res);
#endif
#ifdef HAVE_ADC_VBAT
static void adc_enable_vbat_channel(FAR struct adc_dev_s *dev, bool enable);
#endif
#ifdef HAVE_ADC_POWERDOWN
static int  adc_ioc_change_sleep_between_opers(FAR struct adc_dev_s *dev,
                                               int cmd, bool arg);
static void adc_power_down_idle(FAR struct stm32_dev_s *priv,
                                bool pdi_high);
static void adc_power_down_delay(FAR struct stm32_dev_s *priv,
                                 bool pdd_high);
#endif

#ifdef CONFIG_STM32_STM32L15XX
static void adc_dels_after_conversion(FAR struct stm32_dev_s *priv,
                                      uint32_t delay);
static void adc_select_ch_bank(FAR struct stm32_dev_s *priv,
                               bool chb_selected);
#endif

#ifdef HAVE_HSI_CONTROL
static void adc_enable_hsi(bool enable);
static void adc_reset_hsi_disable(FAR struct adc_dev_s *dev);
#endif

#ifdef ADC_HAVE_TIMER
static void adc_timstart(FAR struct stm32_dev_s *priv, bool enable);
static int  adc_timinit(FAR struct stm32_dev_s *priv);
#endif

#if defined(ADC_HAVE_DMA) && !defined(CONFIG_STM32_ADC_NOIRQ)
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                FAR void *arg);
#endif

static void adc_reg_startconv(FAR struct stm32_dev_s *priv, bool enable);
#ifdef ADC_HAVE_INJECTED
static void adc_inj_startconv(FAR struct stm32_dev_s *priv, bool enable);
static int adc_inj_set_ch(FAR struct adc_dev_s *dev, uint8_t ch);
#endif

#ifdef ADC_HAVE_EXTCFG
static int adc_extcfg_set(FAR struct stm32_dev_s *priv, uint32_t extcfg);
#endif
#ifdef ADC_HAVE_JEXTCFG
static int adc_jextcfg_set(FAR struct stm32_dev_s *priv, uint32_t jextcfg);
#endif

static void adc_dumpregs(FAR struct stm32_dev_s *priv);

#ifdef CONFIG_STM32_ADC_LL_OPS
static int adc_llops_setup(FAR struct stm32_adc_dev_s *dev);
static void adc_llops_shutdown(FAR struct stm32_adc_dev_s *dev);
static void adc_intack(FAR struct stm32_adc_dev_s *dev, uint32_t source);
static void adc_inten(FAR struct stm32_adc_dev_s *dev, uint32_t source);
static void adc_intdis(FAR struct stm32_adc_dev_s *dev, uint32_t source);
static uint32_t adc_intget(FAR struct stm32_adc_dev_s *dev);
static uint32_t adc_regget(FAR struct stm32_adc_dev_s *dev);
static void adc_llops_reg_startconv(FAR struct stm32_adc_dev_s *dev,
                                    bool enable);
static int adc_offset_set(FAR struct stm32_adc_dev_s *dev, uint8_t ch,
                          uint8_t i, uint16_t offset);
#  ifdef ADC_HAVE_EXTCFG
static void adc_llops_extcfg_set(FAR struct stm32_adc_dev_s *dev,
                                 uint32_t extcfg);
#  endif
#  ifdef ADC_HAVE_JEXTCFG
static void adc_llops_jextcfg_set(FAR struct stm32_adc_dev_s *dev,
                                  uint32_t jextcfg);
#  endif
#  ifdef ADC_HAVE_DMA
static int adc_regbufregister(FAR struct stm32_adc_dev_s *dev,
                              uint16_t *buffer, uint8_t len);
#  endif
#  ifdef ADC_HAVE_INJECTED
static uint32_t adc_injget(FAR struct stm32_adc_dev_s *dev, uint8_t chan);
static void adc_llops_inj_startconv(FAR struct stm32_adc_dev_s *dev,
                                    bool enable);
#  endif
#  ifdef CONFIG_STM32_ADC_CHANGE_SAMPLETIME
static void adc_sampletime_set(FAR struct stm32_adc_dev_s *dev,
                               FAR struct adc_sample_time_s *time_samples);
static void adc_sampletime_write(FAR struct stm32_adc_dev_s *dev);
#  endif
static void adc_llops_dumpregs(FAR struct stm32_adc_dev_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC interface operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind        = adc_bind,
#ifdef HAVE_HSI_CONTROL
  .ao_reset       = adc_reset_hsi_disable,
#else
  .ao_reset       = adc_reset,
#endif
  .ao_setup       = adc_setup,
  .ao_shutdown    = adc_shutdown,
  .ao_rxint       = adc_rxint,
  .ao_ioctl       = adc_ioctl,
};

/* Publicly visible ADC lower-half operations */

#ifdef CONFIG_STM32_ADC_LL_OPS
static const struct stm32_adc_ops_s g_adc_llops =
{
  .setup         = adc_llops_setup,
  .shutdown      = adc_llops_shutdown,
  .int_ack       = adc_intack,
  .int_get       = adc_intget,
  .int_en        = adc_inten,
  .int_dis       = adc_intdis,
  .val_get       = adc_regget,
  .reg_startconv = adc_llops_reg_startconv,
  .offset_set    = adc_offset_set,
#  ifdef ADC_HAVE_DMA
  .regbuf_reg    = adc_regbufregister,
#  endif
#  ifdef ADC_HAVE_EXTCFG
  .extcfg_set    = adc_llops_extcfg_set,
#  endif
#  ifdef ADC_HAVE_JEXTCFG
  .jextcfg_set   = adc_llops_jextcfg_set,
#  endif
#  ifdef ADC_HAVE_INJECTED
  .inj_get       = adc_injget,
  .inj_startconv = adc_llops_inj_startconv,
#  endif
#  ifdef CONFIG_STM32_ADC_CHANGE_SAMPLETIME
  .stime_set     = adc_sampletime_set,
  .stime_write   = adc_sampletime_write,
#  endif
  .dump_regs     = adc_llops_dumpregs
};
#endif

/* ADC instances are coupled in blocks for all IP versions except
 * basic ADC IPv1 (F1, F37x).
 */

#ifdef HAVE_ADC_CMN_DATA
#  ifdef HAVE_IP_ADC_V1
#    define ADC1CMN_DATA g_adc123_cmn
#    define ADC2CMN_DATA g_adc123_cmn
#    define ADC3CMN_DATA g_adc123_cmn

/* ADC123 common data */

struct adccmn_data_s g_adc123_cmn =
{
  .refcount = 0
};

#  elif defined(HAVE_IP_ADC_V2)
#    define ADC1CMN_DATA g_adc12_cmn
#    define ADC2CMN_DATA g_adc12_cmn
#    define ADC3CMN_DATA g_adc34_cmn
#    define ADC4CMN_DATA g_adc34_cmn
#    if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2)

/* ADC12 common data */

struct adccmn_data_s g_adc12_cmn =
{
  .refcount = 0
};

#    endif
#    if defined(CONFIG_STM32_ADC3) || defined(CONFIG_STM32_ADC4)

/* ADC34 common data */

struct adccmn_data_s g_adc34_cmn =
{
  .refcount = 0
};

#    endif
#  endif /* !HAVE_IP_ADC_V1 */
#endif /* HAVE_ADC_CMN_DATA */

/* ADC1 state */

#ifdef CONFIG_STM32_ADC1
static struct stm32_dev_s g_adcpriv1 =
{
#ifdef CONFIG_STM32_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
#ifndef CONFIG_STM32_ADC_NOIRQ
#  if defined(STM32_IRQ_ADC1)
  .irq         = STM32_IRQ_ADC1,
  .isr         = adc1_interrupt,
#  elif defined(STM32_IRQ_ADC12)
  .irq         = STM32_IRQ_ADC12,
  .isr         = adc12_interrupt,
#  elif defined(STM32_IRQ_ADC)
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
#  else
#    error "No STM32_IRQ_ADC1 STM32_IRQ_ADC12 or STM32_IRQ_ADC  defined for CONFIG_STM32_ADC1"
#  endif
#endif /* CONFIG_STM32_ADC_NOIRQ */
#ifdef HAVE_ADC_CMN_DATA
  .cmn         = &ADC1CMN_DATA,
#endif
  .intf        = 1,
  .initialized = 0,
  .anioc_trg   = CONFIG_STM32_ADC1_ANIOC_TRIGGER,
#ifdef HAVE_ADC_RESOLUTION
  .resolution  = CONFIG_STM32_ADC1_RESOLUTION,
#endif
  .base        = STM32_ADC1_BASE,
#ifdef ADC1_HAVE_EXTCFG
  .extcfg      = ADC1_EXTCFG_VALUE,
#endif
#ifdef ADC1_HAVE_JEXTCFG
  .jextcfg     = ADC1_JEXTCFG_VALUE,
#endif
#ifdef ADC1_HAVE_TIMER
  .trigger     = CONFIG_STM32_ADC1_TIMTRIG,
  .tbase       = ADC1_TIMER_BASE,
  .pclck       = ADC1_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_ADC1_SAMPLE_FREQUENCY,
#endif
#ifdef ADC1_HAVE_DMA
  .dmachan     = ADC1_DMA_CHAN,
#  ifdef ADC_HAVE_DMACFG
  .dmacfg      = CONFIG_STM32_ADC1_DMA_CFG,
#  endif
  .hasdma      = true,
#endif
#ifdef ADC_HAVE_SCAN
  .scan        = CONFIG_STM32_ADC1_SCAN,
#endif
};

static struct adc_dev_s g_adcdev1 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv1,
};
#endif

/* ADC2 state */

#ifdef CONFIG_STM32_ADC2
static struct stm32_dev_s g_adcpriv2 =
{
#ifdef CONFIG_STM32_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
#ifndef CONFIG_STM32_ADC_NOIRQ
#  if defined(STM32_IRQ_ADC12)
  .irq         = STM32_IRQ_ADC12,
  .isr         = adc12_interrupt,
#  elif defined(STM32_IRQ_ADC)
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
#  else
#    error "No STM32_IRQ_ADC12 or STM32_IRQ_ADC  defined for CONFIG_STM32_ADC2"
#  endif
#endif /* CONFIG_STM32_ADC_NOIRQ */
#ifdef HAVE_ADC_CMN_DATA
  .cmn         = &ADC2CMN_DATA,
#endif
  .intf        = 2,
  .initialized = 0,
  .anioc_trg   = CONFIG_STM32_ADC2_ANIOC_TRIGGER,
#ifdef HAVE_ADC_RESOLUTION
  .resolution  = CONFIG_STM32_ADC2_RESOLUTION,
#endif
  .base        = STM32_ADC2_BASE,
#ifdef ADC2_HAVE_EXTCFG
  .extcfg      = ADC2_EXTCFG_VALUE,
#endif
#ifdef ADC2_HAVE_JEXTCFG
  .jextcfg     = ADC2_JEXTCFG_VALUE,
#endif
#ifdef ADC2_HAVE_TIMER
  .trigger     = CONFIG_STM32_ADC2_TIMTRIG,
  .tbase       = ADC2_TIMER_BASE,
  .pclck       = ADC2_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_ADC2_SAMPLE_FREQUENCY,
#endif
#ifdef ADC2_HAVE_DMA
  .dmachan     = ADC2_DMA_CHAN,
#  ifdef ADC_HAVE_DMACFG
  .dmacfg      = CONFIG_STM32_ADC2_DMA_CFG,
#  endif
  .hasdma      = true,
#endif
#ifdef ADC_HAVE_SCAN
  .scan        = CONFIG_STM32_ADC2_SCAN,
#endif
};

static struct adc_dev_s g_adcdev2 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv2,
};
#endif

/* ADC3 state */

#ifdef CONFIG_STM32_ADC3
static struct stm32_dev_s g_adcpriv3 =
{
#ifdef CONFIG_STM32_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
#ifndef CONFIG_STM32_ADC_NOIRQ
#  if defined(STM32_IRQ_ADC3)
  .irq         = STM32_IRQ_ADC3,
  .isr         = adc3_interrupt,
#  elif defined(STM32_IRQ_ADC)
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
#  else
#    error "No STM32_IRQ_ADC3 or STM32_IRQ_ADC  defined for CONFIG_STM32_ADC3"
#  endif
#endif /* CONFIG_STM32_ADC_NOIRQ */
#ifdef HAVE_ADC_CMN_DATA
  .cmn         = &ADC3CMN_DATA,
#endif
  .intf        = 3,
  .initialized = 0,
  .anioc_trg   = CONFIG_STM32_ADC3_ANIOC_TRIGGER,
#ifdef HAVE_ADC_RESOLUTION
  .resolution  = CONFIG_STM32_ADC3_RESOLUTION,
#endif
  .base        = STM32_ADC3_BASE,
#ifdef ADC3_HAVE_EXTCFG
  .extcfg      = ADC3_EXTCFG_VALUE,
#endif
#ifdef ADC3_HAVE_JEXTCFG
  .jextcfg     = ADC3_JEXTCFG_VALUE,
#endif
#ifdef ADC3_HAVE_TIMER
  .trigger     = CONFIG_STM32_ADC3_TIMTRIG,
  .tbase       = ADC3_TIMER_BASE,
  .pclck       = ADC3_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_ADC3_SAMPLE_FREQUENCY,
#endif
#ifdef ADC3_HAVE_DMA
  .dmachan     = ADC3_DMA_CHAN,
#  ifdef ADC_HAVE_DMACFG
  .dmacfg      = CONFIG_STM32_ADC3_DMA_CFG,
#  endif
  .hasdma      = true,
#endif
#ifdef ADC_HAVE_SCAN
  .scan        = CONFIG_STM32_ADC3_SCAN,
#endif
};

static struct adc_dev_s g_adcdev3 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv3,
};
#endif

/* ADC4 state */

#ifdef CONFIG_STM32_ADC4
static struct stm32_dev_s g_adcpriv4 =
{
#ifdef CONFIG_STM32_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
#ifndef CONFIG_STM32_ADC_NOIRQ
  .irq         = STM32_IRQ_ADC4,
  .isr         = adc4_interrupt,
#endif
#ifdef HAVE_ADC_CMN_DATA
  .cmn         = &ADC4CMN_DATA,
#endif
  .intf        = 4,
  .initialized = 0,
  .anioc_trg   = CONFIG_STM32_ADC4_ANIOC_TRIGGER,
#ifdef HAVE_ADC_RESOLUTION
  .resolution  = CONFIG_STM32_ADC4_RESOLUTION,
#endif
  .base        = STM32_ADC4_BASE,
#ifdef ADC4_HAVE_EXTCFG
  .extcfg      = ADC4_EXTCFG_VALUE,
#endif
#ifdef ADC4_HAVE_JEXTCFG
  .jextcfg     = ADC4_JEXTCFG_VALUE,
#endif
#ifdef ADC4_HAVE_TIMER
  .trigger     = CONFIG_STM32_ADC4_TIMTRIG,
  .tbase       = ADC4_TIMER_BASE,
  .pclck       = ADC4_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_ADC4_SAMPLE_FREQUENCY,
#endif
#ifdef ADC4_HAVE_DMA
  .dmachan     = ADC4_DMA_CHAN,
#  ifdef ADC_HAVE_DMACFG
  .dmacfg      = CONFIG_STM32_ADC4_DMA_CFG,
#  endif
  .hasdma      = true,
#endif
};

static struct adc_dev_s g_adcdev4 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv4,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_modifyreg32
 *
 * Description:
 *   Modify the value of a 32-bit register (not atomic).
 *
 * Input Parameters:
 *   addr    - The address of the register
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef HAVE_BASIC_ADC
static void stm32_modifyreg32(unsigned int addr, uint32_t clrbits,
                              uint32_t setbits)
{
  putreg32((getreg32(addr) & ~clrbits) | setbits, addr);
}
#endif

/****************************************************************************
 * Name: adc_getreg
 *
 * Description:
 *   Read the value of an ADC register.
 *
 * Input Parameters:
 *   priv   - A reference to the ADC block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t adc_getreg(FAR struct stm32_dev_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: adc_putreg
 *
 * Description:
 *   Write a value to an ADC register.
 *
 * Input Parameters:
 *   priv   - A reference to the ADC block status
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void adc_putreg(FAR struct stm32_dev_s *priv, int offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: adc_modifyreg
 *
 * Description:
 *   Modify the value of an ADC register (not atomic).
 *
 * Input Parameters:
 *   priv    - A reference to the ADC block status
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void adc_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits)
{
  adc_putreg(priv, offset, (adc_getreg(priv, offset) & ~clrbits) | setbits);
}

#ifdef HAVE_ADC_CMN_REGS

/****************************************************************************
 * Name: adccmn_base_get
 ****************************************************************************/

static uint32_t adccmn_base_get(FAR struct stm32_dev_s *priv)
{
  uint32_t base = 0;

#if defined(HAVE_IP_ADC_V2)
  if (priv->base == STM32_ADC1_BASE || priv->base == STM32_ADC2_BASE)
    {
      base = STM32_ADC12CMN_BASE;
    }
#  if defined(CONFIG_STM32_ADC3) || defined(CONFIG_STM32_ADC4)
  else
    {
      base = STM32_ADC34CMN_BASE;
    }
#  endif

#elif defined(HAVE_IP_ADC_V1)
  base = STM32_ADCCMN_BASE;
  UNUSED(priv);
#endif

  return base;
}

/****************************************************************************
 * Name: adccmn_modifyreg
 ****************************************************************************/

static void adccmn_modifyreg(FAR struct stm32_dev_s *priv, uint32_t offset,
                             uint32_t clrbits, uint32_t setbits)
{
  uint32_t base = 0;

  /* Get base address for ADC common register */

  base = adccmn_base_get(priv);

  /* Modify register */

  stm32_modifyreg32(offset + base, clrbits, setbits);
}

/****************************************************************************
 * Name: adccmn_getreg
 ****************************************************************************/

static uint32_t adccmn_getreg(FAR struct stm32_dev_s *priv, uint32_t offset)
{
  uint32_t base = 0;

  /* Get base address for ADC common register */

  base = adccmn_base_get(priv);

  /* Return register value */

  return getreg32(base + offset);
}
#endif /* HAVE_ADC_CMN_REGS */

/****************************************************************************
 * Name: tim_getreg
 *
 * Description:
 *   Read the value of an ADC timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the ADC block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static uint16_t tim_getreg(FAR struct stm32_dev_s *priv, int offset)
{
  return getreg16(priv->tbase + offset);
}
#endif

/****************************************************************************
 * Name: tim_putreg
 *
 * Description:
 *   Write a value to an ADC timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the ADC block status
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void tim_putreg(FAR struct stm32_dev_s *priv, int offset,
                       uint16_t value)
{
  putreg16(value, priv->tbase + offset);
}
#endif

/****************************************************************************
 * Name: tim_modifyreg
 *
 * Description:
 *   Modify the value of an ADC timer register (not atomic).
 *
 * Input Parameters:
 *   priv    - A reference to the ADC block status
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void tim_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                          uint16_t clrbits, uint16_t setbits)
{
  tim_putreg(priv, offset, (tim_getreg(priv, offset) & ~clrbits) | setbits);
}
#endif

/****************************************************************************
 * Name: tim_dumpregs
 *
 * Description:
 *   Dump all timer registers.
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void tim_dumpregs(FAR struct stm32_dev_s *priv, FAR const char *msg)
{
  ainfo("%s:\n", msg);
  ainfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
        tim_getreg(priv, STM32_GTIM_CR1_OFFSET),
        tim_getreg(priv, STM32_GTIM_CR2_OFFSET),
        tim_getreg(priv, STM32_GTIM_SMCR_OFFSET),
        tim_getreg(priv, STM32_GTIM_DIER_OFFSET));
  ainfo("   SR: %04x EGR:  0000 CCMR1: %04x CCMR2: %04x\n",
        tim_getreg(priv, STM32_GTIM_SR_OFFSET),
        tim_getreg(priv, STM32_GTIM_CCMR1_OFFSET),
        tim_getreg(priv, STM32_GTIM_CCMR2_OFFSET));
  ainfo(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
        tim_getreg(priv, STM32_GTIM_CCER_OFFSET),
        tim_getreg(priv, STM32_GTIM_CNT_OFFSET),
        tim_getreg(priv, STM32_GTIM_PSC_OFFSET),
        tim_getreg(priv, STM32_GTIM_ARR_OFFSET));
  ainfo(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
        tim_getreg(priv, STM32_GTIM_CCR1_OFFSET),
        tim_getreg(priv, STM32_GTIM_CCR2_OFFSET),
        tim_getreg(priv, STM32_GTIM_CCR3_OFFSET),
        tim_getreg(priv, STM32_GTIM_CCR4_OFFSET));
#if STM32_NATIM > 0
  if (priv->tbase == STM32_TIM1_BASE
#  ifdef STM32_TIM8_BASE
      || priv->tbase == STM32_TIM8_BASE
#  endif
    )
    {
      ainfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
            tim_getreg(priv, STM32_ATIM_RCR_OFFSET),
            tim_getreg(priv, STM32_ATIM_BDTR_OFFSET),
            tim_getreg(priv, STM32_ATIM_DCR_OFFSET),
            tim_getreg(priv, STM32_ATIM_DMAR_OFFSET));
    }
  else
#endif
    {
      ainfo("  DCR: %04x DMAR: %04x\n",
            tim_getreg(priv, STM32_GTIM_DCR_OFFSET),
            tim_getreg(priv, STM32_GTIM_DMAR_OFFSET));
    }
}
#endif

/****************************************************************************
 * Name: adc_timstart
 *
 * Description:
 *   Start (or stop) the timer counter
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *   enable - True: Start conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void adc_timstart(FAR struct stm32_dev_s *priv, bool enable)
{
  ainfo("enable: %d\n", enable ? 1 : 0);

  if (enable)
    {
      /* Start the counter */

      tim_modifyreg(priv, STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);
    }
  else
    {
      /* Disable the counter */

      tim_modifyreg(priv, STM32_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);
    }
}
#endif

/****************************************************************************
 * Name: adc_timinit
 *
 * Description:
 *   Initialize the timer that drivers the ADC sampling for this channel
 *   using the pre-calculated timer divider definitions.
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef ADC_HAVE_TIMER
static int adc_timinit(FAR struct stm32_dev_s *priv)
{
  uint32_t prescaler;
  uint32_t reload;
  uint32_t timclk;

  uint16_t clrbits = 0;
  uint16_t setbits = 0;
  uint16_t cr2;
  uint16_t ccmr1;
  uint16_t ccmr2;
  uint16_t ocmode1;
  uint16_t ocmode2;
  uint16_t ccenable;
  uint16_t ccer;
  uint16_t egr;

  /* If the timer base address is zero, then this ADC was not configured to
   * use a timer.
   */

  if (priv->tbase == 0)
    {
      return ERROR;
    }

  /* NOTE: EXTSEL configuration is done in adc_reset function */

  /* Configure the timer channel to drive the ADC */

  /* Calculate optimal values for the timer prescaler and for the timer
   * reload register.  If freq is the desired frequency, then
   *
   *   reload = timclk / freq
   *   reload = (pclck / prescaler) / freq
   *
   * There are many solutions to do this, but the best solution will be the
   * one that has the largest reload value and the smallest prescaler value.
   * That is the solution that should give us the most accuracy in the timer
   * control.  Subject to:
   *
   *   0 <= prescaler  <= 65536
   *   1 <= reload <= 65535
   *
   * So ( prescaler = pclck / 65535 / freq ) would be optimal.
   */

  prescaler = (priv->pclck / priv->freq + 65534) / 65535;

  /* We need to decrement the prescaler value by one, but only, the value
   * does not underflow.
   */

  if (prescaler < 1)
    {
      awarn("WARNING: Prescaler underflowed.\n");
      prescaler = 1;
    }

  /* Check for overflow */

  else if (prescaler > 65536)
    {
      awarn("WARNING: Prescaler overflowed.\n");
      prescaler = 65536;
    }

  timclk = priv->pclck / prescaler;

  reload = timclk / priv->freq;
  if (reload < 1)
    {
      awarn("WARNING: Reload value underflowed.\n");
      reload = 1;
    }
  else if (reload > 65535)
    {
      awarn("WARNING: Reload value overflowed.\n");
      reload = 65535;
    }

  /* Disable the timer until we get it configured */

  adc_timstart(priv, false);

  /* Set up the timer CR1 register.
   *
   * Select the Counter Mode == count up:
   *
   * ATIM_CR1_EDGE: The counter counts up or down depending on the
   *                direction bit(DIR).
   * ATIM_CR1_DIR: 0: count up, 1: count down
   *
   * Set the clock division to zero for all
   */

  clrbits = GTIM_CR1_DIR | GTIM_CR1_CMS_MASK | GTIM_CR1_CKD_MASK;
  setbits = GTIM_CR1_EDGE;
  tim_modifyreg(priv, STM32_GTIM_CR1_OFFSET, clrbits, setbits);

  /* Set the reload and prescaler values */

  tim_putreg(priv, STM32_GTIM_PSC_OFFSET, prescaler - 1);
  tim_putreg(priv, STM32_GTIM_ARR_OFFSET, reload);

  /* Clear the advanced timers repetition counter in TIM1 */

#if STM32_NATIM > 0
  if (priv->tbase == STM32_TIM1_BASE
#  ifdef STM32_TIM8_BASE
      || priv->tbase == STM32_TIM8_BASE
#  endif
    )
    {
      tim_putreg(priv, STM32_ATIM_RCR_OFFSET, 0);
      tim_putreg(priv, STM32_ATIM_BDTR_OFFSET, ATIM_BDTR_MOE); /* Check me */
    }
#endif

  /* TIMx event generation: Bit 0 UG: Update generation */

  tim_putreg(priv, STM32_GTIM_EGR_OFFSET, GTIM_EGR_UG);

  /* Handle channel specific setup */

  ocmode1 = 0;
  ocmode2 = 0;

  switch (priv->trigger)
    {
      case 0: /* TimerX CC1 event */
        {
          ccenable = ATIM_CCER_CC1E;
          ocmode1  = (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR1_CC1S_SHIFT) |
                     (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC1M_SHIFT) |
                     ATIM_CCMR1_OC1PE;

          /* Set the event CC1 */

          egr      = ATIM_EGR_CC1G;

          /* Set the duty cycle by writing to the CCR register for this
           * channel
           */

          tim_putreg(priv, STM32_GTIM_CCR1_OFFSET, (uint16_t)(reload >> 1));
        }
        break;

      case 1: /* TimerX CC2 event */
        {
          ccenable = ATIM_CCER_CC2E;
          ocmode1  = (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR1_CC2S_SHIFT) |
                     (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT) |
                     ATIM_CCMR1_OC2PE;

          /* Set the event CC2 */

          egr      = ATIM_EGR_CC2G;

          /* Set the duty cycle by writing to the CCR register for this
           * channel
           */

          tim_putreg(priv, STM32_GTIM_CCR2_OFFSET, (uint16_t)(reload >> 1));
        }
        break;

      case 2: /* TimerX CC3 event */
        {
          ccenable = ATIM_CCER_CC3E;
          ocmode2  = (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR2_CC3S_SHIFT) |
                     (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC3M_SHIFT) |
                     ATIM_CCMR2_OC3PE;

          /* Set the event CC3 */

          egr      = ATIM_EGR_CC3G;

          /* Set the duty cycle by writing to the CCR register for this
           * channel
           */

          tim_putreg(priv, STM32_GTIM_CCR3_OFFSET, (uint16_t)(reload >> 1));
        }
        break;

      case 3: /* TimerX CC4 event */
        {
          ccenable = ATIM_CCER_CC4E;
          ocmode2  = (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR2_CC4S_SHIFT) |
                     (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR2_OC4M_SHIFT) |
                     ATIM_CCMR2_OC4PE;

          /* Set the event CC4 */

          egr      = ATIM_EGR_CC4G;

          /* Set the duty cycle by writing to the CCR register for this
           * channel
           */

          tim_putreg(priv, STM32_GTIM_CCR4_OFFSET, (uint16_t)(reload >> 1));
        }
        break;

      case 4: /* TimerX TRGO event */
        {
          /* TODO: TRGO support not yet implemented */

          /* Set the event TRGO */

          ccenable = 0;
          egr      = GTIM_EGR_TG;

          /* Set the duty cycle by writing to the CCR register for this
           * channel
           */

          tim_putreg(priv, STM32_GTIM_CCR4_OFFSET, (uint16_t)(reload >> 1));
        }
        break;

      default:
        aerr("ERROR: No such trigger: %d\n", priv->trigger);
        return -EINVAL;
    }

  /* Disable the Channel by resetting the CCxE Bit in the CCER register */

  ccer = tim_getreg(priv, STM32_GTIM_CCER_OFFSET);
  ccer &= ~ccenable;
  tim_putreg(priv, STM32_GTIM_CCER_OFFSET, ccer);

  /* Fetch the CR2, CCMR1, and CCMR2 register (already have ccer) */

  cr2   = tim_getreg(priv, STM32_GTIM_CR2_OFFSET);
  ccmr1 = tim_getreg(priv, STM32_GTIM_CCMR1_OFFSET);
  ccmr2 = tim_getreg(priv, STM32_GTIM_CCMR2_OFFSET);

  /* Reset the Output Compare Mode Bits and set the select output compare
   * mode
   */

  ccmr1 &= ~(ATIM_CCMR1_CC1S_MASK | ATIM_CCMR1_OC1M_MASK | ATIM_CCMR1_OC1PE |
             ATIM_CCMR1_CC2S_MASK | ATIM_CCMR1_OC2M_MASK | ATIM_CCMR1_OC2PE);
  ccmr2 &= ~(ATIM_CCMR2_CC3S_MASK | ATIM_CCMR2_OC3M_MASK | ATIM_CCMR2_OC3PE |
             ATIM_CCMR2_CC4S_MASK | ATIM_CCMR2_OC4M_MASK | ATIM_CCMR2_OC4PE);
  ccmr1 |= ocmode1;
  ccmr2 |= ocmode2;

  /* Reset the output polarity level of all channels (selects high
   * polarity)
   */

  ccer &= ~(ATIM_CCER_CC1P | ATIM_CCER_CC2P |
            ATIM_CCER_CC3P | ATIM_CCER_CC4P);

  /* Enable the output state of the selected channel (only) */

  ccer &= ~(ATIM_CCER_CC1E | ATIM_CCER_CC2E |
            ATIM_CCER_CC3E | ATIM_CCER_CC4E);
  ccer |= ccenable;

  /* TODO: revisit and simplify logic below */

#if STM32_NATIM > 0
  if (priv->tbase == STM32_TIM1_BASE
#  ifdef STM32_TIM8_BASE
      || priv->tbase == STM32_TIM8_BASE
#  endif
    )
    {
      /* Reset output N polarity level, output N state, output compare state,
       * output compare N idle state.
       */

      ccer &= ~(ATIM_CCER_CC1NE | ATIM_CCER_CC1NP |
                ATIM_CCER_CC2NE | ATIM_CCER_CC2NP |
                ATIM_CCER_CC3NE | ATIM_CCER_CC3NP);

      /* Reset the output compare and output compare N IDLE State */

      cr2 &= ~(ATIM_CR2_OIS1 | ATIM_CR2_OIS1N |
               ATIM_CR2_OIS2 | ATIM_CR2_OIS2N |
               ATIM_CR2_OIS3 | ATIM_CR2_OIS3N |
               ATIM_CR2_OIS4);
    }
#  if defined(HAVE_GTIM_CCXNP)
  else
    {
      ccer &= ~(GTIM_CCER_CC1NP | GTIM_CCER_CC2NP | GTIM_CCER_CC3NP |
              GTIM_CCER_CC4NP);
    }
#  endif

#else  /* No ADV TIM */

  /* For the STM32L15XX family only these timers can be used: 2-4, 6, 7, 9,
   * 10. Reset the output compare and output compare N IDLE State
   */

  if (priv->tbase >= STM32_TIM2_BASE && priv->tbase <= STM32_TIM4_BASE)
    {
      /* Reset output N polarity level, output N state, output compare state,
       * output compare N idle state.
       */

      ccer &= ~(GTIM_CCER_CC1NE | GTIM_CCER_CC1NP |
                GTIM_CCER_CC2NE | GTIM_CCER_CC2NP |
                GTIM_CCER_CC3NE | GTIM_CCER_CC3NP |
                GTIM_CCER_CC4NP);
    }
#endif

  /* Save the modified register values */

  tim_putreg(priv, STM32_GTIM_CR2_OFFSET, cr2);
  tim_putreg(priv, STM32_GTIM_CCMR1_OFFSET, ccmr1);
  tim_putreg(priv, STM32_GTIM_CCMR2_OFFSET, ccmr2);
  tim_putreg(priv, STM32_GTIM_CCER_OFFSET, ccer);
  tim_putreg(priv, STM32_GTIM_EGR_OFFSET, egr);

  /* Set the ARR Preload Bit */

  tim_modifyreg(priv, STM32_GTIM_CR1_OFFSET, 0, GTIM_CR1_ARPE);

  /* Enable the timer counter */

  adc_timstart(priv, true);

  tim_dumpregs(priv, "After starting timers");

  return OK;
}
#endif

/****************************************************************************
 * Name: adc_reg_startconv
 *
 * Description:
 *   Start (or stop) the ADC regular conversion process
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *   enable - True: Start conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(HAVE_IP_ADC_V2)
static void adc_reg_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  uint32_t regval;

  ainfo("reg enable: %d\n", enable ? 1 : 0);

  if (enable)
    {
      /* Start the conversion of regular channels */

      adc_modifyreg(priv, STM32_ADC_CR_OFFSET, 0, ADC_CR_ADSTART);
    }
  else
    {
      regval = adc_getreg(priv, STM32_ADC_CR_OFFSET);

      /* Is a conversion ongoing? */

      if ((regval & ADC_CR_ADSTART) != 0)
        {
          /* Stop the conversion */

          adc_putreg(priv, STM32_ADC_CR_OFFSET, regval | ADC_CR_ADSTP);

          /* Wait for the conversion to stop */

          while ((adc_getreg(priv, STM32_ADC_CR_OFFSET) &
                  ADC_CR_ADSTP) != 0);
        }
    }
}
#elif defined(HAVE_IP_ADC_V1) && !defined(HAVE_BASIC_ADC)
static void adc_reg_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  ainfo("reg enable: %d\n", enable ? 1 : 0);

  if (enable)
    {
      /* Start the conversion of regular channels */

      adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, 0, ADC_CR2_SWSTART);
    }
  else
    {
      /* Stop the conversion */

      adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, ADC_CR2_SWSTART, 0);
    }
}
#else  /* ADV IPv1 BASIC */
static void adc_reg_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  ainfo("reg enable: %d\n", enable ? 1 : 0);

  if (!enable)
    {
      /* Clear ADON to stop the conversion and put the ADC in the
       * power down state.
       */

      adc_enable(priv, false);
    }

  /* If the ADC is already on, set ADON again to start the conversion.
   * Otherwise, set ADON once to wake up the ADC from the power down state.
   */

  adc_enable(priv, true);
}
#endif

#ifdef ADC_HAVE_INJECTED

/****************************************************************************
 * Name: adc_inj_startconv
 *
 * Description:
 *   Start (or stop) the ADC injected conversion process
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *   enable - True: Start conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(HAVE_IP_ADC_V2)
static void adc_inj_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  uint32_t regval;

  ainfo("inj enable: %d\n", enable ? 1 : 0);

  if (enable)
    {
      /* Start the conversion of regular channels */

      adc_modifyreg(priv, STM32_ADC_CR_OFFSET, 0, ADC_CR_JADSTART);
    }
  else
    {
      regval = adc_getreg(priv, STM32_ADC_CR_OFFSET);

      /* Is a conversion ongoing? */

      if ((regval & ADC_CR_JADSTART) != 0)
        {
          /* Stop the conversion */

          adc_putreg(priv, STM32_ADC_CR_OFFSET, regval | ADC_CR_JADSTP);

          /* Wait for the conversion to stop */

          while ((adc_getreg(priv, STM32_ADC_CR_OFFSET) &
                  ADC_CR_JADSTP) != 0);
        }
    }
}
#elif defined(HAVE_IP_ADC_V1)
static void adc_inj_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  ainfo("inj enable: %d\n", enable ? 1 : 0);

  if (enable)
    {
      /* Start the conversion of injected channels */

      adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, 0, ADC_CR2_JSWSTART);
    }
  else
    {
      /* Stop the conversion */

      adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, ADC_CR2_JSWSTART, 0);
    }
}
#endif

#endif /* ADC_HAVE_INJECTED */

/****************************************************************************
 * Name: adccmn_lock
 ****************************************************************************/

#ifdef HAVE_ADC_CMN_DATA
static int adccmn_lock(FAR struct stm32_dev_s *priv, bool lock)
{
  int ret;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&priv->cmn->lock);
    }
  else
    {
      ret = nxsem_post(&priv->cmn->lock);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: adc_rccreset
 *
 * Description:
 *   Deinitializes the ADCx peripheral registers to their default
 *   reset values. It could set all the ADCs configured.
 *
 * Input Parameters:
 *   regaddr - The register to read
 *   reset - Condition, set or reset
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(HAVE_IP_ADC_V1) && defined(HAVE_BASIC_ADC)
static void adc_rccreset(FAR struct stm32_dev_s *priv, bool reset)
{
  uint32_t adcbit;

  /* Pick the appropriate bit in the RCC reset register.
   * For the basic STM32 ADC IPv1, there is an individual bit to reset
   * each ADC (ADC12 and ADC34).
   */

  switch (priv->intf)
    {
#ifdef CONFIG_STM32_ADC1
      case 1:
        {
          adcbit = RCC_RSTR_ADC1RST;
          break;
        }

#endif
#ifdef CONFIG_STM32_ADC2
      case 2:
        {
          adcbit = RCC_RSTR_ADC2RST;
          break;
        }

#endif
#ifdef CONFIG_STM32_ADC3
      case 3:
        {
          adcbit = RCC_RSTR_ADC3RST;
          break;
        }

#endif
#ifdef CONFIG_STM32_ADC4
      case 4:
        {
          adcbit = RCC_RSTR_ADC4RST;
          break;
        }

#endif
      default:
        {
          return;
        }
    }

  /* Set or clear the selected bit in the RCC reset register */

  if (reset)
    {
      /* Enable ADC reset state */

      modifyreg32(STM32_RCC_RSTR, 0, adcbit);
    }
  else
    {
      /* Release ADC from reset state */

      modifyreg32(STM32_RCC_RSTR, adcbit, 0);
    }
}
#elif defined(HAVE_IP_ADC_V1)
static void adc_rccreset(FAR struct stm32_dev_s *priv, bool reset)
{
  uint32_t adcbit;

  /* Pick the appropriate bit in the RCC reset register.
   * For the STM32 ADC IPv1, there is one common reset for all ADCs.
   */

  switch (priv->intf)
    {
      case 1:
      case 2:
      case 3:
        {
          adcbit = RCC_RSTR_ADC123RST;
          break;
        }

      default:
        {
          return;
        }
    }

  /* Set or clear the selected bit in the RCC reset register */

  if (reset)
    {
      /* Enable ADC reset state */

      modifyreg32(STM32_RCC_RSTR, 0, adcbit);
    }
  else
    {
      /* Release ADC from reset state */

      modifyreg32(STM32_RCC_RSTR, adcbit, 0);
    }
}
#elif defined(HAVE_IP_ADC_V2)
static void adc_rccreset(FAR struct stm32_dev_s *priv, bool reset)
{
  uint32_t adcbit;

  /* Pick the appropriate bit in the RCC reset register.
   * For the STM32 ADC IPv2, there is an individual bit to reset each
   * ADC block.
   */

  switch (priv->intf)
    {
#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2)
      case 1:
      case 2:
        {
          adcbit = RCC_RSTR_ADC12RST;
          break;
        }

#endif
#if defined(CONFIG_STM32_ADC3) || defined(CONFIG_STM32_ADC4)
      case 3:
      case 4:
        {
          adcbit = RCC_RSTR_ADC34RST;
          break;
        }

#endif
      default:
        {
          return;
        }
    }

  /* Set or clear the selected bit in the RCC reset register */

  if (reset)
    {
      /* Enable ADC reset state */

      modifyreg32(STM32_RCC_RSTR, 0, adcbit);
    }
  else
    {
      /* Release ADC from reset state */

      modifyreg32(STM32_RCC_RSTR, adcbit, 0);
    }
}
#endif

/****************************************************************************
 * Name: adc_power_down_idle
 *
 * Description:
 *   Enables or disables power down during the idle phase.
 *
 * Input Parameters:
 *
 *   priv     - pointer to the adc device structure
 *   pdi_high - true:  The ADC is powered down when waiting for a start event
 *              false: The ADC is powered up when waiting for a start event
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_STM32L15XX
static void adc_power_down_idle(FAR struct stm32_dev_s *priv, bool pdi_high)
{
  uint32_t regval;

  ainfo("PDI: %d\n", pdi_high ? 1 : 0);

  regval = adc_getreg(priv, STM32_ADC_CR1_OFFSET);

  if ((STM32_ADC1_CR2 & ADC_CR2_ADON) == 0)
    {
      if (pdi_high)
        {
          regval |= ADC_CR1_PDI;
        }
      else
        {
          regval &= ~ADC_CR1_PDI;
        }

      adc_putreg(priv, STM32_ADC_CR1_OFFSET, regval);
    }
}
#endif

/****************************************************************************
 * Name: adc_power_down_delay
 *
 * Description:
 *   Enables or disables power down during the delay phase.
 *
 * Input Parameters:
 *
 *   priv     - pointer to the adc device structure
 *   pdd_high - true:  The ADC is powered down when waiting for a start event
 *              false: The ADC is powered up when waiting for a start event
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_STM32L15XX
static void adc_power_down_delay(FAR struct stm32_dev_s *priv, bool pdd_high)
{
  uint32_t regval;

  ainfo("PDD: %d\n", pdd_high ? 1 : 0);

  regval = adc_getreg(priv, STM32_ADC_CR1_OFFSET);

  if ((STM32_ADC1_CR2 & ADC_CR2_ADON) == 0)
    {
      if (pdd_high)
        {
          regval |= ADC_CR1_PDD;
        }
      else
        {
          regval &= ~ADC_CR1_PDD;
        }

      adc_putreg(priv, STM32_ADC_CR1_OFFSET, regval);
    }
}
#endif

/****************************************************************************
 * Name: adc_dels_after_conversion
 *
 * Description:
 *   Defines the length of the delay which is applied after a conversion or
 *   a sequence of conversions.
 *
 * Input Parameters:
 *
 *   priv  - pointer to the adc device structure
 *   delay - delay selection (see definition in chip/chip/stm32_adc.h
 *           starting from line 284)
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_STM32L15XX
static void adc_dels_after_conversion(FAR struct stm32_dev_s *priv,
                                      uint32_t delay)
{
  ainfo("Delay selected: 0x%08x\n", delay);

  adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, ADC_CR2_DELS_MASK, delay);
}
#endif

/****************************************************************************
 * Name: adc_select_ch_bank
 *
 * Description:
 *   Selects the bank of channels to be converted
 *    (! Must be modified only when no conversion is on going !)
 *
 * Input Parameters:
 *
 *   priv   - pointer to the adc device structure
 *   enable - true:  bank of channels B selected
 *            false: bank of channels A selected
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_STM32L15XX
static void adc_select_ch_bank(FAR struct stm32_dev_s *priv,
                               bool chb_selected)
{
  ainfo("Bank of channels selected: %c\n", chb_selected ? 'B' : 'A');

  if (chb_selected)
    {
      adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, 0, ADC_CR2_CFG);
    }
  else
    {
      adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, ADC_CR2_CFG, 0);
    }
}
#endif

/****************************************************************************
 * Name: adc_enable
 *
 * Description:
 *   Enables or disables the specified ADC peripheral.  Also, starts a
 *   conversion when the ADC is not triggered by timers
 *
 * Input Parameters:
 *
 *   enable - true:  enable ADC conversion
 *            false: disable ADC conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(HAVE_IP_ADC_V2)
static void adc_enable(FAR struct stm32_dev_s *priv, bool enable)
{
  uint32_t regval;

  ainfo("enable: %d\n", enable ? 1 : 0);

  regval = adc_getreg(priv, STM32_ADC_CR_OFFSET);

  if (enable)
    {
      /* Enable the ADC */

      adc_putreg(priv, STM32_ADC_CR_OFFSET, regval | ADC_CR_ADEN);

      /* Wait for the ADC to be ready */

      while ((adc_getreg(priv, STM32_ADC_ISR_OFFSET) & ADC_INT_ARDY) == 0);
    }
  else if ((regval & ADC_CR_ADEN) != 0 && (regval & ADC_CR_ADDIS) == 0)
    {
      /* Stop ongoing regular conversions */

      adc_reg_startconv(priv, false);

      /* Disable the ADC */

      adc_putreg(priv, STM32_ADC_CR_OFFSET, regval | ADC_CR_ADDIS);

      /* Wait for the ADC to be disabled */

      while ((adc_getreg(priv, STM32_ADC_CR_OFFSET) & ADC_CR_ADEN) != 0);
    }
}
#else  /* HAVE_IP_ADC_V1 */
static void adc_enable(FAR struct stm32_dev_s *priv, bool enable)
{
#ifdef ADC_SR_ADONS
  bool enabled = (adc_getreg(priv, STM32_ADC_SR_OFFSET) & ADC_SR_ADONS) != 0;
#else
  bool enabled = false;
#endif

  ainfo("enable: %d\n", enable ? 1 : 0);

  if (!enabled && enable)
    {
      adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, 0, ADC_CR2_ADON);
    }
  else if (enabled && !enable)
    {
      adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, ADC_CR2_ADON, 0);
    }
}
#endif

/****************************************************************************
 * Name: adc_dmaconvcallback
 *
 * Description:
 *   Callback for DMA.  Called from the DMA transfer complete interrupt after
 *   all channels have been converted and transferred with DMA.
 *
 * Input Parameters:
 *
 *   handle - handle to DMA
 *   isr -
 *   arg - adc device
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(ADC_HAVE_DMA) && !defined(CONFIG_STM32_ADC_NOIRQ)
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                FAR void *arg)
{
  FAR struct adc_dev_s   *dev  = (FAR struct adc_dev_s *)arg;
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int i;

  /* Verify that the upper-half driver has bound its callback functions */

  if (priv->cb != NULL)
    {
      DEBUGASSERT(priv->cb->au_receive != NULL);

      for (i = 0; i < priv->rnchannels; i++)
        {
          priv->cb->au_receive(dev, priv->r_chanlist[priv->current],
                               priv->r_dmabuffer[priv->current]);
          priv->current++;
          if (priv->current >= priv->rnchannels)
            {
              /* Restart the conversion sequence from the beginning */

              priv->current = 0;
            }
        }
    }

  /* Restart DMA for the next conversion series */

  adc_modifyreg(priv, STM32_ADC_DMAREG_OFFSET, ADC_DMAREG_DMA, 0);
  adc_modifyreg(priv, STM32_ADC_DMAREG_OFFSET, 0, ADC_DMAREG_DMA);
}
#endif

/****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int adc_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
#ifdef ADC_HAVE_CB
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
#else
  UNUSED(dev);
  UNUSED(callback);
#endif

  return OK;
}

/****************************************************************************
 * Name: adc_watchdog_cfg
 ****************************************************************************/

#if defined(HAVE_IP_ADC_V2)
static void adc_watchdog_cfg(FAR struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  /* Initialize the watchdog 1 threshold register */

  adc_putreg(priv, STM32_ADC_TR1_OFFSET, 0x0fff0000);

  /* Enable the analog watchdog */

  clrbits = ADC_CFGR1_AWD1CH_MASK;
  setbits = ADC_CFGR1_AWD1EN | ADC_CFGR1_AWD1SGL |
            (priv->r_chanlist[0] << ADC_CFGR1_AWD1CH_SHIFT);

  /* Modify CFGR configuration */

  adc_modifyreg(priv, STM32_ADC_CFGR1_OFFSET, clrbits, setbits);
}
#else
static void adc_watchdog_cfg(FAR struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  /* Initialize the watchdog high threshold register */

  adc_putreg(priv, STM32_ADC_HTR_OFFSET, 0x00000fff);

  /* Initialize the watchdog low threshold register */

  adc_putreg(priv, STM32_ADC_LTR_OFFSET, 0x00000000);

  clrbits = ADC_CR1_AWDCH_MASK;
  setbits = ADC_CR1_AWDEN | (priv->r_chanlist[0] << ADC_CR1_AWDCH_SHIFT);

  /* Modify CR1 configuration */

  adc_modifyreg(priv, STM32_ADC_CR1_OFFSET, clrbits, setbits);
}
#endif

/****************************************************************************
 * Name: adc_calibrate
 ****************************************************************************/

#if defined(HAVE_IP_ADC_V2)
static void adc_calibrate(FAR struct stm32_dev_s *priv)
{
#if 0 /* Doesn't work */
  /* Calibrate the ADC */

  adc_modifyreg(priv, STM32_ADC_CR_OFFSET, ADC_CR_ADCALDIF, AD_CR_ADCAL);

  /* Wait for the calibration to complete */

  while ((adc_getreg(priv, STM32_ADC_CR_OFFSET) & ADC_CR_ADCAL) != 0);

#else
  UNUSED(priv);
#endif
}
#elif defined(HAVE_IP_ADC_V1) && defined(HAVE_BASIC_ADC)
static void adc_calibrate(FAR struct stm32_dev_s *priv)
{
  /* Power on the ADC */

  adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, 0, ADC_CR2_ADON);

  /* Wait for the ADC power on at least 2 ADCCLK cycles */

  up_udelay(10);

  /* Reset calibration registers */

  adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, 0, ADC_CR2_RSTCAL);

  /* Wait for the calibration register reset to complete */

  while ((adc_getreg(priv, STM32_ADC_CR2_OFFSET) & ADC_CR2_RSTCAL) != 0);

  /* Start ADC auto-calibration procedure  */

  adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, 0, ADC_CR2_CAL);

  /* Wait for the calibration procedure to complete */

  while ((adc_getreg(priv, STM32_ADC_CR2_OFFSET) & ADC_CR2_CAL) != 0);

  /* Power off the ADC */

  adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, ADC_CR2_ADON, 0);
}
#else
#  define adc_calibrate(priv)
#endif

/****************************************************************************
 * Name: adc_mode_cfg
 ****************************************************************************/

#ifdef HAVE_IP_ADC_V2
static void adc_mode_cfg(FAR struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  /* Disable continuous mode and set align to right */

  clrbits = ADC_CFGR1_CONT | ADC_CFGR1_ALIGN;

  /* Disable external trigger for regular channels */

  clrbits |= ADC_CFGR1_EXTEN_MASK;
  setbits |= ADC_CFGR1_EXTEN_NONE;

  /* Set CFGR configuration */

  adc_modifyreg(priv, STM32_ADC_CFGR1_OFFSET, clrbits, setbits);
}
#else
static void adc_mode_cfg(FAR struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

#ifdef HAVE_BASIC_ADC
  /* Set independent mode */

  clrbits |= ADC_CR1_DUALMOD_MASK;
  setbits |= ADC_CR1_IND;
#endif

#ifdef ADC_HAVE_SCAN
  if (priv->scan == true)
    {
      setbits |= ADC_CR1_SCAN;
    }
#endif

  /* Set CR1 configuration */

  adc_modifyreg(priv, STM32_ADC_CR1_OFFSET, clrbits, setbits);

  /* REVISIT: */

#ifdef CONFIG_STM32_STM32L15XX

  /* Select the bank of channels A */

  adc_select_ch_bank(priv, false);

#  ifdef HAVE_ADC_POWERDOWN
  /* Disables power down during the delay phase */

  adc_power_down_idle(priv, false);
  adc_power_down_delay(priv, false);
#  endif

  /* Delay until the converted data has been read */

  adc_dels_after_conversion(priv, ADC_CR2_DELS_TILLRD);
#endif

  /* Disable continuous mode and set align to right */

  clrbits = ADC_CR2_CONT | ADC_CR2_ALIGN;
  setbits = 0;

  /* Disable external trigger for regular channels */

  clrbits |= ADC_EXTREG_EXTEN_MASK;
  setbits |= ADC_EXTREG_EXTEN_NONE;

  /* Enable software trigger for regular channels
   * REVISIT: SWSTART must be set if no EXT trigger and basic ADC IPv1
   */

#ifdef CONFIG_STM32_STM32F37XX
  clrbits |= ADC_CR2_EXTSEL_MASK;
  setbits |= ADC_CR2_EXTSEL_SWSTART | ADC_CR2_EXTTRIG; /* SW is considered as external trigger */
#endif

  /* Set CR2 configuration */

  adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, clrbits, setbits);
}
#endif

/****************************************************************************
 * Name: adc_voltreg_cfg
 ****************************************************************************/

#if defined(HAVE_IP_ADC_V2)
static void adc_voltreg_cfg(FAR struct stm32_dev_s *priv)
{
  /* Set ADC voltage regulator to intermediate state */

  adc_modifyreg(priv, STM32_ADC_CR_OFFSET, ADC_CR_ADVREGEN_MASK,
                ADC_CR_ADVREGEN_INTER);

  /* Enable the ADC voltage regulator */

  adc_modifyreg(priv, STM32_ADC_CR_OFFSET, ADC_CR_ADVREGEN_MASK,
                ADC_CR_ADVREGEN_ENABLED);

  /* Wait for the ADC voltage regulator to startup */

  up_udelay(10);
}
#else
static void adc_voltreg_cfg(FAR struct stm32_dev_s *priv)
{
  /* Nothing to do here */

  UNUSED(priv);
}
#endif

/****************************************************************************
 * Name: adc_voltreg_cfg
 ****************************************************************************/

static void adc_sampletime_cfg(FAR struct adc_dev_s *dev)
{
  /* Initialize the same sample time for each ADC.
   * During sample cycles channel selection bits must remain unchanged.
   */

#ifdef CONFIG_STM32_ADC_CHANGE_SAMPLETIME
  adc_sampletime_write((FAR struct stm32_adc_dev_s *)dev->ad_priv);
#else
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  adc_putreg(priv, STM32_ADC_SMPR1_OFFSET, ADC_SMPR1_DEFAULT);
  adc_putreg(priv, STM32_ADC_SMPR2_OFFSET, ADC_SMPR2_DEFAULT);
#  ifdef STM32_ADC_SMPR3_OFFSET
  adc_putreg(priv, STM32_ADC_SMPR3_OFFSET, ADC_SMPR3_DEFAULT);
#  endif
#  ifdef STM32_ADC_SMPR0_OFFSET
  adc_putreg(priv, STM32_ADC_SMPR0_OFFSET, ADC_SMPR0_DEFAULT);
#  endif
#endif
}

/****************************************************************************
 * Name: adc_common_cfg
 ****************************************************************************/

#if defined(HAVE_IP_ADC_V2)
static void adc_common_cfg(FAR struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  /* REVISIT: */

  clrbits = ADC_CCR_DUAL_MASK | ADC_CCR_DELAY_MASK | ADC_CCR_DMACFG |
            ADC_CCR_MDMA_MASK | ADC_CCR_CKMODE_MASK | ADC_CCR_VREFEN |
            ADC_CCR_TSEN | ADC_CCR_VBATEN;
  setbits = ADC_CCR_DUAL_IND | ADC_CCR_DELAY(0) | ADC_CCR_MDMA_DISABLED |
            ADC_CCR_CKMODE_ASYNCH;

  adccmn_modifyreg(priv, STM32_ADC_CCR_OFFSET, clrbits, setbits);
}
#elif defined(HAVE_IP_ADC_V1) && !defined(HAVE_BASIC_ADC)
static void adc_common_cfg(FAR struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  clrbits  = ADC_CCR_ADCPRE_MASK | ADC_CCR_TSVREFE;
  setbits  = ADC_CCR_ADCPRE_DIV2;

  /* REVISIT: */

#if !defined(CONFIG_STM32_STM32L15XX)
  clrbits |= ADC_CCR_MULTI_MASK | ADC_CCR_DELAY_MASK | ADC_CCR_DDS |
             ADC_CCR_DMA_MASK | ADC_CCR_VBATEN;
  setbits |= ADC_CCR_MULTI_NONE | ADC_CCR_DMA_DISABLED;
#endif /* !defined(CONFIG_STM32_STM32L15XX) */

  adccmn_modifyreg(priv, STM32_ADC_CCR_OFFSET, clrbits, setbits);
}
#else
static void adc_common_cfg(FAR struct stm32_dev_s *priv)
{
  /* Do nothing here */

  UNUSED(priv);
}
#endif

#ifdef ADC_HAVE_DMA
/****************************************************************************
 * Name: adc_dma_cfg
 ****************************************************************************/

#ifdef HAVE_IP_ADC_V2
static void adc_dma_cfg(FAR struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  /* Set DMA mode */

  if (priv->dmacfg == 0)
    {
      /* One Shot Mode */

      clrbits |= ADC_CFGR1_DMACFG;
    }
  else
    {
      /* Circular Mode */

      setbits |= ADC_CFGR1_DMACFG;
    }

  /* Enable DMA */

  setbits |= ADC_CFGR1_DMAEN;

  /* Modify CFGR configuration */

  adc_modifyreg(priv, STM32_ADC_CFGR1_OFFSET, clrbits, setbits);
}
#else
static void adc_dma_cfg(FAR struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

#ifdef ADC_HAVE_DMACFG
  /* Set DMA mode */

  if (priv->dmacfg == 0)
    {
      /* One Shot Mode */

      clrbits |= ADC_CR2_DDS;
    }
  else
    {
      /* Circular Mode */

      setbits |= ADC_CR2_DDS;
    }
#endif

  /* Enable DMA */

  setbits |= ADC_CR2_DMA;

  /* Modify CR2 configuration */

  adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, clrbits, setbits);
}
#endif

/****************************************************************************
 * Name: adc_dma_start
 ****************************************************************************/

static void adc_dma_start(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  /* Stop and free DMA if it was started before */

  if (priv->dma != NULL)
    {
      stm32_dmastop(priv->dma);
      stm32_dmafree(priv->dma);
    }

  priv->dma = stm32_dmachannel(priv->dmachan);

#ifndef CONFIG_STM32_ADC_NOIRQ
  /* Start DMA only if standard ADC interrupts used */

  stm32_dmasetup(priv->dma,
                 priv->base + STM32_ADC_DR_OFFSET,
                 (uint32_t)priv->r_dmabuffer,
                 priv->rnchannels,
                 ADC_DMA_CONTROL_WORD);

  stm32_dmastart(priv->dma, adc_dmaconvcallback, dev, false);
#endif
}
#endif /* ADC_HAVE_DMA */

/****************************************************************************
 * Name: adc_configure
 ****************************************************************************/

static void adc_configure(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  /* Turn off the ADC before configuration */

  adc_enable(priv, false);

  /* Configure voltage regulator if present */

  adc_voltreg_cfg(priv);

  /* Calibrate ADC - doesn't work for now */

  adc_calibrate(priv);

  /* Initialize the ADC watchdog */

  adc_watchdog_cfg(priv);

  /* Initialize the ADC sample time */

  adc_sampletime_cfg(dev);

  /* Set ADC working mode */

  adc_mode_cfg(priv);

  /* Configuration of the channel conversions */

  if (priv->cr_channels > 0)
    {
      adc_set_ch(dev, 0);
    }

#ifdef ADC_HAVE_INJECTED
  /* Configuration of the injected channel conversions after adc enabled */

  if (priv->cj_channels > 0)
    {
      adc_inj_set_ch(dev, 0);
    }
#endif

  /* ADC common register configuration */

  adc_common_cfg(priv);

#ifdef ADC_HAVE_DMA
  /* Configure ADC DMA if enabled */

  if (priv->hasdma)
    {
      /* Configure ADC DMA */

      adc_dma_cfg(priv);

      /* Start ADC DMA */

      adc_dma_start(dev);
    }
#endif

#ifdef HAVE_ADC_RESOLUTION
  /* Configure ADC resolution */

  adc_resolution_set(dev, priv->resolution);
#endif

#ifdef ADC_HAVE_EXTCFG
  /* Configure external event for regular group */

  adc_extcfg_set(priv, priv->extcfg);
#endif

  /* Enable ADC */

  adc_enable(priv, true);

#ifdef ADC_HAVE_JEXTCFG
  /* Configure external event for injected group when ADC enabled */

  adc_jextcfg_set(priv, priv->jextcfg);

#if defined(HAVE_IP_ADC_V2)
  /* For ADC IPv2 there is queue of context for injected conversion.
   * JEXTCFG configuration is the second write to JSQR register which means
   * configuration is stored on queue.
   * We trigger single INJ conversion here to update context.
   */

  adc_inj_startconv(priv, true);
#endif
#endif

  /* Dump regs */

  adc_dumpregs(priv);
}

/****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware.
 *   This is called, before adc_setup() and on error conditions.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  irqstate_t flags;

  ainfo("intf: %d\n", priv->intf);
  flags = enter_critical_section();

  /* Do nothing if ADC instance is currently in use */

  if (priv->initialized > 0)
    {
      goto out;
    }

#ifdef HAVE_HSI_CONTROL
  /* The STM32L15XX family uses HSI as an independent clock-source
   * for the ADC
   */

  adc_enable_hsi(true);
#endif

#if defined(HAVE_IP_ADC_V2)
  /* Turn off the ADC so we can write the RCC bits */

  adc_enable(priv, false);
#endif

  /* Only if this is the first initialzied ADC instance in the ADC block */

#ifdef HAVE_ADC_CMN_DATA
  if (adccmn_lock(priv, true) < 0)
    {
      goto out;
    }

  if (priv->cmn->refcount == 0)
#endif
    {
      /* Enable ADC reset state */

      adc_rccreset(priv, true);

      /* Release ADC from reset state */

      adc_rccreset(priv, false);
    }

#ifdef HAVE_ADC_CMN_DATA
  adccmn_lock(priv, false);
#endif

out:
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: adc_reset_hsi_disable
 *
 * Description:
 *   Reset the ADC device with HSI and ADC shut down. Called early to
 *   initialize the hardware. This is called, before adc_setup() and on
 *   error conditions. In STM32L15XX case sometimes HSI must be shut
 *   down after the first initialization
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef HAVE_HSI_CONTROL
static void adc_reset_hsi_disable(FAR struct adc_dev_s *dev)
{
    adc_reset(dev);
    adc_shutdown(dev);
}
#endif

/****************************************************************************
 * Name: adc_setup
 *
 * Description:
 *   Configure the ADC. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int ret = OK;

  /* Do nothing when the ADC device is already set up */

  if (priv->initialized > 0)
    {
      return OK;
    }

  /* Attach the ADC interrupt */

#ifndef CONFIG_STM32_ADC_NOIRQ
  ret = irq_attach(priv->irq, priv->isr, NULL);
  if (ret < 0)
    {
      ainfo("irq_attach failed: %d\n", ret);
      return ret;
    }
#endif

  /* Make sure that the ADC device is in the powered up, reset state */

  adc_reset(dev);

  /* Configure ADC device */

  adc_configure(dev);

#ifdef ADC_HAVE_TIMER
  /* Configure timer */

  if (priv->tbase != 0)
    {
      ret = adc_timinit(priv);
      if (ret < 0)
        {
          aerr("ERROR: adc_timinit failed: %d\n", ret);
        }
    }
#endif

  /* As default conversion is started here.
   *
   * NOTE: for ADC IPv2 (J)ADSTART bit must be set to start ADC conversion
   *       even if hardware trigger is selected.
   *       This can be done here during the opening of the ADC device
   *       or later with ANIOC_TRIGGER ioctl call.
   */

#ifndef CONFIG_STM32_ADC_NO_STARTUP_CONV
  /* Start regular conversion */

  adc_reg_startconv(priv, true);

#  ifdef ADC_HAVE_INJECTED
  /* Start injected conversion */

  adc_inj_startconv(priv, true);
#  endif
#endif

#ifdef HAVE_ADC_CMN_DATA
  /* Increase instances counter */

  ret = adccmn_lock(priv, true);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->cmn->refcount == 0)
#endif
    {
      /* Enable the ADC interrupt */

#ifndef CONFIG_STM32_ADC_NOIRQ
      ainfo("Enable the ADC interrupt: irq=%d\n", priv->irq);
      up_enable_irq(priv->irq);
#endif
    }

#ifdef HAVE_ADC_CMN_DATA
  priv->cmn->refcount += 1;
  adccmn_lock(priv, false);
#endif

  /* The ADC device is ready */

  priv->initialized += 1;

  return ret;
}

/****************************************************************************
 * Name: adc_shutdown
 *
 * Description:
 *   Disable the ADC.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  /* Decrement count only when ADC device is in use */

  if (priv->initialized > 0)
    {
      priv->initialized -= 1;
    }

  /* Shutdown the ADC device only when not in use */

  if (priv->initialized > 0)
    {
      return;
    }

  /* Disable ADC */

  adc_enable(priv, false);

#ifdef HAVE_HSI_CONTROL
  adc_enable_hsi(false);
#endif

#ifdef HAVE_ADC_CMN_DATA
  if (adccmn_lock(priv, true) < 0)
    {
      return;
    }

  if (priv->cmn->refcount <= 1)
#endif
    {
#ifndef CONFIG_STM32_ADC_NOIRQ
      /* Disable ADC interrupts and detach the ADC interrupt handler */

      up_disable_irq(priv->irq);
      irq_detach(priv->irq);
#endif

      /* Disable and reset the ADC module.
       *
       * NOTE: The ADC block will be reset to its reset state only if all
       *       ADC block instances are closed. This means that the closed
       *       ADC may not be reset which in turn may affect low-power
       *       applications. (But ADC is turned off here, is not that
       *       enough?)
       */

      adc_rccreset(priv, true);
    }

#ifdef ADC_HAVE_TIMER
  /* Disable timer */

  if (priv->tbase != 0)
    {
      adc_timstart(priv, false);
    }
#endif

#ifdef HAVE_ADC_CMN_DATA
  /* Decrease instances counter */

  if (priv->cmn->refcount > 0)
    {
      priv->cmn->refcount -= 1;
    }

  adccmn_lock(priv, false);
#endif
}

/****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  uint32_t regval;

  ainfo("intf: %d enable: %d\n", priv->intf, enable ? 1 : 0);

  if (enable)
    {
      /* Enable the analog watchdog / overrun interrupts, and if no DMA,
       * end-of-conversion ADC.
       */

      regval = ADC_IER_ALLINTS;
#ifdef ADC_HAVE_DMA
      if (priv->hasdma)
        {
          regval &= ~(ADC_IER_EOC | ADC_IER_JEOC);
        }
#endif

      adc_modifyreg(priv, STM32_ADC_IER_OFFSET, 0, regval);
    }
  else
    {
      /* Disable all ADC interrupts */

      adc_modifyreg(priv, STM32_ADC_IER_OFFSET, ADC_IER_ALLINTS, 0);
    }
}

/****************************************************************************
 * Name: adc_enable_tvref_register
 *
 * Description:
 *   Enable/disable the temperature sensor and the VREFINT channel.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   enable - true:  Temperature sensor and V REFINT channel enabled
 *                   (ch 16 and 17)
 *            false: Temperature sensor and V REFINT channel disabled
 *                   (ch 16 and 17)
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if defined(HAVE_IP_ADC_V1)
static void adc_ioc_enable_tvref_register(FAR struct adc_dev_s *dev,
                                          bool enable)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

#ifdef HAVE_BASIC_ADC
#  if defined(CONFIG_STM32_ADC1)
  /* TSVREF bit is only available in the STM32_ADC1_CR2 register. */

  if (priv->intf == 1)
    {
      if (enable)
        {
          adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, 0, ADC_CR2_TSVREFE);
        }
      else
        {
          adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, ADC_CR2_TSVREFE, 0);
        }
    }

  ainfo("STM32_ADC_CR2 value: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_CR2_OFFSET));
#  endif /* CONFIG_STM32_ADC1 */
#else    /* !HAVE_BASIC_ADC */
  if (enable)
    {
      adccmn_modifyreg(priv, STM32_ADC_CCR_OFFSET, 0, ADC_CCR_TSVREFE);
    }
  else
    {
      adccmn_modifyreg(priv, STM32_ADC_CCR_OFFSET, ADC_CCR_TSVREFE, 0);
    }

  ainfo("STM32_ADC_CCR value: 0x%08" PRIx32 "\n",
        adccmn_getreg(priv, STM32_ADC_CCR_OFFSET));
#endif
}
#endif /* HAVE_IP_ADC_V1 */

/****************************************************************************
 * Name: adc_resolution_set
 ****************************************************************************/

#ifdef HAVE_ADC_RESOLUTION
static int adc_resolution_set(FAR struct adc_dev_s *dev, uint8_t res)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int ret = OK;

  /* Check input */

  if (res > 3)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Modify appropriate register */

#if defined(HAVE_IP_ADC_V1)
  adc_modifyreg(priv, STM32_ADC_CR1_OFFSET, ADC_CR1_RES_MASK,
                res << ADC_CR1_RES_SHIFT);
#elif defined(HAVE_IP_ADC_V2)
  adc_modifyreg(priv, STM32_ADC_CFGR1_OFFSET, ADC_CFGR1_RES_MASK,
                res << ADC_CFGR1_RES_SHIFT);
#endif

errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: adc_extcfg_set
 ****************************************************************************/

#ifdef ADC_HAVE_EXTCFG
static int adc_extcfg_set(FAR struct stm32_dev_s *priv, uint32_t extcfg)
{
  uint32_t exten  = 0;
  uint32_t extsel = 0;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  /* Get EXTEN and EXTSEL from input */

  exten = (extcfg & ADC_EXTREG_EXTEN_MASK);
  extsel = (extcfg & ADC_EXTREG_EXTSEL_MASK);

  /* EXTSEL selection: These bits select the external event used
   * to trigger the start of conversion of a regular group.  NOTE:
   *
   * - The position with of the EXTSEL field varies from one STM32 MCU
   *   to another.
   * - The width of the EXTSEL field varies from one STM32 MCU to another.
   */

  if (exten > 0)
    {
      setbits = (extsel | exten);
      clrbits = (ADC_EXTREG_EXTEN_MASK | ADC_EXTREG_EXTSEL_MASK);

      ainfo("Initializing extsel = 0x%08" PRIx32 "\n", extsel);

      /* Write register */

      adc_modifyreg(priv, STM32_ADC_EXTREG_OFFSET, clrbits, setbits);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: adc_jextcfg_set
 ****************************************************************************/

#ifdef ADC_HAVE_JEXTCFG
static int adc_jextcfg_set(FAR struct stm32_dev_s *priv, uint32_t jextcfg)
{
  uint32_t jexten =  0;
  uint32_t jextsel = 0;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  /* Get JEXTEN and JEXTSEL from input */

  jexten = (jextcfg & ADC_JEXTREG_JEXTEN_MASK);
  jextsel = (jextcfg & ADC_JEXTREG_JEXTSEL_MASK);

  /* JEXTSEL selection: These bits select the external event used
   * to trigger the start of conversion of a injected group.  NOTE:
   *
   * - The position with of the JEXTSEL field varies from one STM32 MCU
   *   to another.
   * - The width of the JEXTSEL field varies from one STM32 MCU to another.
   */

  if (jexten > 0)
    {
      setbits = (jexten | jextsel);
      clrbits = (ADC_JEXTREG_JEXTEN_MASK | ADC_JEXTREG_JEXTSEL_MASK);

      ainfo("Initializing jextsel = 0x%08" PRIx32 "\n", jextsel);

      /* Write register */

      adc_modifyreg(priv, STM32_ADC_JEXTREG_OFFSET, clrbits, setbits);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: adc_dumpregs
 ****************************************************************************/

static void adc_dumpregs(FAR struct stm32_dev_s *priv)
{
  UNUSED(priv);

#if defined(HAVE_IP_ADC_V2)
  ainfo("ISR:  0x%08" PRIx32 " IER:  0x%08" PRIx32
        " CR:   0x%08" PRIx32 " CFGR1: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_ISR_OFFSET),
        adc_getreg(priv, STM32_ADC_IER_OFFSET),
        adc_getreg(priv, STM32_ADC_CR_OFFSET),
        adc_getreg(priv, STM32_ADC_CFGR1_OFFSET));
#else
  ainfo("SR:   0x%08" PRIx32 " CR1:  0x%08" PRIx32
        " CR2:  0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SR_OFFSET),
        adc_getreg(priv, STM32_ADC_CR1_OFFSET),
        adc_getreg(priv, STM32_ADC_CR2_OFFSET));
#endif

  ainfo("SQR1: 0x%08" PRIx32 " SQR2: 0x%08" PRIx32
        " SQR3: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SQR1_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR2_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR3_OFFSET));

  ainfo("SMPR1: 0x%08" PRIx32 " SMPR2: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SMPR1_OFFSET),
        adc_getreg(priv, STM32_ADC_SMPR2_OFFSET));

#if defined(STM32_ADC_SQR4_OFFSET)
  ainfo("SQR4: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SQR4_OFFSET));
#endif

#if defined(STM32_ADC_SQR5_OFFSET)
  ainfo("SQR5: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SQR5_OFFSET));
#endif

#ifdef ADC_HAVE_INJECTED
  ainfo("JSQR: 0x%08" PRIx32 "\n", adc_getreg(priv, STM32_ADC_JSQR_OFFSET));
#endif

#if defined(HAVE_IP_ADC_V2) || (defined(HAVE_IP_ADC_V1) && !defined(HAVE_BASIC_ADC))
  ainfo("CCR:  0x%08" PRIx32 "\n",
        adccmn_getreg(priv, STM32_ADC_CCR_OFFSET));
#endif
}

/****************************************************************************
 * Name: adc_enable_vbat_channel
 *
 * Description:
 *   Enable/disable the Vbat voltage measurement channel.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   enable - true:  Vbat input channel enabled (ch 18)
 *            false: Vbat input channel disabled (ch 18)
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef HAVE_ADC_VBAT
static void adc_enable_vbat_channel(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  if (enable)
    {
      adccmn_modifyreg(priv, STM32_ADC_CCR_OFFSET, 0, ADC_CCR_VBATEN);
    }
  else
    {
      adccmn_modifyreg(priv, STM32_ADC_CCR_OFFSET, ADC_CCR_VBATEN, 0);
    }

  ainfo("STM32_ADC_CCR value: 0x%08" PRIx32 "\n",
        adccmn_getreg(priv, STM32_ADC_CCR_OFFSET));
}
#endif

/****************************************************************************
 * Name: adc_ioc_change_sleep_between_opers
 *
 * Description:
 *   Changes PDI and PDD bits to save battery.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   cmd - command
 *   arg - arguments passed with command
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef HAVE_ADC_POWERDOWN
static int adc_ioc_change_sleep_between_opers(FAR struct adc_dev_s *dev,
                                              int cmd, bool arg)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int ret = OK;

  adc_enable(priv, false);

  switch (cmd)
    {
      case IO_ENABLE_DISABLE_PDI:
        adc_power_down_idle(priv, arg);
        break;

      case IO_ENABLE_DISABLE_PDD:
        adc_power_down_delay(priv, arg);
        break;

      case IO_ENABLE_DISABLE_PDD_PDI:
        adc_power_down_idle(priv, arg);
        adc_power_down_delay(priv, arg);
        break;

      default:
        ainfo("unknown cmd: %d\n", cmd);
        break;
    }

  adc_enable(priv, true);

  return ret;
}
#endif

/****************************************************************************
 * Name: adc_ioc_enable_awd_int
 *
 * Description:
 *   Turns ON/OFF ADC analog watchdog interrupt.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   arg - true:  Turn ON interrupt
 *         false: Turn OFF interrupt
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_ioc_enable_awd_int(FAR struct stm32_dev_s *priv, bool enable)
{
  if (enable)
    {
      adc_modifyreg(priv, STM32_ADC_IER_OFFSET, 0, ADC_IER_AWD);
    }
  else
    {
      adc_modifyreg(priv, STM32_ADC_IER_OFFSET, ADC_IER_AWD, 0);
    }
}

/****************************************************************************
 * Name: adc_ioc_enable_eoc_int
 *
 * Description:
 *   Turns ON/OFF ADC EOC interrupt.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   arg - true:  Turn ON interrupt
 *         false: Turn OFF interrupt
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_ioc_enable_eoc_int(FAR struct stm32_dev_s *priv, bool enable)
{
  if (enable)
    {
      adc_modifyreg(priv, STM32_ADC_IER_OFFSET, 0, ADC_IER_EOC);
    }
  else
    {
      adc_modifyreg(priv, STM32_ADC_IER_OFFSET, ADC_IER_EOC, 0);
    }
}

/****************************************************************************
 * Name: adc_ioc_enable_jeoc_int
 *
 * Description:
 *   Turns ON/OFF ADC injected channels interrupt.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   arg - true:  Turn ON interrupt
 *         false: Turn OFF interrupt
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_ioc_enable_jeoc_int(FAR struct stm32_dev_s *priv,
                                    bool enable)
{
  if (enable)
    {
      adc_modifyreg(priv, STM32_ADC_IER_OFFSET, 0, ADC_IER_JEOC);
    }
  else
    {
      adc_modifyreg(priv, STM32_ADC_IER_OFFSET, ADC_IER_JEOC, 0);
    }
}

/****************************************************************************
 * Name: adc_ioc_enable_ovr_int
 *
 * Description:
 *   Turns ON/OFF ADC overrun interrupt.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   arg - true:  Turn ON interrupt
 *         false: Turn OFF interrupt
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_ioc_enable_ovr_int(FAR struct stm32_dev_s *priv, bool enable)
{
  if (enable)
    {
      adc_modifyreg(priv, STM32_ADC_IER_OFFSET, 0, ADC_IER_OVR);
    }
  else
    {
      adc_modifyreg(priv, STM32_ADC_IER_OFFSET, ADC_IER_OVR, 0);
    }
}

/****************************************************************************
 * Name: adc_ioc_change_ints
 *
 * Description:
 *   Turns ON/OFF ADC interrupts.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   cmd - command
 *   arg - arguments passed with command
 *
 * Returned Value:
 *
 ****************************************************************************/

static int adc_ioc_change_ints(FAR struct adc_dev_s *dev, int cmd, bool arg)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case IO_ENABLE_DISABLE_AWDIE:
        adc_ioc_enable_awd_int(priv, arg);
        break;

      case IO_ENABLE_DISABLE_EOCIE:
        adc_ioc_enable_eoc_int(priv, arg);
        break;

      case IO_ENABLE_DISABLE_JEOCIE:
        adc_ioc_enable_jeoc_int(priv, arg);
        break;

      case IO_ENABLE_DISABLE_OVRIE:
        adc_ioc_enable_ovr_int(priv, arg);
        break;

      case IO_ENABLE_DISABLE_ALL_INTS:
        adc_ioc_enable_awd_int(priv, arg);
        adc_ioc_enable_eoc_int(priv, arg);
        adc_ioc_enable_jeoc_int(priv, arg);
        adc_ioc_enable_ovr_int(priv, arg);
        break;

      default:
        ainfo("unknown cmd: %d\n", cmd);
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: adc_ioc_wait_rcnr_zeroed
 *
 * Description:
 *   For the STM3215XX-family the ADC_SR_RCNR bit must be zeroed,
 *   before next conversion.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_STM32L15XX
static int adc_ioc_wait_rcnr_zeroed(FAR struct stm32_dev_s *priv)
{
  int i;

  for (i = 0; i < 30000; i++)
    {
      if ((adc_getreg(priv, STM32_ADC_SR_OFFSET) & ADC_SR_RCNR) == 0)
        {
          return OK;
        }
    }

  return -ENODATA;
}
#endif

/****************************************************************************
 * Name: adc_enable_hsi
 *
 * Description:
 *   Enable/Disable HSI clock
 *
 * Input Parameters:
 *   enable - true  : HSI clock for ADC enabled
 *            false : HSI clock for ADC disabled
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef HAVE_HSI_CONTROL
static void adc_enable_hsi(bool enable)
{
  if (enable)
    {
      /* Enable the HSI */

      stm32_modifyreg32(STM32_RCC_CR, 0, RCC_CR_HSION);
      while ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) == 0);
    }
  else
    {
      /* Disable the HSI */

      stm32_modifyreg32(STM32_RCC_CR, RCC_CR_HSION, 0);
    }
}
#endif

/****************************************************************************
 * Name: adc_sqrbits
 ****************************************************************************/

static uint32_t adc_sqrbits(FAR struct stm32_dev_s *priv, int first,
                            int last, int offset)
{
  uint32_t bits = 0;
  int i;

  for (i = first - 1;
       i < priv->rnchannels && i < last;
       i++, offset += ADC_SQ_OFFSET)
    {
      bits |= (uint32_t)priv->r_chanlist[i] << offset;
    }

  return bits;
}

/****************************************************************************
 * Name: adc_set_ch
 *
 * Description:
 *   Sets the ADC channel.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   ch  - ADC channel number + 1. 0 reserved for all configured channels
 *
 * Returned Value:
 *   int - errno
 *
 ****************************************************************************/

static int adc_set_ch(FAR struct adc_dev_s *dev, uint8_t ch)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  uint32_t bits;
  int i;

  if (ch == 0)
    {
      priv->current   = 0;
      priv->rnchannels = priv->cr_channels;
    }
  else
    {
      for (i = 0; i < priv->cr_channels && priv->r_chanlist[i] != ch - 1;
           i++);

      if (i >= priv->cr_channels)
        {
          return -ENODEV;
        }

      priv->current   = i;
      priv->rnchannels = 1;
    }

#ifdef STM32_ADC_SQR5_OFFSET
  bits = adc_sqrbits(priv, ADC_SQR5_FIRST, ADC_SQR5_LAST,
                     ADC_SQR5_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR5_OFFSET, ~ADC_SQR5_RESERVED, bits);
#endif

#ifdef STM32_ADC_SQR4_OFFSET
  bits = adc_sqrbits(priv, ADC_SQR4_FIRST, ADC_SQR4_LAST,
                     ADC_SQR4_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR4_OFFSET, ~ADC_SQR4_RESERVED, bits);
#endif

  bits = adc_sqrbits(priv, ADC_SQR3_FIRST, ADC_SQR3_LAST,
                     ADC_SQR3_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR3_OFFSET, ~ADC_SQR3_RESERVED, bits);

  bits = adc_sqrbits(priv, ADC_SQR2_FIRST, ADC_SQR2_LAST,
                     ADC_SQR2_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR2_OFFSET, ~ADC_SQR2_RESERVED, bits);

  bits = ((uint32_t)priv->rnchannels - 1) << ADC_SQR1_L_SHIFT;
  bits |= adc_sqrbits(priv, ADC_SQR1_FIRST,
                      ADC_SQR1_LAST, ADC_SQR1_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR1_OFFSET, ~ADC_SQR1_RESERVED, bits);

  return OK;
}

#ifdef ADC_HAVE_INJECTED

/****************************************************************************
 * Name: adc_inj_set_ch
 ****************************************************************************/

static int adc_inj_set_ch(FAR struct adc_dev_s *dev, uint8_t ch)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  uint32_t clrbits;
  uint32_t setbits;
  int i;

  /* Configure injected sequence length */

  setbits = ADC_JSQR_JL(priv->cj_channels);
  clrbits = ADC_JEXTREG_JEXTSEL_MASK | ADC_JSQR_JL_MASK;

  /* Configure injected channels */

  for (i = 0 ; i < priv->cj_channels; i += 1)
    {
#if defined(HAVE_IP_ADC_V1)
      /* Injected channels sequence for for ADC IPv1:
       *
       *           1      2     3      4
       *   IL=1: JSQR4,
       *   IL=2: JSQR3, JSQR4
       *   IL=3: JSQR2, JSQR3, JSQR4
       *   IL=4: JSQR1, JSQR2, JSQR3, JSQR4
       */

      setbits |= (priv->j_chanlist[priv->cj_channels - 1 - i] <<
                  (ADC_JSQR_JSQ4_SHIFT - ADC_JSQR_JSQ_SHIFT * i));
#else
      setbits |= priv->j_chanlist[i] << (ADC_JSQR_JSQ1_SHIFT +
                                         ADC_JSQR_JSQ_SHIFT * i);
#endif
    }

  /* Write register */

  adc_modifyreg(priv, STM32_ADC_JSQR_OFFSET, clrbits, setbits);

  return OK;
}
#endif

/****************************************************************************
 * Name: adc_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   cmd - command
 *   arg - arguments passed with command
 *
 * Returned Value:
 *
 ****************************************************************************/

static int adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int ret                      = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          /* Start regular conversion if regular channels configured */

          if (priv->anioc_trg & ANIOC_TRIGGER_REGULAR)
            {
              if (priv->cr_channels > 0)
                {
                  adc_reg_startconv(priv, true);
                }
            }

#ifdef ADC_HAVE_INJECTED
          /* Start injected conversion if injected channels configured */

          if (priv->anioc_trg & ANIOC_TRIGGER_INJECTED)
            {
              if (priv->cj_channels > 0)
                {
                  adc_inj_startconv(priv, true);
                }
            }
#endif

          break;
        }

      case IO_TRIGGER_REG:
        {
          /* Start regular conversion if regular channels configured */

          if (priv->cr_channels > 0)
            {
              adc_reg_startconv(priv, true);
            }

          break;
        }

#ifdef ADC_HAVE_INJECTED
      case IO_TRIGGER_INJ:
        {
          /* Start injected conversion if injected channels configured */

          if (priv->cj_channels > 0)
            {
              adc_inj_startconv(priv, true);
            }

          break;
        }
#endif

      case IO_ENABLE_DISABLE_AWDIE:
      case IO_ENABLE_DISABLE_EOCIE:
      case IO_ENABLE_DISABLE_JEOCIE:
      case IO_ENABLE_DISABLE_OVRIE:
      case IO_ENABLE_DISABLE_ALL_INTS:
        {
          adc_ioc_change_ints(dev, cmd, *(bool *)arg);
          break;
        }

#if defined(HAVE_IP_ADC_V1)
      case IO_ENABLE_TEMPER_VOLT_CH:
        {
          adc_ioc_enable_tvref_register(dev, *(bool *)arg);
          break;
        }
#endif

#ifdef HAVE_ADC_VBAT
      case IO_ENABLE_DISABLE_VBAT_CH:
        {
          adc_enable_vbat_channel(dev, *(bool *)arg);
          break;
        }
#endif

#ifdef HAVE_ADC_POWERDOWN
      case IO_ENABLE_DISABLE_PDI:
      case IO_ENABLE_DISABLE_PDD:
      case IO_ENABLE_DISABLE_PDD_PDI:
        {
          adc_ioc_change_sleep_between_opers(dev, cmd, *(bool *)arg);
          break;
        }
#endif

      case IO_STOP_ADC:
        {
          adc_enable(priv, false);
#ifdef HAVE_HSI_CONTROL
          adc_enable_hsi(false);
#endif
          break;
        }

      case IO_START_ADC:
        {
#ifdef HAVE_HSI_CONTROL
          adc_enable_hsi(true);
#endif
          adc_enable(priv, true);
          break;
        }

      case IO_START_CONV:
        {
          uint8_t ch = ((uint8_t)arg);

#ifdef CONFIG_STM32_STM32L15XX
          ret = adc_ioc_wait_rcnr_zeroed(priv);
          if (ret < 0)
            {
              return ret;
            }
#endif

          ret = adc_set_ch(dev, ch);
          if (ret < 0)
            {
              return ret;
            }

#ifdef CONFIG_ADC
          if (ch)
            {
              /* Clear fifo if upper-half driver enabled */

              dev->ad_recv.af_head = 0;
              dev->ad_recv.af_tail = 0;
            }
#endif

          adc_reg_startconv(priv, true);
          break;
        }

      default:
        {
          aerr("ERROR: Unknown cmd: %d\n", cmd);
          ret = -ENOTTY;
          break;
        }
    }

  return ret;
}

#ifndef CONFIG_STM32_ADC_NOIRQ

/****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   Common ADC interrupt handler.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int adc_interrupt(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  uint32_t regval;
  uint32_t pending;
  int32_t  data;

  regval  = adc_getreg(priv, STM32_ADC_ISR_OFFSET);
  pending = regval & ADC_ISR_ALLINTS;
  if (pending == 0)
    {
      return OK;
    }

  /* Identifies the interruption AWD, OVR or EOC */

  if ((regval & ADC_ISR_AWD) != 0)
    {
      awarn("WARNING: Analog Watchdog, Value converted out of range!\n");
    }

  if ((regval & ADC_ISR_OVR) != 0)
    {
      awarn("WARNING: Overrun has occurred!\n");
    }

  /* EOC: End of conversion */

  if ((regval & ADC_ISR_EOC) != 0)
    {
      /* Read the converted value and clear EOC bit
       * (It is cleared by reading the ADC_DR)
       */

      data = adc_getreg(priv, STM32_ADC_DR_OFFSET) & ADC_DR_RDATA_MASK;

      /* Verify that the upper-half driver has bound its callback functions */

      if (priv->cb != NULL)
        {
          /* Give the ADC data to the ADC driver.  The ADC receive() method
           * accepts 3 parameters:
           *
           * 1) The first is the ADC device instance for this ADC block.
           * 2) The second is the channel number for the data, and
           * 3) The third is the converted data for the channel.
           */

          DEBUGASSERT(priv->cb->au_receive != NULL);
          priv->cb->au_receive(dev, priv->r_chanlist[priv->current], data);
        }

      /* Set the channel number of the next channel that will complete
       * conversion.
       */

      priv->current++;

      if (priv->current >= priv->rnchannels)
        {
          /* Restart the conversion sequence from the beginning */

          priv->current = 0;
        }
    }

  /* Clear pending interrupts */

  adc_putreg(priv, STM32_ADC_ISR_OFFSET, pending);

  return OK;
}

/****************************************************************************
 * Name: adc1_interrupt
 *
 * Description:
 *   ADC interrupt handler for the STM32 L15XX family.
 *
 * Input Parameters:
 *   irq     - The IRQ number that generated the interrupt.
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(STM32_IRQ_ADC1)
static int adc1_interrupt(int irq, FAR void *context, FAR void *arg)
{
  adc_interrupt(&g_adcdev1);

  return OK;
}
#endif

/****************************************************************************
 * Name: adc12_interrupt
 *
 * Description:
 *   ADC1/2 interrupt handler for the STM32 F1/F3 families.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(STM32_IRQ_ADC12) && \
    (defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2))
static int adc12_interrupt(int irq, FAR void *context, FAR void *arg)
{
#ifdef CONFIG_STM32_ADC1
  adc_interrupt(&g_adcdev1);
#endif

#ifdef CONFIG_STM32_ADC2
  adc_interrupt(&g_adcdev2);
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: adc3_interrupt
 *
 * Description:
 *   ADC3 interrupt handler for the STM32 F1 family.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(STM32_IRQ_ADC3) && defined(CONFIG_STM32_ADC3)
static int adc3_interrupt(int irq, FAR void *context, FAR void *arg)
{
  adc_interrupt(&g_adcdev3);

  return OK;
}
#endif

/****************************************************************************
 * Name: adc4_interrupt
 *
 * Description:
 *   ADC4 interrupt handler for the STM32 F3 family.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(STM32_IRQ_ADC4) && defined(CONFIG_STM32_ADC4)
static int adc4_interrupt(int irq, FAR void *context, FAR void *arg)
{
  adc_interrupt(&g_adcdev4);

  return OK;
}
#endif

/****************************************************************************
 * Name: adc123_interrupt
 *
 * Description:
 *   ADC1/2/3 interrupt handler for the STM32 F2/F4 families.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(STM32_IRQ_ADC)
static int adc123_interrupt(int irq, FAR void *context, FAR void *arg)
{
#ifdef CONFIG_STM32_ADC1
  adc_interrupt(&g_adcdev1);
#endif

#ifdef CONFIG_STM32_ADC2
  adc_interrupt(&g_adcdev2);
#endif

#ifdef CONFIG_STM32_ADC3
  adc_interrupt(&g_adcdev3);
#endif

  return OK;
}
#endif
#endif /* CONFIG_STM32_ADC_NOIRQ */

#ifdef CONFIG_STM32_ADC_LL_OPS

/****************************************************************************
 * Name: adc_llops_setup
 ****************************************************************************/

static int adc_llops_setup(FAR struct stm32_adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  return adc_setup(priv->dev);
}

/****************************************************************************
 * Name: adc_llops_shutdown
 ****************************************************************************/

static void adc_llops_shutdown(FAR struct stm32_adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  adc_shutdown(priv->dev);
}

/****************************************************************************
 * Name: adc_intack
 ****************************************************************************/

static void adc_intack(FAR struct stm32_adc_dev_s *dev, uint32_t source)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  /* Clear pending interrupts */

#ifdef HAVE_IP_ADC_V2
  /* Cleared by writing 1 to it */

  adc_putreg(priv, STM32_ADC_ISR_OFFSET, (source & ADC_ISR_ALLINTS));
#else
  /* Cleared by writing 0 to it */

  adc_modifyreg(priv, STM32_ADC_ISR_OFFSET, (source & ADC_ISR_ALLINTS), 0);
#endif
}

/****************************************************************************
 * Name: adc_inten
 ****************************************************************************/

static void adc_inten(FAR struct stm32_adc_dev_s *dev, uint32_t source)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  /* Enable interrupts */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, 0, (source & ADC_IER_ALLINTS));
}

/****************************************************************************
 * Name: adc_intdis
 ****************************************************************************/

static void adc_intdis(FAR struct stm32_adc_dev_s *dev, uint32_t source)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  /* Disable interrupts */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, (source & ADC_IER_ALLINTS), 0);
}

/****************************************************************************
 * Name: adc_ackget
 ****************************************************************************/

static uint32_t adc_intget(FAR struct stm32_adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;
  uint32_t regval;
  uint32_t pending;

  regval  = adc_getreg(priv, STM32_ADC_ISR_OFFSET);
  pending = regval & ADC_ISR_ALLINTS;

  return pending;
}

/****************************************************************************
 * Name: adc_regget
 ****************************************************************************/

static uint32_t adc_regget(FAR struct stm32_adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  return adc_getreg(priv, STM32_ADC_DR_OFFSET) & ADC_DR_RDATA_MASK;
}

/****************************************************************************
 * Name: adc_llops_reg_startconv
 ****************************************************************************/

static void adc_llops_reg_startconv(FAR struct stm32_adc_dev_s *dev,
                                    bool enable)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  adc_reg_startconv(priv, enable);
}

/****************************************************************************
 * Name: adc_offset_set
 ****************************************************************************/

#ifdef HAVE_IP_ADC_V2
static int adc_offset_set(FAR struct stm32_adc_dev_s *dev, uint8_t ch,
                          uint8_t i, uint16_t offset)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;
  uint32_t regval = 0;
  uint32_t reg    = 0;
  int      ret    = OK;

  if (i >= 4)
    {
      /* There are only four offset registers. */

      ret = -E2BIG;
      goto errout;
    }

  reg = STM32_ADC_OFR1_OFFSET + i * 4;

  regval = ADC_OFR_OFFSETY_EN;
  adc_putreg(priv, reg, regval);

  regval |= ADC_OFR_OFFSETY_CH(ch) | ADC_OFR_OFFSETY(offset);
  adc_putreg(priv, reg, regval);

errout:
  return ret;
}
#else  /* HAVE_IP_ADC_V1 */
static int adc_offset_set(FAR struct stm32_adc_dev_s *dev, uint8_t ch,
                          uint8_t i, uint16_t offset)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;
  uint32_t reg = 0;
  int      ret = OK;

  /* WARNING: Offset only for injected channels! */

  UNUSED(ch);

  if (i >= 4)
    {
      /* There are only four offset registers. */

      ret = -E2BIG;
      goto errout;
    }

  reg = STM32_ADC_JOFR1_OFFSET + i * 4;

  adc_putreg(priv, reg, offset);

errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: adc_llops_extcfg_set
 ****************************************************************************/

#ifdef ADC_HAVE_EXTCFG
static void adc_llops_extcfg_set(FAR struct stm32_adc_dev_s *dev,
                                 uint32_t extcfg)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  adc_extcfg_set(priv, extcfg);
}
#endif

/****************************************************************************
 * Name: adc_llops_jextcfg_set
 ****************************************************************************/

#ifdef ADC_HAVE_JEXTCFG
static void  adc_llops_jextcfg_set(FAR struct stm32_adc_dev_s *dev,
                                   uint32_t jextcfg)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  adc_jextcfg_set(priv, jextcfg);
}
#endif

/****************************************************************************
 * Name: adc_regbufregister
 ****************************************************************************/

#ifdef ADC_HAVE_DMA
static int adc_regbufregister(FAR struct stm32_adc_dev_s *dev,
                              uint16_t *buffer, uint8_t len)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  stm32_dmasetup(priv->dma,
                 priv->base + STM32_ADC_DR_OFFSET,
                 (uint32_t)buffer,
                 len,
                 ADC_DMA_CONTROL_WORD);

  /* No DMA callback */

  stm32_dmastart(priv->dma, NULL, dev, false);

  return OK;
}
#endif /* ADC_HAVE_DMA */

/****************************************************************************
 * Name: adc_injget
 ****************************************************************************/

#ifdef ADC_HAVE_INJECTED
static uint32_t adc_injget(FAR struct stm32_adc_dev_s *dev, uint8_t chan)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;
  uint32_t regval = 0;

  if (chan > (priv->cj_channels - 1))
    {
      /* REVISIT: return valute with MSB set to indicate error ? */

      goto errout;
    }

  regval = adc_getreg(priv, STM32_ADC_JDR1_OFFSET + 4 * (chan)) &
           ADC_JDR_JDATA_MASK;

errout:
  return regval;
}

/****************************************************************************
 * Name: adc_llops_inj_startconv
 ****************************************************************************/

static void adc_llops_inj_startconv(FAR struct stm32_adc_dev_s *dev,
                                    bool enable)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  adc_inj_startconv(priv, enable);
}

#endif /* ADC_HAVE_INJECTED */

/****************************************************************************
 * Name: adc_sampletime_write
 *
 * Description:
 *   Writes previously defined values into ADC_SMPRx registers.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_ADC_CHANGE_SAMPLETIME
static void adc_sampletime_write(FAR struct stm32_adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;
  uint32_t value = 0;
  uint8_t i;
  uint8_t shift;

  /* Sampling time individually for each channel.
   * It's different for families.
   */

  for (i = 0, shift = 0; i < priv->adc_channels; i++)
    {
      value |= priv->sample_rate[i] << (shift * 3);
      switch (i)
        {
#if defined(STM32_ADC_SMPR0_OFFSET) && defined(STM32_ADC_SMPR3_OFFSET)
          case 9:
            {
              adc_putreg(priv, STM32_ADC_SMPR3_OFFSET, value);
              shift = 0;
              value = 0;
              break;
            }

          case 19:
            {
              adc_putreg(priv, STM32_ADC_SMPR2_OFFSET, value);
              shift = 0;
              value = 0;
              break;
            }

          case 29:
            {
              adc_putreg(priv, STM32_ADC_SMPR1_OFFSET, value);
              shift = 0;
              value = 0;
              break;
            }

          case (ADC_CHANNELS_NUMBER - 1):
            {
              adc_putreg(priv, STM32_ADC_SMPR0_OFFSET, value);
              shift = 0;
              value = 0;
              break;
            }

#elif defined(STM32_ADC_SMPR1_OFFSET) && defined(STM32_ADC_SMPR2_OFFSET)
          case (ADC_CHANNELS_NUMBER - 1):
            {
              adc_putreg(priv, STM32_ADC_SMPR2_OFFSET, value);
              shift = 0;
              value = 0;
              break;
            }

          case 9:
            {
              adc_putreg(priv, STM32_ADC_SMPR1_OFFSET, value);
              shift = 0;
              value = 0;
              break;
            }
#else
#  error "Not supported SMPRx configuration"
#endif

          default:
            {
              shift++;
              break;
            }
        }
    }
}

/****************************************************************************
 * Name: adc_sampletime_set
 *
 * Description:
 *   Changes sample times for specified channels. This method
 *   doesn't make any register writing. So, it's only stores the information.
 *   Values provided by user will be written in registers only on the next
 *   ADC peripheral start, as it was told to do in manual. However, before
 *   very first start, user can call this method and override default values
 *   either for every channels or for only some predefined by user channel(s)
 *
 * Input Parameters:
 *   priv     - pointer to the adc device structure
 *   pdi_high - true:  The ADC is powered down when waiting for a start event
 *              false: The ADC is powered up when waiting for a start event
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void adc_sampletime_set(FAR struct stm32_adc_dev_s *dev,
                        FAR struct adc_sample_time_s *time_samples)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;
  uint8_t ch_index;
  uint8_t i;

  /* Check if user wants to assign the same value for all channels
   * or just wants to change sample time values for certain channels
   */

  if (time_samples->all_same)
    {
      memset(priv->sample_rate, time_samples->all_ch_sample_time,
             ADC_CHANNELS_NUMBER);
    }
  else
    {
      for (i = 0; i < time_samples->channels_nbr; i++)
        {
          ch_index = time_samples->channel[i].channel;
          if (ch_index >= ADC_CHANNELS_NUMBER)
            {
              break;
            }

          priv->sample_rate[ch_index] = time_samples->channel[i].sample_time;
        }
    }
}
#endif /* CONFIG_STM32_ADC_CHANGE_SAMPLETIME */

/****************************************************************************
 * Name: adc_llops_dumpregs
 ****************************************************************************/

static void adc_llops_dumpregs(FAR struct stm32_adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  adc_dumpregs(priv);
}

#endif /* CONFIG_STM32_ADC_LL_OPS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adcinitialize
 *
 * Description:
 *   Initialize the ADC.
 *
 *   The logic allow initialize ADC regular and injected channels.
 *
 *   The number of injected channels for given ADC is selected from Kconfig
 *   with CONFIG_STM32_ADCx_INJECTED_CHAN definitions
 *
 *   The number of regular channels is obtained from the equation:
 *
 *     cr_channels = channels - cj_channels
 *
 *   where:
 *     cr_channels - regular channels
 *     cj_channels - injected channels
 *     channels    - this function parameter
 *
 *   The chanlist array store both regular channels and injected channels
 *   configuration so that regular channels are the first in order:
 *
 *     # regular channels start from here
 *     chanlist[0]                  -> ADC_SQRx_SQ1
 *     chanlist[1]                  -> ADC_SQRx_SQ2
 *     ...
 *     # injected channels start from here
 *     chanlist[channels - (y - 1)] -> ADC_JSQR_JSQ1
 *     ...
 *     chanlist[channels]           -> ADC_JSQR_ISQy
 *
 *   where:
 *      y = CONFIG_STM32_ADCx_INJECTED_CHAN, and y > 0
 *
 *   If CONFIG_STM32_ADCx_INJECTED_CHAN = 0, then all channels from chanlist
 *   are regular channels.
 *
 * Input Parameters:
 *   intf      - Could be {1,2,3,4} for ADC1, ADC2, ADC3 or ADC4
 *   chanlist  - The list of channels (regular + injected)
 *   channels  - Number of channels (regular + injected)
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *stm32_adcinitialize(int intf, FAR const uint8_t *chanlist,
                                      int channels)
{
  FAR struct adc_dev_s   *dev;
  FAR struct stm32_dev_s *priv;
  uint8_t     cr_channels = 0;
  uint8_t     cj_channels = 0;
#ifdef ADC_HAVE_INJECTED
  FAR uint8_t *j_chanlist = NULL;
#endif

  switch (intf)
    {
#ifdef CONFIG_STM32_ADC1
      case 1:
        {
          ainfo("ADC1 selected\n");
          dev = &g_adcdev1;
          cj_channels = CONFIG_STM32_ADC1_INJECTED_CHAN;
          cr_channels = channels - cj_channels;
#  ifdef ADC_HAVE_INJECTED
          if (cj_channels > 0)
            {
              j_chanlist  = (FAR uint8_t *)chanlist + cr_channels;
            }
#  endif
          break;
        }

#endif /* CONFIG_STM32_ADC1 */
#ifdef CONFIG_STM32_ADC2
      case 2:
        {
          ainfo("ADC2 selected\n");
          dev = &g_adcdev2;
          cj_channels = CONFIG_STM32_ADC2_INJECTED_CHAN;
          cr_channels = channels - cj_channels;
#  ifdef ADC_HAVE_INJECTED
          if (cj_channels > 0)
            {
              j_chanlist  = (FAR uint8_t *)chanlist + cr_channels;
            }
#  endif
          break;
        }

#endif /* CONFIG_STM32_ADC2 */
#ifdef CONFIG_STM32_ADC3
      case 3:
        {
          ainfo("ADC3 selected\n");
          dev = &g_adcdev3;
          cj_channels = CONFIG_STM32_ADC3_INJECTED_CHAN;
          cr_channels = channels - cj_channels;
#  ifdef ADC_HAVE_INJECTED
          if (cj_channels > 0)
            {
              j_chanlist  = (FAR uint8_t *)chanlist + cr_channels;
            }
#  endif
          break;
        }

#endif /* CONFIG_STM32_ADC3 */
#ifdef CONFIG_STM32_ADC4
      case 4:
        {
          ainfo("ADC4 selected\n");
          dev = &g_adcdev4;
          cj_channels = CONFIG_STM32_ADC4_INJECTED_CHAN;
          cr_channels = channels - cj_channels;
#  ifdef ADC_HAVE_INJECTED
          if (cj_channels > 0)
            {
              j_chanlist  = (FAR uint8_t *)chanlist + cr_channels;
            }
#  endif
          break;
        }

#endif /* CONFIG_STM32_ADC4 */
      default:
        {
          aerr("ERROR: No ADC interface defined\n");
          return NULL;
        }
    }

  /* Configure the selected ADC */

  priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  /* Configure regular channels */

  DEBUGASSERT(cr_channels <= CONFIG_STM32_ADC_MAX_SAMPLES);

  priv->cr_channels = cr_channels;
  memcpy(priv->r_chanlist, chanlist, cr_channels);

#ifdef ADC_HAVE_INJECTED
  /* Configure injected channels */

  DEBUGASSERT(cj_channels <= ADC_INJ_MAX_SAMPLES);

  priv->cj_channels = cj_channels;
  memcpy(priv->j_chanlist, j_chanlist, cj_channels);
#endif

#ifdef CONFIG_STM32_ADC_CHANGE_SAMPLETIME
  /* Assign default values for the sample time table */

  memset(priv->sample_rate, ADC_SMPR_DEFAULT, ADC_CHANNELS_NUMBER);
  priv->adc_channels = ADC_CHANNELS_NUMBER;
#endif

#ifdef ADC_HAVE_CB
  priv->cb        = NULL;
#endif

#ifdef CONFIG_STM32_ADC_LL_OPS
  /* Store reference to the upper-half ADC device */

  priv->dev = dev;
#endif

#ifdef ADC_HAVE_INJECTED
  ainfo("intf: %d cr_channels: %d, cj_channels: %d\n",
        intf, priv->cr_channels, priv->cj_channels);
#else
  ainfo("intf: %d cr_channels: %d\n", intf, priv->cr_channels);
#endif

#ifdef HAVE_ADC_CMN_DATA
  /* Initialize the ADC common data semaphore.
   *
   * REVISIT: This will be done several times for each initialzied ADC in
   *          the ADC block.
   */

  nxsem_init(&priv->cmn->lock, 0, 1);
#endif

  return dev;
}

#endif /* CONFIG_STM32_ADC1 || CONFIG_STM32_ADC2 ||
        * CONFIG_STM32_ADC3 || CONFIG_STM32_ADC4
        */
#endif /* CONFIG_STM32_ADC */
