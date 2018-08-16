/****************************************************************************
 * arch/arm/src/stm32/stm32_adc.c
 *
 *   Copyright (C) 2011, 2013, 2015-2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
 *            Paul Alexander Patience <paul-a.patience@polymtl.ca>
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
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "stm32.h"
#include "stm32_dma.h"
#include "stm32_adc.h"

/* ADC "upper half" support must be enabled */

#ifdef CONFIG_STM32_ADC

/* Some ADC peripheral must be enabled */

#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2) || \
    defined(CONFIG_STM32_ADC3) || defined(CONFIG_STM32_ADC4)

/* This implementation is for the STM32 F1, F2, F3, F4 and STM32L15XX only */

#if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F20XX) || \
    defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F37XX) || \
    defined(CONFIG_STM32_STM32F4XXX) || defined(CONFIG_STM32_STM32L15XX)

/* At the moment there is no proper implementation for timers external
 * trigger in STM32L15XX May be added latter
 */

#if defined(ADC_HAVE_TIMER) && defined(CONFIG_STM32_STM32L15XX)
#  warning "There is no proper implementation for TIMER TRIGGERS at the moment"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* RCC reset ****************************************************************/

#if defined(CONFIG_STM32_STM32F10XX)
#  define STM32_RCC_RSTR   STM32_RCC_APB2RSTR
#  define RCC_RSTR_ADC1RST RCC_APB2RSTR_ADC1RST
#  define RCC_RSTR_ADC2RST RCC_APB2RSTR_ADC2RST
#  define RCC_RSTR_ADC3RST RCC_APB2RSTR_ADC3RST
#  define RCC_RSTR_ADC4RST RCC_APB2RSTR_ADC4RST
#elif defined(CONFIG_STM32_STM32F30XX)
#  define STM32_RCC_RSTR   STM32_RCC_AHBRSTR
#  define RCC_RSTR_ADC1RST RCC_AHBRSTR_ADC12RST
#  define RCC_RSTR_ADC2RST RCC_AHBRSTR_ADC12RST
#  define RCC_RSTR_ADC3RST RCC_AHBRSTR_ADC34RST
#  define RCC_RSTR_ADC4RST RCC_AHBRSTR_ADC34RST
#elif defined(CONFIG_STM32_STM32F37XX)
#  define STM32_RCC_RSTR   STM32_RCC_APB2RSTR
#  define RCC_RSTR_ADC1RST RCC_APB2RSTR_ADCRST
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define STM32_RCC_RSTR   STM32_RCC_APB2RSTR
#  define RCC_RSTR_ADC1RST RCC_APB2RSTR_ADCRST
#  define RCC_RSTR_ADC2RST RCC_APB2RSTR_ADCRST
#  define RCC_RSTR_ADC3RST RCC_APB2RSTR_ADCRST
#  define RCC_RSTR_ADC4RST RCC_APB2RSTR_ADCRST
#elif defined(CONFIG_STM32_STM32L15XX)
#  define STM32_RCC_RSTR   STM32_RCC_APB2RSTR
#  define RCC_RSTR_ADC1RST RCC_APB2RSTR_ADC1RST
#endif

/* ADC interrupts ***********************************************************/

#if defined(CONFIG_STM32_STM32F30XX)
#  define STM32_ADC_DMAREG_OFFSET    STM32_ADC_CFGR_OFFSET
#  define ADC_DMAREG_DMA             ADC_CFGR_DMAEN
#  define STM32_ADC_EXTREG_OFFSET    STM32_ADC_CFGR_OFFSET
#  define ADC_EXTREG_EXTSEL_MASK     ADC_CFGR_EXTSEL_MASK
#  define ADC_EXTREG_EXTEN_MASK      ADC_CFGR_EXTEN_MASK
#  define ADC_EXTREG_EXTEN_DEFAULT   ADC_CFGR_EXTEN_RISING
#  define ADC_ISR_EOC                ADC_INT_EOC
#  define ADC_IER_EOC                ADC_INT_EOC
#  define ADC_ISR_AWD                ADC_INT_AWD1
#  define ADC_IER_AWD                ADC_INT_AWD1
#  define ADC_ISR_JEOC               ADC_INT_JEOC
#  define ADC_IER_JEOC               ADC_INT_JEOC
#  define ADC_ISR_OVR                ADC_INT_OVR
#  define ADC_IER_OVR                ADC_INT_OVR
#else
#  define STM32_ADC_DMAREG_OFFSET    STM32_ADC_CR2_OFFSET
#  define ADC_DMAREG_DMA             ADC_CR2_DMA
#  define STM32_ADC_EXTREG_OFFSET    STM32_ADC_CR2_OFFSET
#  define ADC_EXTREG_EXTSEL_MASK     ADC_CR2_EXTSEL_MASK
#  define STM32_ADC_ISR_OFFSET       STM32_ADC_SR_OFFSET
#  define STM32_ADC_IER_OFFSET       STM32_ADC_CR1_OFFSET
#  define ADC_ISR_EOC                ADC_SR_EOC
#  define ADC_IER_EOC                ADC_CR1_EOCIE
#  define ADC_ISR_AWD                ADC_SR_AWD
#  define ADC_IER_AWD                ADC_CR1_AWDIE
#  define ADC_ISR_JEOC               ADC_SR_JEOC
#  define ADC_IER_JEOC               ADC_CR1_JEOCIE
#  if defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F37XX)
#    define ADC_EXTREG_EXTEN_MASK    ADC_CR2_EXTTRIG
#    define ADC_EXTREG_EXTEN_NONE    0
#    define ADC_EXTREG_EXTEN_DEFAULT ADC_CR2_EXTTRIG
#    define ADC_ISR_OVR              0
#    define ADC_IER_OVR              0
#  else
#    define ADC_EXTREG_EXTEN_MASK    ADC_CR2_EXTEN_MASK
#    define ADC_EXTREG_EXTEN_NONE    ADC_CR2_EXTEN_NONE
#    define ADC_EXTREG_EXTEN_DEFAULT ADC_CR2_EXTEN_RISING
#    define ADC_ISR_OVR              ADC_SR_OVR
#    define ADC_IER_OVR              ADC_CR1_OVRIE
#  endif
#endif

#define ADC_ISR_ALLINTS (ADC_ISR_EOC | ADC_ISR_AWD | ADC_ISR_JEOC | \
                         ADC_ISR_OVR)
#define ADC_IER_ALLINTS (ADC_IER_EOC | ADC_IER_AWD | ADC_IER_JEOC | \
                         ADC_IER_OVR)

/* ADC Channels/DMA ********************************************************/
/* The maximum number of channels that can be sampled.  If DMA support is
 * not enabled, then only a single channel can be sampled.  Otherwise,
 * data overruns would occur.
 */

#define ADC_MAX_CHANNELS_DMA   16
#define ADC_MAX_CHANNELS_NODMA 1

#ifdef ADC_HAVE_DMA
#  define ADC_MAX_SAMPLES ADC_MAX_CHANNELS_DMA
#else
#  if defined(CONFIG_STM32_STM32F30XX)
#    define ADC_MAX_SAMPLES ADC_MAX_CHANNELS_DMA /* Works without DMA should sampling frequency be reduced */
#  elif defined(CONFIG_STM32_STM32L15XX)
#    define ADC_MAX_SAMPLES ADC_MAX_CHANNELS_DMA /* Works without DMA as IO_START_CONV can switch channels on the fly */
#  else
#    define ADC_MAX_SAMPLES ADC_MAX_CHANNELS_NODMA
#  endif
#endif

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
#  define ADC_DMA_CONTROL_WORD (DMA_SCR_MSIZE_16BITS | \
                                DMA_SCR_PSIZE_16BITS | \
                                DMA_SCR_MINC | \
                                DMA_SCR_CIRC | \
                                DMA_SCR_DIR_P2M)
#else
#  define ADC_DMA_CONTROL_WORD (DMA_CCR_MSIZE_16BITS | \
                                DMA_CCR_PSIZE_16BITS | \
                                DMA_CCR_MINC | \
                                DMA_CCR_CIRC)
#endif

/* DMA channels and interface values differ for the F1 and F4 families */

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
#elif defined(CONFIG_STM32_STM32F30XX)
#  if defined(ADC_HAVE_DMA) || (ADC_MAX_SAMPLES == 1)
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
#  define ADC_CHANNELS_NUMBER 32
#  define ADC_SMPR_DEFAULT    ADC_SMPR_384
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one ADC block */

struct stm32_dev_s
{
  FAR const struct adc_callback_s *cb;
  uint8_t irq;          /* Interrupt generated by this ADC block */
  uint8_t nchannels;    /* Number of channels */
  uint8_t cchannels;    /* Number of configured channels */
  uint8_t intf;         /* ADC interface number */
  uint8_t current;      /* Current ADC channel being converted */
#ifdef ADC_HAVE_DMA
  uint8_t dmachan;      /* DMA channel needed by this ADC */
  bool    hasdma;       /* True: This channel supports DMA */
#endif
#ifdef CONFIG_STM32_STM32L15XX
  /* Sample time selection. These bits must be written only when ADON=0 */

  uint8_t sample_rate[ADC_CHANNELS_NUMBER];
#endif
#ifdef ADC_HAVE_TIMER
  uint8_t trigger;      /* Timer trigger channel: 0=CC1, 1=CC2, 2=CC3,
                         * 3=CC4, 4=TRGO */
#endif
  xcpt_t   isr;         /* Interrupt handler for this ADC block */
  uint32_t base;        /* Base address of registers unique to this ADC
                         * block */
#ifdef ADC_HAVE_TIMER
  uint32_t tbase;       /* Base address of timer used by this ADC block */
  uint32_t extsel;      /* EXTSEL value used by this ADC block */
  uint32_t pclck;       /* The PCLK frequency that drives this timer */
  uint32_t freq;        /* The desired frequency of conversions */
#endif
#ifdef ADC_HAVE_DMA
  DMA_HANDLE dma;       /* Allocated DMA channel */

  /* DMA transfer buffer */

  uint16_t dmabuffer[ADC_MAX_SAMPLES];
#endif

  /* List of selected ADC channels to sample */

  uint8_t  chanlist[ADC_MAX_SAMPLES];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC Register access */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined(CONFIG_STM32_STM32F37XX) || defined(CONFIG_STM32_STM32F4XXX) || \
    defined(CONFIG_STM32_STM32L15XX)
static void stm32_modifyreg32(unsigned int addr, uint32_t clrbits,
                              uint32_t setbits);
#endif
static uint32_t adc_getreg(FAR struct stm32_dev_s *priv, int offset);
static void     adc_putreg(FAR struct stm32_dev_s *priv, int offset,
                           uint32_t value);
static void     adc_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                              uint32_t clrbits, uint32_t setbits);
#ifdef ADC_HAVE_TIMER
static uint16_t tim_getreg(FAR struct stm32_dev_s *priv, int offset);
static void     tim_putreg(FAR struct stm32_dev_s *priv, int offset,
                           uint16_t value);
static void     tim_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                              uint16_t clrbits, uint16_t setbits);
static void     tim_dumpregs(FAR struct stm32_dev_s *priv,
                             FAR const char *msg);
#endif

static void adc_rccreset(FAR struct stm32_dev_s *priv, bool reset);

/* ADC Interrupt Handler */

static int adc_interrupt(FAR struct adc_dev_s *dev);
#if defined(STM32_IRQ_ADC1) && defined(CONFIG_STM32_ADC1)
static int adc1_interrupt(int irq, FAR void *context, FAR void *arg);
#endif
#if defined(STM32_IRQ_ADC12) && (defined(CONFIG_STM32_ADC1) || \
                                 defined(CONFIG_STM32_ADC2))
static int adc12_interrupt(int irq, FAR void *context, FAR void *arg);
#endif
#if (defined(STM32_IRQ_ADC3) && defined(CONFIG_STM32_ADC3))
static int adc3_interrupt(int irq, FAR void *context, FAR void *arg);
#endif
#if defined(STM32_IRQ_ADC4) && defined(CONFIG_STM32_ADC4)
static int adc4_interrupt(int irq, FAR void *context, FAR void *arg);
#endif
#if defined(STM32_IRQ_ADC)
static int adc123_interrupt(int irq, FAR void *context, FAR void *arg);
#endif

/* ADC Driver Methods */

static int  adc_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void adc_reset(FAR struct adc_dev_s *dev);
static int  adc_setup(FAR struct adc_dev_s *dev);
static void adc_shutdown(FAR struct adc_dev_s *dev);
static void adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);
static void adc_enable(FAR struct stm32_dev_s *priv, bool enable);

static uint32_t adc_sqrbits(FAR struct stm32_dev_s *priv, int first, int last,
                            int offset);
static int      adc_set_ch(FAR struct adc_dev_s *dev, uint8_t ch);

#ifdef CONFIG_STM32_STM32L15XX
static void adc_power_down_idle(FAR struct stm32_dev_s *priv,
                                bool pdi_high);
static void adc_power_down_delay(FAR struct stm32_dev_s *priv,
                                 bool pdd_high);
static void adc_dels_after_conversion(FAR struct stm32_dev_s *priv,
                                      uint32_t delay);
static void adc_select_ch_bank(FAR struct stm32_dev_s *priv,
                               bool chb_selected);
static int  adc_ioc_change_ints(FAR struct adc_dev_s *dev, int cmd,
                                bool arg);
#if STM32_CFGR_PLLSRC != 0 || STM32_SYSCLK_SW != RCC_CFGR_SW_HSI
static void adc_enable_hsi(bool enable);
static void adc_reset_hsi_disable(FAR struct adc_dev_s *dev);
#endif
#endif

#ifdef ADC_HAVE_TIMER
static void adc_timstart(FAR struct stm32_dev_s *priv, bool enable);
static int  adc_timinit(FAR struct stm32_dev_s *priv);
#endif

#ifdef ADC_HAVE_DMA
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                FAR void *arg);
#endif

static void adc_startconv(FAR struct stm32_dev_s *priv, bool enable);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC interface operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind        = adc_bind,
#if defined(CONFIG_STM32_STM32L15XX) && \
    (STM32_CFGR_PLLSRC != 0 || STM32_SYSCLK_SW != RCC_CFGR_SW_HSI)
  .ao_reset       = adc_reset_hsi_disable,
#else
  .ao_reset       = adc_reset,
#endif
  .ao_setup       = adc_setup,
  .ao_shutdown    = adc_shutdown,
  .ao_rxint       = adc_rxint,
  .ao_ioctl       = adc_ioctl,
};

/* ADC1 state */

#ifdef CONFIG_STM32_ADC1
static struct stm32_dev_s g_adcpriv1 =
{
#if defined(STM32_IRQ_ADC1)
  .irq         = STM32_IRQ_ADC1,
  .isr         = adc1_interrupt,
#elif defined(STM32_IRQ_ADC12)
  .irq         = STM32_IRQ_ADC12,
  .isr         = adc12_interrupt,
#elif defined(STM32_IRQ_ADC)
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
#else
#  error "No STM32_IRQ_ADC1 STM32_IRQ_ADC12 or STM32_IRQ_ADC  defined for CONFIG_STM32_ADC1"
#endif
  .intf        = 1,
  .base        = STM32_ADC1_BASE,
#ifdef ADC1_HAVE_TIMER
  .trigger     = CONFIG_STM32_ADC1_TIMTRIG,
  .tbase       = ADC1_TIMER_BASE,
  .extsel      = ADC1_EXTSEL_VALUE,
  .pclck       = ADC1_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_ADC1_SAMPLE_FREQUENCY,
#endif
#ifdef ADC1_HAVE_DMA
  .dmachan     = ADC1_DMA_CHAN,
  .hasdma      = true,
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
#if defined(STM32_IRQ_ADC12)
  .irq         = STM32_IRQ_ADC12,
  .isr         = adc12_interrupt,
#elif defined(STM32_IRQ_ADC)
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
#else
#  error "No STM32_IRQ_ADC12 or STM32_IRQ_ADC  defined for CONFIG_STM32_ADC2"
#endif
  .intf        = 2,
  .base        = STM32_ADC2_BASE,
#ifdef ADC2_HAVE_TIMER
  .trigger     = CONFIG_STM32_ADC2_TIMTRIG,
  .tbase       = ADC2_TIMER_BASE,
  .extsel      = ADC2_EXTSEL_VALUE,
  .pclck       = ADC2_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_ADC2_SAMPLE_FREQUENCY,
#endif
#ifdef ADC2_HAVE_DMA
  .dmachan     = ADC2_DMA_CHAN,
  .hasdma      = true,
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
#if defined(STM32_IRQ_ADC3)
  .irq         = STM32_IRQ_ADC3,
  .isr         = adc3_interrupt,
#elif defined(STM32_IRQ_ADC)
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
#else
#  error "No STM32_IRQ_ADC3 or STM32_IRQ_ADC  defined for CONFIG_STM32_ADC3"
#endif
  .intf        = 3,
  .base        = STM32_ADC3_BASE,
#ifdef ADC3_HAVE_TIMER
  .trigger     = CONFIG_STM32_ADC3_TIMTRIG,
  .tbase       = ADC3_TIMER_BASE,
  .extsel      = ADC3_EXTSEL_VALUE,
  .pclck       = ADC3_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_ADC3_SAMPLE_FREQUENCY,
#endif
#ifdef ADC3_HAVE_DMA
  .dmachan     = ADC3_DMA_CHAN,
  .hasdma      = true,
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
  .irq         = STM32_IRQ_ADC4,
  .isr         = adc4_interrupt,
  .intf        = 4,
  .base        = STM32_ADC4_BASE,
#ifdef ADC4_HAVE_TIMER
  .trigger     = CONFIG_STM32_ADC4_TIMTRIG,
  .tbase       = ADC4_TIMER_BASE,
  .extsel      = ADC4_EXTSEL_VALUE,
  .pclck       = ADC4_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_ADC4_SAMPLE_FREQUENCY,
#endif
#ifdef ADC4_HAVE_DMA
  .dmachan     = ADC4_DMA_CHAN,
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

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F30XX) || \
    defined(CONFIG_STM32_STM32F37XX) || defined(CONFIG_STM32_STM32F4XXX) || \
    defined(CONFIG_STM32_STM32L15XX)
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
#ifndef CONFIG_STM32_STM32L15XX
  if (priv->tbase == STM32_TIM1_BASE || priv->tbase == STM32_TIM8_BASE)
    {
      ainfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
            tim_getreg(priv, STM32_ATIM_RCR_OFFSET),
            tim_getreg(priv, STM32_ATIM_BDTR_OFFSET),
            tim_getreg(priv, STM32_ATIM_DCR_OFFSET),
            tim_getreg(priv, STM32_ATIM_DMAR_OFFSET));
    }
  else
    {
      ainfo("  DCR: %04x DMAR: %04x\n",
            tim_getreg(priv, STM32_GTIM_DCR_OFFSET),
            tim_getreg(priv, STM32_GTIM_DMAR_OFFSET));
    }
#endif
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

  /* EXTSEL selection: These bits select the external event used to trigger
   * the start of conversion of a regular group.  NOTE:
   *
   * - The position with of the EXTSEL field varies from one STM32 MCU
   *   to another.
   * - The width of the EXTSEL field varies from one STM32 MCU to another.
   * - The value in priv->extsel is already shifted into the correct bit
   *   position.
   */

  ainfo("Initializing timers extsel = 0x%08x\n", priv->extsel);

  adc_modifyreg(priv, STM32_ADC_EXTREG_OFFSET,
                ADC_EXTREG_EXTEN_MASK | ADC_EXTREG_EXTSEL_MASK,
                ADC_EXTREG_EXTEN_DEFAULT | priv->extsel);

  /* Configure the timer channel to drive the ADC */

  /* Caculate optimal values for the timer prescaler and for the timer
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

  tim_putreg(priv, STM32_GTIM_PSC_OFFSET, prescaler-1);
  tim_putreg(priv, STM32_GTIM_ARR_OFFSET, reload);

  /* Clear the advanced timers repetition counter in TIM1 */

#ifndef CONFIG_STM32_STM32L15XX
  if (priv->tbase == STM32_TIM1_BASE || priv->tbase == STM32_TIM8_BASE)
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

#ifndef CONFIG_STM32_STM32L15XX

  if (priv->tbase == STM32_TIM1_BASE || priv->tbase == STM32_TIM8_BASE)
    {
      /* Reset output N polarity level, output N state, output compare state,
       * output compare N idle state.
       */

#  if defined(CONFIG_STM32_STM32F20XX) || \
      defined(CONFIG_STM32_STM32F30XX) || \
      defined(CONFIG_STM32_STM32F4XXX)
      ccer &= ~(ATIM_CCER_CC1NE | ATIM_CCER_CC1NP |
                ATIM_CCER_CC2NE | ATIM_CCER_CC2NP |
                ATIM_CCER_CC3NE | ATIM_CCER_CC3NP |
                ATIM_CCER_CC4NP);
#  else
      ccer &= ~(ATIM_CCER_CC1NE | ATIM_CCER_CC1NP |
                ATIM_CCER_CC2NE | ATIM_CCER_CC2NP |
                ATIM_CCER_CC3NE | ATIM_CCER_CC3NP);
#  endif

      /* Reset the output compare and output compare N IDLE State */

      cr2 &= ~(ATIM_CR2_OIS1 | ATIM_CR2_OIS1N |
               ATIM_CR2_OIS2 | ATIM_CR2_OIS2N |
               ATIM_CR2_OIS3 | ATIM_CR2_OIS3N |
               ATIM_CR2_OIS4);
    }
#  if defined(CONFIG_STM32_STM32F20XX) || \
      defined(CONFIG_STM32_STM32F30XX) || \
      defined(CONFIG_STM32_STM32F4XXX)
  else
    {
      ccer &= ~(GTIM_CCER_CC1NP | GTIM_CCER_CC2NP | GTIM_CCER_CC3NP);
    }
#  endif

#else

  /* For the STM32L15XX family only these timers can be used: 2-4, 6, 7, 9, 10
   * Reset the output compare and output compare N IDLE State
   */

  if (priv->tbase >= STM32_TIM2_BASE && priv->tbase <= STM32_TIM4_BASE)
    {
      /* Reset output N polarity level, output N state, output compare state,
       * output compare N idle state.
       */

      ccer &= ~(ATIM_CCER_CC1NE | ATIM_CCER_CC1NP |
                ATIM_CCER_CC2NE | ATIM_CCER_CC2NP |
                ATIM_CCER_CC3NE | ATIM_CCER_CC3NP |
                ATIM_CCER_CC4NP);
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
 * Name: adc_startconv
 *
 * Description:
 *   Start (or stop) the ADC conversion process
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *   enable - True: Start conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

#if defined(CONFIG_STM32_STM32F10XX)
static void adc_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  ainfo("enable: %d\n", enable ? 1 : 0);

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
#elif defined(CONFIG_STM32_STM32F30XX)
static void adc_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  uint32_t regval;

  ainfo("enable: %d\n", enable ? 1 : 0);

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

          while ((adc_getreg(priv, STM32_ADC_CR_OFFSET) & ADC_CR_ADSTP) != 0);
        }
    }
}
#else
static void adc_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  ainfo("enable: %d\n", enable ? 1 : 0);

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

static void adc_rccreset(FAR struct stm32_dev_s *priv, bool reset)
{
  uint32_t adcbit;

  /* Pick the appropriate bit in the APB2 reset register.
   * For the STM32 F1, there is an individual bit to reset each ADC,
   * but for the STM32 F2/F4, there is one common reset for all ADCs.
   * THIS will probably cause some problems!
   */

  switch (priv->intf)
    {
#ifdef CONFIG_STM32_ADC1
      case 1:
        adcbit = RCC_RSTR_ADC1RST;
        break;
#endif
#ifdef CONFIG_STM32_ADC2
      case 2:
        adcbit = RCC_RSTR_ADC2RST;
        break;
#endif
#ifdef CONFIG_STM32_ADC3
      case 3:
        adcbit = RCC_RSTR_ADC3RST;
        break;
#endif
#ifdef CONFIG_STM32_ADC4
      case 4:
        adcbit = RCC_RSTR_ADC4RST;
        break;
#endif
      default:
        return;
    }

  /* Set or clear the selected bit in the APB2 reset register.
   * modifyreg32() disables interrupts.  Disabling interrupts is necessary
   * because the APB2RTSR register is used by several different drivers.
   */

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
 *                  (! Must be modified only when no conversion is on going !)
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

#if defined(CONFIG_STM32_STM32F30XX)
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
      /* Stop ongoing conversions */

      adc_startconv(priv, false);

      /* Disable the ADC */

      adc_putreg(priv, STM32_ADC_CR_OFFSET, regval | ADC_CR_ADDIS);

      /* Wait for the ADC to be disabled */

      while ((adc_getreg(priv, STM32_ADC_CR_OFFSET) & ADC_CR_ADEN) != 0);
    }
}
#else
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
 * Name: adc_write_sample_time_registers
 *
 * Description:
 *   Writes previously defined values into ADC_SMPR0, ADC_SMPR1, ADC_SMPR2
 *   and ADC_SMPR3 registers
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_STM32L15XX
static void adc_write_sample_time_registers(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  uint32_t value = 0;
  uint8_t i;
  uint8_t shift;

  /* Sampling time individually for each channel
   * 000: 4 cycles
   * 001: 9 cycles
   * 010: 16 cycles
   * 011: 24 cycles
   * 100: 48 cycles
   * 101: 96 cycles
   * 110: 192 cycles
   * 111: 384 cycles    - selected for all channels
   */

  for (i = 0, shift = 0; i < 32; i++)
    {
      value |= priv->sample_rate[i] << (shift * 3);
      switch (i)
        {
          case 9:
            adc_putreg(priv, STM32_ADC_SMPR3_OFFSET, value);
            shift = 0;
            value = 0;
            break;

          case 19:
            adc_putreg(priv, STM32_ADC_SMPR2_OFFSET, value);
            shift = 0;
            value = 0;
            break;

          case 29:
            adc_putreg(priv, STM32_ADC_SMPR1_OFFSET, value);
            shift = 0;
            value = 0;
            break;

          case 31:
            adc_putreg(priv, STM32_ADC_SMPR0_OFFSET, value);
            shift = 0;
            value = 0;
            break;

          default:
            shift++;
            break;
        }
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

#ifdef ADC_HAVE_DMA
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr, FAR void *arg)
{
  FAR struct adc_dev_s   *dev  = (FAR struct adc_dev_s *)arg;
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int i;

  /* Verify that the upper-half driver has bound its callback functions */

  if (priv->cb != NULL)
    {
      DEBUGASSERT(priv->cb->au_receive != NULL);

      for (i = 0; i < priv->nchannels; i++)
        {
          priv->cb->au_receive(dev, priv->chanlist[priv->current], priv->dmabuffer[priv->current]);
          priv->current++;
          if (priv->current >= priv->nchannels)
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
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
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
  uint32_t clrbits;
  uint32_t setbits;
#ifdef ADC_HAVE_TIMER
  int ret;
#endif

  ainfo("intf: %d\n", priv->intf);
  flags = enter_critical_section();

#if defined(CONFIG_STM32_STM32L15XX) && \
    (STM32_CFGR_PLLSRC != 0 || STM32_SYSCLK_SW != RCC_CFGR_SW_HSI)

  /* The STM32L15XX family uses HSI as an independent clock-source
   * for the ADC
   */

  adc_enable_hsi(true);

#endif

#if defined(CONFIG_STM32_STM32F30XX)

  /* Turn off the ADC so we can write the RCC bits */

  adc_enable(priv, false);

#endif

  /* Enable ADC reset state */

  adc_rccreset(priv, true);

  /* Release ADC from reset state */

  adc_rccreset(priv, false);

#if defined(CONFIG_STM32_STM32F30XX)

  /* Set voltage regular enable to intermediate state */

  adc_modifyreg(priv, STM32_ADC_CR_OFFSET, ADC_CR_ADVREGEN_MASK,
                ADC_CR_ADVREGEN_INTER);

  /* Enable the ADC voltage regulator */

  adc_modifyreg(priv, STM32_ADC_CR_OFFSET, ADC_CR_ADVREGEN_MASK,
                ADC_CR_ADVREGEN_ENABLED);

  /* Wait for the ADC voltage regulator to startup */

  up_udelay(10);

#if 0 /* Doesn't work */

  /* Calibrate the ADC */

  adc_modifyreg(priv, STM32_ADC_CR_OFFSET, ADC_CR_ADCALDIF, AD_CR_ADCAL);

  /* Wait for the calibration to complete */

  while ((adc_getreg(priv, STM32_ADC_CR_OFFSET) & ADC_CR_ADCAL) != 0);

#endif

  /* Initialize the watchdog 1 threshold register */

  adc_putreg(priv, STM32_ADC_TR1_OFFSET, 0x0fff0000);

#else

  /* Initialize the watchdog high threshold register */

  adc_putreg(priv, STM32_ADC_HTR_OFFSET, 0x00000fff);

  /* Initialize the watchdog low threshold register */

  adc_putreg(priv, STM32_ADC_LTR_OFFSET, 0x00000000);

#endif

  /* Initialize the same sample time for each ADC.
   * During sample cycles channel selection bits must remain unchanged.
   */

#ifdef CONFIG_STM32_STM32L15XX
  adc_write_sample_time_registers(dev);
#else
  adc_putreg(priv, STM32_ADC_SMPR1_OFFSET, ADC_SMPR1_DEFAULT);
  adc_putreg(priv, STM32_ADC_SMPR2_OFFSET, ADC_SMPR2_DEFAULT);
#endif

#if defined(CONFIG_STM32_STM32F30XX)

  /* Enable the analog watchdog */

  clrbits = ADC_CFGR_AWD1CH_MASK;
  setbits = ADC_CFGR_AWD1EN | ADC_CFGR_AWD1SGL |
            (priv->chanlist[0] << ADC_CFGR_AWD1CH_SHIFT);

  /* Set the resolution of the conversion */

  clrbits |= ADC_CFGR_RES_MASK;
  setbits |= ADC_CFGR_RES_12BIT;

#ifdef ADC_HAVE_DMA
  if (priv->hasdma)
    {
      /* Set DMA one shot mode */

      clrbits |= ADC_CFGR_DMACFG;

      /* Enable DMA */

      setbits |= ADC_CFGR_DMAEN;
    }
#endif

  /* Disable continuous mode and set align to right */

  clrbits |= ADC_CFGR_CONT | ADC_CFGR_ALIGN;

  /* Disable external trigger for regular channels */

  clrbits |= ADC_CFGR_EXTEN_MASK;
  setbits |= ADC_CFGR_EXTEN_NONE;

  /* Set CFGR configuration */

  adc_modifyreg(priv, STM32_ADC_CFGR_OFFSET, clrbits, setbits);

  /* Enable interrupt flags, but disable overrun interrupt */

  clrbits = ADC_IER_OVR;
  setbits = ADC_IER_ALLINTS & ~ADC_IER_OVR;

  /* Set IER configuration */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, clrbits, setbits);

#else /* CONFIG_STM32_STM32F30XX */

  /* Enable the analog watchdog */

  clrbits = ADC_CR1_AWDCH_MASK;
  setbits = ADC_CR1_AWDEN | (priv->chanlist[0] << ADC_CR1_AWDCH_SHIFT);

#ifdef CONFIG_STM32_STM32F10XX

  /* Set independent mode */

  clrbits |= ADC_CR1_DUALMOD_MASK;
  setbits |= ADC_CR1_IND;

#elif (! defined(CONFIG_STM32_STM32F37XX))

  /* Set the resolution of the conversion */

  clrbits |= ADC_CR1_RES_MASK;
  setbits |= ADC_CR1_RES_12BIT;

#endif

#ifdef ADC_HAVE_DMA
  if (priv->hasdma)
    {
      setbits |= ADC_CR1_SCAN;
    }
#endif

  /* Enable interrupt flags, but disable overrun interrupt: TODO this is
   * done later by upper half when opening device by adc_rxint().
   */

#ifndef CONFIG_STM32_STM32F37XX
  clrbits |= ADC_IER_OVR;
  setbits |= ADC_IER_ALLINTS & ~ADC_IER_OVR;
#else
  /* TODO NON DMA mode */
#endif

  /* Set CR1 configuration */

  adc_modifyreg(priv, STM32_ADC_CR1_OFFSET, clrbits, setbits);

#ifdef CONFIG_STM32_STM32L15XX

  /* Disables power down during the delay phase */

  adc_power_down_idle(priv, false);
  adc_power_down_delay(priv, false);

  /* Select the bank of channels A */

  adc_select_ch_bank(priv, false);

  /* Delay until the converted data has been read */

  adc_dels_after_conversion(priv, ADC_CR2_DELS_TILLRD);

#endif

  /* Disable continuous mode and set align to right */

  clrbits = ADC_CR2_CONT | ADC_CR2_ALIGN;
  setbits = 0;

  /* Disable external trigger for regular channels */

  clrbits |= ADC_EXTREG_EXTEN_MASK;
  setbits |= ADC_EXTREG_EXTEN_NONE;

  /* Enable software trigger for regular channels */

#ifdef CONFIG_STM32_STM32F37XX
  clrbits |= ADC_CR2_EXTSEL_MASK;
  setbits |= ADC_CR2_EXTSEL_SWSTART | ADC_CR2_EXTTRIG; /* SW is considered as external trigger */
#endif

#ifdef ADC_HAVE_DMA
  if (priv->hasdma)
    {
      setbits |= ADC_CR2_DMA;
    }
#endif

  /* Set CR2 configuration */

  adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, clrbits, setbits);

#endif

  /* Configuration of the channel conversions */

  adc_set_ch(dev, 0);

  /* ADC CCR configuration */

#if defined(CONFIG_STM32_STM32F30XX)
  clrbits = ADC_CCR_DUAL_MASK | ADC_CCR_DELAY_MASK | ADC_CCR_DMACFG |
            ADC_CCR_MDMA_MASK | ADC_CCR_CKMODE_MASK | ADC_CCR_VREFEN |
            ADC_CCR_TSEN | ADC_CCR_VBATEN;
  setbits = ADC_CCR_DUAL_IND | ADC_CCR_DELAY(0) | ADC_CCR_MDMA_DISABLED |
            ADC_CCR_CKMODE_ASYNCH;

  if (priv->base == STM32_ADC1_BASE || priv->base == STM32_ADC2_BASE)
    {
      stm32_modifyreg32(STM32_ADC12_CCR, clrbits, setbits);
    }
  else
    {
      stm32_modifyreg32(STM32_ADC34_CCR, clrbits, setbits);
    }
#elif defined(CONFIG_STM32_STM32F20XX) || \
      defined(CONFIG_STM32_STM32F4XXX) || \
      defined(CONFIG_STM32_STM32L15XX)
  clrbits  = ADC_CCR_ADCPRE_MASK | ADC_CCR_TSVREFE;
  setbits  = ADC_CCR_ADCPRE_DIV2;

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
  clrbits |= ADC_CCR_MULTI_MASK | ADC_CCR_DELAY_MASK | ADC_CCR_DDS |
             ADC_CCR_DMA_MASK | ADC_CCR_VBATE;
  setbits |= ADC_CCR_MULTI_NONE | ADC_CCR_DMA_DISABLED;
#endif

  stm32_modifyreg32(STM32_ADC_CCR, clrbits, setbits);
#endif

#ifdef ADC_HAVE_DMA

  /* Enable DMA */

  if (priv->hasdma)
    {
      /* Stop and free DMA if it was started before */

      if (priv->dma != NULL)
        {
          stm32_dmastop(priv->dma);
          stm32_dmafree(priv->dma);
        }

      priv->dma = stm32_dmachannel(priv->dmachan);

      stm32_dmasetup(priv->dma,
                     priv->base + STM32_ADC_DR_OFFSET,
                     (uint32_t)priv->dmabuffer,
                     priv->nchannels,
                     ADC_DMA_CONTROL_WORD);

      stm32_dmastart(priv->dma, adc_dmaconvcallback, dev, false);
    }

#endif

  /* Set ADON to wake up the ADC from the power down state */

  adc_enable(priv, true);

#ifdef ADC_HAVE_TIMER
  if (priv->tbase != 0)
    {
      ret = adc_timinit(priv);
      if (ret < 0)
        {
          aerr("ERROR: adc_timinit failed: %d\n", ret);
        }
    }
#ifndef CONFIG_STM32_ADC_NO_STARTUP_CONV
  else
#endif
#endif
#ifndef CONFIG_STM32_ADC_NO_STARTUP_CONV
    {
      adc_startconv(priv, true);
    }
#endif

  leave_critical_section(flags);

#if defined(CONFIG_STM32_STM32F30XX)
  ainfo("ISR:  0x%08x CR:   0x%08x CFGR: 0x%08x\n",
        adc_getreg(priv, STM32_ADC_ISR_OFFSET),
        adc_getreg(priv, STM32_ADC_CR_OFFSET),
        adc_getreg(priv, STM32_ADC_CFGR_OFFSET));
#else
  ainfo("SR:   0x%08x CR1:  0x%08x CR2:  0x%08x\n",
        adc_getreg(priv, STM32_ADC_SR_OFFSET),
        adc_getreg(priv, STM32_ADC_CR1_OFFSET),
        adc_getreg(priv, STM32_ADC_CR2_OFFSET));
#endif

  ainfo("SQR1: 0x%08x SQR2: 0x%08x SQR3: 0x%08x\n",
        adc_getreg(priv, STM32_ADC_SQR1_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR2_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR3_OFFSET));

#if defined(CONFIG_STM32_STM32F30XX)
  ainfo("SQR4: 0x%08x\n", adc_getreg(priv, STM32_ADC_SQR4_OFFSET));
#elif defined(CONFIG_STM32_STM32L15XX)
  ainfo("SQR4: 0x%08x SQR5: 0x%08x\n",
        adc_getreg(priv, STM32_ADC_SQR4_OFFSET)
        adc_getreg(priv, STM32_ADC_SQR5_OFFSET));
#endif

#if defined(CONFIG_STM32_STM32F30XX)
  if (priv->base == STM32_ADC1_BASE || priv->base == STM32_ADC2_BASE)
    {
      ainfo("CCR:  0x%08x\n", getreg32(STM32_ADC12_CCR));
    }
  else
    {
      ainfo("CCR:  0x%08x\n", getreg32(STM32_ADC34_CCR));
    }
#elif defined(CONFIG_STM32_STM32F20XX) || \
      defined(CONFIG_STM32_STM32F4XXX) || \
      defined(CONFIG_STM32_STM32L15XX)
  ainfo("CCR:  0x%08x\n", getreg32(STM32_ADC_CCR));
#endif
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

#if defined(CONFIG_STM32_STM32L15XX) && \
    (STM32_CFGR_PLLSRC != 0 || STM32_SYSCLK_SW != RCC_CFGR_SW_HSI)
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
  int ret;

  /* Attach the ADC interrupt */

  ret = irq_attach(priv->irq, priv->isr, NULL);
  if (ret < 0)
    {
      ainfo("irq_attach failed: %d\n", ret);
      return ret;
    }

  /* Make sure that the ADC device is in the powered up, reset state */

  adc_reset(dev);

  /* Enable the ADC interrupt */

  ainfo("Enable the ADC interrupt: irq=%d\n", priv->irq);
  up_enable_irq(priv->irq);

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

  adc_enable(priv, false);
#if defined(CONFIG_STM32_STM32L15XX) && \
    (STM32_CFGR_PLLSRC != 0 || STM32_SYSCLK_SW != RCC_CFGR_SW_HSI)
  adc_enable_hsi(false);
#endif

  /* Disable ADC interrupts and detach the ADC interrupt handler */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  /* Disable and reset the ADC module */

  adc_rccreset(priv, true);
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

#if defined(CONFIG_STM32_STM32L15XX) || defined(CONFIG_STM32_STM32F20XX) || \
    defined(CONFIG_STM32_STM32F4XXX)
static void adc_ioc_enable_tvref_register(FAR struct adc_dev_s *dev,
                                          bool enable)
{
  if (enable)
    {
      stm32_modifyreg32(STM32_ADC_CCR, 0, ADC_CCR_TSVREFE);
    }
  else
    {
      stm32_modifyreg32(STM32_ADC_CCR, ADC_CCR_TSVREFE, 0);
    }

  ainfo("STM32_ADC_CCR value: 0x%08x\n", getreg32(STM32_ADC_CCR));
}

#elif defined(CONFIG_STM32_STM32F10XX)
static void adc_ioc_enable_tvref_register(FAR struct adc_dev_s *dev,
                                          bool enable)
{
#if defined(CONFIG_STM32_ADC1)
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

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

  ainfo("STM32_ADC_CR2 value: 0x%08x\n",
        adc_getreg(priv, STM32_ADC_CR2_OFFSET));
#endif /* CONFIG_STM32_ADC1 */
}
#endif

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

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
static void adc_enable_vbat_channel(FAR struct adc_dev_s *dev, bool enable)
{
  if (enable)
    {
      stm32_modifyreg32(STM32_ADC_CCR, 0, ADC_CCR_VBATE);
    }
  else
    {
      stm32_modifyreg32(STM32_ADC_CCR, ADC_CCR_VBATE, 0);
    }

  ainfo("STM32_ADC_CCR value: 0x%08x\n", getreg32(STM32_ADC_CCR));
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

#ifdef CONFIG_STM32_STM32L15XX
static int adc_ioc_change_sleep_between_opers(FAR struct adc_dev_s *dev,
                                              int cmd, bool arg)
{
  int ret = OK;
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

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

#ifdef CONFIG_STM32_STM32L15XX
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
#endif

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

#ifdef CONFIG_STM32_STM32L15XX
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
#endif

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

#ifdef CONFIG_STM32_STM32L15XX
static void adc_ioc_enable_jeoc_int(FAR struct stm32_dev_s *priv, bool enable)
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
#endif

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

#ifdef CONFIG_STM32_STM32L15XX
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
#endif

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

#ifdef CONFIG_STM32_STM32L15XX
static int adc_ioc_change_ints(FAR struct adc_dev_s *dev, int cmd, bool arg)
{
  int ret = OK;
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

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
#endif

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

#if defined(CONFIG_STM32_STM32L15XX) && \
    (STM32_CFGR_PLLSRC != 0 || STM32_SYSCLK_SW != RCC_CFGR_SW_HSI)
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

static uint32_t adc_sqrbits(FAR struct stm32_dev_s *priv, int first, int last,
                            int offset)
{
  uint32_t bits = 0;
  int i;

  for (i = first - 1;
       i < priv->nchannels && i < last;
       i++, offset += ADC_SQ_OFFSET)
    {
      bits |= (uint32_t)priv->chanlist[i] << offset;
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
      priv->nchannels = priv->cchannels;
    }
  else
    {
      for (i = 0; i < priv->cchannels && priv->chanlist[i] != ch - 1; i++);

      if (i >= priv->cchannels)
        {
          return -ENODEV;
        }

      priv->current   = i;
      priv->nchannels = 1;
    }

#ifdef STM32_ADC_SQR5_OFFSET
  bits = adc_sqrbits(priv, ADC_SQR5_FIRST, ADC_SQR5_LAST, ADC_SQR5_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR5_OFFSET, ~ADC_SQR5_RESERVED, bits);
#endif

#ifdef STM32_ADC_SQR4_OFFSET
  bits = adc_sqrbits(priv, ADC_SQR4_FIRST, ADC_SQR4_LAST, ADC_SQR4_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR4_OFFSET, ~ADC_SQR4_RESERVED, bits);
#endif

  bits = adc_sqrbits(priv, ADC_SQR3_FIRST, ADC_SQR3_LAST, ADC_SQR3_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR3_OFFSET, ~ADC_SQR3_RESERVED, bits);

  bits = adc_sqrbits(priv, ADC_SQR2_FIRST, ADC_SQR2_LAST, ADC_SQR2_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR2_OFFSET, ~ADC_SQR2_RESERVED, bits);

  bits = ((uint32_t)priv->nchannels - 1) << ADC_SQR1_L_SHIFT |
         adc_sqrbits(priv, ADC_SQR1_FIRST, ADC_SQR1_LAST, ADC_SQR1_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR1_OFFSET, ~ADC_SQR1_RESERVED, bits);

  return OK;
}

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
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        adc_startconv(priv, true);
        break;

#if defined(CONFIG_STM32_STM32F10XX)
      case IO_ENABLE_TEMPER_VOLT_CH:
        adc_ioc_enable_tvref_register(dev, *(bool *)arg);
        break;

#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F4XXX)
      case IO_ENABLE_TEMPER_VOLT_CH:
        adc_ioc_enable_tvref_register(dev, *(bool *)arg);
        break;

      case IO_ENABLE_DISABLE_VBAT_CH:
        adc_enable_vbat_channel(dev, *(bool *)arg);
        break;

#elif defined(CONFIG_STM32_STM32L15XX)
      case IO_ENABLE_TEMPER_VOLT_CH:
        adc_ioc_enable_tvref_register(dev, *(bool *)arg);
        break;

      case IO_ENABLE_DISABLE_PDI:
      case IO_ENABLE_DISABLE_PDD:
      case IO_ENABLE_DISABLE_PDD_PDI:
        adc_ioc_change_sleep_between_opers(dev, cmd, *(bool *)arg);
        break;

      case IO_ENABLE_DISABLE_AWDIE:
      case IO_ENABLE_DISABLE_EOCIE:
      case IO_ENABLE_DISABLE_JEOCIE:
      case IO_ENABLE_DISABLE_OVRIE:
      case IO_ENABLE_DISABLE_ALL_INTS:
        adc_ioc_change_ints(dev, cmd, *(bool *)arg);
        break;

      case IO_START_CONV:
        {
          uint8_t ch = ((uint8_t)arg);

          ret = adc_ioc_wait_rcnr_zeroed(priv);
          if (ret < 0)
            {
              return ret;
            }

          ret = adc_set_ch(dev, ch);
          if (ret < 0)
            {
              return ret;
            }

          if (ch)
            {
              /* Clear fifo */

              dev->ad_recv.af_head = 0;
              dev->ad_recv.af_tail = 0;
            }

          adc_startconv(priv, true);
        }
        break;

#if STM32_CFGR_PLLSRC != 0 || STM32_SYSCLK_SW != RCC_CFGR_SW_HSI
      case IO_STOP_ADC:
        adc_enable(priv, false);
        adc_enable_hsi(false);
        break;

      case IO_START_ADC:
        adc_enable_hsi(true);
        adc_enable(priv, true);
        break;
#endif
#endif /* CONFIG_STM32_STM32L15XX */

      default:
        aerr("ERROR: Unknown cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

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
          priv->cb->au_receive(dev, priv->chanlist[priv->current], data);
        }

      /* Set the channel number of the next channel that will complete
       * conversion.
       */

      priv->current++;

      if (priv->current >= priv->nchannels)
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_change_sample_time
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

#ifdef CONFIG_STM32_STM32L15XX
void stm32_adcchange_sample_time(FAR struct adc_dev_s *dev,
                                 FAR struct adc_sample_time_s *time_samples)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  uint8_t ch_index;
  uint8_t i;

  /* Check if user wants to assign the same value for all channels
   * or just wants to change sample time values for certain channels */

  if (time_samples->all_same)
    {
      memset(priv->sample_rate, time_samples->all_ch_sample_time,
             ADC_CHANNELS_NUMBER);
    }
  else
    {
      for (i = 0; i < time_samples->channels_nbr; i++)
        {
          ch_index = time_samples->channel->channel;
          if (ch_index >= ADC_CHANNELS_NUMBER)
            {
              break;
            }

          priv->sample_rate[ch_index] = time_samples->channel->sample_time;
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_adcinitialize
 *
 * Description:
 *   Initialize the ADC.
 *
 *   The logic is, save nchannels : # of channels (conversions) in ADC_SQR1_L
 *   Then, take the chanlist array and store it in the SQR Regs,
 *     chanlist[0] -> ADC_SQR3_SQ1
 *     chanlist[1] -> ADC_SQR3_SQ2
 *     ...
 *     chanlist[15]-> ADC_SQR1_SQ16
 *
 *   up to
 *     chanlist[nchannels]
 *
 * Input Parameters:
 *   intf      - Could be {1,2,3} for ADC1, ADC2, or ADC3
 *   chanlist  - The list of channels
 *   cchannels - Number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *stm32_adcinitialize(int intf, FAR const uint8_t *chanlist,
                                      int cchannels)
{
  FAR struct adc_dev_s   *dev;
  FAR struct stm32_dev_s *priv;

  ainfo("intf: %d cchannels: %d\n", intf, cchannels);

  switch (intf)
    {
#ifdef CONFIG_STM32_ADC1
      case 1:
        ainfo("ADC1 selected\n");
        dev = &g_adcdev1;
        break;
#endif
#ifdef CONFIG_STM32_ADC2
      case 2:
        ainfo("ADC2 selected\n");
        dev = &g_adcdev2;
        break;
#endif
#ifdef CONFIG_STM32_ADC3
      case 3:
        ainfo("ADC3 selected\n");
        dev = &g_adcdev3;
        break;
#endif
#ifdef CONFIG_STM32_ADC4
      case 4:
        ainfo("ADC4 selected\n");
        dev = &g_adcdev4;
        break;
#endif
      default:
        aerr("ERROR: No ADC interface defined\n");
        return NULL;
    }

  /* Configure the selected ADC */

  priv = (FAR struct stm32_dev_s *)dev->ad_priv;

#if defined(CONFIG_STM32_STM32L15XX)

  /* Assign default values for the sample time table */

  memset(priv->sample_rate, ADC_SMPR_DEFAULT, ADC_CHANNELS_NUMBER);

#endif

  DEBUGASSERT(cchannels <= ADC_MAX_SAMPLES);

  priv->cb        = NULL;
  priv->cchannels = cchannels;

  memcpy(priv->chanlist, chanlist, cchannels);

  return dev;
}

#endif /* CONFIG_STM32_STM32F10XX || CONFIG_STM32_STM32F20XX ||
        * CONFIG_STM32_STM32F30XX || CONFIG_STM32_STM32F47XX ||
        * CONFIG_STM32_STM32F4XXX || CONFIG_STM32_STM32L15XX
        */
#endif /* CONFIG_STM32_ADC1 || CONFIG_STM32_ADC2 ||
        * CONFIG_STM32_ADC3 || CONFIG_STM32_ADC4
        */
#endif /* CONFIG_STM32_ADC */
