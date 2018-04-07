/****************************************************************************
 * arch/arm/src/stm32/stm32f33xxx_adc.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *
 *   based on stm32_adc.c
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

#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This implementation is for the STM32F33XXX only.
 *
 * STM32F33XXX chips are intended for digital power conversion applications,
 * which casue some differences in use comapred to the standard STM32 ADC driver.
 * This include:
 *   - lower half ADC driver can be used not only with the upper-half ADC driver,
 *     but also as the lower-half logic for the power-related drivers (eg. SMPS
 *     upper driver),
 *   - ADC will be used in time-critical operations (eg. converter control loop),
 *     therfore it is necessary to support the high performecne, zero latency ADC
 *     interrupts
 *   - ADC triggering from the high resolution timer (HRTIM) and external lines
 *   - support for injected sequence
 */

#if defined(CONFIG_STM32_STM32F33XX)

#warning "ADC support for STM32F33XX under development !"

#if defined(ADC1_INJECTED_CHAN) && !defined(CONFIG_STM32_ADC1_INJECTED)
#  warning
#endif
#if defined(ADC2_INJECTED_CHAN) && !defined(CONFIG_STM32_ADC2_INJECTED)
#  warning
#endif

#if defined(CONFIG_STM32_ADC1_INJECTED) || defined(CONFIG_STM32_ADC2_INJECTED)
#  define ADC_HAVE_INJECTED 1
#endif

#ifndef CONFIG_STM32_ADC1_INJECTED
#  define ADC1_INJECTED_CHAN 0
#else
#  define ADC1_HAVE_JEXTSEL
#endif
#ifndef CONFIG_STM32_ADC2_INJECTED
#  define ADC2_INJECTED_CHAN 0
#else
#  define ADC2_HAVE_JEXTSEL
#endif

#if defined(CONFIG_STM32_ADC_NOIRQ) && defined(ADC_HAVE_DMA)
#  error "ADC DMA support requires common ADC interrupts"
#endif

/* RCC reset ****************************************************************/

#define STM32_RCC_RSTR    STM32_RCC_AHBRSTR
#define STM32_RCC_ENR     STM32_RCC_AHBENR
#define RCC_RSTR_ADC12RST RCC_AHBRSTR_ADC12RST
#define RCC_RSTR_ADC12EN  RCC_AHBENR_ADC12EN

/* ADC interrupts ***********************************************************/

#define STM32_ADC_DMAREG_OFFSET    STM32_ADC_CFGR_OFFSET
#define ADC_DMAREG_DMA             ADC_CFGR_DMAEN
#define STM32_ADC_EXTREG_OFFSET    STM32_ADC_CFGR_OFFSET
#define ADC_EXTREG_EXTSEL_MASK     ADC_CFGR_EXTSEL_MASK
#define ADC_EXTREG_EXTEN_MASK      ADC_CFGR_EXTEN_MASK
#define ADC_EXTREG_EXTEN_DEFAULT   ADC_CFGR_EXTEN_RISING
#define STM32_ADC_JEXTREG_OFFSET   STM32_ADC_JSQR_OFFSET
#define ADC_JEXTREG_JEXTSEL_MASK   ADC_JSQR_JEXTSEL_MASK
#define ADC_JEXTREG_JEXTEN_MASK    ADC_JSQR_JEXTEN_MASK
#define ADC_JEXTREG_JEXTEN_DEFAULT ADC_JSQR_JEXTEN_RISING

#define ADC_ISR_EOC                ADC_INT_EOC
#define ADC_IER_EOC                ADC_INT_EOC
#define ADC_ISR_AWD                ADC_INT_AWD1
#define ADC_IER_AWD                ADC_INT_AWD1
#define ADC_ISR_JEOC               ADC_INT_JEOC
#define ADC_IER_JEOC               ADC_INT_JEOC
#define ADC_ISR_JEOS               ADC_INT_JEOS
#define ADC_IER_JEOS               ADC_INT_JEOS
#define ADC_ISR_OVR                ADC_INT_OVR
#define ADC_IER_OVR                ADC_INT_OVR

#define ADC_ISR_ALLINTS (ADC_ISR_EOC | ADC_ISR_AWD | ADC_ISR_JEOC | \
                         ADC_ISR_JEOS | ADC_ISR_OVR)
#define ADC_IER_ALLINTS (ADC_IER_EOC | ADC_IER_AWD | ADC_IER_JEOC | \
                         ADC_IER_JEOS| ADC_IER_OVR)

/* ADC Channels/DMA ********************************************************/
/* The maximum number of channels that can be sampled.  If DMA support is
 * not enabled, then only a single channel can be sampled.  Otherwise,
 * data overruns would occur.
 */

#define ADC_MAX_CHANNELS_DMA   16
#define ADC_MAX_CHANNELS_NOIRQ 16
#define ADC_MAX_CHANNELS_NODMA 1

#if defined(ADC_HAVE_DMA)
#  define ADC_REG_MAX_SAMPLES ADC_MAX_CHANNELS_DMA
#elif defined(CONFIG_STM32_ADC_NOIRQ)
#  define ADC_REG_MAX_SAMPLES ADC_MAX_CHANNELS_NOIRQ
#else
#  define ADC_REG_MAX_SAMPLES ADC_MAX_CHANNELS_NODMA
#endif

#define ADC_INJ_MAX_SAMPLES   4

#define ADC_DMA_CONTROL_WORD (DMA_CCR_MSIZE_16BITS | \
                              DMA_CCR_PSIZE_16BITS | \
                              DMA_CCR_MINC |         \
                              DMA_CCR_CIRC)

#if defined(ADC_HAVE_DMA) || (ADC_REG_MAX_SAMPLES == 1)
#  define ADC_SMPR_DEFAULT    ADC_SMPR_61p5
#else /* Slow down sampling frequency */
#  define ADC_SMPR_DEFAULT    ADC_SMPR_601p5
#endif

#ifndef ADC1_SMP1
#  define ADC1_SMP1 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP2
#  define ADC1_SMP2 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP3
#  define ADC1_SMP3 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP4
#  define ADC1_SMP4 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP5
#  define ADC1_SMP5 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP6
#  define ADC1_SMP6 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP7
#  define ADC1_SMP7 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP8
#  define ADC1_SMP8 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP9
#  define ADC1_SMP9 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP10
#  define ADC1_SMP10 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP11
#  define ADC1_SMP11 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP12
#  define ADC1_SMP12 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP13
#  define ADC1_SMP13 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP14
#  define ADC1_SMP14 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP15
#  define ADC1_SMP15 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP16
#  define ADC1_SMP16 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP17
#  define ADC1_SMP17 ADC_SMPR_DEFAULT
#endif
#ifndef ADC1_SMP18
#  define ADC1_SMP18 ADC_SMPR_DEFAULT
#endif

#ifndef ADC2_SMP1
#  define ADC2_SMP1 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP2
#  define ADC2_SMP2 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP3
#  define ADC2_SMP3 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP4
#  define ADC2_SMP4 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP5
#  define ADC2_SMP5 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP6
#  define ADC2_SMP6 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP7
#  define ADC2_SMP7 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP8
#  define ADC2_SMP8 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP9
#  define ADC2_SMP9 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP10
#  define ADC2_SMP10 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP11
#  define ADC2_SMP11 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP12
#  define ADC2_SMP12 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP13
#  define ADC2_SMP13 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP14
#  define ADC2_SMP14 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP15
#  define ADC2_SMP15 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP16
#  define ADC2_SMP16 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP17
#  define ADC2_SMP17 ADC_SMPR_DEFAULT
#endif
#ifndef ADC2_SMP18
#  define ADC2_SMP18 ADC_SMPR_DEFAULT
#endif

#define ADC1_SMPR1   ((ADC1_SMP1 << ADC_SMPR1_SMP1_SHIFT) |  \
                      (ADC1_SMP2 << ADC_SMPR1_SMP2_SHIFT) |  \
                      (ADC1_SMP3 << ADC_SMPR1_SMP3_SHIFT) |  \
                      (ADC1_SMP4 << ADC_SMPR1_SMP4_SHIFT) |  \
                      (ADC1_SMP5 << ADC_SMPR1_SMP5_SHIFT) |  \
                      (ADC1_SMP6 << ADC_SMPR1_SMP6_SHIFT) |  \
                      (ADC1_SMP7 << ADC_SMPR1_SMP7_SHIFT) |  \
                      (ADC1_SMP8 << ADC_SMPR1_SMP8_SHIFT) |  \
                      (ADC1_SMP9 << ADC_SMPR1_SMP9_SHIFT))
#define ADC1_SMPR2   ((ADC1_SMP10 << ADC_SMPR2_SMP10_SHIFT) | \
                      (ADC1_SMP11 << ADC_SMPR2_SMP11_SHIFT) | \
                      (ADC1_SMP12 << ADC_SMPR2_SMP12_SHIFT) | \
                      (ADC1_SMP13 << ADC_SMPR2_SMP13_SHIFT) | \
                      (ADC1_SMP14 << ADC_SMPR2_SMP14_SHIFT) | \
                      (ADC1_SMP15 << ADC_SMPR2_SMP15_SHIFT) | \
                      (ADC1_SMP16 << ADC_SMPR2_SMP16_SHIFT) | \
                      (ADC1_SMP17 << ADC_SMPR2_SMP17_SHIFT) | \
                      (ADC1_SMP18 << ADC_SMPR2_SMP18_SHIFT))

#define ADC2_SMPR1   ((ADC2_SMP1 << ADC_SMPR1_SMP1_SHIFT) | \
                      (ADC2_SMP2 << ADC_SMPR1_SMP2_SHIFT) | \
                      (ADC2_SMP3 << ADC_SMPR1_SMP3_SHIFT) | \
                      (ADC2_SMP4 << ADC_SMPR1_SMP4_SHIFT) | \
                      (ADC2_SMP5 << ADC_SMPR1_SMP5_SHIFT) | \
                      (ADC2_SMP6 << ADC_SMPR1_SMP6_SHIFT) | \
                      (ADC2_SMP7 << ADC_SMPR1_SMP7_SHIFT) | \
                      (ADC2_SMP8 << ADC_SMPR1_SMP8_SHIFT) | \
                      (ADC2_SMP9 << ADC_SMPR1_SMP9_SHIFT))
#define ADC2_SMPR2   ((ADC2_SMP10 << ADC_SMPR2_SMP10_SHIFT) | \
                      (ADC2_SMP11 << ADC_SMPR2_SMP11_SHIFT) | \
                      (ADC2_SMP12 << ADC_SMPR2_SMP12_SHIFT) | \
                      (ADC2_SMP13 << ADC_SMPR2_SMP13_SHIFT) | \
                      (ADC2_SMP14 << ADC_SMPR2_SMP14_SHIFT) | \
                      (ADC2_SMP15 << ADC_SMPR2_SMP15_SHIFT) | \
                      (ADC2_SMP16 << ADC_SMPR2_SMP16_SHIFT) | \
                      (ADC2_SMP17 << ADC_SMPR2_SMP17_SHIFT) | \
                      (ADC2_SMP18 << ADC_SMPR2_SMP18_SHIFT))

/* Last bit of the extsel fields indicate if external trigger is in use */

#define HAVE_EXTSEL_MASK  (1<<31)

#if defined(ADC1_HAVE_TIMER) || defined(ADC1_HAVE_HRTIM) || defined(ADC1_HAVE_EXTI)
#  define ADC1_HAVE_EXTSEL 1
#else
#  undef ADC1_HAVE_EXTSEL
#endif

#if defined(ADC2_HAVE_TIMER) || defined(ADC2_HAVE_HRTIM) || defined(ADC2_HAVE_EXTI)
#  define ADC2_HAVE_EXTSEL 1
#else
#  undef ADC2_HAVE_EXTSEL
#endif

#if defined(ADC1_HAVE_EXTSEL) || defined(ADC2_HAVE_EXTSEL)
#  define ADC_HAVE_EXTSEL
#endif

#if defined(ADC1_HAVE_JEXTSEL) || defined(ADC2_HAVE_JEXTSEL)
#  define ADC_HAVE_JEXTSEL
#endif

/* Default ADC resolution */

#ifndef ADC1_RESOLUTION
#  define ADC1_RESOLUTION ADC_RESOLUTION_12BIT
#endif
#ifndef ADC2_RESOLUTION
#  define ADC2_RESOLUTION ADC_RESOLUTION_12BIT
#endif

/* Default ADC DMA configuration */

#ifndef ADC1_DMA_CFG
#  define ADC1_DMA_CFG ADC_CFGR_DMACFG
#endif
#ifndef ADC2_DMA_CFG
#  define ADC2_DMA_CFG ADC_CFGR_DMACFG
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Data common to all ADC instances */

struct adc_cmn_s
{
  uint8_t initialized; /* How many ADC instances are currently in use */
                       /* TODO: dual ADC mode */
};

/* This structure describes the state of one ADC block */

struct stm32_dev_s
{
#ifdef CONFIG_STM32_ADC_NOIRQ
  FAR const struct stm32_adc_ops_s *ops; /* Publicly visible portion */
#else
  FAR const struct adc_callback_s *cb;
  uint8_t irq;          /* Interrupt generated by this ADC block */
#endif
  struct adc_cmn_s *cmn; /* Common ADC data */
  uint8_t rnchannels;   /* Number of regular channels */
  uint8_t cr_channels;  /* Number of configured regular channels */
#ifdef ADC_HAVE_INJECTED
  uint8_t cj_channels;  /* Number of configured injected channels */
#endif
  uint8_t intf;         /* ADC interface number */
  uint8_t resolution;   /* ADC resolution*/
  uint8_t current;      /* Current ADC channel being converted */
#ifdef ADC_HAVE_DMA
  uint8_t dmachan;      /* DMA channel needed by this ADC */
  uint8_t dmacfg;       /* DMA channel configuration */
  bool    hasdma;       /* True: This channel supports DMA */
#endif
#ifdef ADC_HAVE_TIMER
  uint8_t trigger;      /* Timer trigger channel: 0=CC1, 1=CC2, 2=CC3,
                         * 3=CC4, 4=TRGO */
#endif
#ifndef CONFIG_STM32_ADC_NOIRQ
  xcpt_t   isr;         /* Interrupt handler for this ADC block */
#endif
  uint32_t base;        /* Base address of registers unique to this ADC
                         * block */
  uint32_t smp1;        /* Sample time part 1 */
  uint32_t smp2;        /* Sample time part 2 */
#ifdef ADC_HAVE_EXTSEL
  uint32_t extsel;      /* EXTSEL value used by this ADC block */
#endif
#ifdef ADC_HAVE_JEXTSEL
  uint32_t jextsel;     /* JEXTSEL value used by this ADC block */
#endif
#ifdef ADC_HAVE_TIMER
  uint32_t tbase;       /* Base address of timer used by this ADC block */
  uint32_t pclck;       /* The PCLK frequency that drives this timer */
  uint32_t freq;        /* The desired frequency of conversions */
#endif
#ifdef ADC_HAVE_DMA
  DMA_HANDLE dma;       /* Allocated DMA channel */

  /* DMA transfer buffer for regular channels */

  uint16_t r_dmabuffer[ADC_REG_MAX_SAMPLES];
#endif

  /* List of selected ADC regular channels to sample */

  uint8_t  r_chanlist[ADC_REG_MAX_SAMPLES];

#ifdef ADC_HAVE_INJECTED
  /* List of selected ADC injected channels to sample */

  uint8_t  j_chanlist[ADC_INJ_MAX_SAMPLES];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* ADC Register access */

static void stm32_modifyreg32(unsigned int addr, uint32_t clrbits,
                              uint32_t setbits);
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

#ifndef CONFIG_STM32_ADC_NOIRQ
static int adc_interrupt(FAR struct adc_dev_s *dev);
static int adc12_interrupt(int irq, FAR void *context, FAR void *arg);
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

#ifdef ADC_HAVE_TIMER
static void adc_timstart(FAR struct stm32_dev_s *priv, bool enable);
static int  adc_timinit(FAR struct stm32_dev_s *priv);
#endif

#if defined(ADC_HAVE_DMA) && !defined(CONFIG_STM32_ADC_NOIRQ)
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                FAR void *arg);
#endif

static void adc_startconv(FAR struct stm32_dev_s *priv, bool enable);
#ifdef ADC_HAVE_INJECTED
static void adc_inj_startconv(FAR struct stm32_dev_s *priv, bool enable);
static int adc_inj_set_ch(FAR struct adc_dev_s *dev, uint8_t ch);
#endif

#ifdef CONFIG_STM32_ADC_NOIRQ
static void adc_intack(FAR struct stm32_adc_dev_s *dev, uint32_t source);
static void adc_inten(FAR struct stm32_adc_dev_s *dev, uint32_t source);
static void adc_intdis(FAR struct stm32_adc_dev_s *dev, uint32_t source);
static uint32_t adc_intget(FAR struct stm32_adc_dev_s *dev);
static uint32_t adc_regget(FAR struct stm32_adc_dev_s *dev);
#  ifdef ADC_HAVE_DMA
static int adc_regbufregister(FAR struct stm32_adc_dev_s *dev, uint16_t *buffer, uint8_t len);
#  endif
#  ifdef ADC_HAVE_INJECTED
static uint32_t adc_injget(FAR struct stm32_adc_dev_s *dev, uint8_t chan);
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC interface operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind        = adc_bind,
  .ao_reset       = adc_reset,
  .ao_setup       = adc_setup,
  .ao_shutdown    = adc_shutdown,
  .ao_rxint       = adc_rxint,
  .ao_ioctl       = adc_ioctl,
};

/* Publicly visible ADC lower-half operations */

#ifdef CONFIG_STM32_ADC_NOIRQ
static const struct stm32_adc_ops_s g_adc_lowerops =
{
  .int_ack = adc_intack,
  .int_get = adc_intget,
  .int_en  = adc_inten,
  .int_dis = adc_intdis,
  .val_get = adc_regget,
#ifdef ADC_HAVE_DMA
  .regbuf_reg = adc_regbufregister,
#endif
#ifdef ADC_HAVE_INJECTED
  .inj_get    = adc_injget,
#endif
};
#endif

/* Common ADC12 data */

struct adc_cmn_s g_adc12_cmn =
{
  .initialized = 0
};

/* ADC1 state */

#ifdef CONFIG_STM32_ADC1
static struct stm32_dev_s g_adcpriv1 =
{
#ifdef CONFIG_STM32_ADC_NOIRQ
  .ops         = &g_adc_lowerops,
#else
  .irq         = STM32_IRQ_ADC12,
  .isr         = adc12_interrupt,
#endif
  .cmn         = &g_adc12_cmn,
  .intf        = 1,
  .resolution  = ADC1_RESOLUTION,
  .base        = STM32_ADC1_BASE,
  .smp1        = ADC1_SMPR1,
  .smp2        = ADC1_SMPR2,
#if defined(ADC1_HAVE_EXTSEL)
  .extsel      = ADC1_EXTSEL_VALUE | HAVE_EXTSEL_MASK,
#endif
#if defined(ADC1_HAVE_JEXTSEL)
  .jextsel     = ADC1_JEXTSEL_VALUE | HAVE_EXTSEL_MASK,
#endif
#ifdef ADC1_HAVE_TIMER
  .trigger     = CONFIG_STM32_ADC1_TIMTRIG,
  .tbase       = ADC1_TIMER_BASE,
  .pclck       = ADC1_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_ADC1_SAMPLE_FREQUENCY,
#endif
#ifdef ADC1_HAVE_DMA
  .dmachan     = ADC1_DMA_CHAN,
  .dmacfg      = ADC1_DMA_CFG,
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
#ifdef CONFIG_STM32_ADC_NOIRQ
  .ops         = &g_adc_lowerops,
#else
  .irq         = STM32_IRQ_ADC12,
  .isr         = adc12_interrupt,
#endif
  .cmn         = &g_adc12_cmn,
  .intf        = 2,
  .resolution  = ADC2_RESOLUTION,
  .base        = STM32_ADC2_BASE,
  .smp1        = ADC2_SMPR1,
  .smp2        = ADC2_SMPR2,
#ifdef ADC2_HAVE_EXTSEL
  .extsel      = ADC2_EXTSEL_VALUE | HAVE_EXTSEL_MASK,
#endif
#ifdef ADC2_HAVE_JEXTSEL
  .jextsel     = ADC2_JEXTSEL_VALUE | HAVE_EXTSEL_MASK,
#endif
#ifdef ADC2_HAVE_TIMER
  .trigger     = CONFIG_STM32_ADC2_TIMTRIG,
  .tbase       = ADC2_TIMER_BASE,
  .pclck       = ADC2_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32_ADC2_SAMPLE_FREQUENCY,
#endif
#ifdef ADC2_HAVE_DMA
  .dmachan     = ADC2_DMA_CHAN,
  .dmacfg      = ADC2_DMA_CFG,
  .hasdma      = true,
#endif
};

static struct adc_dev_s g_adcdev2 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv2,
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

static void stm32_modifyreg32(unsigned int addr, uint32_t clrbits,
                              uint32_t setbits)
{
  putreg32((getreg32(addr) & ~clrbits) | setbits, addr);
}

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

  if (priv->tbase == STM32_TIM1_BASE)
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

  if (priv->tbase == STM32_TIM1_BASE)
    {
      /* Reset output N polarity level, output N state, output compare state,
       * output compare N idle state.
       */

      ccer &= ~(ATIM_CCER_CC1NE | ATIM_CCER_CC1NP |
                ATIM_CCER_CC2NE | ATIM_CCER_CC2NP |
                ATIM_CCER_CC3NE | ATIM_CCER_CC3NP |
                ATIM_CCER_CC4NP);

      /* Reset the output compare and output compare N IDLE State */

      cr2 &= ~(ATIM_CR2_OIS1 | ATIM_CR2_OIS1N |
               ATIM_CR2_OIS2 | ATIM_CR2_OIS2N |
               ATIM_CR2_OIS3 | ATIM_CR2_OIS3N |
               ATIM_CR2_OIS4);
    }
  else
    {
      ccer &= ~(GTIM_CCER_CC1NP | GTIM_CCER_CC2NP | GTIM_CCER_CC3NP);
    }

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

static void adc_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  uint32_t regval;

  ainfo("regular enable: %d\n", enable ? 1 : 0);

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

/****************************************************************************
 * Name: adc_inj_startconv
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

#ifdef ADC_HAVE_INJECTED
static void adc_inj_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  uint32_t regval;

  ainfo("injected enable: %d\n", enable ? 1 : 0);

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

          while ((adc_getreg(priv, STM32_ADC_CR_OFFSET) & ADC_CR_JADSTP) != 0);
        }
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
  uint32_t rstbit;
  uint32_t enbit;
  uint32_t rst_reg;
  uint32_t en_reg;

  /* Pick the appropriate bit in the APB2 reset register.
   * For the STM32 F1, there is an individual bit to reset each ADC,
   * but for the STM32 F2/F4, there is one common reset for all ADCs.
   * THIS will probably cause some problems!
   */

  switch (priv->intf)
    {
#if defined(CONFIG_STM32_ADC1) || defined(CONFIG_STM32_ADC2)
      case 1:
      case 2:
        {
          rstbit = RCC_RSTR_ADC12RST;
          enbit  = RCC_RSTR_ADC12EN;
          rst_reg = RCC_RSTR_ADC12RST;
          en_reg = RCC_RSTR_ADC12EN;
          break;
        }
#endif
      default:
        {
          return;
        }
    }

  /* Set or clear the selected bit in the APB2 reset register.
   * modifyreg32() disables interrupts.  Disabling interrupts is necessary
   * because the APB2RTSR register is used by several different drivers.
   */

  if (reset)
    {
      /* Enable ADC reset state */

      modifyreg32(rst_reg, 0, rstbit);
    }
  else
    {
      /* Release ADC from reset state */

      modifyreg32(en_reg, enbit, 1);
    }
}

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

      adc_startconv(priv, false);

#ifdef ADC_HAVE_INJECTED
      /* Stop ongoing injected conversion */

      adc_inj_startconv(priv, false);
#endif

      /* Disable the ADC */

      adc_putreg(priv, STM32_ADC_CR_OFFSET, regval | ADC_CR_ADDIS);

      /* Wait for the ADC to be disabled */

      while ((adc_getreg(priv, STM32_ADC_CR_OFFSET) & ADC_CR_ADEN) != 0);
    }
}

/****************************************************************************
 * Name: adc_dmaconvcallback
 *
 * Description:
 *   Callback for DMA.  Called from the DMA transfer complete interrupt after
 *   all channels have been converted and transferred with DMA.
 *   Only for regular conversion.
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
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr, FAR void *arg)
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
#ifndef CONFIG_STM32_ADC_NOIRQ
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
#endif

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

  /* Turn off the ADC so we can write the RCC bits */

  adc_enable(priv, false);

  /* Only if this is the first instance */

  if (priv->cmn->initialized <= 0)
    {
      /* Enable ADC reset state */

      adc_rccreset(priv, true);

      /* Release ADC from reset state */

      adc_rccreset(priv, false);
    }

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

  /* Initialize the same sample time for each ADC.
   * During sample cycles channel selection bits must remain unchanged.
   */

  adc_putreg(priv, STM32_ADC_SMPR1_OFFSET, priv->smp1);
  adc_putreg(priv, STM32_ADC_SMPR2_OFFSET, priv->smp2);

  /* Enable the analog watchdog */

  clrbits = ADC_CFGR_AWD1CH_MASK;
  setbits = ADC_CFGR_AWD1EN | ADC_CFGR_AWD1SGL |
            (priv->r_chanlist[0] << ADC_CFGR_AWD1CH_SHIFT);

  /* Set the resolution of the conversion */

  clrbits |= ADC_CFGR_RES_MASK;
  setbits |= priv->resolution << ADC_CFGR_RES_SHIFT;

#ifdef ADC_HAVE_DMA
  if (priv->hasdma)
    {
      /* Set DMA mode */

      clrbits |= ADC_CFGR_DMACFG;

      setbits |= priv->dmacfg;

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

#ifndef CONFIG_STM32_ADC_NOIRQ
  /* Enable interrupt flags, but disable overrun interrupt */

  clrbits = ADC_IER_OVR;
  setbits = ADC_IER_ALLINTS & ~ADC_IER_OVR;

  /* Set IER configuration */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, clrbits, setbits);
#endif

  /* Configuration of the regular channel conversions */

  if (priv->cr_channels > 0)
    {
      adc_set_ch(dev, 0);
    }

  /* ADC CCR configuration */

  clrbits = ADC_CCR_DUAL_MASK | ADC_CCR_DELAY_MASK | ADC_CCR_DMACFG |
            ADC_CCR_MDMA_MASK | ADC_CCR_CKMODE_MASK | ADC_CCR_VREFEN |
            ADC_CCR_TSEN | ADC_CCR_VBATEN;
  setbits = ADC_CCR_DUAL_IND | ADC_CCR_DELAY(0) | ADC_CCR_MDMA_DISABLED |
            ADC_CCR_CKMODE_ASYNCH;

  if (priv->base == STM32_ADC1_BASE || priv->base == STM32_ADC2_BASE)
    {
      stm32_modifyreg32(STM32_ADC12_CCR, clrbits, setbits);
    }

  stm32_modifyreg32(STM32_ADC12_CCR, clrbits, setbits);

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

#ifndef CONFIG_STM32_ADC_NOIRQ
      stm32_dmasetup(priv->dma,
                     priv->base + STM32_ADC_DR_OFFSET,
                     (uint32_t)priv->r_dmabuffer,
                     priv->rnchannels,
                     ADC_DMA_CONTROL_WORD);

      stm32_dmastart(priv->dma, adc_dmaconvcallback, dev, false);
#endif
    }

#endif

  /* Set ADON to wake up the ADC from the power down state */

  adc_enable(priv, true);

  /* EXTSEL selection: These bits select the external event used
   * to trigger the start of conversion of a regular group.  NOTE:
   *
   * - The position with of the EXTSEL field varies from one STM32 MCU
   *   to another.
   * - The width of the EXTSEL field varies from one STM32 MCU to another.
   * - The value in priv->extsel is already shifted into the correct bit
   *   position.
   */

#ifdef ADC_HAVE_EXTSEL
  if (priv->extsel & HAVE_EXTSEL_MASK)
    {
      ainfo("Initializing extsel = 0x%08x\n", (priv->extsel & ~HAVE_EXTSEL_MASK));

      adc_modifyreg(priv, STM32_ADC_EXTREG_OFFSET,
                    ADC_EXTREG_EXTEN_MASK | ADC_EXTREG_EXTSEL_MASK,
                    ADC_EXTREG_EXTEN_DEFAULT | (priv->extsel & ~HAVE_EXTSEL_MASK));
    }
#endif

#ifdef ADC_HAVE_INJECTED
  /* Configuration of the injected channel conversions after adc enabled */

  if (priv->cj_channels > 0)
    {
      adc_inj_set_ch(dev, 0);
    }
#endif

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

#ifdef ADC_HAVE_INJECTED
      adc_inj_startconv(priv, true);
#endif
    }
#endif

  leave_critical_section(flags);

  ainfo("ISR:  0x%08x CR:   0x%08x CFGR: 0x%08x\n",
        adc_getreg(priv, STM32_ADC_ISR_OFFSET),
        adc_getreg(priv, STM32_ADC_CR_OFFSET),
        adc_getreg(priv, STM32_ADC_CFGR_OFFSET));

  ainfo("SQR1: 0x%08x SQR2: 0x%08x SQR3: 0x%08x\n",
        adc_getreg(priv, STM32_ADC_SQR1_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR2_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR3_OFFSET));

  ainfo("SQR4: 0x%08x\n", adc_getreg(priv, STM32_ADC_SQR4_OFFSET));

  if (priv->base == STM32_ADC1_BASE || priv->base == STM32_ADC2_BASE)
    {
      ainfo("CCR:  0x%08x\n", getreg32(STM32_ADC12_CCR));
    }

  ainfo("SMPR1: 0x%08x SMPR2: 0x%08x\n",
        adc_getreg(priv, STM32_ADC_SMPR1_OFFSET),
        adc_getreg(priv, STM32_ADC_SMPR2_OFFSET));

  ainfo("JSQR: 0x%08x DIFSEL: 0x%08x IER: 0x%08x\n",
        adc_getreg(priv, STM32_ADC_JSQR_OFFSET),
        adc_getreg(priv, STM32_ADC_DIFSEL_OFFSET),
        adc_getreg(priv, STM32_ADC_IER_OFFSET));
}

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
#ifndef CONFIG_STM32_ADC_NOIRQ
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
#endif
  int ret = OK;

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

  /* Enable the ADC interrupt */

#ifndef CONFIG_STM32_ADC_NOIRQ
  ainfo("Enable the ADC interrupt: irq=%d\n", priv->irq);
  up_enable_irq(priv->irq);
#endif

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

  /* Disable ADC interrupts and detach the ADC interrupt handler */

#ifndef CONFIG_STM32_ADC_NOIRQ
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
#endif

  if (priv->cmn->initialized <= 1)
    {
      /* Disable and reset the ADC module only when last instance */

      adc_rccreset(priv, true);

      /* Decrease instances counter */
    }

  priv->cmn->initialized -= 1;
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
 * Name: adc_sqrbits
 ****************************************************************************/

static uint32_t adc_sqrbits(FAR struct stm32_dev_s *priv, int first, int last,
                            int offset)
{
  uint32_t bits = 0;
  int i;

  for (i = first - 1;
       i < priv->rnchannels && i < last;
       i++, offset += ADC_SQ_OFFSET)
    {
      bits |= ((uint32_t)priv->r_chanlist[i]) << offset;
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
      for (i = 0; i < priv->cr_channels && priv->r_chanlist[i] != ch - 1; i++);

      if (i >= priv->cr_channels)
        {
          return -ENODEV;
        }

      priv->current   = i;
      priv->rnchannels = 1;
    }

  bits = adc_sqrbits(priv, ADC_SQR4_FIRST, ADC_SQR4_LAST, ADC_SQR4_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR4_OFFSET, ~ADC_SQR4_RESERVED, bits);

  bits = adc_sqrbits(priv, ADC_SQR3_FIRST, ADC_SQR3_LAST, ADC_SQR3_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR3_OFFSET, ~ADC_SQR3_RESERVED, bits);

  bits = adc_sqrbits(priv, ADC_SQR2_FIRST, ADC_SQR2_LAST, ADC_SQR2_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR2_OFFSET, ~ADC_SQR2_RESERVED, bits);

  bits = ((uint32_t)priv->rnchannels - 1) << ADC_SQR1_L_SHIFT |
         adc_sqrbits(priv, ADC_SQR1_FIRST, ADC_SQR1_LAST, ADC_SQR1_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR1_OFFSET, ~ADC_SQR1_RESERVED, bits);

  return OK;
}

#ifdef ADC_HAVE_INJECTED
static int adc_inj_set_ch(FAR struct adc_dev_s *dev, uint8_t ch)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  uint32_t regval = 0;
  int i;

  /* Configure JEXTSEL */

  if (priv->jextsel & HAVE_EXTSEL_MASK)
    {
      ainfo("Initializing jextsel = 0x%08x\n", (priv->jextsel & ~HAVE_EXTSEL_MASK));

      regval = ADC_JEXTREG_JEXTEN_DEFAULT | (priv->jextsel & ~HAVE_EXTSEL_MASK);
    }

  /* Configure injected sequence length */

  regval |= ADC_JSQR_JL(priv->cj_channels);

  /* Configure injected channels */

  for (i = 0 ; i < priv->cj_channels; i += 1)
    {
      regval |= priv->j_chanlist[i] << (ADC_JSQR_JSQ1_SHIFT + 6 * i);
    }

  adc_putreg(priv, STM32_ADC_JSQR_OFFSET, regval);

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
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          /* Start regular conversion if regular channels configured */

          if (priv->cr_channels > 0)
            {
              adc_startconv(priv, true);
            }

#ifdef ADC_HAVE_INJECTED
          /* Start injected conversion if injected channels configured */

          if (priv->cj_channels > 0)
            {
              adc_inj_startconv(priv, true);
            }
#endif

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

#ifndef CONFIG_STM32_ADC_NOIRQ
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

#ifndef CONFIG_STM32_ADC_NOIRQ
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

#ifdef CONFIG_STM32_ADC_NOIRQ

/****************************************************************************
 * Name: adc_intack
 ****************************************************************************/

static void adc_intack(FAR struct stm32_adc_dev_s *dev, uint32_t source)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  /* Clear pending interrupts */

  adc_putreg(priv, STM32_ADC_ISR_OFFSET, source);
}

/****************************************************************************
 * Name: adc_inten
 ****************************************************************************/

static void adc_inten(FAR struct stm32_adc_dev_s *dev, uint32_t source)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  /* Enable interrupts */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, 0, source);
}

/****************************************************************************
 * Name: adc_intdis
 ****************************************************************************/

static void adc_intdis(FAR struct stm32_adc_dev_s *dev, uint32_t source)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  /* Disable interrupts */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, source, 0);
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
 * Name: adc_regbufregister
 ****************************************************************************/
#ifdef ADC_HAVE_DMA
static int adc_regbufregister(FAR struct stm32_adc_dev_s *dev, uint16_t *buffer, uint8_t len)
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
#endif

/****************************************************************************
 * Name: adc_inj_get
 ****************************************************************************/

#ifdef ADC_HAVE_INJECTED
static uint32_t adc_injget(FAR struct stm32_adc_dev_s *dev, uint8_t chan)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;
  uint32_t regval = 0;

  if (chan > priv->cj_channels-1)
    {
      goto errout;
    }

  regval = adc_getreg(priv, STM32_ADC_JDR1_OFFSET+4*(chan)) & ADC_JDR_JDATA_MASK;

errout:
  return regval;
}
#endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adcinitialize
 *
 * Description:
 *   Initialize the ADC.
 *
 *   TODO: describe injected channels configuration.
 *
 *   The logic is, save rnchannels : # of channels (conversions) in ADC_SQR1_L
 *   Then, take the chanlist array and store it in the SQR Regs,
 *     chanlist[0] -> ADC_SQR3_SQ1
 *     chanlist[1] -> ADC_SQR3_SQ2
 *     ...
 *     chanlist[15]-> ADC_SQR1_SQ16
 *
 *   up to
 *     chanlist[rnchannels]
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
                                      int channels)
{
  FAR struct adc_dev_s *dev;
  FAR struct stm32_dev_s *priv;
  uint8_t cr_channels = 0;
#ifdef ADC_HAVE_INJECTED
  uint8_t cj_channels = 0;
  FAR uint8_t *j_chanlist = NULL;
#endif

  switch (intf)
    {
#ifdef CONFIG_STM32_ADC1
      case 1:
        {
          ainfo("ADC1 selected\n");
          dev         = &g_adcdev1;
          cr_channels = channels - ADC1_INJECTED_CHAN;
#ifdef ADC_HAVE_INJECTED
          cj_channels = ADC1_INJECTED_CHAN;
          j_chanlist  = (FAR uint8_t *)chanlist + cr_channels;
#endif
          break;
        }
#endif
#ifdef CONFIG_STM32_ADC2
      case 2:
        {
          ainfo("ADC2 selected\n");
          dev         = &g_adcdev2;
          cr_channels = channels - ADC2_INJECTED_CHAN;
#ifdef ADC_HAVE_INJECTED
          cj_channels = ADC2_INJECTED_CHAN;
          j_chanlist  = (FAR uint8_t *)chanlist + cr_channels;
#endif
          break;
        }
#endif
      default:
        {
          aerr("ERROR: No ADC interface defined\n");
          return NULL;
        }
    }

  /* Configure the selected ADC */

  priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  DEBUGASSERT(cr_channels <= ADC_REG_MAX_SAMPLES);

  /* Configure regular channels */

  priv->cr_channels = cr_channels;
  memcpy(priv->r_chanlist, chanlist, cr_channels);

#ifdef ADC_HAVE_INJECTED
  /* Configure injected channels */

  DEBUGASSERT(cj_channels <= ADC_INJ_MAX_SAMPLES);

  priv->cj_channels = cj_channels;
  memcpy(priv->j_chanlist, j_chanlist, cj_channels);
#endif

#ifndef CONFIG_STM32_ADC_NOIRQ
  priv->cb          = NULL;
#endif

#ifdef ADC_HAVE_INJECTED
  ainfo("intf: %d cr_channels: %d, cj_channels: %d\n",
        intf, priv->cr_channels, priv->cj_channels);
#else
  ainfo("intf: %d cr_channels: %d\n", intf, priv->cr_channels);
#endif

  /* Increase instances counter */

  priv->cmn->initialized += 1;

  return dev;
}

#endif /* CONFIG_STM32_STM32F33XX */
#endif /* CONFIG_STM32_ADC1 || CONFIG_STM32_ADC2 */
#endif /* CONFIG_STM32_ADC */
