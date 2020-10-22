/*****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_adc.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2015 Motorola Mobility, LLC. All rights reserved.
 *   Copyright (C) 2015 Omni Hoverboards Inc. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Diego Sanchez <dsanchez@nx-engineering.com>
 *           Paul Alexander Patience <paul-a.patience@polymtl.ca>
 *           Mateusz Szafoni <raiden00@railab.me>
 *           Juha Niskanen <juha.niskanen@haltian.com>
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <string.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/power/pm.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "chip.h"
#include "stm32l4_rcc.h"
#include "stm32l4_tim.h"
#include "stm32l4_dma.h"
#include "stm32l4_adc.h"

/* ADC "upper half" support must be enabled */

#ifdef CONFIG_ADC

/* Some ADC peripheral must be enabled */

#if defined(CONFIG_STM32L4_ADC1) || defined(CONFIG_STM32L4_ADC2) || \
    defined(CONFIG_STM32L4_ADC3)

#if !(defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L4X5) || \
      defined(CONFIG_STM32L4_STM32L4X6) || defined(CONFIG_STM32L4_STM32L4XR))
#  error "Unrecognized STM32 chip"
#endif

#if defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L4XR)
#  if defined(CONFIG_STM32L4_ADC2) || defined(CONFIG_STM32L4_ADC3)
#    error "Using non-existent ADC"
#  endif
#endif

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

#ifndef container_of
#  define container_of(ptr, type, member) \
          ((type *)((intptr_t)(ptr) - offsetof(type, member)))
#endif

/* RCC reset *****************************************************************/

#define STM32L4_RCC_RSTR STM32L4_RCC_AHB2RSTR
#define RCC_RSTR_ADC1RST RCC_AHB2RSTR_ADCRST
#define RCC_RSTR_ADC2RST RCC_AHB2RSTR_ADCRST
#define RCC_RSTR_ADC3RST RCC_AHB2RSTR_ADCRST

/* ADC interrupts ************************************************************/

#if defined(CONFIG_STM32L4_STM32L4X3) || defined(CONFIG_STM32L4_STM32L4XR)
#  define STM32L4_IRQ_ADC12 STM32L4_IRQ_ADC1
#endif

/* ADC Channels/DMA **********************************************************/

/* The maximum number of channels that can be sampled.  While DMA support is
 * very nice for reliable multi-channel sampling, the STM32L4 can function
 * without, although there is a risk of overrun.
 */

#define ADC_MAX_CHANNELS_DMA   16
#define ADC_MAX_CHANNELS_NODMA 16

#ifdef ADC_HAVE_DMA
#  if !defined(CONFIG_STM32L4_DMA1) && !defined(CONFIG_STM32L4_DMA2)
#    /* REVISIT: check accordingly to which one is configured in board.h */
#    error "STM32L4 ADC DMA support requires CONFIG_STM32L4_DMA1 or CONFIG_STM32L4_DMA2"
#  endif
#endif

#ifdef ADC_HAVE_DMA
#  define ADC_MAX_SAMPLES ADC_MAX_CHANNELS_DMA
#else
#  define ADC_MAX_SAMPLES ADC_MAX_CHANNELS_NODMA
#endif

/* DMA channels and interface values */

#define ADC_DMA_CONTROL_WORD (DMA_CCR_MSIZE_16BITS | \
                              DMA_CCR_PSIZE_16BITS | \
                              DMA_CCR_MINC | \
                              DMA_CCR_CIRC)

/* Sample time default configuration */

#ifndef CONFIG_STM32L4_ADC_SMPR
#  define ADC_SMPR_DEFAULT    ADC_SMPR_640p5
#else
#  define ADC_SMPR_DEFAULT CONFIG_STM32L4_ADC_SMPR
#endif

#define ADC_SMPR1_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR1_SMP0_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP1_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP2_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP3_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP4_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP5_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP6_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP7_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP8_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP9_SHIFT))
#define ADC_SMPR2_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR2_SMP10_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP11_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP12_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP13_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP14_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP15_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP16_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP17_SHIFT) | \
                             (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP18_SHIFT))

/* The channels not in this range are internal channels for reading Vref
 * (ADC1 only), Vbat/3, Vts or DAC outputs. The DAC outputs on ADC3 are an
 * exception and not classified as internal channels by this implementation,
 * as no dedicated register bits are needed for reading those.
 */

#define ADC_EXTERNAL_CHAN_MIN  1
#define ADC_EXTERNAL_CHAN_MAX  16

/* ADCx_EXTSEL_VALUE can be set by this driver (look at stm32l4_adc.h) or
 * by board specific logic in board.h file.
 */

#define ADC_EXTREG_EXTSEL_MASK       ADC_CFGR_EXTSEL_MASK
#define ADC_EXTREG_EXTEN_MASK        ADC_CFGR_EXTEN_MASK
#define ADC_EXTREG_EXTEN_DEFAULT     ADC_CFGR_EXTEN_RISING

#ifdef ADC1_EXTSEL_VALUE
#  define ADC1_HAVE_EXTCFG  1
#  define ADC1_EXTCFG_VALUE (ADC1_EXTSEL_VALUE | ADC_EXTREG_EXTEN_DEFAULT)
#else
#  undef ADC1_HAVE_EXTCFG
#endif
#ifdef ADC2_EXTSEL_VALUE
#  define ADC2_HAVE_EXTCFG  1
#  define ADC2_EXTCFG_VALUE (ADC2_EXTSEL_VALUE | ADC_EXTREG_EXTEN_DEFAULT)
#else
#  undef ADC2_HAVE_EXTCFG
#endif
#ifdef ADC3_EXTSEL_VALUE
#  define ADC3_HAVE_EXTCFG  1
#  define ADC3_EXTCFG_VALUE (ADC3_EXTSEL_VALUE | ADC_EXTREG_EXTEN_DEFAULT)
#else
#  undef ADC3_HAVE_EXTCFG
#endif
#ifdef ADC4_EXTSEL_VALUE
#  define ADC4_HAVE_EXTCFG  1
#  define ADC4_EXTCFG_VALUE (ADC4_EXTSEL_VALUE | ADC_EXTREG_EXTEN_DEFAULT)
#else
#  undef ADC4_HAVE_EXTCFG
#endif

#if defined(ADC1_HAVE_EXTCFG) || defined(ADC2_HAVE_EXTCFG) || \
    defined(ADC3_HAVE_EXTCFG) || defined(ADC3_HAVE_EXTCFG)
#  define ADC_HAVE_EXTCFG
#endif

/* ADC resolution supported */

#define HAVE_ADC_RESOLUTION

/*****************************************************************************
 * Private Types
 *****************************************************************************/

/* This structure describes the state of one ADC block */

struct stm32_dev_s
{
#ifdef CONFIG_STM32L4_ADC_LL_OPS
  FAR const struct stm32_adc_ops_s *llops; /* Low-level ADC ops */
#endif
#ifndef CONFIG_STM32L4_ADC_NOIRQ
  FAR const struct adc_callback_s *cb;
  uint8_t irq;          /* Interrupt generated by this ADC block */
#endif
  uint8_t nchannels;    /* Number of channels */
  uint8_t cchannels;    /* Number of configured channels */
  uint8_t intf;         /* ADC interface number */
  uint8_t current;      /* Current ADC channel being converted */
#ifdef HAVE_ADC_RESOLUTION
  uint8_t resolution;   /* ADC resolution (0-3) */
#endif
#ifdef ADC_HAVE_DMA
  uint8_t dmachan;      /* DMA channel needed by this ADC */
  uint8_t dmacfg;       /* DMA channel configuration */
  bool    hasdma;       /* True: This ADC supports DMA */
#endif
#ifdef ADC_HAVE_DFSDM
  bool    hasdfsdm;     /* True: This ADC routes its output to DFSDM */
#endif
#ifdef ADC_HAVE_TIMER
  uint8_t trigger;      /* Timer trigger channel: 0=CC1, 1=CC2, 2=CC3,
                         * 3=CC4, 4=TRGO, 5=TRGO2 */
#endif
  xcpt_t   isr;         /* Interrupt handler for this ADC block */
  uint32_t base;        /* Base address of registers unique to this ADC
                         * block */
#if defined(ADC_HAVE_TIMER) || defined(ADC_HAVE_EXTCFG)
  uint32_t extcfg;      /* External event configuration for regular group */
#endif
#ifdef ADC_HAVE_TIMER
  uint32_t tbase;       /* Base address of timer used by this ADC block */
  uint32_t pclck;       /* The PCLK frequency that drives this timer */
  uint32_t freq;        /* The desired frequency of conversions */
#endif

#ifdef CONFIG_PM
  struct pm_callback_s pm_callback;
#endif

#ifdef ADC_HAVE_DMA
  DMA_HANDLE dma;       /* Allocated DMA channel */

  /* DMA transfer buffer */

  uint16_t dmabuffer[ADC_MAX_SAMPLES];
#endif

  /* List of selected ADC channels to sample */

  uint8_t  chanlist[ADC_MAX_SAMPLES];
};

/*****************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

/* ADC Register access */

static void stm32_modifyreg32(unsigned int addr, uint32_t clrbits,
                              uint32_t setbits);
static uint32_t adc_getreg(FAR struct stm32_dev_s *priv, int offset);
static void     adc_putreg(FAR struct stm32_dev_s *priv, int offset,
                           uint32_t value);
static void     adc_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                              uint32_t clrbits, uint32_t setbits);
static void     adc_dumpregs(FAR struct stm32_dev_s *priv);

#ifdef ADC_HAVE_TIMER
static uint16_t tim_getreg(FAR struct stm32_dev_s *priv, int offset);
static void     tim_putreg(FAR struct stm32_dev_s *priv, int offset,
                           uint16_t value);
static void     tim_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                              uint16_t clrbits, uint16_t setbits);
static void     tim_dumpregs(FAR struct stm32_dev_s *priv,
                             FAR const char *msg);
#endif

/* ADC Miscellaneous Helpers */

static void     adc_rccreset(FAR struct stm32_dev_s *priv, bool reset);
static void     adc_enable(FAR struct stm32_dev_s *priv);
static uint32_t adc_sqrbits(FAR struct stm32_dev_s *priv, int first,
                            int last, int offset);
static int      adc_set_ch(FAR struct adc_dev_s *dev, uint8_t ch);
static bool     adc_internal(FAR struct stm32_dev_s * priv,
                             uint32_t *adc_ccr);
#ifdef HAVE_ADC_RESOLUTION
static int      adc_resolution_set(FAR struct adc_dev_s *dev, uint8_t res);
#endif
static void     adc_sample_time_set(FAR struct adc_dev_s *dev);
static void     adc_startconv(FAR struct stm32_dev_s *priv, bool enable);
#ifdef ADC_HAVE_TIMER
static void     adc_timstart(FAR struct stm32_dev_s *priv, bool enable);
static int      adc_timinit(FAR struct stm32_dev_s *priv);
#endif
#ifdef ADC_HAVE_DMA
static void     adc_dma_cfg(FAR struct stm32_dev_s *priv);
static void     adc_dma_start(FAR struct adc_dev_s *dev);
#  ifndef CONFIG_STM32L4_ADC_NOIRQ
static void     adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                    FAR void *arg);
#  endif
#endif
#if defined(ADC_HAVE_DFSDM) || defined(CONFIG_STM32L4_ADC_LL_OPS)
static int      adc_offset_set(FAR struct stm32_dev_s *priv, uint8_t ch,
                               uint8_t i, uint16_t offset);
#endif
#ifdef ADC_HAVE_EXTCFG
static int      adc_extsel_set(FAR struct adc_dev_s *dev, uint32_t extcfg);
#endif
#ifdef CONFIG_PM
static int      adc_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e state);
#endif
#ifdef CONFIG_STM32L4_ADC_LL_OPS
static void     adc_llops_intack(FAR struct stm32_adc_dev_s *dev,
                                 uint32_t source);
static void     adc_llops_inten(FAR struct stm32_adc_dev_s *dev,
                                uint32_t source);
static void     adc_llops_intdis(FAR struct stm32_adc_dev_s *dev,
                                 uint32_t source);
static uint32_t adc_llops_intget(FAR struct stm32_adc_dev_s *dev);
static uint32_t adc_llops_regget(FAR struct stm32_adc_dev_s *dev);
static void     adc_llops_startconv(FAR struct stm32_adc_dev_s *dev,
                                    bool enable);
static int      adc_llops_offset_set(FAR struct stm32_adc_dev_s *dev,
                                     uint8_t ch, uint8_t i, uint16_t offset);
#  ifdef ADC_HAVE_DMA
static int      adc_regbufregister(FAR struct stm32_adc_dev_s *dev,
                                   uint16_t *buffer, uint8_t len);
#  endif
static void     adc_llops_dumpregs(FAR struct stm32_adc_dev_s *dev);
#endif /* CONFIG_STM32L4_ADC_LL_OPS */

/* ADC Interrupt Handler */

#ifndef CONFIG_STM32L4_ADC_NOIRQ
static int adc_interrupt(FAR struct adc_dev_s *dev, uint32_t regval);
#  if defined(CONFIG_STM32L4_ADC1) || defined(CONFIG_STM32L4_ADC2)
static int adc12_interrupt(int irq, FAR void *context, FAR void *arg);
#  endif
#  if defined(CONFIG_STM32L4_ADC3)
static int adc3_interrupt(int irq, FAR void *context, FAR void *arg);
#  endif
#endif /* CONFIG_STM32L4_ADC_NOIRQ */

/* ADC Driver Methods */

static int  adc_bind(FAR struct adc_dev_s *dev,
                     FAR const struct adc_callback_s *callback);
static void adc_reset(FAR struct adc_dev_s *dev);
static int  adc_setup(FAR struct adc_dev_s *dev);
static void adc_shutdown(FAR struct adc_dev_s *dev);
static void adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

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

#ifdef CONFIG_STM32L4_ADC_LL_OPS
static const struct stm32_adc_ops_s g_adc_llops =
{
  .int_ack       = adc_llops_intack,
  .int_get       = adc_llops_intget,
  .int_en        = adc_llops_inten,
  .int_dis       = adc_llops_intdis,
  .val_get       = adc_llops_regget,
  .reg_startconv = adc_llops_startconv,
  .offset_set    = adc_llops_offset_set,
#  ifdef ADC_HAVE_DMA
  .regbuf_reg    = adc_regbufregister,
#  endif
  .dump_regs     = adc_llops_dumpregs
};
#endif /* CONFIG_STM32L4_ADC_LL_OPS */

/* ADC1 state */

#ifdef CONFIG_STM32L4_ADC1
static struct stm32_dev_s g_adcpriv1 =
{
#ifdef CONFIG_STM32L4_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
#ifndef CONFIG_STM32L4_ADC_NOIRQ
  .irq         = STM32L4_IRQ_ADC12,
  .isr         = adc12_interrupt,
#endif /* CONFIG_STM32L4_ADC_NOIRQ */
  .intf        = 1,
#ifdef HAVE_ADC_RESOLUTION
  .resolution  = CONFIG_STM32L4_ADC1_RESOLUTION,
#endif  
  .base        = STM32L4_ADC1_BASE,
#if defined(ADC1_HAVE_TIMER) || defined(ADC1_HAVE_EXTCFG)
  .extcfg      = ADC1_EXTCFG_VALUE,
#endif
#ifdef ADC1_HAVE_TIMER
  .trigger     = CONFIG_STM32L4_ADC1_TIMTRIG,
  .tbase       = ADC1_TIMER_BASE,
  .pclck       = ADC1_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32L4_ADC1_SAMPLE_FREQUENCY,
#endif
#ifdef ADC1_HAVE_DMA
  .dmachan     = ADC1_DMA_CHAN,
  .dmacfg      = CONFIG_STM32L4_ADC1_DMA_CFG,
  .hasdma      = true,
#endif
#ifdef ADC1_HAVE_DFSDM
  .hasdfsdm    = true,
#endif
#ifdef CONFIG_PM
  .pm_callback =
    {
      .prepare = adc_pm_prepare,
    }
#endif
};

static struct adc_dev_s g_adcdev1 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv1,
};
#endif

/* ADC2 state */

#ifdef CONFIG_STM32L4_ADC2
static struct stm32_dev_s g_adcpriv2 =
{
#ifdef CONFIG_STM32L4_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
#ifndef CONFIG_STM32L4_ADC_NOIRQ
  .irq         = STM32L4_IRQ_ADC12,
  .isr         = adc12_interrupt,
#endif /* CONFIG_STM32L4_ADC_NOIRQ */
  .intf        = 2,
#ifdef HAVE_ADC_RESOLUTION
  .resolution  = CONFIG_STM32L4_ADC2_RESOLUTION,
#endif
  .base        = STM32L4_ADC2_BASE,
#if defined(ADC2_HAVE_TIMER) || defined(ADC2_HAVE_EXTCFG)
  .extcfg      = ADC2_EXTCFG_VALUE,
#endif
#ifdef ADC2_HAVE_TIMER
  .trigger     = CONFIG_STM32L4_ADC2_TIMTRIG,
  .tbase       = ADC2_TIMER_BASE,
  .pclck       = ADC2_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32L4_ADC2_SAMPLE_FREQUENCY,
#endif
#ifdef ADC2_HAVE_DMA
  .dmachan     = ADC2_DMA_CHAN,
  .dmacfg      = CONFIG_STM32L4_ADC1_DMA_CFG,
  .hasdma      = true,
#endif
#ifdef ADC2_HAVE_DFSDM
  .hasdfsdm    = true,
#endif
#ifdef CONFIG_PM
  .pm_callback =
    {
      .prepare = adc_pm_prepare,
    }
#endif
};

static struct adc_dev_s g_adcdev2 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv2,
};
#endif

/* ADC3 state */

#ifdef CONFIG_STM32L4_ADC3
static struct stm32_dev_s g_adcpriv3 =
{
#ifdef CONFIG_STM32L4_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
#ifndef CONFIG_STM32L4_ADC_NOIRQ
  .irq         = STM32L4_IRQ_ADC3,
  .isr         = adc3_interrupt,
#endif /* CONFIG_STM32L4_ADC_NOIRQ */
  .intf        = 3,
#ifdef HAVE_ADC_RESOLUTION
  .resolution  = CONFIG_STM32L4_ADC3_RESOLUTION,
#endif  
  .base        = STM32L4_ADC3_BASE,
#if defined(ADC3_HAVE_TIMER) || defined(ADC3_HAVE_EXTCFG)
  .extcfg      = ADC3_EXTCFG_VALUE,
#endif
#ifdef ADC3_HAVE_TIMER
  .trigger     = CONFIG_STM32L4_ADC3_TIMTRIG,
  .tbase       = ADC3_TIMER_BASE,
  .pclck       = ADC3_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32L4_ADC3_SAMPLE_FREQUENCY,
#endif
#ifdef ADC3_HAVE_DMA
  .dmachan     = ADC3_DMA_CHAN,
  .dmacfg      = CONFIG_STM32L4_ADC1_DMA_CFG,
  .hasdma      = true,
#endif
#ifdef ADC3_HAVE_DFSDM
  .hasdfsdm    = true,
#endif
#ifdef CONFIG_PM
  .pm_callback =
    {
      .prepare = adc_pm_prepare,
    }
#endif
};

static struct adc_dev_s g_adcdev3 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv3,
};
#endif

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
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
 *****************************************************************************/

static void stm32_modifyreg32(unsigned int addr, uint32_t clrbits,
                                      uint32_t setbits)
{
  putreg32((getreg32(addr) & ~clrbits) | setbits, addr);
}

/*****************************************************************************
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
 *****************************************************************************/

static uint32_t adc_getreg(FAR struct stm32_dev_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/*****************************************************************************
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
 *****************************************************************************/

static void adc_putreg(FAR struct stm32_dev_s *priv, int offset,
                       uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/*****************************************************************************
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
 *****************************************************************************/

static void adc_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits)
{
  adc_putreg(priv, offset, (adc_getreg(priv, offset) & ~clrbits) | setbits);
}

/*****************************************************************************
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
 *****************************************************************************/

#ifdef ADC_HAVE_TIMER
static uint16_t tim_getreg(FAR struct stm32_dev_s *priv, int offset)
{
  return getreg16(priv->tbase + offset);
}
#endif

/*****************************************************************************
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
 *****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void tim_putreg(FAR struct stm32_dev_s *priv, int offset,
                       uint16_t value)
{
  putreg16(value, priv->tbase + offset);
}
#endif

/*****************************************************************************
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
 *****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void tim_modifyreg(FAR struct stm32_dev_s *priv, int offset,
                          uint16_t clrbits, uint16_t setbits)
{
  tim_putreg(priv, offset, (tim_getreg(priv, offset) & ~clrbits) | setbits);
}
#endif

/*****************************************************************************
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
 *****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void tim_dumpregs(FAR struct stm32_dev_s *priv, FAR const char *msg)
{
  ainfo("%s:\n", msg);
  ainfo("  CR1: %04x CR2:  %04x SMCR:  %04x DIER:  %04x\n",
        tim_getreg(priv, STM32L4_GTIM_CR1_OFFSET),
        tim_getreg(priv, STM32L4_GTIM_CR2_OFFSET),
        tim_getreg(priv, STM32L4_GTIM_SMCR_OFFSET),
        tim_getreg(priv, STM32L4_GTIM_DIER_OFFSET));
  ainfo("   SR: %04x EGR:  0000 CCMR1: %04x CCMR2: %04x\n",
        tim_getreg(priv, STM32L4_GTIM_SR_OFFSET),
        tim_getreg(priv, STM32L4_GTIM_CCMR1_OFFSET),
        tim_getreg(priv, STM32L4_GTIM_CCMR2_OFFSET));
  ainfo(" CCER: %04x CNT:  %04x PSC:   %04x ARR:   %04x\n",
        tim_getreg(priv, STM32L4_GTIM_CCER_OFFSET),
        tim_getreg(priv, STM32L4_GTIM_CNT_OFFSET),
        tim_getreg(priv, STM32L4_GTIM_PSC_OFFSET),
        tim_getreg(priv, STM32L4_GTIM_ARR_OFFSET));
  ainfo(" CCR1: %04x CCR2: %04x CCR3:  %04x CCR4:  %04x\n",
        tim_getreg(priv, STM32L4_GTIM_CCR1_OFFSET),
        tim_getreg(priv, STM32L4_GTIM_CCR2_OFFSET),
        tim_getreg(priv, STM32L4_GTIM_CCR3_OFFSET),
        tim_getreg(priv, STM32L4_GTIM_CCR4_OFFSET));

  if (priv->tbase == STM32L4_TIM1_BASE || priv->tbase == STM32L4_TIM8_BASE)
    {
      ainfo("  RCR: %04x BDTR: %04x DCR:   %04x DMAR:  %04x\n",
            tim_getreg(priv, STM32L4_ATIM_RCR_OFFSET),
            tim_getreg(priv, STM32L4_ATIM_BDTR_OFFSET),
            tim_getreg(priv, STM32L4_ATIM_DCR_OFFSET),
            tim_getreg(priv, STM32L4_ATIM_DMAR_OFFSET));
    }
  else
    {
      ainfo("  DCR: %04x DMAR: %04x\n",
            tim_getreg(priv, STM32L4_GTIM_DCR_OFFSET),
            tim_getreg(priv, STM32L4_GTIM_DMAR_OFFSET));
    }
}
#endif

/*****************************************************************************
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
 *****************************************************************************/

#ifdef ADC_HAVE_TIMER
static void adc_timstart(FAR struct stm32_dev_s *priv, bool enable)
{
  ainfo("enable: %d\n", enable ? 1 : 0);

  if (enable)
    {
      /* Start the counter */

      tim_modifyreg(priv, STM32L4_GTIM_CR1_OFFSET, 0, GTIM_CR1_CEN);
    }
  else
    {
      /* Disable the counter */

      tim_modifyreg(priv, STM32L4_GTIM_CR1_OFFSET, GTIM_CR1_CEN, 0);
    }
}
#endif

/*****************************************************************************
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
 *****************************************************************************/

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

  /* NOTE: EXTSEL configuration was done during adc configuration */

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
  tim_modifyreg(priv, STM32L4_GTIM_CR1_OFFSET, clrbits, setbits);

  /* Set the reload and prescaler values */

  tim_putreg(priv, STM32L4_GTIM_PSC_OFFSET, prescaler - 1);
  tim_putreg(priv, STM32L4_GTIM_ARR_OFFSET, reload);

  /* Clear the advanced timers repetition counter in TIM1 */

  if (priv->tbase == STM32L4_TIM1_BASE || priv->tbase == STM32L4_TIM8_BASE)
    {
      tim_putreg(priv, STM32L4_ATIM_RCR_OFFSET, 0);
      tim_putreg(priv, STM32L4_ATIM_BDTR_OFFSET, ATIM_BDTR_MOE); /* Check me */
    }

  /* TIMx event generation: Bit 0 UG: Update generation */

  tim_putreg(priv, STM32L4_GTIM_EGR_OFFSET, GTIM_EGR_UG);

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

          tim_putreg(priv, STM32L4_GTIM_CCR1_OFFSET, (uint16_t)(reload >> 1));
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

          tim_putreg(priv, STM32L4_GTIM_CCR2_OFFSET, (uint16_t)(reload >> 1));
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

          tim_putreg(priv, STM32L4_GTIM_CCR3_OFFSET, (uint16_t)(reload >> 1));
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

          tim_putreg(priv, STM32L4_GTIM_CCR4_OFFSET, (uint16_t)(reload >> 1));
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

          tim_putreg(priv, STM32L4_GTIM_CCR4_OFFSET, (uint16_t)(reload >> 1));
        }
        break;

      default:
        aerr("ERROR: No such trigger: %d\n", priv->trigger);
        return -EINVAL;
    }

  /* Disable the Channel by resetting the CCxE Bit in the CCER register */

  ccer = tim_getreg(priv, STM32L4_GTIM_CCER_OFFSET);
  ccer &= ~ccenable;
  tim_putreg(priv, STM32L4_GTIM_CCER_OFFSET, ccer);

  /* Fetch the CR2, CCMR1, and CCMR2 register (already have ccer) */

  cr2   = tim_getreg(priv, STM32L4_GTIM_CR2_OFFSET);
  ccmr1 = tim_getreg(priv, STM32L4_GTIM_CCMR1_OFFSET);
  ccmr2 = tim_getreg(priv, STM32L4_GTIM_CCMR2_OFFSET);

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

  if (priv->tbase == STM32L4_TIM1_BASE || priv->tbase == STM32L4_TIM8_BASE)
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

  tim_putreg(priv, STM32L4_GTIM_CR2_OFFSET, cr2);
  tim_putreg(priv, STM32L4_GTIM_CCMR1_OFFSET, ccmr1);
  tim_putreg(priv, STM32L4_GTIM_CCMR2_OFFSET, ccmr2);
  tim_putreg(priv, STM32L4_GTIM_CCER_OFFSET, ccer);
  tim_putreg(priv, STM32L4_GTIM_EGR_OFFSET, egr);

  /* Set the ARR Preload Bit */

  tim_modifyreg(priv, STM32L4_GTIM_CR1_OFFSET, 0, GTIM_CR1_ARPE);

  /* Enable the timer counter */

  adc_timstart(priv, true);

  tim_dumpregs(priv, "After starting timers");

  return OK;
}
#endif

/*****************************************************************************
 * Name: adc_pm_prepare
 *
 * Description:
 *   Called by power management framework when it wants to enter low power
 *   states. Check if ADC is in progress and if so prevent from entering STOP.
 *
 *****************************************************************************/

#ifdef CONFIG_PM
static int adc_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e state)
{
  FAR struct stm32_dev_s *priv =
      container_of(cb, struct stm32_dev_s, pm_callback);
  uint32_t regval;

  regval = adc_getreg(priv, STM32L4_ADC_CR_OFFSET);
  if ((state >= PM_IDLE) && (regval & ADC_CR_ADSTART))
    {
      return -EBUSY;
    }

  return OK;
}
#endif

/*****************************************************************************
 * Name: adc_wdog_enable
 *
 * Description:
 *   Enable the analog watchdog.
 *
 *****************************************************************************/

static void adc_wdog_enable(struct stm32_dev_s *priv)
{
  uint32_t regval;

  /* Initialize the Analog watchdog enable */

  regval  = adc_getreg(priv, STM32L4_ADC_CFGR_OFFSET);
  regval |= ADC_CFGR_AWD1EN | ADC_CFGR_CONT | ADC_CFGR_OVRMOD;
  adc_putreg(priv, STM32L4_ADC_CFGR_OFFSET, regval);

  /* Switch to analog watchdog interrupt */

  regval = adc_getreg(priv, STM32L4_ADC_IER_OFFSET);
  regval |= ADC_INT_AWD1;
  regval &= ~ADC_INT_EOC;
  adc_putreg(priv, STM32L4_ADC_IER_OFFSET, regval);
}

/*****************************************************************************
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
 *****************************************************************************/

static void adc_startconv(FAR struct stm32_dev_s *priv, bool enable)
{
  uint32_t regval;

  ainfo("enable: %d\n", enable ? 1 : 0);

  regval = adc_getreg(priv, STM32L4_ADC_CR_OFFSET);
  if (enable)
    {
      /* Start conversion of regular channels */

      regval |= ADC_CR_ADSTART;
    }
  else
    {
      /* Disable the conversion of regular channels */

      regval |= ADC_CR_ADSTP;
    }

  adc_putreg(priv, STM32L4_ADC_CR_OFFSET, regval);
}

/*****************************************************************************
 * Name: adc_rccreset
 *
 * Description:
 *   Deinitializes the ADCx peripheral registers to their default
 *   reset values. It could set all the ADCs configured.
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *   reset - Condition, set or reset
 *
 * Returned Value:
 *
 *****************************************************************************/

static void adc_rccreset(FAR struct stm32_dev_s *priv, bool reset)
{
  irqstate_t flags;
  uint32_t regval;

  /* Pick the appropriate bit in the AHB2 reset register.
   * The STM32L4 is like STM32F2/F4: there is one common reset for all ADCs.
   *
   * First must disable interrupts because the AHB2RSTR register is used by
   * several different drivers.
   */

  flags = enter_critical_section();

  /* Set or clear the selected bit in the AHB2 reset register */

  regval = getreg32(STM32L4_RCC_AHB2RSTR);
  if (reset)
    {
      regval |= RCC_AHB2RSTR_ADCRST;
    }
  else
    {
      regval &= ~RCC_AHB2RSTR_ADCRST;
    }

  putreg32(regval, STM32L4_RCC_AHB2RSTR);
  leave_critical_section(flags);
}

/*****************************************************************************
 * Name: adc_enable
 *
 * Description:
 *   Enables the specified ADC peripheral.
 *
 * Input Parameters:
 *   priv - A reference to the ADC block status
 *
 * Returned Value:
 *
 *****************************************************************************/

static void adc_enable(FAR struct stm32_dev_s *priv)
{
  uint32_t regval;

  regval = adc_getreg(priv, STM32L4_ADC_CR_OFFSET);

  /* Exit deep power down mode and enable voltage regulator */

  regval &= ~ADC_CR_DEEPPWD;
  regval |= ADC_CR_ADVREGEN;
  adc_putreg(priv, STM32L4_ADC_CR_OFFSET, regval);

  /* Wait for voltage regulator to power up */

  up_udelay(20);

  /* Enable ADC calibration. ADCALDIF == 0 so this is only for
   * single-ended conversions, not for differential ones.
   */

  regval |= ADC_CR_ADCAL;
  adc_putreg(priv, STM32L4_ADC_CR_OFFSET, regval);

  /* Wait for calibration to complete */

  while (adc_getreg(priv, STM32L4_ADC_CR_OFFSET) & ADC_CR_ADCAL);

  /* Enable ADC
   * Note: ADEN bit cannot be set during ADCAL=1 and 4 ADC clock cycle
   * after the ADCAL bit is cleared by hardware. If we are using SYSCLK
   * as ADC clock source, this is the same as time taken to execute 4
   * ARM instructions.
   */

  regval  = adc_getreg(priv, STM32L4_ADC_CR_OFFSET);
  regval |= ADC_CR_ADEN;
  adc_putreg(priv, STM32L4_ADC_CR_OFFSET, regval);

  /* Wait for hardware to be ready for conversions */

  while (!(adc_getreg(priv, STM32L4_ADC_ISR_OFFSET) & ADC_INT_ADRDY));
}

/*****************************************************************************
 * Name: adc_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 *****************************************************************************/

static int adc_bind(FAR struct adc_dev_s *dev,
                    FAR const struct adc_callback_s *callback)
{
#ifndef CONFIG_STM32L4_ADC_NOIRQ
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
#endif

  return OK;
}

/*****************************************************************************
 * Name: adc_reset
 *
 * Description:
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before adc_setup() and on error conditions.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 *****************************************************************************/

static void adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  ainfo("intf: ADC%d\n", priv->intf);

  /* Enable ADC reset state */

  adc_rccreset(priv, true);

  /* Release ADC from reset state */

  adc_rccreset(priv, false);
}

/*****************************************************************************
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
 *****************************************************************************/

static int adc_setup(FAR struct adc_dev_s *dev)
{
#if !defined(CONFIG_STM32L4_ADC_NOIRQ) ||  defined(ADC_HAVE_TIMER) || \
    !defined(CONFIG_STM32L4_ADC_NO_STARTUP_CONV)
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
#endif
  int ret = OK;
  irqstate_t flags;
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  /* Attach the ADC interrupt */

#ifndef CONFIG_STM32L4_ADC_NOIRQ
  ret = irq_attach(priv->irq, priv->isr, NULL);
  if (ret < 0)
    {
      ainfo("irq_attach failed: %d\n", ret);
      return ret;
    }
#endif

  flags = enter_critical_section();

  /* Make sure that the ADC device is in the powered up, reset state. */

  adc_reset(dev);

  /* Initialize the same sample time for each ADC. */

  adc_sample_time_set(dev);

#ifdef HAVE_ADC_RESOLUTION
  /* Set the resolution of the conversion. */

  adc_resolution_set(dev, priv->resolution);
#endif

  /* Disable continuous mode and set align to right */

  clrbits |= ADC_CFGR_CONT | ADC_CFGR_ALIGN;

  /* Disable external trigger for regular channels */

  clrbits |= ADC_CFGR_EXTEN_MASK;
  setbits |= ADC_CFGR_EXTEN_NONE;

  /* Set CFGR configuration */

  adc_modifyreg(priv, STM32L4_ADC_CFGR_OFFSET, clrbits, setbits);

  /* Configuration of the channel conversions */

  adc_set_ch(dev, 0);

  /* ADC CCR configuration */

  clrbits = ADC_CCR_PRESC_MASK | ADC_CCR_VREFEN |
            ADC_CCR_TSEN | ADC_CCR_VBATEN;
  setbits = ADC_CCR_PRESC_NOT_DIV;

  /* On STM32L4X3 devices DAC1 and DAC2 outputs are multiplexed with ADC1 TS
   * and VBAT. adc_internal() knows about this and does not set TSEN or VBATEN
   * bits if configuration has requested DAC output to be connected to ADC.
   */

  adc_internal(priv, &setbits);

  stm32_modifyreg32(STM32L4_ADC_CCR, clrbits, setbits);

#ifdef ADC_HAVE_DMA
  if (priv->hasdma)
    {
      /* Configure ADC DMA */

      adc_dma_cfg(priv);

      /* Start ADC DMA */

      adc_dma_start(dev);
    }
#endif

#ifdef ADC_HAVE_EXTCFG
  /* Configure external event for regular group */

  if (priv->cchannels > 0)
    {
      adc_extsel_set(dev, priv->extcfg);
    }
  else
    {
      awarn("WARNING: External event for regular channels not configured.\n");
    }
#endif

  /* Set ADEN to wake up the ADC from Power Down. */

  adc_enable(priv);

#ifdef ADC_HAVE_TIMER
  if (priv->tbase != 0)
    {
      ret = adc_timinit(priv);
      if (ret < 0)
        {
          aerr("ERROR: adc_timinit failed: %d\n", ret);
        }
    }
#endif

  leave_critical_section(flags);

  /* Dump regs */

  adc_dumpregs(priv);

/* As default conversion is started here.
   *
   * NOTE: (J)ADSTART bit must be set to start ADC conversion
   *       even if hardware trigger is selected.
   *       This can be done here during the opening of the ADC device
   *       or later with ANIOC_TRIGGER ioctl call.
   */

#ifndef CONFIG_STM32L4_ADC_NO_STARTUP_CONV
  /* Start regular conversion */

  adc_startconv(priv, true);
#endif

  /* Enable the ADC interrupt */

#ifndef CONFIG_STM32L4_ADC_NOIRQ
  ainfo("Enable the ADC interrupt: irq=%d\n", priv->irq);
  up_enable_irq(priv->irq);
#endif

  return ret;
}

/*****************************************************************************
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
 *****************************************************************************/

static void adc_shutdown(FAR struct adc_dev_s *dev)
{
#ifndef CONFIG_STM32L4_ADC_NOIRQ
  /* Disable ADC interrupts and detach the ADC interrupt handler */

  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
#endif

  /* Disable and reset the ADC module */

  adc_reset(dev);
}

/*****************************************************************************
 * Name: adc_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 *****************************************************************************/

static void adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  uint32_t regval;

  ainfo("intf: %d enable: %d\n", priv->intf, enable ? 1 : 0);

  regval = adc_getreg(priv, STM32L4_ADC_IER_OFFSET);
  if (enable)
    {
      /* Enable end of conversion interrupt */

      regval |= ADC_INT_EOC;
    }
  else
    {
      /* Disable all interrupts */

      regval &= ~ADC_INT_MASK;
    }

  adc_putreg(priv, STM32L4_ADC_IER_OFFSET, regval);
}

/*****************************************************************************
 * Name: adc_resolution_set
 *****************************************************************************/

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

  adc_modifyreg(priv, STM32L4_ADC_CFGR_OFFSET, ADC_CFGR_RES_MASK,
                res << ADC_CFGR_RES_SHIFT);

errout:
  return ret;
}
#endif

/*****************************************************************************
 * Name: adc_sample_time_set
 *****************************************************************************/

static void adc_sample_time_set(FAR struct adc_dev_s *dev)
{
  /* Initialize the same sample time for each ADC.
   * During sample cycles channel selection bits must remain unchanged.
   */

  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  adc_putreg(priv, STM32L4_ADC_SMPR1_OFFSET, ADC_SMPR1_DEFAULT);
  adc_putreg(priv, STM32L4_ADC_SMPR2_OFFSET, ADC_SMPR2_DEFAULT);
}

/*****************************************************************************
 * Name: adc_extsel_set
 *****************************************************************************/

#ifdef ADC_HAVE_EXTCFG
static int adc_extsel_set(FAR struct adc_dev_s *dev, uint32_t extcfg)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
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
   * - The position with of the EXTSEL field varies from one STM32L4 MCU
   *   to another.
   * - The width of the EXTSEL field varies from one STM32L4 MCU to another.
   */

  if (exten > 0)
    {
      setbits = (extsel | exten);
      clrbits = (ADC_EXTREG_EXTEN_MASK | ADC_EXTREG_EXTSEL_MASK);

      ainfo("Initializing extsel = 0x%08x\n", extsel);

      /* Write register */

      adc_modifyreg(priv, STM32L4_ADC_CFGR_OFFSET, clrbits, setbits);
    }

  return OK;
}
#endif

/*****************************************************************************
 * Name: adc_dumpregs
 *****************************************************************************/

static void adc_dumpregs(FAR struct stm32_dev_s *priv)
{
  UNUSED(priv);

  ainfo("ISR:  0x%08x IER:  0x%08x CR:   0x%08x CFGR1: 0x%08x\n",
        adc_getreg(priv, STM32L4_ADC_ISR_OFFSET),
        adc_getreg(priv, STM32L4_ADC_IER_OFFSET),
        adc_getreg(priv, STM32L4_ADC_CR_OFFSET),
        adc_getreg(priv, STM32L4_ADC_CFGR_OFFSET));

  ainfo("SQR1: 0x%08x SQR2: 0x%08x SQR3: 0x%08x SQR4: 0x%08x\n",
        adc_getreg(priv, STM32L4_ADC_SQR1_OFFSET),
        adc_getreg(priv, STM32L4_ADC_SQR2_OFFSET),
        adc_getreg(priv, STM32L4_ADC_SQR3_OFFSET),
        adc_getreg(priv, STM32L4_ADC_SQR4_OFFSET));

  ainfo("SMPR1: 0x%08x SMPR2: 0x%08x\n",
        adc_getreg(priv, STM32L4_ADC_SMPR1_OFFSET),
        adc_getreg(priv, STM32L4_ADC_SMPR2_OFFSET));
}

/*****************************************************************************
 * Name: adc_sqrbits
 *****************************************************************************/

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

/*****************************************************************************
 * Name: adc_internal
 *****************************************************************************/

static bool adc_internal(FAR struct stm32_dev_s * priv, uint32_t *adc_ccr)
{
  int i;
  bool internal = false;

  if (priv->intf == 1 || priv->intf == 3)
    {
      for (i = 0; i < priv->nchannels; i++)
        {
          if (priv->chanlist[i] < ADC_EXTERNAL_CHAN_MIN || \
              priv->chanlist[i] > ADC_EXTERNAL_CHAN_MAX)
            {
              internal = true;
              switch (priv->chanlist[i])
                {
                  case 0:
                    if (priv->intf == 1)
                      {
                        *adc_ccr |= ADC_CCR_VREFEN;
                      }
                    break;

                  case 17:
#if !(defined(CONFIG_STM32L4_STM32L4X3) && defined(CONFIG_STM32L4_DAC1_OUTPUT_ADC))
                    *adc_ccr |= ADC_CCR_TSEN;
#endif
                    break;
                  case 18:
#if !(defined(CONFIG_STM32L4_STM32L4X3) && defined(CONFIG_STM32L4_DAC2_OUTPUT_ADC))
                    *adc_ccr |= ADC_CCR_VBATEN;
#endif
                    break;
                }
            }
        }
    }

  /* REVISIT: ADC2 or ADC3 DAC outputs not considered internal. */

  return internal;
}

/*****************************************************************************
 * Name: adc_offset_set
 *****************************************************************************/

#if defined(ADC_HAVE_DFSDM) || defined(CONFIG_STM32L4_ADC_LL_OPS)
static int adc_offset_set(FAR struct stm32_dev_s *priv, uint8_t ch, uint8_t i,
                          uint16_t offset)
{
  uint32_t regval = 0;
  uint32_t reg    = 0;
  int      ret    = OK;

  if (i >= 4)
    {
      /* There are only four offset registers. */

      ret = -E2BIG;
      goto errout;
    }

  reg = STM32L4_ADC_OFR1_OFFSET + i * 4;

  regval = ADC_OFR_OFFSETY_EN;
  adc_putreg(priv, reg, regval);

  regval |= ADC_OFR_OFFSETY_CH(ch) | ADC_OFR_OFFSETY(offset);
  adc_putreg(priv, reg, regval);

errout:
  return ret;
}
#endif

/*****************************************************************************
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
 *****************************************************************************/

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

  DEBUGASSERT(priv->nchannels <= ADC_MAX_SAMPLES);

  bits = adc_sqrbits(priv, ADC_SQR4_FIRST, ADC_SQR4_LAST, ADC_SQR4_SQ_OFFSET);
  adc_modifyreg(priv, STM32L4_ADC_SQR4_OFFSET, ~ADC_SQR4_RESERVED, bits);

  bits = adc_sqrbits(priv, ADC_SQR3_FIRST, ADC_SQR3_LAST, ADC_SQR3_SQ_OFFSET);
  adc_modifyreg(priv, STM32L4_ADC_SQR3_OFFSET, ~ADC_SQR3_RESERVED, bits);

  bits = adc_sqrbits(priv, ADC_SQR2_FIRST, ADC_SQR2_LAST, ADC_SQR2_SQ_OFFSET);
  adc_modifyreg(priv, STM32L4_ADC_SQR2_OFFSET, ~ADC_SQR2_RESERVED, bits);

  bits = ((uint32_t)priv->nchannels - 1) << ADC_SQR1_L_SHIFT |
         adc_sqrbits(priv, ADC_SQR1_FIRST, ADC_SQR1_LAST, ADC_SQR1_SQ_OFFSET);
  adc_modifyreg(priv, STM32L4_ADC_SQR1_OFFSET, ~ADC_SQR1_RESERVED, bits);

#ifdef ADC_HAVE_DFSDM
  if (priv->hasdfsdm)
    {
      /* Convert 12-bit ADC result to signed 16-bit. */

      if (ch == 0)
        {
          for (i = 0; i < priv->cchannels; i++)
            {
              adc_setoffset(priv, priv->chanlist[i], i, 0x800);
            }
        }
      else
        {
          adc_setoffset(priv, priv->current, 0, 0x800);
        }
    }
#endif

  return OK;
}

/*****************************************************************************
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
 *****************************************************************************/

static int adc_ioctl(FAR struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  uint32_t regval;
  uint32_t tmp;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        adc_startconv(priv, true);
        break;

      case ANIOC_WDOG_UPPER: /* Set watchdog upper threshold */
        {
          regval = adc_getreg(priv, STM32L4_ADC_TR1_OFFSET);

          /* Verify new upper threshold greater than lower threshold */

          tmp = (regval & ADC_TR1_LT_MASK) >> ADC_TR1_LT_SHIFT;
          if (arg < tmp)
            {
              ret = -EINVAL;
              break;
            }

          /* Set the watchdog threshold register */

          regval &= ~ADC_TR1_HT_MASK;
          regval |= ((arg << ADC_TR1_HT_SHIFT) & ADC_TR1_HT_MASK);
          adc_putreg(priv, STM32L4_ADC_TR1_OFFSET, regval);

          /* Ensure analog watchdog is enabled */

          adc_wdog_enable(priv);
        }
        break;

      case ANIOC_WDOG_LOWER: /* Set watchdog lower threshold */
        {
          regval = adc_getreg(priv, STM32L4_ADC_TR1_OFFSET);

          /* Verify new lower threshold less than upper threshold */

          tmp = (regval & ADC_TR1_HT_MASK) >> ADC_TR1_HT_SHIFT;
          if (arg > tmp)
            {
              ret = -EINVAL;
              break;
            }

          /* Set the watchdog threshold register */

          regval &= ~ADC_TR1_LT_MASK;
          regval |= ((arg << ADC_TR1_LT_SHIFT) & ADC_TR1_LT_MASK);
          adc_putreg(priv, STM32L4_ADC_TR1_OFFSET, regval);

          /* Ensure analog watchdog is enabled */

          adc_wdog_enable(priv);
        }
        break;

      default:
        aerr("ERROR: Unknown cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

#ifndef CONFIG_STM32L4_ADC_NOIRQ

/*****************************************************************************
 * Name: adc_interrupt
 *
 * Description:
 *   Common ADC interrupt handler.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 *****************************************************************************/

static int adc_interrupt(FAR struct adc_dev_s *dev, uint32_t adcisr)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int32_t value;

  /* Identifies the interruption AWD or OVR */

  if ((adcisr & ADC_INT_AWD1) != 0)
    {
      value  = adc_getreg(priv, STM32L4_ADC_DR_OFFSET);
      value &= ADC_DR_MASK;

      awarn("WARNING: Analog Watchdog, Value (0x%03x) out of range!\n",
            value);

      /* Stop ADC conversions to avoid continuous interrupts */

      adc_startconv(priv, false);
    }

  if ((adcisr & ADC_INT_OVR) != 0)
    {
      awarn("WARNING: Overrun has occurred!\n");
    }

  /* EOC: End of conversion */

  if ((adcisr & ADC_INT_EOC) != 0)
    {
      /* Read the converted value and clear EOC bit
       * (It is cleared by reading the ADC_DR)
       */

      value  = adc_getreg(priv, STM32L4_ADC_DR_OFFSET);
      value &= ADC_DR_MASK;

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
          priv->cb->au_receive(dev, priv->chanlist[priv->current], value);
        }

      /* Set the channel number of the next channel that will complete
       * conversion
       */

      priv->current++;

      if (priv->current >= priv->nchannels)
        {
          /* Restart the conversion sequence from the beginning */

          priv->current = 0;
        }
    }

  return OK;
}

/*****************************************************************************
 * Name: adc12_interrupt
 *
 * Description:
 *   ADC1/2 interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 *****************************************************************************/

#if defined(CONFIG_STM32L4_ADC1) || defined(CONFIG_STM32L4_ADC2)
static int adc12_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t regval;
  uint32_t pending;

#ifdef CONFIG_STM32L4_ADC1
  regval  = getreg32(STM32L4_ADC1_ISR);
  pending = regval & ADC_INT_MASK;
  if (pending != 0)
    {
      adc_interrupt(&g_adcdev1, regval);

      /* Clear interrupts */

      putreg32(regval, STM32L4_ADC1_ISR);
    }
#endif

#ifdef CONFIG_STM32L4_ADC2
  regval  = getreg32(STM32L4_ADC2_ISR);
  pending = regval & ADC_INT_MASK;
  if (pending != 0)
    {
      adc_interrupt(&g_adcdev2, regval);

      /* Clear interrupts */

      putreg32(regval, STM32L4_ADC2_ISR);
    }
#endif

  return OK;
}
#endif

/*****************************************************************************
 * Name: adc3_interrupt
 *
 * Description:
 *   ADC3 interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 *****************************************************************************/

#ifdef CONFIG_STM32L4_ADC3
static int adc3_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t regval;
  uint32_t pending;

  regval  = getreg32(STM32L4_ADC3_ISR);
  pending = regval & ADC_INT_MASK;
  if (pending != 0)
    {
      adc_interrupt(&g_adcdev3, regval);

      /* Clear interrupts */

      putreg32(regval, STM32L4_ADC3_ISR);
    }

  return OK;
}
#endif
#endif  /* CONFIG_STM32L4_ADC_NOIRQ */

#ifdef ADC_HAVE_DMA
/*****************************************************************************
 * Name: adc_dma_cfg
 *****************************************************************************/

static void adc_dma_cfg(FAR struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  /* Set DMA mode */

  if (priv->dmacfg == 0)
    {
      /* One Shot Mode */

      clrbits |= ADC_CFGR_DMACFG;
    }
  else
    {
      /* Circular Mode */

      setbits |= ADC_CFGR_DMACFG;
    }

  /* Enable DMA */

  setbits |= ADC_CFGR_DMAEN;

#ifdef ADC_HAVE_DFSDM
  if (priv->hasdfsdm)
    {
      /* Disable DMA */

      clrbits |= ADC_CFGR_DMAEN;

      /* Enable routing to DFSDM */

      setbits |= ADC_CFGR_DFSDMCFG;
    }
#endif

  /* Modify CFGR configuration */

  adc_modifyreg(priv, STM32L4_ADC_CFGR_OFFSET, clrbits, setbits);
}

/*****************************************************************************
 * Name: adc_dma_start
 *****************************************************************************/

static void adc_dma_start(FAR struct adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;

  /* Stop and free DMA if it was started before */

  if (priv->dma != NULL)
    {
      stm32l4_dmastop(priv->dma);
      stm32l4_dmafree(priv->dma);
    }

  priv->dma = stm32l4_dmachannel(priv->dmachan);

#ifndef CONFIG_STM32L4_ADC_NOIRQ
  stm32l4_dmasetup(priv->dma,
                   priv->base + STM32L4_ADC_DR_OFFSET,
                   (uint32_t)priv->dmabuffer,
                   priv->nchannels,
                   ADC_DMA_CONTROL_WORD);

  stm32l4_dmastart(priv->dma, adc_dmaconvcallback, dev, false);
#endif
}

/*****************************************************************************
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
 *****************************************************************************/

#ifndef CONFIG_STM32L4_ADC_NOIRQ
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
          priv->cb->au_receive(dev, priv->chanlist[priv->current],
                               priv->dmabuffer[priv->current]);
          priv->current++;
          if (priv->current >= priv->nchannels)
            {
              /* Restart the conversion sequence from the beginning */

              priv->current = 0;
            }
        }
    }

  /* Restart DMA for the next conversion series */

  adc_modifyreg(priv, STM32L4_ADC_CFGR_OFFSET, ADC_CFGR_DMAEN, 0);
  adc_modifyreg(priv, STM32L4_ADC_CFGR_OFFSET, 0, ADC_CFGR_DMAEN);
}
#endif
#endif  /* ADC_HAVE_DMA */

#ifdef CONFIG_STM32L4_ADC_LL_OPS

/*****************************************************************************
 * Name: adc_llops_intack
 *****************************************************************************/

static void adc_llops_intack(FAR struct stm32_adc_dev_s *dev, uint32_t source)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  /* Clear pending interrupts */

  adc_putreg(priv, STM32L4_ADC_ISR_OFFSET, (source & ADC_ISR_ALLINTS));
}

/*****************************************************************************
 * Name: adc_llops_inten
 *****************************************************************************/

static void adc_llops_inten(FAR struct stm32_adc_dev_s *dev, uint32_t source)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  /* Enable interrupts */

  adc_modifyreg(priv, STM32L4_ADC_IER_OFFSET, 0, (source & ADC_IER_ALLINTS));
}

/*****************************************************************************
 * Name: adc_llops_intdis
 *****************************************************************************/

static void adc_llops_intdis(FAR struct stm32_adc_dev_s *dev, uint32_t source)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  /* Disable interrupts */

  adc_modifyreg(priv, STM32L4_ADC_IER_OFFSET, (source & ADC_IER_ALLINTS), 0);
}

/*****************************************************************************
 * Name: adc_llops_intget
 *****************************************************************************/

static uint32_t adc_llops_intget(FAR struct stm32_adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;
  uint32_t regval;
  uint32_t pending;

  regval  = adc_getreg(priv, STM32L4_ADC_ISR_OFFSET);
  pending = regval & ADC_ISR_ALLINTS;

  return pending;
}

/*****************************************************************************
 * Name: adc_llops_regget
 *****************************************************************************/

static uint32_t adc_llops_regget(FAR struct stm32_adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  return adc_getreg(priv, STM32L4_ADC_DR_OFFSET) & ADC_DR_MASK;
}

/*****************************************************************************
 * Name: adc_llops_startconv
 *****************************************************************************/

static void adc_llops_startconv(FAR struct stm32_adc_dev_s *dev, bool enable)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  adc_startconv(priv, enable);
}

/*****************************************************************************
 * Name: adc_llops_offset_set
 *****************************************************************************/

static int  adc_llops_offset_set(FAR struct stm32_adc_dev_s *dev, uint8_t ch,
                                 uint8_t i, uint16_t offset)
{
  int ret;

  ret = adc_offset_set((FAR struct stm32_dev_s *)dev, ch, i, offset);

  return ret;
}

/*****************************************************************************
 * Name: adc_regbufregister
 *****************************************************************************/

#ifdef ADC_HAVE_DMA
static int adc_regbufregister(FAR struct stm32_adc_dev_s *dev,
                              uint16_t *buffer, uint8_t len)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  stm32l4_dmasetup(priv->dma,
                 priv->base + STM32L4_ADC_DR_OFFSET,
                 (uint32_t)buffer,
                 len,
                 ADC_DMA_CONTROL_WORD);

  /* No DMA callback */

  stm32l4_dmastart(priv->dma, NULL, dev, false);

  return OK;
}
#endif  /* ADC_HAVE_DMA */

/*****************************************************************************
 * Name: adc_llops_dumpregs
 *****************************************************************************/

static void adc_llops_dumpregs(FAR struct stm32_adc_dev_s *dev)
{
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev;

  adc_dumpregs(priv);
}

#endif  /* CONFIG_STM32L4_ADC_LL_OPS */

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: stm32l4_adc_initialize
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
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 *****************************************************************************/

struct adc_dev_s *stm32l4_adc_initialize(int intf, FAR
                                const uint8_t *chanlist, int cchannels)
{
  FAR struct adc_dev_s   *dev;
  FAR struct stm32_dev_s *priv;

  ainfo("intf: %d cchannels: %d\n", intf, cchannels);

  switch (intf)
    {
#ifdef CONFIG_STM32L4_ADC1
      case 1:
        ainfo("ADC1 selected\n");
        dev = &g_adcdev1;
        break;
#endif
#ifdef CONFIG_STM32L4_ADC2
      case 2:
        ainfo("ADC2 selected\n");
        dev = &g_adcdev2;
        break;
#endif
#ifdef CONFIG_STM32L4_ADC3
      case 3:
        ainfo("ADC3 selected\n");
        dev = &g_adcdev3;
        break;
#endif
      default:
        aerr("ERROR: No ADC interface defined\n");
        return NULL;
    }

  /* Configure the selected ADC */

  priv = (FAR struct stm32_dev_s *)dev->ad_priv;

#ifndef CONFIG_STM32L4_ADC_NOIRQ
  priv->cb = NULL;
#endif

  DEBUGASSERT(cchannels <= ADC_MAX_SAMPLES);
  if (cchannels > ADC_MAX_SAMPLES)
    {
      cchannels = ADC_MAX_SAMPLES;
    }

  priv->cchannels = cchannels;
  memcpy(priv->chanlist, chanlist, cchannels);

#ifdef CONFIG_PM
  if (pm_register(&priv->pm_callback) != OK)
    {
      aerr("Power management registration failed\n");
      return NULL;
    }
#endif

  return dev;
}

#endif /* CONFIG_STM32L4_ADC1 || CONFIG_STM32L4_ADC2 || CONFIG_STM32L4_ADC3 */
#endif /* CONFIG_ADC */