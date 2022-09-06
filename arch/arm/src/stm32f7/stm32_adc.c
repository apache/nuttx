/****************************************************************************
 * arch/arm/src/stm32f7/stm32_adc.c
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

#include <sys/types.h>
#include <sys/ioctl.h>

#include <inttypes.h>
#include <stdio.h>
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
#include <nuttx/fs/ioctl.h>
#include <nuttx/power/pm.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/semaphore.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32_rcc.h"
#include "stm32_tim.h"
#include "stm32_dma.h"
#include "stm32_adc.h"

/* Based on arch/src/stm32/stm32_adc.c */

/* STM32 ADC "lower-half" support must be enabled */

#ifdef CONFIG_STM32F7_ADC

/* This implementation is for the STM32 ADC IP version 1 */

/* Supported ADC modes:
 *   - SW triggering with/without DMA transfer
 *   - TIM triggering with/without DMA transfer
 *   - external triggering with/without DMA transfer
 *
 * (tested with ADC example app from NuttX apps repo).
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RCC reset ****************************************************************/

#define STM32_RCC_RSTR     STM32_RCC_APB2RSTR
#define RCC_RSTR_ADC123RST RCC_APB2RSTR_ADCRST

/* ADC Channels/DMA *********************************************************/

#define ADC_DMA_CONTROL_WORD (DMA_SCR_MSIZE_16BITS |  \
                              DMA_SCR_PSIZE_16BITS |  \
                              DMA_SCR_MINC |          \
                              DMA_SCR_CIRC |          \
                              DMA_SCR_DIR_P2M)

/* Sample time default configuration */

#define ADC_SMPR_DEFAULT    ADC_SMPR_112
#define ADC_SMPR1_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR1_SMP10_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP11_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP12_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP13_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP14_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP15_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP16_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP17_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR1_SMP18_SHIFT))
#define ADC_SMPR2_DEFAULT   ((ADC_SMPR_DEFAULT << ADC_SMPR2_SMP0_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP1_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP2_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP3_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP4_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP5_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP6_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP7_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP8_SHIFT) | \
                               (ADC_SMPR_DEFAULT << ADC_SMPR2_SMP9_SHIFT))

/* Number of channels per ADC */

#define ADC_CHANNELS_NUMBER 19

/* Max 4 injected channels */

#define ADC_INJ_MAX_SAMPLES   4

/* ADC scan mode support */

#ifndef CONFIG_STM32F7_ADC1_SCAN
#  define CONFIG_STM32F7_ADC1_SCAN 0
#endif
#ifndef CONFIG_STM32F7_ADC2_SCAN
#  define CONFIG_STM32F7_ADC2_SCAN 0
#endif
#ifndef CONFIG_STM32F7_ADC3_SCAN
#  define CONFIG_STM32F7_ADC3_SCAN 0
#endif

/* We have to support ADC callbacks if default ADC interrupts or
 * DMA transfer are enabled
 */

#if !defined(CONFIG_STM32F7_ADC_NOIRQ) || defined(ADC_HAVE_DMA)
#  define ADC_HAVE_CB
#else
#  undef ADC_HAVE_CB
#endif

/* ADC software trigger configuration */

#define ANIOC_TRIGGER_REGULAR  (1 << 0)
#define ANIOC_TRIGGER_INJECTED (1 << 1)

/* The last external channel on ADC 1 to enable Reading Vref or
 * Vbat / Vsence
 */

#define ADC_LAST_EXTERNAL_CHAN 15

/* Assuming VDC 2.4 - 3.6 */

#define ADC_MAX_FADC 36000000

#if STM32_PCLK2_FREQUENCY/2 <= ADC_MAX_FADC
# define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV2
#elif STM32_PCLK2_FREQUENCY/4 <= ADC_MAX_FADC
# define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV4
#elif STM32_PCLK2_FREQUENCY/6 <= ADC_MAX_FADC
# define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV6
#elif STM32_PCLK2_FREQUENCY/8 <= ADC_MAX_FADC
# define ADC_CCR_ADCPRE_DIV     ADC_CCR_ADCPRE_DIV8
#else
# error "PCLK2 too high - no divisor found "
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Data common to all ADC instances */

struct adccmn_data_s
{
  uint8_t refcount; /* How many ADC instances are currently in use */
  mutex_t lock;     /* Exclusive access to common ADC data */
};

/* This structure describes the state of one ADC block */

struct stm32_dev_s
{
#ifdef CONFIG_STM32F7_ADC_LL_OPS
  const struct stm32_adc_ops_s *llops; /* Low-level ADC ops */
  struct adc_dev_s             *dev;   /* Upper-half ADC reference */
#endif
#ifdef ADC_HAVE_CB
  const struct adc_callback_s *cb;
  uint8_t irq;               /* Interrupt generated by this ADC block */
#endif
  struct adccmn_data_s *cmn; /* Common ADC data */
  uint8_t rnchannels;        /* Number of regular channels */
  uint8_t cr_channels;       /* Number of configured regular channels */
#ifdef ADC_HAVE_INJECTED
  uint8_t cj_channels;       /* Number of configured injected channels */
#endif
  uint8_t intf;              /* ADC interface number */
  uint8_t initialized;       /* ADC interface initialization counter */
  uint8_t current;           /* Current ADC channel being converted */
  uint8_t anioc_trg;         /* ANIOC_TRIGGER configuration */
  uint8_t resolution;        /* ADC resolution (0-3) */
#ifdef ADC_HAVE_DMA
  uint8_t dmachan;           /* DMA channel needed by this ADC */
  uint8_t dmacfg;            /* DMA channel configuration, only for ADC IPv2 */
  bool    hasdma;            /* True: This channel supports DMA */
#endif
  bool    scan;              /* True: Scan mode */
#ifdef CONFIG_STM32F7_ADC_CHANGE_SAMPLETIME
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

#ifdef CONFIG_PM
  struct pm_callback_s pm_callback;
#endif

#ifdef ADC_HAVE_DMA
  DMA_HANDLE dma;            /* Allocated DMA channel */

  /* DMA transfer buffer */

  uint16_t r_dmabuffer[CONFIG_STM32F7_ADC_MAX_SAMPLES];
#endif

  /* List of selected ADC channels to sample */

  uint8_t  r_chanlist[CONFIG_STM32F7_ADC_MAX_SAMPLES];

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
static uint32_t adc_getreg(struct stm32_dev_s *priv, int offset);
static void adc_putreg(struct stm32_dev_s *priv, int offset,
                       uint32_t value);
static void adc_modifyreg(struct stm32_dev_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits);
static void adccmn_modifyreg(struct stm32_dev_s *priv, uint32_t offset,
                             uint32_t clrbits, uint32_t setbits);
static uint32_t adccmn_getreg(struct stm32_dev_s *priv, uint32_t offset);

#ifdef ADC_HAVE_TIMER
static uint16_t tim_getreg(struct stm32_dev_s *priv, int offset);
static void tim_putreg(struct stm32_dev_s *priv, int offset,
                       uint16_t value);
static void tim_modifyreg(struct stm32_dev_s *priv, int offset,
                          uint16_t clrbits, uint16_t setbits);
static void tim_dumpregs(struct stm32_dev_s *priv, const char *msg);
#endif

static void adc_rccreset(struct stm32_dev_s *priv, bool reset);

/* ADC Interrupt Handler */

#ifndef CONFIG_STM32F7_ADC_NOIRQ
static int adc_interrupt(struct adc_dev_s *dev);
static int adc123_interrupt(int irq, void *context, void *arg);
#endif /* CONFIG_STM32F7_ADC_NOIRQ */

/* ADC Driver Methods */

static int  adc_bind(struct adc_dev_s *dev,
                     const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int  adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);
static void adc_enable(struct stm32_dev_s *priv, bool enable);

static uint32_t adc_sqrbits(struct stm32_dev_s *priv, int first,
                            int last, int offset);
static int  adc_set_ch(struct adc_dev_s *dev, uint8_t ch);
static bool adc_internal(struct stm32_dev_s *priv);

static int  adc_resolution_set(struct adc_dev_s *dev, uint8_t res);

#ifdef ADC_HAVE_TIMER
static void adc_timstart(struct stm32_dev_s *priv, bool enable);
static int  adc_timinit(struct stm32_dev_s *priv);
#endif

#if defined(ADC_HAVE_DMA) && !defined(CONFIG_STM32F7_ADC_NOIRQ)
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                void *arg);
#endif

#ifdef CONFIG_PM
static int adc_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e state);
#endif

static void adc_reg_startconv(struct stm32_dev_s *priv, bool enable);
#ifdef ADC_HAVE_INJECTED
static void adc_inj_startconv(struct stm32_dev_s *priv, bool enable);
static int adc_inj_set_ch(struct adc_dev_s *dev, uint8_t ch);
#endif

#ifdef ADC_HAVE_EXTCFG
static int adc_extcfg_set(struct stm32_dev_s *priv, uint32_t extcfg);
#endif
#ifdef ADC_HAVE_JEXTCFG
static int adc_jextcfg_set(struct stm32_dev_s *priv, uint32_t jextcfg);
#endif

static void adc_dumpregs(struct stm32_dev_s *priv);

#ifdef CONFIG_STM32F7_ADC_LL_OPS
static int adc_llops_setup(struct stm32_adc_dev_s *dev);
static void adc_llops_shutdown(struct stm32_adc_dev_s *dev);
static void adc_intack(struct stm32_adc_dev_s *dev, uint32_t source);
static void adc_inten(struct stm32_adc_dev_s *dev, uint32_t source);
static void adc_intdis(struct stm32_adc_dev_s *dev, uint32_t source);
static uint32_t adc_intget(struct stm32_adc_dev_s *dev);
static uint32_t adc_regget(struct stm32_adc_dev_s *dev);
static void adc_llops_reg_startconv(struct stm32_adc_dev_s *dev,
                                    bool enable);
static int adc_offset_set(struct stm32_adc_dev_s *dev, uint8_t ch,
                          uint8_t i, uint16_t offset);
#  ifdef ADC_HAVE_EXTCFG
static void adc_llops_extcfg_set(struct stm32_adc_dev_s *dev,
                                 uint32_t extcfg);
#  endif
#  ifdef ADC_HAVE_JEXTCFG
static void adc_llops_jextcfg_set(struct stm32_adc_dev_s *dev,
                                  uint32_t jextcfg);
#  endif
#  ifdef ADC_HAVE_DMA
static int adc_regbufregister(struct stm32_adc_dev_s *dev,
                              uint16_t *buffer, uint8_t len);
#  endif
#  ifdef ADC_HAVE_INJECTED
static uint32_t adc_injget(struct stm32_adc_dev_s *dev, uint8_t chan);
static void adc_llops_inj_startconv(struct stm32_adc_dev_s *dev,
                                    bool enable);
#  endif
#  ifdef CONFIG_STM32F7_ADC_CHANGE_SAMPLETIME
static void adc_sampletime_set(struct stm32_adc_dev_s *dev,
                               struct adc_sample_time_s *time_samples);
static void adc_sampletime_write(struct stm32_adc_dev_s *dev);
#  endif
static void adc_llops_dumpregs(struct stm32_adc_dev_s *dev);
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

#ifdef CONFIG_STM32F7_ADC_LL_OPS
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
#  ifdef CONFIG_STM32F7_ADC_CHANGE_SAMPLETIME
  .stime_set     = adc_sampletime_set,
  .stime_write   = adc_sampletime_write,
#  endif
  .dump_regs     = adc_llops_dumpregs
};
#endif

/* ADC instances are coupled in blocks */

#define ADC1CMN_DATA g_adc123_cmn
#define ADC2CMN_DATA g_adc123_cmn
#define ADC3CMN_DATA g_adc123_cmn

/* ADC123 common data */

struct adccmn_data_s g_adc123_cmn =
{
  .refcount = 0,
  .lock = NXMUTEX_INITIALIZER,
};

/* ADC1 state */

#ifdef CONFIG_STM32F7_ADC1
static struct stm32_dev_s g_adcpriv1 =
{
#ifdef CONFIG_STM32F7_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
#ifndef CONFIG_STM32F7_ADC_NOIRQ
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
#endif /* CONFIG_STM32F7_ADC_NOIRQ */
  .cmn         = &ADC1CMN_DATA,
  .intf        = 1,
  .initialized = 0,
  .anioc_trg   = CONFIG_STM32F7_ADC1_ANIOC_TRIGGER,
  .resolution  = CONFIG_STM32F7_ADC1_RESOLUTION,
  .base        = STM32_ADC1_BASE,
#ifdef ADC1_HAVE_EXTCFG
  .extcfg      = ADC1_EXTCFG_VALUE,
#endif
#ifdef ADC1_HAVE_JEXTCFG
  .jextcfg     = ADC1_JEXTCFG_VALUE,
#endif
#ifdef ADC1_HAVE_TIMER
  .trigger     = CONFIG_STM32F7_ADC1_TIMTRIG,
  .tbase       = ADC1_TIMER_BASE,
  .pclck       = ADC1_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32F7_ADC1_SAMPLE_FREQUENCY,
#endif
#ifdef ADC1_HAVE_DMA
  .dmachan     = ADC1_DMA_CHAN,
  .dmacfg      = CONFIG_STM32F7_ADC1_DMA_CFG,
  .hasdma      = true,
#endif
  .scan        = CONFIG_STM32F7_ADC1_SCAN,
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

#ifdef CONFIG_STM32F7_ADC2
static struct stm32_dev_s g_adcpriv2 =
{
#ifdef CONFIG_STM32F7_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
#ifndef CONFIG_STM32F7_ADC_NOIRQ
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
#endif /* CONFIG_STM32F7_ADC_NOIRQ */
  .cmn         = &ADC2CMN_DATA,
  .intf        = 2,
  .initialized = 0,
  .anioc_trg   = CONFIG_STM32F7_ADC2_ANIOC_TRIGGER,
  .resolution  = CONFIG_STM32F7_ADC2_RESOLUTION,
  .base        = STM32_ADC2_BASE,
#ifdef ADC2_HAVE_EXTCFG
  .extcfg      = ADC2_EXTCFG_VALUE,
#endif
#ifdef ADC2_HAVE_JEXTCFG
  .jextcfg     = ADC2_JEXTCFG_VALUE,
#endif
#ifdef ADC2_HAVE_TIMER
  .trigger     = CONFIG_STM32F7_ADC2_TIMTRIG,
  .tbase       = ADC2_TIMER_BASE,
  .pclck       = ADC2_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32F7_ADC2_SAMPLE_FREQUENCY,
#endif
#ifdef ADC2_HAVE_DMA
  .dmachan     = ADC2_DMA_CHAN,
  .dmacfg      = CONFIG_STM32F7_ADC2_DMA_CFG,
  .hasdma      = true,
#endif
  .scan        = CONFIG_STM32F7_ADC2_SCAN,
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

#ifdef CONFIG_STM32F7_ADC3
static struct stm32_dev_s g_adcpriv3 =
{
#ifdef CONFIG_STM32F7_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
#ifndef CONFIG_STM32F7_ADC_NOIRQ
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
#endif /* CONFIG_STM32F7_ADC_NOIRQ */
  .cmn         = &ADC3CMN_DATA,
  .intf        = 3,
  .initialized = 0,
  .anioc_trg   = CONFIG_STM32F7_ADC3_ANIOC_TRIGGER,
  .resolution  = CONFIG_STM32F7_ADC3_RESOLUTION,
  .base        = STM32_ADC3_BASE,
#ifdef ADC3_HAVE_EXTCFG
  .extcfg      = ADC3_EXTCFG_VALUE,
#endif
#ifdef ADC3_HAVE_JEXTCFG
  .jextcfg     = ADC3_JEXTCFG_VALUE,
#endif
#ifdef ADC3_HAVE_TIMER
  .trigger     = CONFIG_STM32F7_ADC3_TIMTRIG,
  .tbase       = ADC3_TIMER_BASE,
  .pclck       = ADC3_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32F7_ADC3_SAMPLE_FREQUENCY,
#endif
#ifdef ADC3_HAVE_DMA
  .dmachan     = ADC3_DMA_CHAN,
  .dmacfg      = CONFIG_STM32F7_ADC3_DMA_CFG,
  .hasdma      = true,
#endif
  .scan        = CONFIG_STM32F7_ADC3_SCAN,
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

static uint32_t adc_getreg(struct stm32_dev_s *priv, int offset)
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

static void adc_putreg(struct stm32_dev_s *priv, int offset,
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

static void adc_modifyreg(struct stm32_dev_s *priv, int offset,
                          uint32_t clrbits, uint32_t setbits)
{
  adc_putreg(priv, offset, (adc_getreg(priv, offset) & ~clrbits) | setbits);
}

/****************************************************************************
 * Name: adccmn_modifyreg
 ****************************************************************************/

static void adccmn_modifyreg(struct stm32_dev_s *priv, uint32_t offset,
                             uint32_t clrbits, uint32_t setbits)
{
  /* Modify register */

  stm32_modifyreg32(offset + STM32_ADCCMN_BASE, clrbits, setbits);
}

/****************************************************************************
 * Name: adccmn_getreg
 ****************************************************************************/

static uint32_t adccmn_getreg(struct stm32_dev_s *priv, uint32_t offset)
{
  /* Return register value */

  return getreg32(offset + STM32_ADCCMN_BASE);
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
static uint16_t tim_getreg(struct stm32_dev_s *priv, int offset)
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
static void tim_putreg(struct stm32_dev_s *priv, int offset,
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
static void tim_modifyreg(struct stm32_dev_s *priv, int offset,
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
static void tim_dumpregs(struct stm32_dev_s *priv, const char *msg)
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
static void adc_timstart(struct stm32_dev_s *priv, bool enable)
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
static int adc_timinit(struct stm32_dev_s *priv)
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
   * So (prescaler = pclck / 65535 / freq) would be optimal.
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

  if (priv->tbase == STM32_TIM1_BASE || priv->tbase == STM32_TIM8_BASE)
    {
      tim_putreg(priv, STM32_ATIM_RCR_OFFSET, 0);
      tim_putreg(priv, STM32_ATIM_BDTR_OFFSET, ATIM_BDTR_MOE); /* Check me */
    }

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

  if (priv->tbase == STM32_TIM1_BASE || priv->tbase == STM32_TIM8_BASE)
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

  /* Reset the output compare and output compare N IDLE State */

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
 * Name: adc_pm_prepare
 *
 * Description:
 *   Called by power management framework when it wants to enter low power
 *   states. Check if ADC is in progress and if so prevent from entering
 *   STOP.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int adc_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e state)
{
  struct stm32_dev_s *priv =
      (struct stm32_dev_s *)((char *)cb -
                             offsetof(struct stm32_dev_s, pm_callback));
  uint32_t regval;

  regval = adc_getreg(priv, STM32_ADC_CR2_OFFSET);
  if ((state >= PM_IDLE) && (regval & ADC_CR2_SWSTART))
    {
      return -EBUSY;
    }

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

static void adc_reg_startconv(struct stm32_dev_s *priv, bool enable)
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

static void adc_inj_startconv(struct stm32_dev_s *priv, bool enable)
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

#endif /* ADC_HAVE_INJECTED */

/****************************************************************************
 * Name: adc_rccreset
 *
 * Description:
 *   Deinitializes the ADCx peripheral registers to their default
 *   reset values. It could set all the ADCs configured.
 *
 * Input Parameters:
 *   priv  - A reference to the ADC block status
 *   reset - Condition, set or reset
 *
 * Returned Value:
 *
 ****************************************************************************/

static void adc_rccreset(struct stm32_dev_s *priv, bool reset)
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

static void adc_enable(struct stm32_dev_s *priv, bool enable)
{
  ainfo("enable: %d\n", enable ? 1 : 0);

  if (enable)
    {
      adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, 0, ADC_CR2_ADON);
    }
  else
    {
      adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, ADC_CR2_ADON, 0);
    }
}

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

#if defined(ADC_HAVE_DMA) && !defined(CONFIG_STM32F7_ADC_NOIRQ)
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                void *arg)
{
  struct adc_dev_s   *dev  = (struct adc_dev_s *)arg;
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int i;

  up_invalidate_dcache((uintptr_t)priv->r_dmabuffer,
                       (uintptr_t)priv->r_dmabuffer +
                       sizeof(priv->r_dmabuffer));

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

static int adc_bind(struct adc_dev_s *dev,
                    const struct adc_callback_s *callback)
{
#ifdef ADC_HAVE_CB
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

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

static void adc_watchdog_cfg(struct stm32_dev_s *priv)
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

/****************************************************************************
 * Name: adc_mode_cfg
 ****************************************************************************/

static void adc_mode_cfg(struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  if (priv->scan)
    {
      setbits |= ADC_CR1_SCAN;
    }

  /* Set CR1 configuration */

  adc_modifyreg(priv, STM32_ADC_CR1_OFFSET, clrbits, setbits);

  /* Disable continuous mode and set align to right */

  clrbits = ADC_CR2_CONT | ADC_CR2_ALIGN;
  setbits = 0;

  /* Disable external trigger for regular channels */

  clrbits |= ADC_EXTREG_EXTEN_MASK;
  setbits |= ADC_EXTREG_EXTEN_NONE;

  /* Set CR2 configuration */

  adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, clrbits, setbits);
}

/****************************************************************************
 * Name: adc_sampletime_cfg
 ****************************************************************************/

static void adc_sampletime_cfg(struct adc_dev_s *dev)
{
  /* Initialize the same sample time for each ADC.
   * During sample cycles channel selection bits must remain unchanged.
   */

#ifdef CONFIG_STM32F7_ADC_CHANGE_SAMPLETIME
  adc_sampletime_write((struct stm32_adc_dev_s *)dev->ad_priv);
#else
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  adc_putreg(priv, STM32_ADC_SMPR1_OFFSET, ADC_SMPR1_DEFAULT);
  adc_putreg(priv, STM32_ADC_SMPR2_OFFSET, ADC_SMPR2_DEFAULT);
#endif
}

/****************************************************************************
 * Name: adc_common_cfg
 ****************************************************************************/

static void adc_common_cfg(struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  clrbits  = ADC_CCR_ADCPRE_MASK;
  setbits  = ADC_CCR_ADCPRE_DIV;

  if (adc_internal(priv))
    {
      setbits |= ADC_CCR_TSVREFE;
    }

  clrbits |= ADC_CCR_MULTI_MASK | ADC_CCR_DELAY_MASK | ADC_CCR_DDS |
             ADC_CCR_DMA_MASK | ADC_CCR_VBATE;
  setbits |= ADC_CCR_MULTI_NONE | ADC_CCR_DMA_DISABLED;

  adccmn_modifyreg(priv, STM32_ADC_CCR_OFFSET, clrbits, setbits);
}

#ifdef ADC_HAVE_DMA
/****************************************************************************
 * Name: adc_dma_cfg
 ****************************************************************************/

static void adc_dma_cfg(struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

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

  /* Enable DMA */

  setbits |= ADC_CR2_DMA;

  /* Modify CR2 configuration */

  adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, clrbits, setbits);
}

/****************************************************************************
 * Name: adc_dma_start
 ****************************************************************************/

static void adc_dma_start(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  /* Stop and free DMA if it was started before */

  if (priv->dma != NULL)
    {
      stm32_dmastop(priv->dma);
      stm32_dmafree(priv->dma);
    }

  priv->dma = stm32_dmachannel(priv->dmachan);

#ifndef CONFIG_STM32F7_ADC_NOIRQ
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

static void adc_configure(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  /* Turn off the ADC before configuration */

  adc_enable(priv, false);

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

  /* Configure ADC resolution */

  adc_resolution_set(dev, priv->resolution);

#ifdef ADC_HAVE_EXTCFG
  /* Configure external event for regular group */

  adc_extcfg_set(priv, priv->extcfg);
#endif

  /* Enable ADC */

  adc_enable(priv, true);

#ifdef ADC_HAVE_JEXTCFG
  /* Configure external event for injected group when ADC enabled */

  adc_jextcfg_set(priv, priv->jextcfg);
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

static void adc_reset(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  irqstate_t flags;

  ainfo("intf: %d\n", priv->intf);
  flags = enter_critical_section();

  /* Do nothing if ADC instance is currently in use */

  if (priv->initialized > 0)
    {
      goto out;
    }

  /* Only if this is the first initialzied ADC instance in the ADC block */

  if (nxmutex_lock(&priv->cmn->lock) < 0)
    {
      goto out;
    }

  if (priv->cmn->refcount == 0)
    {
      /* Enable ADC reset state */

      adc_rccreset(priv, true);

      /* Release ADC from reset state */

      adc_rccreset(priv, false);
    }

  nxmutex_unlock(&priv->cmn->lock);

out:
  leave_critical_section(flags);
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

static int adc_setup(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int ret = OK;

  /* Do nothing when the ADC device is already set up */

  if (priv->initialized > 0)
    {
      return OK;
    }

  /* Attach the ADC interrupt */

#ifndef CONFIG_STM32F7_ADC_NOIRQ
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

  /* As default conversion is started here */

#ifndef CONFIG_STM32F7_ADC_NO_STARTUP_CONV
  /* Start regular conversion */

  adc_reg_startconv(priv, true);

#  ifdef ADC_HAVE_INJECTED
  /* Start injected conversion */

  adc_inj_startconv(priv, true);
#  endif
#endif

  /* Increase instances counter */

  ret = nxmutex_lock(&priv->cmn->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->cmn->refcount == 0)
    {
      /* Enable the ADC interrupt */

#ifndef CONFIG_STM32F7_ADC_NOIRQ
      ainfo("Enable the ADC interrupt: irq=%d\n", priv->irq);
      up_enable_irq(priv->irq);
#endif
    }

  priv->cmn->refcount += 1;
  nxmutex_unlock(&priv->cmn->lock);

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

static void adc_shutdown(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

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

  if (nxmutex_lock(&priv->cmn->lock) < 0)
    {
      return;
    }

  if (priv->cmn->refcount <= 1)
    {
#ifndef CONFIG_STM32F7_ADC_NOIRQ
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

  /* Decrease instances counter */

  if (priv->cmn->refcount > 0)
    {
      priv->cmn->refcount -= 1;
    }

  nxmutex_unlock(&priv->cmn->lock);
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

static void adc_rxint(struct adc_dev_s *dev, bool enable)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
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
 * Name: adc_resolution_set
 ****************************************************************************/

static int adc_resolution_set(struct adc_dev_s *dev, uint8_t res)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int ret = OK;

  /* Check input */

  if (res > 3)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Modify appropriate register */

  adc_modifyreg(priv, STM32_ADC_CR1_OFFSET, ADC_CR1_RES_MASK,
                res << ADC_CR1_RES_SHIFT);

errout:
  return ret;
}

/****************************************************************************
 * Name: adc_extcfg_set
 ****************************************************************************/

#ifdef ADC_HAVE_EXTCFG
static int adc_extcfg_set(struct stm32_dev_s *priv, uint32_t extcfg)
{
  uint32_t exten  = 0;
  uint32_t extsel = 0;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  /* Get EXTEN and EXTSEL from input */

  exten = extcfg & ADC_EXTREG_EXTEN_MASK;
  extsel = extcfg & ADC_EXTREG_EXTSEL_MASK;

  /* EXTSEL selection: These bits select the external event used
   * to trigger the start of conversion of a regular group.  NOTE:
   *
   * - The position with of the EXTSEL field varies from one STM32 MCU
   *   to another.
   * - The width of the EXTSEL field varies from one STM32 MCU to another.
   */

  if (exten > 0)
    {
      setbits = extsel | exten;
      clrbits = ADC_EXTREG_EXTEN_MASK | ADC_EXTREG_EXTSEL_MASK;

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
static int adc_jextcfg_set(struct stm32_dev_s *priv, uint32_t jextcfg)
{
  uint32_t jexten =  0;
  uint32_t jextsel = 0;
  uint32_t setbits = 0;
  uint32_t clrbits = 0;

  /* Get JEXTEN and JEXTSEL from input */

  jexten = jextcfg & ADC_JEXTREG_JEXTEN_MASK;
  jextsel = jextcfg & ADC_JEXTREG_JEXTSEL_MASK;

  /* JEXTSEL selection: These bits select the external event used
   * to trigger the start of conversion of a injected group.  NOTE:
   *
   * - The position with of the JEXTSEL field varies from one STM32 MCU
   *   to another.
   * - The width of the JEXTSEL field varies from one STM32 MCU to another.
   */

  if (jexten > 0)
    {
      setbits = jexten | jextsel;
      clrbits = ADC_JEXTREG_JEXTEN_MASK | ADC_JEXTREG_JEXTSEL_MASK;

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

static void adc_dumpregs(struct stm32_dev_s *priv)
{
  UNUSED(priv);

  ainfo("SR:   0x%08" PRIx32 " CR1:  0x%08" PRIx32
        " CR2:  0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SR_OFFSET),
        adc_getreg(priv, STM32_ADC_CR1_OFFSET),
        adc_getreg(priv, STM32_ADC_CR2_OFFSET));

  ainfo("SQR1: 0x%08" PRIx32 " SQR2: 0x%08" PRIx32
        " SQR3: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SQR1_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR2_OFFSET),
        adc_getreg(priv, STM32_ADC_SQR3_OFFSET));

  ainfo("SMPR1: 0x%08" PRIx32 " SMPR2: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SMPR1_OFFSET),
        adc_getreg(priv, STM32_ADC_SMPR2_OFFSET));

#ifdef ADC_HAVE_INJECTED
  ainfo("JSQR: 0x%08" PRIx32 "\n", adc_getreg(priv, STM32_ADC_JSQR_OFFSET));
#endif

  ainfo("CCR:  0x%08" PRIx32 "\n",
        adccmn_getreg(priv, STM32_ADC_CCR_OFFSET));
}

/****************************************************************************
 * Name: adc_sqrbits
 ****************************************************************************/

static uint32_t adc_sqrbits(struct stm32_dev_s *priv, int first,
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
 * Name: adc_internal
 ****************************************************************************/

static bool adc_internal(struct stm32_dev_s *priv)
{
  int i;

  if (priv->intf == 1)
    {
      for (i = 0; i < priv->cr_channels; i++)
        {
          if (priv->r_chanlist[i] > ADC_LAST_EXTERNAL_CHAN)
            {
              return true;
            }
        }

#ifdef ADC_HAVE_INJECTED
      for (i = 0; i < priv->cj_channels; i++)
        {
          if (priv->j_chanlist[i] > ADC_LAST_EXTERNAL_CHAN)
            {
              return true;
            }
        }
#endif
    }

  return false;
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

static int adc_set_ch(struct adc_dev_s *dev, uint8_t ch)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
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

      priv->current    = i;
      priv->rnchannels = 1;
    }

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

static int adc_inj_set_ch(struct adc_dev_s *dev, uint8_t ch)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t clrbits;
  uint32_t setbits;
  int i;

  /* Configure injected sequence length */

  setbits = ADC_JSQR_JL(priv->cj_channels);
  clrbits = ADC_JEXTREG_JEXTSEL_MASK | ADC_JSQR_JL_MASK;

  /* Configure injected channels */

  for (i = 0 ; i < priv->cj_channels; i += 1)
    {
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

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int ret = OK;

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

      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = priv->rnchannels;
        }
        break;

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

      case IO_STOP_ADC:
        {
          adc_enable(priv, false);
          break;
        }

      case IO_START_ADC:
        {
          adc_enable(priv, true);
          break;
        }

      case IO_START_CONV:
        {
          uint8_t ch = ((uint8_t)arg);

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

#ifndef CONFIG_STM32F7_ADC_NOIRQ

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

static int adc_interrupt(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
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
 * Name: adc123_interrupt
 *
 * Description:
 *   ADC1/2/3 interrupt handler
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int adc123_interrupt(int irq, void *context, void *arg)
{
#ifdef CONFIG_STM32F7_ADC1
  adc_interrupt(&g_adcdev1);
#endif

#ifdef CONFIG_STM32F7_ADC2
  adc_interrupt(&g_adcdev2);
#endif

#ifdef CONFIG_STM32F7_ADC3
  adc_interrupt(&g_adcdev3);
#endif

  return OK;
}
#endif /* CONFIG_STM32F7_ADC_NOIRQ */

#ifdef CONFIG_STM32F7_ADC_LL_OPS

/****************************************************************************
 * Name: adc_llops_setup
 ****************************************************************************/

static int adc_llops_setup(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  return adc_setup(priv->dev);
}

/****************************************************************************
 * Name: adc_llops_shutdown
 ****************************************************************************/

static void adc_llops_shutdown(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_shutdown(priv->dev);
}

/****************************************************************************
 * Name: adc_intack
 ****************************************************************************/

static void adc_intack(struct stm32_adc_dev_s *dev, uint32_t source)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Cleared by writing 0 to it */

  adc_modifyreg(priv, STM32_ADC_ISR_OFFSET, (source & ADC_ISR_ALLINTS), 0);
}

/****************************************************************************
 * Name: adc_inten
 ****************************************************************************/

static void adc_inten(struct stm32_adc_dev_s *dev, uint32_t source)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Enable interrupts */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, 0, (source & ADC_IER_ALLINTS));
}

/****************************************************************************
 * Name: adc_intdis
 ****************************************************************************/

static void adc_intdis(struct stm32_adc_dev_s *dev, uint32_t source)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Disable interrupts */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, (source & ADC_IER_ALLINTS), 0);
}

/****************************************************************************
 * Name: adc_ackget
 ****************************************************************************/

static uint32_t adc_intget(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t regval;
  uint32_t pending;

  regval  = adc_getreg(priv, STM32_ADC_ISR_OFFSET);
  pending = regval & ADC_ISR_ALLINTS;

  return pending;
}

/****************************************************************************
 * Name: adc_regget
 ****************************************************************************/

static uint32_t adc_regget(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  return adc_getreg(priv, STM32_ADC_DR_OFFSET) & ADC_DR_RDATA_MASK;
}

/****************************************************************************
 * Name: adc_llops_reg_startconv
 ****************************************************************************/

static void adc_llops_reg_startconv(struct stm32_adc_dev_s *dev,
                                    bool enable)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_reg_startconv(priv, enable);
}

/****************************************************************************
 * Name: adc_offset_set
 ****************************************************************************/

static int adc_offset_set(struct stm32_adc_dev_s *dev, uint8_t ch,
                          uint8_t i, uint16_t offset)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
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

/****************************************************************************
 * Name: adc_llops_extcfg_set
 ****************************************************************************/

#ifdef ADC_HAVE_EXTCFG
static void adc_llops_extcfg_set(struct stm32_adc_dev_s *dev,
                                 uint32_t extcfg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_extcfg_set(priv, extcfg);
}
#endif

/****************************************************************************
 * Name: adc_llops_jextcfg_set
 ****************************************************************************/

#ifdef ADC_HAVE_JEXTCFG
static void  adc_llops_jextcfg_set(struct stm32_adc_dev_s *dev,
                                   uint32_t jextcfg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_jextcfg_set(priv, jextcfg);
}
#endif

/****************************************************************************
 * Name: adc_regbufregister
 ****************************************************************************/

#ifdef ADC_HAVE_DMA
static int adc_regbufregister(struct stm32_adc_dev_s *dev,
                              uint16_t *buffer, uint8_t len)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

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
static uint32_t adc_injget(struct stm32_adc_dev_s *dev, uint8_t chan)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
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

static void adc_llops_inj_startconv(struct stm32_adc_dev_s *dev,
                                    bool enable)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

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

#ifdef CONFIG_STM32F7_ADC_CHANGE_SAMPLETIME
static void adc_sampletime_write(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t value = 0;
  uint8_t i;
  uint8_t shift;

  /* Sampling time individually for each channel */

  for (i = 0, shift = 0; i < priv->adc_channels; i++)
    {
      value |= priv->sample_rate[i] << (shift * 3);
      switch (i)
        {
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
 *   dev          - pointer to the adc device structure
 *   time_samples - pointe to the adc sample time configuration data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void adc_sampletime_set(struct stm32_adc_dev_s *dev,
                        struct adc_sample_time_s *time_samples)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
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
#endif /* CONFIG_STM32F7_ADC_CHANGE_SAMPLETIME */

/****************************************************************************
 * Name: adc_llops_dumpregs
 ****************************************************************************/

static void adc_llops_dumpregs(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_dumpregs(priv);
}

#endif /* CONFIG_STM32F7_ADC_LL_OPS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_initialize
 *
 * Description:
 *   Initialize the ADC.
 *
 *   The logic allow initialize ADC regular and injected channels.
 *
 *   The number of injected channels for given ADC is selected from Kconfig
 *   with CONFIG_STM32F7_ADCx_INJECTED_CHAN definitions
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
 *      y = CONFIG_STM32F7_ADCx_INJECTED_CHAN, and y > 0
 *
 *   If CONFIG_STM32F7_ADCx_INJECTED_CHAN = 0, then all channels from
 *   chanlist are regular channels.
 *
 * Input Parameters:
 *   intf      - Could be {1,2,3,4} for ADC1, ADC2, ADC3
 *   chanlist  - The list of channels (regular + injected)
 *   channels  - Number of channels (regular + injected)
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *stm32_adc_initialize(int intf,
                                       const uint8_t *chanlist,
                                       int channels)
{
  struct adc_dev_s   *dev;
  struct stm32_dev_s *priv;
  uint8_t     cr_channels = 0;
  uint8_t     cj_channels = 0;
#ifdef ADC_HAVE_INJECTED
  uint8_t *j_chanlist = NULL;
#endif

  switch (intf)
    {
#ifdef CONFIG_STM32F7_ADC1
      case 1:
        {
          ainfo("ADC1 selected\n");
          dev = &g_adcdev1;
          cj_channels = CONFIG_STM32F7_ADC1_INJECTED_CHAN;
          cr_channels = channels - cj_channels;
#  ifdef ADC_HAVE_INJECTED
          if (cj_channels > 0)
            {
              j_chanlist  = (uint8_t *)chanlist + cr_channels;
            }
#  endif
          break;
        }
#endif /* CONFIG_STM32F7_ADC1 */

#ifdef CONFIG_STM32F7_ADC2
      case 2:
        {
          ainfo("ADC2 selected\n");
          dev = &g_adcdev2;
          cj_channels = CONFIG_STM32F7_ADC2_INJECTED_CHAN;
          cr_channels = channels - cj_channels;
#  ifdef ADC_HAVE_INJECTED
          if (cj_channels > 0)
            {
              j_chanlist  = (uint8_t *)chanlist + cr_channels;
            }
#  endif
          break;
        }
#endif /* CONFIG_STM32F7_ADC2 */

#ifdef CONFIG_STM32F7_ADC3
      case 3:
        {
          ainfo("ADC3 selected\n");
          dev = &g_adcdev3;
          cj_channels = CONFIG_STM32F7_ADC3_INJECTED_CHAN;
          cr_channels = channels - cj_channels;
#  ifdef ADC_HAVE_INJECTED
          if (cj_channels > 0)
            {
              j_chanlist  = (uint8_t *)chanlist + cr_channels;
            }
#  endif
          break;
        }

#endif /* CONFIG_STM32F7_ADC3 */

      default:
        {
          aerr("ERROR: No ADC interface defined\n");
          return NULL;
        }
    }

  /* Configure the selected ADC */

  priv = (struct stm32_dev_s *)dev->ad_priv;

  /* Configure regular channels */

  DEBUGASSERT(cr_channels <= CONFIG_STM32F7_ADC_MAX_SAMPLES);
  if (cr_channels > CONFIG_STM32F7_ADC_MAX_SAMPLES)
    {
      cr_channels = CONFIG_STM32F7_ADC_MAX_SAMPLES;
    }

  priv->cr_channels = cr_channels;
  memcpy(priv->r_chanlist, chanlist, cr_channels);

#ifdef ADC_HAVE_INJECTED
  /* Configure injected channels */

  DEBUGASSERT(cj_channels <= ADC_INJ_MAX_SAMPLES);
  if (cj_channels > ADC_INJ_MAX_SAMPLES)
    {
      cj_channels = ADC_INJ_MAX_SAMPLES;
    }

  priv->cj_channels = cj_channels;
  memcpy(priv->j_chanlist, j_chanlist, cj_channels);
#endif

#ifdef CONFIG_STM32F7_ADC_CHANGE_SAMPLETIME
  /* Assign default values for the sample time table */

  memset(priv->sample_rate, ADC_SMPR_DEFAULT, ADC_CHANNELS_NUMBER);
  priv->adc_channels = ADC_CHANNELS_NUMBER;
#endif

#ifdef CONFIG_STM32F7_ADC_LL_OPS
  /* Store reference to the upper-half ADC device */

  priv->dev = dev;
#endif

#ifdef ADC_HAVE_INJECTED
  ainfo("intf: %d cr_channels: %d, cj_channels: %d\n",
        intf, priv->cr_channels, priv->cj_channels);
#else
  ainfo("intf: %d cr_channels: %d\n", intf, priv->cr_channels);
#endif

  return dev;
}

#endif /* CONFIG_STM32F7_ADC */
