/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_adc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/mutex.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32.h"
#include "stm32_tim.h"
#include "stm32_dma.h"
#include "stm32_adc.h"

/* STM32 ADC "lower-half" support must be enabled */

#ifdef CONFIG_STM32F0L0G0_ADC

/* Some ADC peripheral must be enabled */

#if defined(CONFIG_STM32F0L0G0_ADC1)

#if defined(CONFIG_STM32F0L0G0_STM32F0)
#  error Not tested
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RCC reset ****************************************************************/

#define STM32_RCC_RSTR      STM32_RCC_APB2RSTR
#define RCC_RSTR_ADC1RST    RCC_APB2RSTR_ADC1RST

/* ADC Channels/DMA *********************************************************/

/* DMA values differs according to STM32 DMA IP core version */

#if  defined(HAVE_IP_DMA_V1)
#  define ADC_DMA_CONTROL_WORD (DMA_CCR_MSIZE_16BITS | \
                                DMA_CCR_PSIZE_16BITS | \
                                DMA_CCR_MINC | \
                                DMA_CCR_CIRC)
#else
#  error Not supported
#endif

/* Sample time default configuration */

/* C0 and G0 support additional sample time selection 2 */

#if defined(CONFIG_STM32F0L0G0_STM32G0) || defined(CONFIG_STM32F0L0G0_STM32C0)
#  define ADC_HAVE_SMPR_SMP2
#endif

#if defined(ADC_HAVE_DMA) || (CONFIG_STM32F0L0G0_ADC_MAX_SAMPLES == 1)
#  if defined(CONFIG_ARCH_CHIP_STM32C0) || defined(CONFIG_ARCH_CHIP_STM32G0)
#    define ADC_SMP1_DEFAULT  ADC_SMPR_12p5
#    define ADC_SMP2_DEFAULT  ADC_SMPR_12p5
#  else
#    define ADC_SMP1_DEFAULT  ADC_SMPR_13p5
#  endif
#else /* Slow down sampling frequency */
#  if defined(CONFIG_ARCH_CHIP_STM32C0) || defined(CONFIG_ARCH_CHIP_STM32G0)
#    define ADC_SMP1_DEFAULT  ADC_SMPR_160p5
#    define ADC_SMP2_DEFAULT  ADC_SMPR_160p5
#  else
#    define ADC_SMP1_DEFAULT  ADC_SMPR_239p5
#  endif
#endif

#ifdef ADC_HAVE_SMPR_SMP2
#  define ADC_SMPSEL_DEFAULT  0 /* For now we use only SMP1 */
#endif

/* Number of channels per ADC:
 *   - F0, L0, G0 - 19, but single SMP for all channels
 *
 * NOTE: this value can be obtained from SMPRx register description
 *       (ST manual)
 */

#if defined(CONFIG_STM32F0L0G0_STM32F0) || \
    defined(CONFIG_STM32F0L0G0_STM32L0) || \
    defined(CONFIG_STM32F0L0G0_STM32C0) || \
    defined(CONFIG_STM32F0L0G0_STM32G0)
#  define ADC_CHANNELS_NUMBER 19
#else
#  error "Not supported"
#endif

/* ADC resolution */

#define HAVE_ADC_RESOLUTION

/* ADC have common registers but only single ADC */

#define HAVE_ADC_CMN_REGS 1

/* ADCx_EXTSEL_VALUE */

#ifdef ADC1_EXTSEL_VALUE
#  define ADC1_HAVE_EXTCFG  1
#  define ADC1_EXTCFG_VALUE (ADC1_EXTSEL_VALUE | ADC_EXTREG_EXTEN_DEFAULT)
#else
#  undef ADC1_HAVE_EXTCFG
#endif

#if defined(ADC1_HAVE_EXTCFG)
#  define ADC_HAVE_EXTCFG
#endif

/* ADC DMA configuration bit support */

#define ADC_HAVE_DMACFG 1

#if defined(CONFIG_STM32F0L0G0_STM32G0) || defined(CONFIG_STM32F0L0G0_STM32L0)
#  ifndef ANIOC_SET_OVERSAMPLE
#    define ANIOC_SET_OVERSAMPLE _ANIOC(0x0f)
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Data common to all ADC instances */

#ifdef HAVE_ADC_CMN_DATA
struct adccmn_data_s
{
  uint8_t initialized; /* How many ADC instances are currently in use */
  mutex_t lock;        /* Exclusive access to common ADC data */
};
#endif

/* This structure describes the state of one ADC block
 * REVISIT: save some space with bit fields.
 */

struct stm32_dev_s
{
#ifdef CONFIG_STM32F0L0G0_ADC_LL_OPS
  const struct stm32_adc_ops_s *llops; /* Low-level ADC ops */
#endif
#if !defined(CONFIG_STM32F0L0G0_ADC_NOIRQ) | defined(ADC_HAVE_DMA)
  const struct adc_callback_s *cb;
  uint8_t irq;               /* Interrupt generated by this ADC block */
#endif
#ifdef HAVE_ADC_CMN_DATA
  struct adccmn_data_s *cmn; /* Common ADC data */
#endif
  uint8_t rnchannels;        /* Number of regular channels */
  uint8_t cr_channels;       /* Number of configured regular channels */
  uint8_t intf;              /* ADC interface number */
  uint8_t current;           /* Current ADC channel being converted */
#ifdef HAVE_ADC_RESOLUTION
  uint8_t resolution;        /* ADC resolution (0-3) */
#endif
#ifdef ADC_HAVE_DMA
  uint8_t dmachan;           /* DMA channel needed by this ADC */
#  ifdef ADC_HAVE_DMACFG
  uint8_t dmacfg;            /* DMA channel configuration, only for ADC IPv2 */
#  endif
  bool    hasdma;            /* True: This channel supports DMA */
  uint16_t dmabatch;         /* Number of conversions for DMA batch */
#endif
#ifdef CONFIG_STM32F0L0G0_ADC_CHANGE_SAMPLETIME
  /* Sample time selection. These bits must be written only when ADON=0. */

#  ifdef ADC_HAVE_SMPR_SMP2
  uint8_t sample_rate[2];    /* [0] for SMP1, [1] for SMP2 */
  uint32_t smpsel;           /* ADC Sample Rate Selection Bits */
#  else
  uint8_t sample_rate[1];    /* Only SMP1 is used */
#  endif
#endif
#ifdef ADC_HAVE_TIMER
  uint8_t trigger;           /* Timer trigger channel: 0=CC1, 1=CC2, 2=CC3,
                              * 3=CC4, 4=TRGO, 5=TRGO2
                              */
#endif
  xcpt_t   isr;              /* Interrupt handler for this ADC block */
  uint32_t base;             /* Base address of registers unique to this ADC
                              * block */
#ifdef ADC_HAVE_EXTCFG
  uint32_t extcfg;           /* External event configuration for regular group */
#endif
#ifdef ADC_HAVE_TIMER
  uint32_t tbase;            /* Base address of timer used by this ADC block */
  uint32_t pclck;            /* The PCLK frequency that drives this timer */
  uint32_t freq;             /* The desired frequency of conversions */
#endif
#ifdef ADC_HAVE_DMA
  DMA_HANDLE dma;            /* Allocated DMA channel */

  /* DMA transfer buffer */

  uint16_t *r_dmabuffer;
#endif

  /* List of selected ADC channels to sample */

  uint8_t  r_chanlist[CONFIG_STM32F0L0G0_ADC_MAX_SAMPLES];
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
#ifdef HAVE_ADC_CMN_REGS
static uint32_t adccmn_base_get(struct stm32_dev_s *priv);
static void adccmn_modifyreg(struct stm32_dev_s *priv, uint32_t offset,
                             uint32_t clrbits, uint32_t setbits);
static uint32_t adccmn_getreg(struct stm32_dev_s *priv, uint32_t offset);
#endif
#ifdef ADC_HAVE_TIMER
static uint16_t tim_getreg(struct stm32_dev_s *priv, int offset);
static void tim_putreg(struct stm32_dev_s *priv, int offset,
                       uint16_t value);
static void tim_modifyreg(struct stm32_dev_s *priv, int offset,
                          uint16_t clrbits, uint16_t setbits);
static void tim_modifyreg32(struct stm32_dev_s *priv, int offset,
                            uint32_t clrbits, uint32_t setbits);
static void tim_dumpregs(struct stm32_dev_s *priv,
                         const char *msg);
#endif

static void adc_rccreset(struct stm32_dev_s *priv, bool reset);

/* ADC Interrupt Handler */

#ifndef CONFIG_STM32F0L0G0_ADC_NOIRQ
static int  adc_interrupt(struct adc_dev_s *dev);
static int  adc1_interrupt(int irq, void *context, void *arg);
#endif /* CONFIG_STM32F0L0G0_ADC_NOIRQ */

/* ADC Driver Methods */

static int  adc_bind(struct adc_dev_s *dev,
                     const struct adc_callback_s *callback);
static void adc_reset(struct adc_dev_s *dev);
static int  adc_setup(struct adc_dev_s *dev);
static void adc_shutdown(struct adc_dev_s *dev);
static void adc_rxint(struct adc_dev_s *dev, bool enable);
static int  adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg);
static void adc_enable(struct stm32_dev_s *priv, bool enable);

static int  adc_set_ch(struct adc_dev_s *dev, uint8_t ch);

static int  adc_ioc_change_ints(struct adc_dev_s *dev, int cmd,
                                bool arg);

#ifdef HAVE_ADC_RESOLUTION
static int  adc_resolution_set(struct adc_dev_s *dev, uint8_t res);
#endif
#ifdef HAVE_ADC_VBAT
static void adc_enable_vbat_channel(struct adc_dev_s *dev, bool enable);
#endif
#ifdef HAVE_ADC_POWERDOWN
static int  adc_ioc_change_sleep_between_opers(struct adc_dev_s *dev,
                                               int cmd, bool arg);
static void adc_power_down_idle(struct stm32_dev_s *priv,
                                bool pdi_high);
static void adc_power_down_delay(struct stm32_dev_s *priv,
                                 bool pdd_high);
#endif

#ifdef ADC_HAVE_TIMER
static void adc_timstart(struct stm32_dev_s *priv, bool enable);
static int  adc_timinit(struct stm32_dev_s *priv);
#endif

#if defined(ADC_HAVE_DMA)
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                void *arg);
#endif

static void adc_reg_startconv(struct stm32_dev_s *priv, bool enable);

#ifdef ADC_HAVE_EXTCFG
static int adc_extcfg_set(struct adc_dev_s *dev, uint32_t extcfg);
#endif

static void adc_dumpregs(struct stm32_dev_s *priv);

#ifdef CONFIG_STM32F0L0G0_ADC_LL_OPS
static void adc_llops_intack(struct stm32_adc_dev_s *dev,
                             uint32_t source);
static void adc_llops_inten(struct stm32_adc_dev_s *dev,
                            uint32_t source);
static void adc_llops_intdis(struct stm32_adc_dev_s *dev,
                             uint32_t source);
static uint32_t adc_llops_intget(struct stm32_adc_dev_s *dev);
static uint32_t adc_llops_regget(struct stm32_adc_dev_s *dev);
static void adc_llops_reg_startconv(struct stm32_adc_dev_s *dev,
                                    bool enable);
#  ifdef ADC_HAVE_DMA
static int adc_llops_regbufregister(struct stm32_adc_dev_s *dev,
                                    uint16_t *buffer, uint8_t len);
#  endif
#  ifdef CONFIG_STM32F0L0G0_ADC_CHANGE_SAMPLETIME
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

#ifdef CONFIG_STM32F0L0G0_ADC_LL_OPS
static const struct stm32_adc_ops_s g_adc_llops =
{
  .int_ack       = adc_llops_intack,
  .int_get       = adc_llops_intget,
  .int_en        = adc_llops_inten,
  .int_dis       = adc_llops_intdis,
  .val_get       = adc_llops_regget,
  .reg_startconv = adc_llops_reg_startconv,
#  ifdef ADC_HAVE_DMA
  .regbuf_reg    = adc_llops_regbufregister,
#  endif
#  ifdef CONFIG_STM32F0L0G0_ADC_CHANGE_SAMPLETIME
  .stime_set     = adc_sampletime_set,
  .stime_write   = adc_sampletime_write,
#  endif
  .dump_regs     = adc_llops_dumpregs
};
#endif

/* ADC1 state */

#ifdef CONFIG_STM32F0L0G0_ADC1

#ifdef ADC1_HAVE_DMA
static uint16_t g_adc1_dmabuffer[CONFIG_STM32F0L0G0_ADC_MAX_SAMPLES *
                                 CONFIG_STM32F0L0G0_ADC1_DMA_BATCH];
#endif

static struct stm32_dev_s g_adcpriv1 =
{
#ifdef CONFIG_STM32F0L0G0_ADC_LL_OPS
  .llops       = &g_adc_llops,
#endif
#ifndef CONFIG_STM32F0L0G0_ADC_NOIRQ
  .irq         = STM32_IRQ_ADC,
  .isr         = adc1_interrupt,
#endif /* CONFIG_STM32F0L0G0_ADC_NOIRQ */
#ifdef HAVE_ADC_CMN_DATA
  .cmn         = &ADC1CMN_DATA,
#endif
  .intf        = 1,
#ifdef HAVE_ADC_RESOLUTION
  .resolution  = CONFIG_STM32F0L0G0_ADC1_RESOLUTION,
#endif
  .base        = STM32_ADC1_BASE,
#ifdef ADC1_HAVE_EXTCFG
  .extcfg      = ADC1_EXTCFG_VALUE,
#endif
#ifdef ADC1_HAVE_TIMER
  .trigger     = CONFIG_STM32F0L0G0_ADC1_TIMTRIG,
  .tbase       = ADC1_TIMER_BASE,
  .pclck       = ADC1_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32F0L0G0_ADC1_SAMPLE_FREQUENCY,
#endif
#ifdef ADC1_HAVE_DMA
  .dmachan     = ADC1_DMA_CHAN,
#  ifdef ADC_HAVE_DMACFG
  .dmacfg      = CONFIG_STM32F0L0G0_ADC1_DMA_CFG,
#  endif
  .hasdma      = true,
  .r_dmabuffer = g_adc1_dmabuffer,
  .dmabatch    = CONFIG_STM32F0L0G0_ADC1_DMA_BATCH
#endif
};

static struct adc_dev_s g_adcdev1 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_adcpriv1,
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

#ifdef HAVE_ADC_CMN_REGS

/****************************************************************************
 * Name: adccmn_base_get
 ****************************************************************************/

static uint32_t adccmn_base_get(struct stm32_dev_s *priv)
{
  uint32_t base = 0;

  if (priv->base == STM32_ADC1_BASE)
    {
      base = STM32_ADC12CMN_BASE;
    }

  return base;
}

/****************************************************************************
 * Name: adccmn_modifyreg
 ****************************************************************************/

static void adccmn_modifyreg(struct stm32_dev_s *priv, uint32_t offset,
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

static uint32_t adccmn_getreg(struct stm32_dev_s *priv, uint32_t offset)
{
  uint32_t base = 0;

  /* Get base address for ADC common register */

  base = adccmn_base_get(priv);

  /* Return register value */

  return getreg32(base + offset);
}
#endif /* HAVE_ADC_CMN_REGS */

#ifdef ADC_HAVE_TIMER
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

static uint16_t tim_getreg(struct stm32_dev_s *priv, int offset)
{
  return getreg16(priv->tbase + offset);
}

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

static void tim_putreg(struct stm32_dev_s *priv, int offset,
                       uint16_t value)
{
  putreg16(value, priv->tbase + offset);
}

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

static void tim_modifyreg(struct stm32_dev_s *priv, int offset,
                          uint16_t clrbits, uint16_t setbits)
{
  tim_putreg(priv, offset, (tim_getreg(priv, offset) & ~clrbits) | setbits);
}

/****************************************************************************
 * Name: tim_modifyreg32
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

static void tim_modifyreg32(struct stm32_dev_s *priv, int offset,
                            uint32_t clrbits, uint32_t setbits)
{
  uint32_t addr = priv->tbase + offset;
  putreg32((getreg32(addr) & ~clrbits) | setbits, addr);
}

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
#if STM32_NATIM > 0
  if (priv->tbase == STM32_TIM1_BASE)
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

  if (priv->tbase == STM32_TIM1_BASE)
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
          /* Set the event TRGO */

          ccenable = 0;
          egr      = GTIM_EGR_TG;

          tim_modifyreg(priv, STM32_GTIM_CR2_OFFSET, clrbits,
                        GTIM_CR2_MMS_UPDATE);
        }
        break;

      case 5: /* TimerX TRGO2 event */
        {
          /* Set the event TRGO2 */

          ccenable = 0;
          egr      = GTIM_EGR_TG;

          tim_modifyreg32(priv, STM32_GTIM_CR2_OFFSET, clrbits,
                          ATIM_CR2_MMS2_UPDATE);
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

  /* Reset the output compare and output compare N IDLE State */

  if (priv->tbase >= STM32_TIM2_BASE && priv->tbase <= STM32_TIM3_BASE)
    {
      /* Reset output N polarity level, output N state, output compare state,
       * output compare N idle state.
       */

      ccer &= ~(GTIM_CCER_CC1NE | GTIM_CCER_CC1NP |
                GTIM_CCER_CC2NP | GTIM_CCER_CC3NP |
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

static void adc_rccreset(struct stm32_dev_s *priv, bool reset)
{
  uint32_t adcbit;

  /* Pick the appropriate bit in the RCC reset register.
   * For the STM32 ADC IPv2, there is an individual bit to reset each ADC
   * block.
   */

  switch (priv->intf)
    {
#if defined(CONFIG_STM32F0L0G0_ADC1)
      case 1:
        {
          adcbit = RCC_RSTR_ADC1RST;
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

#if defined(ADC_HAVE_DMA)
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                void *arg)
{
  struct adc_dev_s   *dev  = (struct adc_dev_s *)arg;
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int i;

  /* Verify that the upper-half driver has bound its callback functions */

  if (priv->cb != NULL)
    {
      DEBUGASSERT(priv->cb->au_receive != NULL);

      for (i = 0; i < priv->rnchannels * priv->dmabatch; i++)
        {
          priv->cb->au_receive(dev, priv->r_chanlist[priv->current],
                               priv->r_dmabuffer[i]);
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
#ifndef CONFIG_STM32F0L0G0_ADC_NOIRQ
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
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

  uint32_t th = 0;

  /* Initialize the watchdog high threshold register */

  th |= 0x0fff << ADC_TR_HT_SHIFT;

  /* Initialize the watchdog low threshold register */

  th |= 0x0000 << ADC_TR_LT_SHIFT;

  /* Write threshold register */

  adc_putreg(priv, STM32_ADC_TR_OFFSET, th);

  clrbits = ADC_CFGR1_AWDCH_MASK;
  setbits = ADC_CFGR1_AWDEN | (priv->r_chanlist[0] << ADC_CFGR1_AWDCH_SHIFT);

  /* Modify CFGR1 configuration */

  adc_modifyreg(priv, STM32_ADC_CFGR1_OFFSET, clrbits, setbits);
}

/****************************************************************************
 * Name: adc_calibrate
 ****************************************************************************/

static void adc_calibrate(struct stm32_dev_s *priv)
{
  /* Calibrate the ADC.
   *   1. ADC must be disabled
   *   2. the voltage regulator must be enabled
   */

  adc_modifyreg(priv, STM32_ADC_CR_OFFSET, 0, ADC_CR_ADCAL);

  /* Wait for the calibration to complete */

  while ((adc_getreg(priv, STM32_ADC_CR_OFFSET) & ADC_CR_ADCAL) != 0);
}

/****************************************************************************
 * Name: adc_mode_cfg
 ****************************************************************************/

static void adc_mode_cfg(struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

  /* Disable continuous mode and set align to right */

  clrbits = ADC_CFGR1_CONT | ADC_CFGR1_ALIGN;

  /* Disable external trigger for regular channels */

  clrbits |= ADC_CFGR1_EXTEN_MASK;
  setbits |= ADC_CFGR1_EXTEN_NONE;

#ifdef CONFIG_STM32F0L0G0_ADC1_CONTINUOUS
  setbits |= ADC_CFGR1_CONT;
#endif

  /* Set CFGR configuration */

  adc_modifyreg(priv, STM32_ADC_CFGR1_OFFSET, clrbits, setbits);
}

/****************************************************************************
 * Name: adc_voltreg_cfg
 ****************************************************************************/

static void adc_voltreg_cfg(struct stm32_dev_s *priv)
{
  /* Enable voltage regulator - required by ADC calibration */

  adc_putreg(priv, STM32_ADC_CR_OFFSET, ADC_CR_ADVREGEN);

  /* Wait for ADC voltage regulator start-up */

  up_udelay(50);
}

/****************************************************************************
 * Name: adc_sampletime_cfg
 ****************************************************************************/

static void adc_sampletime_cfg(struct adc_dev_s *dev)
{
  /* Initialize the same sample time for each ADC.
   * During sample cycles channel selection bits must remain unchanged.
   */
#ifdef CONFIG_STM32F0L0G0_ADC_CHANGE_SAMPLETIME
  struct adc_sample_time_s time_samples = {
#  ifdef STM32_ADC1_SMPR_SMP1
      .smp1    = STM32_ADC1_SMPR_SMP1,
#  else
      .smp1    = ADC_SMP1_DEFAULT,
#  endif

#  ifdef ADC_HAVE_SMPR_SMP2
#    ifdef STM32_ADC1_SMPR_SMP2
      .smp2    = STM32_ADC1_SMPR_SMP2,
#    else
      .smp2    = ADC_SMP2_DEFAULT,
#    endif

#    ifdef STM32_ADC1_SMPR_SMPSEL
      .smpsel  = STM32_ADC1_SMPR_SMPSEL
#    else
      .smpsel  = ADC_SMPSEL_DEFAULT
#    endif
#  else
      .smp2    = 0,
      .smpsel  = 0
#  endif
  };

  adc_sampletime_set((struct stm32_adc_dev_s *)dev, &time_samples);
  adc_sampletime_write((struct stm32_adc_dev_s *)dev);
#else
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t setbits = 0;

  /* Configure sample time 1 */

  setbits |= ADC_SMP1_DEFAULT << ADC_SMPR_SMP1_SHIFT;

#ifdef ADC_HAVE_SMPR_SMP2
  /* Configure sample time 2 */

  setbits |= ADC_SMP2_DEFAULT << ADC_SMPR_SMP2_SHIFT;

  /* Configure sample time selection */

  setbits |= ADC_SMPSEL_DEFAULT << ADC_SMPR_SMPSEL_SHIFT;
#endif

  /* Write SMPR register */

  adc_putreg(priv, STM32_ADC_SMPR_OFFSET, setbits);
#endif
}

/****************************************************************************
 * Name: adc_ckmode_cfg
 ****************************************************************************/

static void adc_ckmode_cfg(struct stm32_dev_s *priv)
{
  uint32_t setbits = 0;
  uint32_t clearbits = ADC_CFGR2_CKMODE_MASK;

#ifdef STM32_ADC_CFGR2_CKMODE
  setbits |= STM32_ADC_CFGR2_CKMODE;
#endif

  adc_modifyreg(priv, STM32_ADC_CFGR2_OFFSET, clearbits, setbits);
}

/****************************************************************************
 * Name: adc_common_cfg
 ****************************************************************************/

static void adc_common_cfg(struct stm32_dev_s *priv)
{
  uint32_t clrbits = 0;
  uint32_t setbits = 0;

#ifdef STM32_ADC_CCR_PRESC
  setbits |= STM32_ADC_CCR_PRESC;
#endif

  /* REVISIT: for now we reset all CCR bits */

  clrbits |= ADC_CCR_PRESC_MASK | ADC_CCR_VREFEN |
             ADC_CCR_TSEN;

#ifdef HAVE_ADC_VBAT
  clrbits |= ADC_CCR_VBATEN;
#endif

#ifdef HAVE_ADC_VLCD
  clrbits |= ADC_CCR_PRESC_MASK;
#endif

#ifdef HAVE_ADC_VLCD
  clrbits |= ADC_CCR_VLCDEN;
#endif

#ifdef HAVE_ADC_LFM
  clrbits |= ADC_CCR_LFMEN;
#endif

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

  stm32_dmasetup(priv->dma,
                 priv->base + STM32_ADC_DR_OFFSET,
                 (uint32_t)priv->r_dmabuffer,
                 priv->rnchannels * priv->dmabatch,
                 ADC_DMA_CONTROL_WORD);

  stm32_dmastart(priv->dma, adc_dmaconvcallback, dev, false);
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

  /* Configure voltage regulator if present */

  adc_voltreg_cfg(priv);

  /* Calibrate ADC */

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

  /* ADC clock mode configuration  */

  adc_ckmode_cfg(priv);

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

  adc_extcfg_set(dev, priv->extcfg);
#endif

  /* Enable ADC */

  adc_enable(priv, true);

  /* Dump regs */

  adc_dumpregs(priv);
}

 #ifdef CONFIG_STM32F0L0G0_ADC_OVERSAMPLE

/****************************************************************************
 * Name: adc_oversample
 ****************************************************************************/

static void adc_oversample(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  uint32_t clrbits = ADC_CFGR2_OVSE | ADC_CFGR2_TOVS |
                     ADC_CFGR2_OVSR_MASK | ADC_CFGR2_OVSS_MASK;

  uint32_t setbits = ADC_CFGR2_OVSE |
                     (CONFIG_STM32F0L0G0_ADC_OVSR << ADC_CFGR2_OVSR_SHIFT) |
                     (CONFIG_STM32F0L0G0_ADC_OVSS << ADC_CFGR2_OVSS_SHIFT);

#  ifdef CONFIG_STM32F0L0G0_ADC_TOVS
  setbits |= ADC_CFGR2_TOVS;
#  endif

  adc_modifyreg(priv, STM32_ADC_CFGR2_OFFSET, clrbits, setbits);
}
#endif

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

#if defined(HAVE_IP_ADC_V2)
  /* Turn off the ADC so we can write the RCC bits */

  adc_enable(priv, false);
#endif

  /* Only if this is the first initialzied ADC instance in the ADC block */

#ifdef HAVE_ADC_CMN_DATA
  if (nxmutex_lock(&priv->cmn->lock) < 0)
    {
      leave_critical_section(flags);
      return;
    }

  if (priv->cmn->initialized == 0)
#endif
    {
      /* Enable ADC reset state */

      adc_rccreset(priv, true);

      /* Release ADC from reset state */

      adc_rccreset(priv, false);
    }

#ifdef HAVE_ADC_CMN_DATA
  nxmutex_unlock(&priv->cmn->lock);
#endif

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
#if !defined(CONFIG_STM32F0L0G0_ADC_NOIRQ) || defined(HAVE_ADC_CMN_DATA) ||   \
     defined(ADC_HAVE_TIMER) || !defined(CONFIG_STM32F0L0G0_ADC_NO_STARTUP_CONV)
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
#endif
  int ret = OK;

  /* Attach the ADC interrupt */

#ifndef CONFIG_STM32F0L0G0_ADC_NOIRQ
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

#ifdef CONFIG_STM32F0L0G0_ADC_OVERSAMPLE
  adc_oversample(dev);
#endif

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

#ifndef CONFIG_STM32F0L0G0_ADC_NO_STARTUP_CONV
  /* Start regular conversion */

  adc_reg_startconv(priv, true);

#endif

  /* Enable the ADC interrupt */

#ifndef CONFIG_STM32F0L0G0_ADC_NOIRQ
  ainfo("Enable the ADC interrupt: irq=%d\n", priv->irq);
  up_enable_irq(priv->irq);
#endif

#ifdef HAVE_ADC_CMN_DATA
  /* Increase instances counter */

  ret = nxmutex_lock(&priv->cmn->lock);
  if (ret < 0)
    {
      return;
    }

  priv->cmn->initialized += 1;
  nxmutex_unlock(&priv->cmn->lock);
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

static void adc_shutdown(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  /* Disable ADC */

  adc_enable(priv, false);

#ifndef CONFIG_STM32F0L0G0_ADC_NOIRQ
  /* Disable ADC interrupts and detach the ADC interrupt handler */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
#endif

#ifdef HAVE_ADC_CMN_DATA
  if (nxmutex_lock(&priv->cmn->lock) < 0)
    {
      return;
    }

  if (priv->cmn->initialized <= 1)
#endif
    {
      /* Disable and reset the ADC module.
       *
       * NOTE: The ADC block will be reset to its reset state only if all
       *       ADC block instances are closed. This means that the closed ADC
       *       may not be reset which in turn may affect low-power
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

  priv->cmn->initialized -= 1;
  nxmutex_unlock(&priv->cmn->lock);
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
          regval &= ~(ADC_IER_EOC);
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

#ifdef HAVE_ADC_RESOLUTION
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

  adc_modifyreg(priv, STM32_ADC_CFGR1_OFFSET, ADC_CFGR1_RES_MASK,
                res << ADC_CFGR1_RES_SHIFT);

errout:
  return ret;
}
#endif

/****************************************************************************
 * Name: adc_extsel_set
 ****************************************************************************/

#ifdef ADC_HAVE_EXTCFG
static int adc_extcfg_set(struct adc_dev_s *dev, uint32_t extcfg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
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

      ainfo("Initializing extsel = 0x%" PRIx32 "\n", extsel);

      /* Write register */

      adc_modifyreg(priv, STM32_ADC_EXTREG_OFFSET, clrbits, setbits);
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

  ainfo("ISR:  0x%08" PRIx32 " IER:  0x%08" PRIx32
        " CR:   0x%08" PRIx32 " CFGR1: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_ISR_OFFSET),
        adc_getreg(priv, STM32_ADC_IER_OFFSET),
        adc_getreg(priv, STM32_ADC_CR_OFFSET),
        adc_getreg(priv, STM32_ADC_CFGR1_OFFSET));

  ainfo("SMPR: 0x%08" PRIx32 " CHSELR: 0x%08" PRIx32 "\n",
        adc_getreg(priv, STM32_ADC_SMPR_OFFSET),
        adc_getreg(priv, STM32_ADC_CHSELR_OFFSET));

  ainfo("CCR:  0x%08" PRIx32 "\n",
        adccmn_getreg(priv, STM32_ADC_CCR_OFFSET));
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
static void adc_enable_vbat_channel(struct adc_dev_s *dev, bool enable)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  if (enable)
    {
      adccmn_modifyreg(priv, STM32_ADC_CCR_OFFSET, 0, ADC_CCR_VBATEN);
    }
  else
    {
      adccmn_modifyreg(priv, STM32_ADC_CCR_OFFSET, ADC_CCR_VBATEN, 0);
    }

  ainfo("STM32_ADC_CCR value: 0x%08x\n",
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
static int adc_ioc_change_sleep_between_opers(struct adc_dev_s *dev,
                                              int cmd, bool arg)
{
  int ret = OK;
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

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

static void adc_ioc_enable_awd_int(struct stm32_dev_s *priv, bool enable)
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

static void adc_ioc_enable_eoc_int(struct stm32_dev_s *priv, bool enable)
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

static void adc_ioc_enable_ovr_int(struct stm32_dev_s *priv, bool enable)
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

static int adc_ioc_change_ints(struct adc_dev_s *dev, int cmd, bool arg)
{
  int ret = OK;
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  switch (cmd)
    {
      case IO_ENABLE_DISABLE_AWDIE:
        adc_ioc_enable_awd_int(priv, arg);
        break;

      case IO_ENABLE_DISABLE_EOCIE:
        adc_ioc_enable_eoc_int(priv, arg);
        break;

      case IO_ENABLE_DISABLE_OVRIE:
        adc_ioc_enable_ovr_int(priv, arg);
        break;

      case IO_ENABLE_DISABLE_ALL_INTS:
        adc_ioc_enable_awd_int(priv, arg);
        adc_ioc_enable_eoc_int(priv, arg);
        adc_ioc_enable_ovr_int(priv, arg);
        break;

      default:
        ainfo("unknown cmd: %d\n", cmd);
        break;
    }

  return ret;
}
#ifdef CONFIG_STM32F0L0G0_ADC_OVERSAMPLE

/****************************************************************************
 * Name: adc_ioc_set_oversample
 *
 * Description:
 *   For STM32G0 and STM32L0: Configure hardware oversampling via CFGR2.
 *
 * Input:
 *   dev - pointer to the ADC device
 *   arg - Packed 32-bit value that matches CFGR2 layout for OVSE, TOVS,
 *         OVSR[2:0] and OVSS[3:0].
 *
 *         Bit fields (match ADC_CFGR2 register layout):
 *           [0]     = OVSE  (enable oversampling)
 *           [1]     = TOVS  (triggered oversampling)
 *           [4:2]   = OVSR  (ratio: 000=2x, ..., 111=256x)
 *           [9:5]   = OVSS  (right shift: 00000=no shift, ..., 11111=31-bit)
 *
 * Returned Value:
 *   OK (0) on success
 *
 ****************************************************************************/

static int adc_ioc_set_oversample(struct adc_dev_s *dev, uint32_t arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t clrbits;
  uint32_t setbits;

  /* Mask out the oversampling-related fields from CFGR2:
   * OVSE | TOVS | OVSR[2:0] | OVSS[3:0]
   */

  clrbits = ADC_CFGR2_OVSE     |
            ADC_CFGR2_TOVS     |
            ADC_CFGR2_OVSR_MASK |
            ADC_CFGR2_OVSS_MASK;

  setbits = arg & (ADC_CFGR2_OVSE     |
                   ADC_CFGR2_TOVS     |
                   ADC_CFGR2_OVSR_MASK |
                   ADC_CFGR2_OVSS_MASK);

  adc_modifyreg(priv, STM32_ADC_CFGR2_OFFSET, clrbits, setbits);
  return OK;
}

#endif /* G0 or L0 */

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
  uint32_t bits = 0;
  int      i    = 0;

  if (ch == 0)
    {
      priv->current    = 0;
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

  for (i = 0; i < priv->rnchannels; i += 1)
    {
      bits |= ADC_CHSELR_CHSEL(priv->r_chanlist[i]);
    }

  /* Write register */

  adc_modifyreg(priv, STM32_ADC_CHSELR_OFFSET, 0, bits);

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

static int adc_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int ret                  = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          /* Start regular conversion if regular channels configured */

          if (priv->cr_channels > 0)
            {
              adc_reg_startconv(priv, true);
            }

          break;
        }

      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = priv->cr_channels;
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

      case IO_ENABLE_DISABLE_AWDIE:
      case IO_ENABLE_DISABLE_EOCIE:
      case IO_ENABLE_DISABLE_OVRIE:
      case IO_ENABLE_DISABLE_ALL_INTS:
        {
          adc_ioc_change_ints(dev, cmd, *(bool *)arg);
          break;
        }

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

#if defined(CONFIG_STM32F0L0G0_ADC_OVERSAMPLE)
      case ANIOC_SET_OVERSAMPLE:
        {
          ret = adc_ioc_set_oversample(dev, arg);
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

  return ret;
}

#ifndef CONFIG_STM32F0L0G0_ADC_NOIRQ

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

static int adc1_interrupt(int irq, void *context, void *arg)
{
  adc_interrupt(&g_adcdev1);

  return OK;
}
#endif /* CONFIG_STM32F0L0G0_ADC_NOIRQ */

#ifdef CONFIG_STM32F0L0G0_ADC_LL_OPS

/****************************************************************************
 * Name: adc_llops_intack
 ****************************************************************************/

static void adc_llops_intack(struct stm32_adc_dev_s *dev,
                             uint32_t source)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

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
 * Name: adc_llops_inten
 ****************************************************************************/

static void adc_llops_inten(struct stm32_adc_dev_s *dev, uint32_t source)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Enable interrupts */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, 0, (source & ADC_IER_ALLINTS));
}

/****************************************************************************
 * Name: adc_llops_intdis
 ****************************************************************************/

static void adc_llops_intdis(struct stm32_adc_dev_s *dev,
                             uint32_t source)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  /* Disable interrupts */

  adc_modifyreg(priv, STM32_ADC_IER_OFFSET, (source & ADC_IER_ALLINTS), 0);
}

/****************************************************************************
 * Name: adc_ackget
 ****************************************************************************/

static uint32_t adc_llops_intget(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t regval;
  uint32_t pending;

  regval  = adc_getreg(priv, STM32_ADC_ISR_OFFSET);
  pending = regval & ADC_ISR_ALLINTS;

  return pending;
}

/****************************************************************************
 * Name: adc_llops_regget
 ****************************************************************************/

static uint32_t adc_llops_regget(struct stm32_adc_dev_s *dev)
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
 * Name: adc_llops_regbufregister
 ****************************************************************************/

#ifdef ADC_HAVE_DMA
static int adc_llops_regbufregister(struct stm32_adc_dev_s *dev,
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

#ifdef CONFIG_STM32F0L0G0_ADC_CHANGE_SAMPLETIME
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

static void adc_sampletime_write(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;
  uint32_t smpr = 0;

  smpr |= ((uint32_t)priv->sample_rate[0] << ADC_SMPR_SMP1_SHIFT);

#ifdef ADC_HAVE_SMPR_SMP2
  smpr |= ((uint32_t)priv->sample_rate[1] << ADC_SMPR_SMP2_SHIFT);
  smpr |= ((uint32_t)priv->smpsel << ADC_SMPR_SMPSEL_SHIFT);
#endif

  adc_putreg(priv, STM32_ADC_SMPR_OFFSET, smpr);
}

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

static void adc_sampletime_set(struct stm32_adc_dev_s *dev,
                               struct adc_sample_time_s *time_samples)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  priv->sample_rate[0] = time_samples->smp1;
#ifdef ADC_HAVE_SMPR_SMP2
  priv->sample_rate[1] = time_samples->smp2;
  priv->smpsel = time_samples->smpsel;
#endif
}
#endif /* CONFIG_STM32F0L0G0_ADC_CHANGE_SAMPLETIME */

/****************************************************************************
 * Name: adc_llops_dumpregs
 ****************************************************************************/

static void adc_llops_dumpregs(struct stm32_adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev;

  adc_dumpregs(priv);
}

#endif /* CONFIG_STM32F0L0G0_ADC_LL_OPS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_initialize
 *
 * Description:
 *   Initialize the ADC.
 *
 * Input Parameters:
 *   intf      - Could be {1} for ADC1
 *   chanlist  - The list of channels
 *   channels - Number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *stm32_adcinitialize(int intf, const uint8_t *chanlist,
                                      int channels)
{
  struct adc_dev_s   *dev;
  struct stm32_dev_s *priv;

  ainfo("intf: %d cchannels: %d\n", intf, channels);

  switch (intf)
    {
#ifdef CONFIG_STM32F0L0G0_ADC1
      case 1:
        {
          ainfo("ADC1 selected\n");
          dev = &g_adcdev1;
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

  priv = (struct stm32_dev_s *)dev->ad_priv;
  priv->cb = NULL;

  DEBUGASSERT(channels <= CONFIG_STM32F0L0G0_ADC_MAX_SAMPLES);

  priv->cr_channels = channels;
  memcpy(priv->r_chanlist, chanlist, channels);

#ifdef CONFIG_PM
  if (pm_register(&priv->pm_callback) != OK)
    {
      aerr("Power management registration failed\n");
      return NULL;
    }
#endif

  return dev;
}

#endif /* CONFIG_STM32F0L0G0_ADC1 */
#endif /* CONFIG_STM32F0L0G0_ADC */
