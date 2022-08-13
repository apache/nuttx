/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_dfsdm.c
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

#include <errno.h>
#include <assert.h>
#include <debug.h>
#include <string.h>

#include <arch/board/board.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "chip.h"
#include "stm32l4_rcc.h"
#include "stm32l4_tim.h"
#include "stm32l4_dma.h"
#include "stm32l4_dfsdm.h"

/* ADC "upper half" support must be enabled. DFSDM are ADC devices. */

#ifdef CONFIG_ADC

/* The peripheral must be enabled */

#ifdef CONFIG_STM32L4_DFSDM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Sanity checking **********************************************************/

#if !defined(CONFIG_STM32L4_DFSDM1_FLT0) && \
    !defined(CONFIG_STM32L4_DFSDM1_FLT1) && \
    !defined(CONFIG_STM32L4_DFSDM1_FLT2) && !defined(CONFIG_STM32L4_DFSDM1_FLT3)
#  error "At least one DFSDM filter must be defined"
#endif

#if defined(CONFIG_STM32L4_STM32L4X3)
#  if defined(CONFIG_STM32L4_DFSDM1_FLT2) || defined(CONFIG_STM32L4_DFSDM1_FLT3)
#    error "Non-existent DFSDM filter defined"
#  endif
#endif

/* Abbreviated register access **********************************************/

#define CHCFGR1_OFFSET(priv)   STM32L4_DFSDM_CHCFGR1_OFFSET((priv)->current)
#define CHCFGR2_OFFSET(priv)   STM32L4_DFSDM_CHCFGR2_OFFSET((priv)->current)

#define FLTCR1_OFFSET(priv)    STM32L4_DFSDM_FLTCR1_OFFSET((priv)->intf)
#define FLTCR2_OFFSET(priv)    STM32L4_DFSDM_FLTCR2_OFFSET((priv)->intf)
#define FLTISR_OFFSET(priv)    STM32L4_DFSDM_FLTISR_OFFSET((priv)->intf)
#define FLTICR_OFFSET(priv)    STM32L4_DFSDM_FLTICR_OFFSET((priv)->intf)
#define FLTFCR_OFFSET(priv)    STM32L4_DFSDM_FLTFCR_OFFSET((priv)->intf)
#define FLTRDATAR_OFFSET(priv) STM32L4_DFSDM_FLTRDATAR_OFFSET((priv)->intf)
#define FLTAWHTR_OFFSET(priv)  STM32L4_DFSDM_FLTAWHTR_OFFSET((priv)->intf)
#define FLTAWLTR_OFFSET(priv)  STM32L4_DFSDM_FLTAWLTR_OFFSET((priv)->intf)
#define FLTAWSR_OFFSET(priv)   STM32L4_DFSDM_FLTAWSR_OFFSET((priv)->intf)
#define FLTAWCFR_OFFSET(priv)  STM32L4_DFSDM_FLTAWCFR_OFFSET((priv)->intf)
#define FLTEXMAX_OFFSET(priv)  STM32L4_DFSDM_FLTEXMAX_OFFSET((priv)->intf)
#define FLTEXMIN_OFFSET(priv)  STM32L4_DFSDM_FLTEXMIN_OFFSET((priv)->intf)

/* DFSDM Filter interrupts **************************************************/

/* These bits are same in FLRCR2 and FLTISR (FLTICR only has JOVR and ROVR) */

#define DFSDM_INT_JEOC   DFSDM_FLTCR2_JEOCIE
#define DFSDM_INT_REOC   DFSDM_FLTCR2_REOCIE
#define DFSDM_INT_JOVR   DFSDM_FLTCR2_JOWRIE
#define DFSDM_INT_ROVR   DFSDM_FLTCR2_ROWRIE
#define DFSDM_INT_AWD    DFSDM_FLTCR2_AWDIE

/* SCDIE and CKABIE are not in this, as the bits exist only in FLT0 filter
 * CR2.
 */

#define DFSDM_INT_MASK (DFSDM_FLTCR2_JEOCIE | DFSDM_FLTCR2_REOCIE | \
                        DFSDM_FLTCR2_JOWRIE | DFSDM_FLTCR2_ROWRIE | \
                        DFSDM_FLTCR2_AWDIE)

/* Similarly, this is missing SCDF and CKABF as special support is needed. */

#define DFSDM_ISR_MASK (DFSDM_FLTISR_JEOCF | DFSDM_FLTISR_REOCF | \
                        DFSDM_FLTISR_JOVRF | DFSDM_FLTISR_ROVRF | \
                        DFSDM_FLTISR_AWDF)

/* DFSDM Channels/DMA *******************************************************/

/* The maximum number of channels that can be sampled.  While DMA support is
 * very nice for reliable multi-channel sampling, the STM32L4 can function
 * without, although there is a risk of overrun.
 */

#if defined(CONFIG_STM32L4_STM32L4X3)
#  define DFSDM_MAX_CHANNELS 4
#  define DFSDM_MAX_FILTERS  2
#else
#  define DFSDM_MAX_CHANNELS 8
#  define DFSDM_MAX_FILTERS  4
#endif

#ifdef DFSDM_HAVE_DMA
#  if !defined(CONFIG_STM32L4_DMA1) && !defined(CONFIG_STM32L4_DMAMUX)
#    error "STM32L4 DFSDM DMA support requires CONFIG_STM32L4_DMA1"
#  endif
#endif

/* We currently only use DMA to read DFSDM conversion results, not for
 * parallel inputs to DFSDM_CHyDATINR register.
 */

#define DFSDM_DMA_CONTROL_WORD (DMA_CCR_MSIZE_32BITS | \
                                DMA_CCR_PSIZE_32BITS | \
                                DMA_CCR_MINC | \
                                DMA_CCR_CIRC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of one DFSDM block */

struct stm32_dev_s
{
  const struct adc_callback_s *cb;
  uint8_t irq;          /* Interrupt generated by this block */
  uint8_t nchannels;    /* Number of channels */
  uint8_t cchannels;    /* Number of configured channels */
  uint8_t intf;         /* DFSDM filter number */
  uint8_t current;      /* Current channel being converted */
#ifdef DFSDM_HAVE_DMA
  uint8_t dmachan;      /* DMA channel needed by this filter */
  bool    hasdma;       /* True: This filter supports DMA */
#endif
#ifdef DFSDM_HAVE_TIMER
  uint8_t trigger;      /* Timer trigger channel: 0=CC1, 1=CC2, 2=CC3,
                         * 3=CC4, 4=TRGO, 5=TRGO2 */
#endif
  xcpt_t   isr;         /* Interrupt handler for this filter block */
  uint32_t base;        /* Base address of registers */
#ifdef DFSDM_HAVE_TIMER
  uint32_t tbase;       /* Base address of timer */
  uint32_t extsel;      /* EXTSEL value used */
  uint32_t pclck;       /* The PCLK frequency that drives this timer */
  uint32_t freq;        /* The desired frequency of conversions */
#endif

#ifdef DFSDM_HAVE_DMA
  DMA_HANDLE dma;       /* Allocated DMA channel */

  /* DMA transfer buffer */

  uint32_t dmabuffer[DFSDM_MAX_CHANNELS];
#endif

  /* List of selected channels to sample */

  uint8_t  chanlist[DFSDM_MAX_CHANNELS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register access */

static uint32_t dfsdm_getreg(struct stm32_dev_s *priv, int offset);
static void     dfsdm_putreg(struct stm32_dev_s *priv, int offset,
                             uint32_t value);
static void     dfsdm_modifyreg(struct stm32_dev_s *priv, int offset,
                                uint32_t clrbits, uint32_t setbits);

#ifdef DFSDM_HAVE_TIMER
static uint16_t tim_getreg(struct stm32_dev_s *priv, int offset);
static void     tim_putreg(struct stm32_dev_s *priv, int offset,
                           uint16_t value);
static void     tim_modifyreg(struct stm32_dev_s *priv, int offset,
                              uint16_t clrbits, uint16_t setbits);
static void     tim_dumpregs(struct stm32_dev_s *priv,
                             const char *msg);
#endif

/* Miscellaneous Helpers */

static void dfsdm_rccreset(struct stm32_dev_s *priv, bool reset);
static void dfsdm_enable(struct stm32_dev_s *priv);
static int  dfsdm_set_ch(struct adc_dev_s *dev, uint8_t ch);

#ifdef DFSDM_HAVE_TIMER
static void dfsdm_timstart(struct stm32_dev_s *priv, bool enable);
static int  dfsdm_timinit(struct stm32_dev_s *priv);
#endif

#ifdef DFSDM_HAVE_DMA
static void dfsdm_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                  void *arg);
#endif

static void dfsdm_startconv(struct stm32_dev_s *priv, bool enable);

/* Interrupt Handler */

static int dfsdm_interrupt(struct adc_dev_s *dev, uint32_t regval);
#if defined(CONFIG_STM32L4_DFSDM1_FLT0)
static int dfsdm_flt0_interrupt(int irq, void *context, void *arg);
#endif
#if defined(CONFIG_STM32L4_DFSDM1_FLT1)
static int dfsdm_flt1_interrupt(int irq, void *context, void *arg);
#endif
#if defined(CONFIG_STM32L4_DFSDM1_FLT2)
static int dfsdm_flt2_interrupt(int irq, void *context, void *arg);
#endif
#if defined(CONFIG_STM32L4_DFSDM1_FLT3)
static int dfsdm_flt3_interrupt(int irq, void *context, void *arg);
#endif

/* ADC Driver Methods */

static int  dfsdm_bind(struct adc_dev_s *dev,
                       const struct adc_callback_s *callback);
static void dfsdm_reset(struct adc_dev_s *dev);
static int  dfsdm_setup(struct adc_dev_s *dev);
static void dfsdm_shutdown(struct adc_dev_s *dev);
static void dfsdm_rxint(struct adc_dev_s *dev, bool enable);
static int  dfsdm_ioctl(struct adc_dev_s *dev,
                        int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ADC interface operations */

static const struct adc_ops_s g_adcops =
{
  .ao_bind        = dfsdm_bind,
  .ao_reset       = dfsdm_reset,
  .ao_setup       = dfsdm_setup,
  .ao_shutdown    = dfsdm_shutdown,
  .ao_rxint       = dfsdm_rxint,
  .ao_ioctl       = dfsdm_ioctl,
};

/* DFSDM FLT0 */

#if defined(CONFIG_STM32L4_DFSDM1_FLT0)
static struct stm32_dev_s g_dfsdmpriv0 =
{
  .irq         = STM32L4_IRQ_DFSDM0,
  .isr         = dfsdm_flt0_interrupt,
  .intf        = 0,
  .base        = STM32L4_DFSDM_BASE,
#ifdef DFSDM_HAVE_TIMER
  .trigger     = CONFIG_STM32L4_DFSDM_TIMTRIG,
  .tbase       = DFSDM_TIMER_BASE,
  .extsel      = DFSDM_JEXTSEL_VALUE,
  .pclck       = DFSDM_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32L4_DFSDM_SAMPLE_FREQUENCY,
#endif
#ifdef DFSDM_HAVE_DMA
  .dmachan     = DMACHAN_DFSDM0,
  .hasdma      = true,
#endif
};

static struct adc_dev_s g_dfsdmdev0 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_dfsdmpriv0,
};
#endif

/* DFSDM FLT1 */

#if defined(CONFIG_STM32L4_DFSDM1_FLT1)
static struct stm32_dev_s g_dfsdmpriv1 =
{
  .irq         = STM32L4_IRQ_DFSDM1,
  .isr         = dfsdm_flt1_interrupt,
  .intf        = 1,
  .base        = STM32L4_DFSDM_BASE,
#ifdef DFSDM_HAVE_TIMER
  .trigger     = CONFIG_STM32L4_DFSDM_TIMTRIG,
  .tbase       = DFSDM_TIMER_BASE,
  .extsel      = DFSDM_JEXTSEL_VALUE,
  .pclck       = DFSDM_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32L4_DFSDM_SAMPLE_FREQUENCY,
#endif
#ifdef DFSDM_HAVE_DMA
  .dmachan     = DMACHAN_DFSDM1,
  .hasdma      = true,
#endif
};

static struct adc_dev_s g_dfsdmdev1 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_dfsdmpriv1,
};
#endif

/* DFSDM FLT2 */

#if defined(CONFIG_STM32L4_DFSDM1_FLT2)
static struct stm32_dev_s g_dfsdmpriv2 =
{
  .irq         = STM32L4_IRQ_DFSDM2,
  .isr         = dfsdm_flt2_interrupt,
  .intf        = 0,
  .base        = STM32L4_DFSDM_BASE,
#ifdef DFSDM_HAVE_TIMER
  .trigger     = CONFIG_STM32L4_DFSDM_TIMTRIG,
  .tbase       = DFSDM_TIMER_BASE,
  .extsel      = DFSDM_JEXTSEL_VALUE,
  .pclck       = DFSDM_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32L4_DFSDM_SAMPLE_FREQUENCY,
#endif
#ifdef DFSDM_HAVE_DMA
  .dmachan     = DMACHAN_DFSDM2,
  .hasdma      = true,
#endif
};

static struct adc_dev_s g_dfsdmdev2 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_dfsdmpriv2,
};
#endif

/* DFSDM FLT3 */

#if defined(CONFIG_STM32L4_DFSDM1_FLT3)
static struct stm32_dev_s g_dfsdmpriv3 =
{
  .irq         = STM32L4_IRQ_DFSDM3,
  .isr         = dfsdm_flt3_interrupt,
  .intf        = 0,
  .base        = STM32L4_DFSDM_BASE,
#ifdef DFSDM_HAVE_TIMER
  .trigger     = CONFIG_STM32L4_DFSDM_TIMTRIG,
  .tbase       = DFSDM_TIMER_BASE,
  .extsel      = DFSDM_JEXTSEL_VALUE,
  .pclck       = DFSDM_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32L4_DFSDM_SAMPLE_FREQUENCY,
#endif
#ifdef DFSDM_HAVE_DMA
  .dmachan     = DMACHAN_DFSDM3,
  .hasdma      = true,
#endif
};

static struct adc_dev_s g_dfsdmdev3 =
{
  .ad_ops      = &g_adcops,
  .ad_priv     = &g_dfsdmpriv3,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dfsdm_getreg
 *
 * Description:
 *   Read the value of a DFSDM register.
 *
 * Input Parameters:
 *   priv   - A reference to the DFSDM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

static uint32_t dfsdm_getreg(struct stm32_dev_s *priv, int offset)
{
  return getreg32(priv->base + offset);
}

/****************************************************************************
 * Name: dfsdm_putreg
 *
 * Description:
 *   Write a value to an DFSDM register.
 *
 * Input Parameters:
 *   priv   - A reference to the DFSDM block status
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dfsdm_putreg(struct stm32_dev_s *priv, int offset,
                         uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: dfsdm_modifyreg
 *
 * Description:
 *   Modify the value of an DFSDM register (not atomic).
 *
 * Input Parameters:
 *   priv    - A reference to the DFSDM block status
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void dfsdm_modifyreg(struct stm32_dev_s *priv, int offset,
                            uint32_t clrbits, uint32_t setbits)
{
  dfsdm_putreg(priv, offset,
              (dfsdm_getreg(priv, offset) & ~clrbits) | setbits);
}

/****************************************************************************
 * Name: tim_getreg
 *
 * Description:
 *   Read the value of an DFSDM timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the DFSDM block status
 *   offset - The offset to the register to read
 *
 * Returned Value:
 *   The current contents of the specified register
 *
 ****************************************************************************/

#ifdef DFSDM_HAVE_TIMER
static uint16_t tim_getreg(struct stm32_dev_s *priv, int offset)
{
  return getreg16(priv->tbase + offset);
}
#endif

/****************************************************************************
 * Name: tim_putreg
 *
 * Description:
 *   Write a value to an DFSDM timer register.
 *
 * Input Parameters:
 *   priv   - A reference to the DFSDM block status
 *   offset - The offset to the register to write to
 *   value  - The value to write to the register
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef DFSDM_HAVE_TIMER
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
 *   Modify the value of an DFSDM timer register (not atomic).
 *
 * Input Parameters:
 *   priv    - A reference to the DFSDM block status
 *   offset  - The offset to the register to modify
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef DFSDM_HAVE_TIMER
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
 *   priv - A reference to the DFSDM block status
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef DFSDM_HAVE_TIMER
static void tim_dumpregs(struct stm32_dev_s *priv, const char *msg)
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

/****************************************************************************
 * Name: dfsdm_timstart
 *
 * Description:
 *   Start (or stop) the timer counter
 *
 * Input Parameters:
 *   priv - A reference to the DFSDM block status
 *   enable - True: Start conversion
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef DFSDM_HAVE_TIMER
static void dfsdm_timstart(struct stm32_dev_s *priv, bool enable)
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

/****************************************************************************
 * Name: dfsdm_timinit
 *
 * Description:
 *   Initialize the timer that drivers the DFSDM sampling for this channel
 *   using the pre-calculated timer divider definitions.
 *
 * Input Parameters:
 *   priv - A reference to the DFSDM block status
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef DFSDM_HAVE_TIMER
static int dfsdm_timinit(struct stm32_dev_s *priv)
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

  /* If the timer base address is zero, then this DFSDM was not configured to
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

  dfsdm_modifyreg(priv, FLTCR2_OFFSET(priv),
                DFSDM_FLTCR1_JEXTEN_MASK | DFSDM_FLTCR1_JEXTSEL_MASK,
                DFSDM_FLTCR1_JEXTEN_RISING | priv->extsel);

  /* Configure the timer channel. */

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

  dfsdm_timstart(priv, false);

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

          tim_putreg(priv, STM32L4_GTIM_CCR1_OFFSET,
                    (uint16_t)(reload >> 1));
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

          tim_putreg(priv, STM32L4_GTIM_CCR2_OFFSET,
                    (uint16_t)(reload >> 1));
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

          tim_putreg(priv, STM32L4_GTIM_CCR3_OFFSET,
                    (uint16_t)(reload >> 1));
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

          tim_putreg(priv, STM32L4_GTIM_CCR4_OFFSET,
                    (uint16_t)(reload >> 1));
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

          tim_putreg(priv, STM32L4_GTIM_CCR4_OFFSET,
                    (uint16_t)(reload >> 1));
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

  dfsdm_timstart(priv, true);

  tim_dumpregs(priv, "After starting timers");

  return OK;
}
#endif

/****************************************************************************
 * Name: dfsdm_wdog_enable
 *
 * Description:
 *   Enable the analog watchdog. This is for filter watchdog only, channel
 *   watchdogs are not supported by current implementation.
 *
 ****************************************************************************/

static void dfsdm_wdog_enable(struct stm32_dev_s *priv)
{
  uint32_t regval;

  /* Initialize the Analog watchdog for current channel
   * and switch to analog watchdog interrupt
   */

  regval  = dfsdm_getreg(priv, FLTCR2_OFFSET(priv));
  regval |= DFSDM_FLTCR2_AWDCH(priv->current);
  regval |= DFSDM_INT_AWD;
  regval &= ~(DFSDM_INT_REOC | DFSDM_INT_JEOC);
  dfsdm_putreg(priv, FLTCR2_OFFSET(priv), regval);
}

/****************************************************************************
 * Name: dfsdm_startconv
 *
 * Description:
 *   Start (or stop) the ADC conversion process
 *
 * Input Parameters:
 *   priv - A reference to the DFSDM block status
 *   enable - True: Start conversion
 *
 ****************************************************************************/

static void dfsdm_startconv(struct stm32_dev_s *priv, bool enable)
{
  uint32_t regval;

  ainfo("enable: %d\n", enable ? 1 : 0);

  regval = dfsdm_getreg(priv, FLTCR1_OFFSET(priv));
  if (enable)
    {
      /* Enable the filter */

      regval |= DFSDM_FLTCR1_DFEN;
    }
  else
    {
      /* Disable the filter */

      regval &= ~DFSDM_FLTCR1_DFEN;
    }

  dfsdm_putreg(priv, FLTCR1_OFFSET(priv), regval);

  if (enable)
    {
      /* Start conversion of regular channels */

      regval |= DFSDM_FLTCR1_RSWSTART;
      dfsdm_putreg(priv, FLTCR1_OFFSET(priv), regval);
    }
}

/****************************************************************************
 * Name: dfsdm_rccreset
 *
 * Description:
 *   Deinitializes the DFSDM peripheral registers to their default
 *   reset values.
 *
 * Input Parameters:
 *   priv - A reference to the DFSDM block status
 *   reset - Condition, set or reset
 *
 ****************************************************************************/

static void dfsdm_rccreset(struct stm32_dev_s *priv, bool reset)
{
  irqstate_t flags;
  uint32_t regval;

  /* First must disable interrupts because the APB2RSTR register is used by
   * several different drivers.
   */

  flags = enter_critical_section();

  /* Set or clear the selected bit in the APB2 reset register */

  regval = getreg32(STM32L4_RCC_APB2RSTR);
  if (reset)
    {
      regval |= RCC_APB2RSTR_DFSDMRST;
    }
  else
    {
      regval &= ~RCC_APB2RSTR_DFSDMRST;
    }

  putreg32(regval, STM32L4_RCC_APB2RSTR);
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: dfsdm_enable
 *
 * Description:
 *   Enables the DFSDM peripheral.
 *
 * Input Parameters:
 *   priv - A reference to the DFSDM block status
 *
 ****************************************************************************/

static void dfsdm_enable(struct stm32_dev_s *priv)
{
  uint32_t regval;

  regval = dfsdm_getreg(priv, STM32L4_DFSDM_CH0CFGR1_OFFSET);

  /* Enable DFSMDM */

  regval |= DFSDM_CH0CFGR1_DFSDMEN;
  dfsdm_putreg(priv, STM32L4_DFSDM_CH0CFGR1_OFFSET, regval);
}

/****************************************************************************
 * Name: dfsdm_bind
 *
 * Description:
 *   Bind the upper-half driver callbacks to the lower-half implementation.
 *   This must be called early in order to receive ADC event notifications.
 *
 ****************************************************************************/

static int dfsdm_bind(struct adc_dev_s *dev,
                      const struct adc_callback_s *callback)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: dfsdm_reset
 *
 * Description:
 *   Reset the DFSDM device.  Called early to initialize the hardware. This
 *   is called, before dfsdm_setup() and on error conditions.
 *
 ****************************************************************************/

static void dfsdm_reset(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  /* Enable DFSDM reset state */

  dfsdm_rccreset(priv, true);

  /* Release DFSDM from reset state */

  dfsdm_rccreset(priv, false);
}

/****************************************************************************
 * Name: dfsdm_setup
 *
 * Description:
 *   Configure the DFSDM. This method is called the first time that the ADC
 *   device is opened.  This will occur when the port is first opened.
 *   This setup includes configuring and attaching ADC interrupts.
 *   Interrupts are all disabled upon return.
 *
 ****************************************************************************/

static int dfsdm_setup(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  int ret;
  irqstate_t flags;
  uint32_t clrbits;
  uint32_t setbits;

  /* Attach the interrupt */

  ret = irq_attach(priv->irq, priv->isr, NULL);
  if (ret < 0)
    {
      ainfo("irq_attach failed: %d\n", ret);
      return ret;
    }

  flags = enter_critical_section();

  /* Make sure that the device is in the powered up, reset state. */

  dfsdm_reset(dev);

  /* Initialize the filter settings.  */

  clrbits = 0;
  setbits = DFSDM_FLTFCR_FORD_SINC1 | \
            DFSDM_FLTFCR_FOSR(0) | \
            DFSDM_FLTFCR_IOSR(0);

  dfsdm_modifyreg(priv, FLTFCR_OFFSET(priv), clrbits, setbits);

  setbits = 0;

#ifdef DFSDM_HAVE_DMA
  if (priv->hasdma)
    {
      /* Enable DMA */

      setbits |= DFSDM_FLTCR1_RDMAEN;
    }
#endif

  /* Disable continuous mode and synchronized mode */

  clrbits = DFSDM_FLTCR1_RCONT | DFSDM_FLTCR1_RSYNC;

  /* Set CR1 configuration */

  dfsdm_modifyreg(priv, FLTCR1_OFFSET(priv), clrbits, setbits);

  /* Configuration of the channel conversions */

  dfsdm_set_ch(dev, 0);

#ifdef DFSDM_HAVE_DMA

  /* Enable DMA */

  if (priv->hasdma)
    {
      /* Stop and free DMA if it was started before */

      if (priv->dma != NULL)
        {
          stm32l4_dmastop(priv->dma);
          stm32l4_dmafree(priv->dma);
        }

      priv->dma = stm32l4_dmachannel(priv->dmachan);

      stm32l4_dmasetup(priv->dma,
                       priv->base + FLTRDATAR_OFFSET(priv),
                       (uint32_t)priv->dmabuffer,
                       priv->nchannels,
                       DFSDM_DMA_CONTROL_WORD);

      stm32l4_dmastart(priv->dma, dfsdm_dmaconvcallback, dev, false);
    }
#endif

  dfsdm_enable(priv);

#ifdef DFSDM_HAVE_TIMER

  /* TODO: timers would require injected conversions, which are
   * not implemented at the moment.
   */
#  warning "DFSDM Timer not fully implemented!"

  if (priv->tbase != 0)
    {
      ret = dfsdm_timinit(priv);
      if (ret < 0)
        {
          aerr("ERROR: dfsdm_timinit failed: %d\n", ret);
        }
    }
#endif

  leave_critical_section(flags);

  ainfo("ISR:   0x%08x FCR:    0x%08x CR1:  0x%08x CR2: 0x%08x\n",
        dfsdm_getreg(priv, FLTISR_OFFSET(priv)),
        dfsdm_getreg(priv, FLTFCR_OFFSET(priv)),
        dfsdm_getreg(priv, FLTCR1_OFFSET(priv)),
        dfsdm_getreg(priv, FLTCR2_OFFSET(priv)));

  /* Enable the DFSDM FLT interrupt */

  ainfo("Enable the FLT interrupt: irq=%d\n", priv->irq);
  up_enable_irq(priv->irq);

  return ret;
}

/****************************************************************************
 * Name: dfsdm_shutdown
 *
 * Description:
 *   Disable the DFSDM.  This method is called when the ADC device is closed.
 *   This method reverses the operation the setup method.
 *
 ****************************************************************************/

static void dfsdm_shutdown(struct adc_dev_s *dev)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;

  /* Disable interrupts and detach the FLT interrupt handler */

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);

  /* Disable and reset the DFSDM module */

  dfsdm_reset(dev);
}

/****************************************************************************
 * Name: dfsdm_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static void dfsdm_rxint(struct adc_dev_s *dev, bool enable)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t regval;

  ainfo("intf: %d enable: %d\n", priv->intf, enable ? 1 : 0);

  regval = dfsdm_getreg(priv, FLTCR2_OFFSET(priv));
  if (enable)
    {
      /* Enable end of conversion interrupt */

      regval |= DFSDM_INT_REOC;
    }
  else
    {
      /* Disable all interrupts */

      regval &= ~DFSDM_INT_MASK;
    }

  dfsdm_putreg(priv, FLTCR2_OFFSET(priv), regval);
}

/****************************************************************************
 * Name: dfsdm_set_ch
 *
 * Description:
 *   Sets the DFSDM channel.
 *
 * Input Parameters:
 *   dev - pointer to device structure used by the driver
 *   ch  - ADC channel number + 1. 0 reserved for all configured channels
 *
 * Returned Value:
 *   int - errno
 *
 ****************************************************************************/

static int dfsdm_set_ch(struct adc_dev_s *dev, uint8_t ch)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t regval;
  int i;

  if (ch == 0)
    {
      priv->current   = 0;
      priv->nchannels = priv->cchannels;
    }
  else
    {
      for (i = 0; i < priv->cchannels && priv->chanlist[i] != ch - 1; i++)
        {
        }

      if (i >= priv->cchannels)
        {
          return -ENODEV;
        }

      priv->current   = i;
      priv->nchannels = 1;
    }

  DEBUGASSERT(priv->nchannels <= DFSDM_MAX_CHANNELS);

  /* Select channel for filter.
   * TODO: regular conversion supports one channel only.
   */

  /* Set channel in CR1 configuration */

  regval  = dfsdm_getreg(priv, FLTCR1_OFFSET(priv));
  regval |= DFSDM_FLTCR1_RCH(priv->current);
  dfsdm_putreg(priv, FLTCR1_OFFSET(priv), regval);

  /* Set CHCFGR1 input data configuration */

  regval  = dfsdm_getreg(priv, CHCFGR1_OFFSET(priv));
#ifdef ADC_HAVE_DFSDM
  regval |= DFSDM_CHCFGR1_DATMPX_ADC;
#else
  regval |= DFSDM_CHCFGR1_DATMPX_EXT;

  /* regval |= DFSDM_CHCFGR1_DATMPX_DATINR; */
#endif
  dfsdm_putreg(priv, CHCFGR1_OFFSET(priv), regval);

  /* Enable the channel */

  regval |= DFSDM_CHCFGR1_CHEN;
  dfsdm_putreg(priv, CHCFGR1_OFFSET(priv), regval);

  return OK;
}

/****************************************************************************
 * Name: dfsdm_ioctl
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

static int dfsdm_ioctl(struct adc_dev_s *dev, int cmd, unsigned long arg)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t regval;
  uint32_t tmp;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        dfsdm_startconv(priv, true);
        break;

      case ANIOC_WDOG_UPPER: /* Set watchdog upper threshold */
        {
          regval = dfsdm_getreg(priv, FLTAWLTR_OFFSET(priv));

          /* Verify new upper threshold greater than lower threshold */

          tmp = (regval & DFSDM_AWLTR_AWLT_MASK) >> DFSDM_AWLTR_AWLT_SHIFT;
          if (arg < tmp)
            {
              ret = -EINVAL;
              break;
            }

          /* Set the watchdog threshold register */

          regval = ((arg << DFSDM_AWHTR_AWHT_SHIFT) & DFSDM_AWHTR_AWHT_MASK);

          dfsdm_putreg(priv, FLTAWHTR_OFFSET(priv), regval);

          /* Ensure analog watchdog is enabled */

          dfsdm_wdog_enable(priv);
        }
        break;

      case ANIOC_WDOG_LOWER: /* Set watchdog lower threshold */
        {
          regval = dfsdm_getreg(priv, FLTAWHTR_OFFSET(priv));

          /* Verify new lower threshold less than upper threshold */

          tmp = (regval & DFSDM_AWHTR_AWHT_MASK) >> DFSDM_AWHTR_AWHT_SHIFT;
          if (arg > tmp)
            {
              ret = -EINVAL;
              break;
            }

          /* Set the watchdog threshold register */

          regval = ((arg << DFSDM_AWLTR_AWLT_SHIFT) & DFSDM_AWLTR_AWLT_MASK);
          dfsdm_putreg(priv, FLTAWLTR_OFFSET(priv), regval);

          /* Ensure analog watchdog is enabled */

          dfsdm_wdog_enable(priv);
        }
        break;

      default:
        aerr("ERROR: Unknown cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: dfsdm_interrupt
 *
 * Description:
 *   Common DFSDM interrupt handler.
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/

static int dfsdm_interrupt(struct adc_dev_s *dev, uint32_t isr)
{
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t value;

  /* Identifies the interruption AWD or OVR */

  if ((isr & DFSDM_INT_AWD) != 0)
    {
      value = dfsdm_getreg(priv, FLTAWSR_OFFSET(priv));

      awarn("WARNING: Analog Watchdog, extreme value on channels (0x%x)!\n",
            value);

      /* Clear AWD flags */

      dfsdm_putreg(priv, FLTAWCFR_OFFSET(priv), value);

      /* Stop ADC conversions to avoid continuous interrupts */

      dfsdm_startconv(priv, false);
    }

  if ((isr & DFSDM_INT_ROVR) != 0)
    {
      awarn("WARNING: Regular conversion overrun has occurred!\n");

      /* Clear overrun flag */

      dfsdm_putreg(priv, FLTAWCFR_OFFSET(priv), DFSDM_INT_ROVR);
    }

#if 0
  if ((isr & DFSDM_INT_JOVR) != 0)
    {
      awarn("WARNING: Injected conversion overrun has occurred!\n");

      /* Clear overrun flag */

      dfsdm_putreg(priv, FLTAWCFR_OFFSET(priv), DFSDM_INT_JOVR);
    }
#endif

  /* EOC: End of conversion */

  if ((isr & DFSDM_INT_REOC) != 0)
    {
      /* Read the converted value and clear EOC bit
       * (It is cleared by reading the FLTRDATAR)
       */

      value = dfsdm_getreg(priv, FLTRDATAR_OFFSET(priv));
      value = (value & DFSDM_FLTRDATAR_RDATA_MASK) >>
               DFSDM_FLTRDATAR_RDATA_SHIFT;

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

/****************************************************************************
 * Name: dfsdm_flt0_interrupt
 *
 * Description:
 *   DFSDM filter interrupt handler
 *
 ****************************************************************************/

#if defined(CONFIG_STM32L4_DFSDM1_FLT0)
static int dfsdm_flt0_interrupt(int irq, void *context, void *arg)
{
  uint32_t regval;
  uint32_t pending;

  regval = getreg32(STM32L4_DFSDM_FLTISR(0));
  pending = regval & DFSDM_ISR_MASK;
  if (pending != 0)
    {
      dfsdm_interrupt(&g_dfsdmdev0, regval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: dfsdm_flt1_interrupt
 *
 * Description:
 *   DFSDM filter interrupt handler
 *
 ****************************************************************************/

#if defined(CONFIG_STM32L4_DFSDM1_FLT1)
static int dfsdm_flt1_interrupt(int irq, void *context, void *arg)
{
  uint32_t regval;
  uint32_t pending;

  regval = getreg32(STM32L4_DFSDM_FLTISR(1));
  pending = regval & DFSDM_ISR_MASK;
  if (pending != 0)
    {
      dfsdm_interrupt(&g_dfsdmdev1, regval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: dfsdm_flt2_interrupt
 *
 * Description:
 *   DFSDM filter interrupt handler
 *
 ****************************************************************************/

#if defined(CONFIG_STM32L4_DFSDM1_FLT2)
static int dfsdm_flt2_interrupt(int irq, void *context, void *arg)
{
  uint32_t regval;
  uint32_t pending;

  regval = getreg32(STM32L4_DFSDM_FLTISR(2));
  pending = regval & DFSDM_ISR_MASK;
  if (pending != 0)
    {
      dfsdm_interrupt(&g_dfsdmdev2, regval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: dfsdm_flt3_interrupt
 *
 * Description:
 *   DFSDM filter interrupt handler
 *
 ****************************************************************************/

#if defined(CONFIG_STM32L4_DFSDM1_FLT3)
static int dfsdm_flt3_interrupt(int irq, void *context, void *arg)
{
  uint32_t regval;
  uint32_t pending;

  regval = getreg32(STM32L4_DFSDM_FLTISR(3));
  pending = regval & DFSDM_ISR_MASK;
  if (pending != 0)
    {
      dfsdm_interrupt(&g_dfsdmdev3, regval);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: dfsdm_dmaconvcallback
 *
 * Description:
 *   Callback for DMA.  Called from the DMA transfer complete interrupt after
 *   all channels have been converted and transferred with DMA.
 *
 * Input Parameters:
 *
 *   handle - handle to DMA
 *   isr -
 *   arg - device
 *
 * Returned Value:
 *
 ****************************************************************************/

#ifdef DFSDM_HAVE_DMA
static void dfsdm_dmaconvcallback(DMA_HANDLE handle,
                                  uint8_t isr, void *arg)
{
  struct adc_dev_s   *dev  = (struct adc_dev_s *)arg;
  struct stm32_dev_s *priv = (struct stm32_dev_s *)dev->ad_priv;
  uint32_t value;
  int i;

  /* Verify that the upper-half driver has bound its callback functions */

  if (priv->cb != NULL)
    {
      DEBUGASSERT(priv->cb->au_receive != NULL);

      for (i = 0; i < priv->nchannels; i++)
        {
          value = priv->dmabuffer[priv->current];
          value = (value & DFSDM_FLTRDATAR_RDATA_MASK) >>
                   DFSDM_FLTRDATAR_RDATA_SHIFT;

          priv->cb->au_receive(dev, priv->chanlist[priv->current], value);
          priv->current++;
          if (priv->current >= priv->nchannels)
            {
              /* Restart the conversion sequence from the beginning */

              priv->current = 0;
            }
        }
    }

  /* Restart DMA for the next conversion series */

  dfsdm_modifyreg(priv, FLTCR1_OFFSET(priv), DFSDM_FLTCR1_RDMAEN, 0);
  dfsdm_modifyreg(priv, FLTCR1_OFFSET(priv), 0, DFSDM_FLTCR1_RDMAEN);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_dfsdm_initialize
 *
 * Description:
 *   Initialize the DFSDM.
 *
 * Input Parameters:
 *   intf      - Could be {0,1,2,3} for DFSDM1 FLT0, FLT1, FLT2, FLT3
 *   chanlist  - The list of channels
 *   cchannels - Number of channels
 *
 * Returned Value:
 *   Valid ADC device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

struct adc_dev_s *stm32l4_dfsdm_initialize(int intf,
                                           const uint8_t *chanlist,
                                           int cchannels)
{
  struct adc_dev_s   *dev;
  struct stm32_dev_s *priv;

  ainfo("intf: %d cchannels: %d\n", intf, cchannels);

  switch (intf)
    {
#if defined(CONFIG_STM32L4_DFSDM1_FLT0)
      case 0:
        ainfo("DFSDM FLT0 selected\n");
        dev = &g_dfsdmdev0;
        break;
#endif
#if defined(CONFIG_STM32L4_DFSDM1_FLT1)
      case 1:
        ainfo("DFSDM FLT1 selected\n");
        dev = &g_dfsdmdev1;
        break;
#endif
#if defined(CONFIG_STM32L4_DFSDM1_FLT2)
      case 2:
        ainfo("DFSDM FLT2 selected\n");
        dev = &g_dfsdmdev2;
        break;
#endif
#if defined(CONFIG_STM32L4_DFSDM1_FLT3)
      case 3:
        ainfo("DFSDM FLT3 selected\n");
        dev = &g_dfsdmdev3;
        break;
#endif
      default:
        aerr("ERROR: No DFSDM interface defined\n");
        return NULL;
    }

  /* Configure the selected DFSDM */

  priv = (struct stm32_dev_s *)dev->ad_priv;
  priv->cb = NULL;

  DEBUGASSERT(cchannels <= DFSDM_MAX_CHANNELS);

  /* TODO: regular conversion supports one channel only. */

#if 0
  if (cchannels > DFSDM_MAX_CHANNELS)
    {
      cchannels = DFSDM_MAX_CHANNELS;
    }
#else
  if (cchannels > 1)
    {
      cchannels = 1;
#  warning "Only one channel supported at the moment!"
    }
#endif

  priv->cchannels = cchannels;
  memcpy(priv->chanlist, chanlist, cchannels);

  return dev;
}

#endif /* CONFIG_STM32L4_DFSDM */
#endif /* CONFIG_ADC */
