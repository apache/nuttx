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

#include "arm_internal.h"
#include "arm_arch.h"

#include "chip.h"
#include "stm32_rcc.h"
#include "stm32_tim.h"
#include "stm32_dma.h"
#include "stm32_adc.h"

/* ADC "upper half" support must be enabled */

#ifdef CONFIG_ADC

/* Some ADC peripheral must be enabled */

#if defined(CONFIG_STM32F7_ADC1) || defined(CONFIG_STM32F7_ADC2) || \
    defined(CONFIG_STM32F7_ADC3)

/* This implementation is for the STM32 F7[4-7] only */

#if defined(CONFIG_STM32F7_STM32F72XX) || defined(CONFIG_STM32F7_STM32F73XX) || \
    defined(CONFIG_STM32F7_STM32F74XX) || defined(CONFIG_STM32F7_STM32F75XX) || \
    defined(CONFIG_STM32F7_STM32F76XX) || defined(CONFIG_STM32F7_STM32F77XX)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RCC reset ****************************************************************/

#define STM32_RCC_RSTR   STM32_RCC_APB2RSTR
#define RCC_RSTR_ADC1RST RCC_APB2RSTR_ADCRST
#define RCC_RSTR_ADC2RST RCC_APB2RSTR_ADCRST
#define RCC_RSTR_ADC3RST RCC_APB2RSTR_ADCRST

/* ADC interrupts ***********************************************************/

#define STM32_ADC_DMAREG_OFFSET    STM32_ADC_CR2_OFFSET
#define ADC_DMAREG_DMA             ADC_CR2_DMA
#define STM32_ADC_EXTREG_OFFSET    STM32_ADC_CR2_OFFSET
#define ADC_EXTREG_EXTSEL_MASK     ADC_CR2_EXTSEL_MASK
#define STM32_ADC_ISR_OFFSET       STM32_ADC_SR_OFFSET
#define STM32_ADC_IER_OFFSET       STM32_ADC_CR1_OFFSET
#define ADC_ISR_EOC                ADC_SR_EOC
#define ADC_IER_EOC                ADC_CR1_EOCIE
#define ADC_ISR_AWD                ADC_SR_AWD
#define ADC_IER_AWD                ADC_CR1_AWDIE
#define ADC_ISR_JEOC               ADC_SR_JEOC
#define ADC_IER_JEOC               ADC_CR1_JEOCIE
#define ADC_EXTREG_EXTEN_MASK      ADC_CR2_EXTEN_MASK
#define ADC_EXTREG_EXTEN_NONE      ADC_CR2_EXTEN_NONE
#define ADC_EXTREG_EXTEN_DEFAULT   ADC_CR2_EXTEN_RISING
#define ADC_ISR_OVR                ADC_SR_OVR
#define ADC_IER_OVR                ADC_CR1_OVRIE

#define ADC_ISR_ALLINTS (ADC_ISR_EOC | ADC_ISR_AWD | ADC_ISR_JEOC | \
                         ADC_ISR_OVR)
#define ADC_IER_ALLINTS (ADC_IER_EOC | ADC_IER_AWD | ADC_IER_JEOC | \
                         ADC_IER_OVR)

/* ADC Channels/DMA *********************************************************/

/* The maximum number of channels that can be sampled.  If DMA support is
 * not enabled, then only a single channel can be sampled.  Otherwise,
 * data overruns would occur.
 */

#define ADC_MAX_CHANNELS_DMA   16
#define ADC_MAX_CHANNELS_NODMA 1

#ifdef ADC_HAVE_DMA
#  ifndef CONFIG_STM32F7_DMA2
#    error "STM32F7 ADC DMA support requires CONFIG_STM32F7_DMA2"
#  endif
#endif

#ifdef ADC_HAVE_DMA
#  define ADC_MAX_SAMPLES ADC_MAX_CHANNELS_DMA
#else
#  define ADC_MAX_SAMPLES ADC_MAX_CHANNELS_NODMA
#endif

#define ADC_DMA_CONTROL_WORD (DMA_SCR_MSIZE_16BITS | \
                                DMA_SCR_PSIZE_16BITS | \
                                DMA_SCR_MINC | \
                                DMA_SCR_CIRC | \
                                DMA_SCR_DIR_P2M)

/* DMA channels and interface values */

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

static int adc_interrupt(FAR struct adc_dev_s *dev);
static int adc123_interrupt(int irq, FAR void *context, FAR void *arg);

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
                            int last,
                            int offset);
static int      adc_set_ch(FAR struct adc_dev_s *dev, uint8_t ch);
static bool     adc_internal(FAR struct stm32_dev_s * priv);

#ifdef ADC_HAVE_TIMER
static void adc_timstart(FAR struct stm32_dev_s *priv, bool enable);
static int  adc_timinit(FAR struct stm32_dev_s *priv);
#endif

#ifdef ADC_HAVE_DMA
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                FAR void *arg);
#endif

static void adc_startconv(FAR struct stm32_dev_s *priv, bool enable);

#ifdef CONFIG_PM
static int adc_pm_prepare(struct pm_callback_s *cb, int domain,
                          enum pm_state_e state);
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

/* ADC1 state */

#ifdef CONFIG_STM32F7_ADC1
static struct stm32_dev_s g_adcpriv1 =
{
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
  .intf        = 1,
  .base        = STM32_ADC1_BASE,
#ifdef ADC1_HAVE_TIMER
  .trigger     = CONFIG_STM32F7_ADC1_TIMTRIG,
  .tbase       = ADC1_TIMER_BASE,
  .extsel      = ADC1_EXTSEL_VALUE,
  .pclck       = ADC1_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32F7_ADC1_SAMPLE_FREQUENCY,
#endif
#ifdef ADC1_HAVE_DMA
  .dmachan     = ADC1_DMA_CHAN,
  .hasdma      = true,
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

#ifdef CONFIG_STM32F7_ADC2
static struct stm32_dev_s g_adcpriv2 =
{
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
  .intf        = 2,
  .base        = STM32_ADC2_BASE,
#ifdef ADC2_HAVE_TIMER
  .trigger     = CONFIG_STM32F7_ADC2_TIMTRIG,
  .tbase       = ADC2_TIMER_BASE,
  .extsel      = ADC2_EXTSEL_VALUE,
  .pclck       = ADC2_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32F7_ADC2_SAMPLE_FREQUENCY,
#endif
#ifdef ADC2_HAVE_DMA
  .dmachan     = ADC2_DMA_CHAN,
  .hasdma      = true,
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

#ifdef CONFIG_STM32F7_ADC3
static struct stm32_dev_s g_adcpriv3 =
{
  .irq         = STM32_IRQ_ADC,
  .isr         = adc123_interrupt,
  .intf        = 3,
  .base        = STM32_ADC3_BASE,
#ifdef ADC3_HAVE_TIMER
  .trigger     = CONFIG_STM32F7_ADC3_TIMTRIG,
  .tbase       = ADC3_TIMER_BASE,
  .extsel      = ADC3_EXTSEL_VALUE,
  .pclck       = ADC3_TIMER_PCLK_FREQUENCY,
  .freq        = CONFIG_STM32F7_ADC3_SAMPLE_FREQUENCY,
#endif
#ifdef ADC3_HAVE_DMA
  .dmachan     = ADC3_DMA_CHAN,
  .hasdma      = true,
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

  ainfo("Initializing timers extsel = 0x%08" PRIx32 "\n", priv->extsel);
  adc_modifyreg(priv, STM32_ADC_EXTREG_OFFSET,
                ADC_EXTREG_EXTEN_MASK | ADC_EXTREG_EXTSEL_MASK,
                ADC_EXTREG_EXTEN_DEFAULT | priv->extsel);

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
#ifdef CONFIG_STM32F7_ADC1
      case 1:
        adcbit = RCC_RSTR_ADC1RST;
        break;
#endif
#ifdef CONFIG_STM32F7_ADC2
      case 2:
        adcbit = RCC_RSTR_ADC2RST;
        break;
#endif
#ifdef CONFIG_STM32F7_ADC3
      case 3:
        adcbit = RCC_RSTR_ADC3RST;
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
 * Name: adc_dmacovcallback
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
static void adc_dmaconvcallback(DMA_HANDLE handle, uint8_t isr,
                                FAR void *arg)
{
  FAR struct adc_dev_s   *dev  = (FAR struct adc_dev_s *)arg;
  FAR struct stm32_dev_s *priv = (FAR struct stm32_dev_s *)dev->ad_priv;
  int i;

  up_invalidate_dcache((uintptr_t)priv->dmabuffer,
                       (uintptr_t)priv->dmabuffer + sizeof(priv->dmabuffer));

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
 *   Reset the ADC device.  Called early to initialize the hardware. This
 *   is called, before adc_setup() and on error conditions.
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

  /* Enable ADC reset state */

  adc_rccreset(priv, true);

  /* Release ADC from reset state */

  adc_rccreset(priv, false);

  /* Initialize the watchdog high threshold register */

  adc_putreg(priv, STM32_ADC_HTR_OFFSET, 0x00000fff);

  /* Initialize the watchdog low threshold register */

  adc_putreg(priv, STM32_ADC_LTR_OFFSET, 0x00000000);

  /* Initialize the same sample time for each ADC.
   * During sample cycles channel selection bits must remain unchanged.
   */

  adc_putreg(priv, STM32_ADC_SMPR1_OFFSET, ADC_SMPR1_DEFAULT);
  adc_putreg(priv, STM32_ADC_SMPR2_OFFSET, ADC_SMPR2_DEFAULT);

  /* Enable the analog watchdog */

  clrbits = ADC_CR1_AWDCH_MASK;
  setbits = ADC_CR1_AWDEN | (priv->chanlist[0] << ADC_CR1_AWDCH_SHIFT);

  /* Set the resolution of the conversion */

  clrbits |= ADC_CR1_RES_MASK;
  setbits |= ADC_CR1_RES_12BIT;

#ifdef ADC_HAVE_DMA
  if (priv->hasdma)
    {
      setbits |= ADC_CR1_SCAN;
    }
#endif

  /* Enable interrupt flags, but disable overrun interrupt */

  clrbits |= ADC_IER_OVR;
  setbits |= ADC_IER_ALLINTS & ~ADC_IER_OVR;

  /* Set CR1 configuration */

  adc_modifyreg(priv, STM32_ADC_CR1_OFFSET, clrbits, setbits);

  /* Disable continuous mode and set align to right */

  clrbits = ADC_CR2_CONT | ADC_CR2_ALIGN;
  setbits = 0;

  /* Disable external trigger for regular channels */

  clrbits |= ADC_EXTREG_EXTEN_MASK;
  setbits |= ADC_EXTREG_EXTEN_NONE;

#ifdef ADC_HAVE_DMA
  if (priv->hasdma)
    {
      setbits |= ADC_CR2_DMA;
    }
#endif

  /* Set CR2 configuration */

  adc_modifyreg(priv, STM32_ADC_CR2_OFFSET, clrbits, setbits);

  /* Configuration of the channel conversions */

  adc_set_ch(dev, 0);

  /* ADC CCR configuration */

  clrbits = ADC_CCR_ADCPRE_MASK | ADC_CCR_TSVREFE;
  setbits = ADC_CCR_ADCPRE_DIV;

  if (adc_internal(priv))
    {
      setbits |= ADC_CCR_TSVREFE;
    }

  clrbits |= ADC_CCR_MULTI_MASK | ADC_CCR_DELAY_MASK | ADC_CCR_DDS |
             ADC_CCR_DMA_MASK | ADC_CCR_VBATE;
  setbits |= ADC_CCR_MULTI_NONE | ADC_CCR_DMA_DISABLED;

  stm32_modifyreg32(STM32_ADC_CCR, clrbits, setbits);

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
#ifndef CONFIG_STM32F7_ADC_NO_STARTUP_CONV
  else
#endif
#endif
#ifndef CONFIG_STM32F7_ADC_NO_STARTUP_CONV
    {
      adc_startconv(priv, true);
    }
#endif

  leave_critical_section(flags);

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

  ainfo("CCR:  0x%08" PRIx32 "\n", getreg32(STM32_ADC_CCR));
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

  ainfo("intf: %d enable: %d\n", priv->intf, enable ? 1 : 0);

  if (enable)
    {
      /* Enable the end-of-conversion ADC and analog watchdog interrupts */

      adc_modifyreg(priv, STM32_ADC_IER_OFFSET, 0, ADC_IER_ALLINTS);
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

static uint32_t adc_sqrbits(FAR struct stm32_dev_s *priv, int first,
                            int last,
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
 * Name: adc_internal
 ****************************************************************************/

static bool adc_internal(FAR struct stm32_dev_s * priv)
{
  int i;

  if (priv->intf == 1)
    {
      for (i  = 0; i < priv->nchannels; i++)
        {
          if (priv->chanlist[i] > ADC_LAST_EXTERNAL_CHAN)
            {
              return true;
            }
        }
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

  bits = adc_sqrbits(priv, ADC_SQR3_FIRST, ADC_SQR3_LAST,
                     ADC_SQR3_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR3_OFFSET, ~ADC_SQR3_RESERVED, bits);

  bits = adc_sqrbits(priv, ADC_SQR2_FIRST, ADC_SQR2_LAST,
                     ADC_SQR2_SQ_OFFSET);
  adc_modifyreg(priv, STM32_ADC_SQR2_OFFSET, ~ADC_SQR2_RESERVED, bits);

  bits = ((uint32_t)priv->nchannels - 1) << ADC_SQR1_L_SHIFT |
         adc_sqrbits(priv, ADC_SQR1_FIRST, ADC_SQR1_LAST,
                     ADC_SQR1_SQ_OFFSET);
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
        {
          adc_startconv(priv, true);
        }
        break;

      case ANIOC_GET_NCHANNELS:
        {
          /* Return the number of configured channels */

          ret = priv->cchannels;
        }
        break;

      default:
        {
          aerr("ERROR: Unknown cmd: %d\n", cmd);
          ret = -ENOTTY;
        }
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

  regval &= ~pending;
  adc_putreg(priv, STM32_ADC_ISR_OFFSET, regval);
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

static int adc123_interrupt(int irq, FAR void *context, FAR void *arg)
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_initialize
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
 ****************************************************************************/

struct adc_dev_s *stm32_adc_initialize(int intf, FAR const uint8_t *chanlist,
                                       int cchannels)
{
  FAR struct adc_dev_s   *dev;
  FAR struct stm32_dev_s *priv;

  ainfo("intf: %d cchannels: %d\n", intf, cchannels);

  switch (intf)
    {
#ifdef CONFIG_STM32F7_ADC1
      case 1:
        ainfo("ADC1 selected\n");
        dev = &g_adcdev1;
        break;
#endif
#ifdef CONFIG_STM32F7_ADC2
      case 2:
        ainfo("ADC2 selected\n");
        dev = &g_adcdev2;
        break;
#endif
#ifdef CONFIG_STM32F7_ADC3
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

  priv     = (FAR struct stm32_dev_s *)dev->ad_priv;
  priv->cb = NULL;

  DEBUGASSERT(cchannels <= ADC_MAX_SAMPLES);
  if (cchannels > ADC_MAX_SAMPLES)
    {
      cchannels = ADC_MAX_SAMPLES;
    }

  priv->cchannels = cchannels;

  memcpy(priv->chanlist, chanlist, cchannels);

  return dev;
}

#endif /* CONFIG_STM32F7_STM32F74XX */
#endif /* CONFIG_STM32F7_ADC1 || CONFIG_STM32F7_ADC2 ||
        * CONFIG_STM32F7_ADC3
        */
#endif /* CONFIG_ADC */
