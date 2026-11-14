/****************************************************************************
 * arch/arm/src/common/stm32/stm32_ptp_pps.c
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

/* PPS disciplined PTP clock on a 32-bit STM32 general purpose timer.
 *
 * TIM2 or TIM5 runs free at the full timer clock (for example 84 MHz on an
 * STM32F407, a resolution of 11.9 ns) and is the counter of the PTP_PPS
 * software clock.  One channel of the same timer is in input capture mode
 * on the pin that receives the PPS of the GNSS receiver: the hardware
 * copies the counter into CCRx on the PPS edge, so the time stamp is exact
 * to one timer clock and does not depend on the interrupt latency, on
 * critical sections or on the system load.  The interrupt only has to read
 * CCRx before the next edge.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/debug.h>
#include <nuttx/timers/ptp_clock.h>
#include <nuttx/timers/ptp_pps.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"
#include "stm32_tim_clk.h"
#include "stm32_ptp_pps.h"

#ifdef CONFIG_STM32_PTP_PPS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PPS_CHAN  CONFIG_STM32_PTP_PPS_CHANNEL

#if defined(CONFIG_STM32_PTP_PPS_TIM2)
#  define PPS_TIM_BASE    STM32_TIM2_BASE
#  define PPS_TIM_IRQ     STM32_IRQ_TIM2
#  define PPS_TIM_CLKIN   STM32_TIM2_CLKIN
#  define PPS_RCC_EN_REG  STM32_RCC_TIM2_EN_REG
#  define PPS_RCC_EN      STM32_RCC_TIM2_EN
#  if PPS_CHAN == 1 && defined(GPIO_TIM2_CH1IN)
#    define PPS_GPIO      GPIO_TIM2_CH1IN
#  elif PPS_CHAN == 2 && defined(GPIO_TIM2_CH2IN)
#    define PPS_GPIO      GPIO_TIM2_CH2IN
#  elif PPS_CHAN == 3 && defined(GPIO_TIM2_CH3IN)
#    define PPS_GPIO      GPIO_TIM2_CH3IN
#  elif PPS_CHAN == 4 && defined(GPIO_TIM2_CH4IN)
#    define PPS_GPIO      GPIO_TIM2_CH4IN
#  endif
#elif defined(CONFIG_STM32_PTP_PPS_TIM5)
#  define PPS_TIM_BASE    STM32_TIM5_BASE
#  define PPS_TIM_IRQ     STM32_IRQ_TIM5
#  define PPS_TIM_CLKIN   STM32_TIM5_CLKIN
#  define PPS_RCC_EN_REG  STM32_RCC_TIM5_EN_REG
#  define PPS_RCC_EN      STM32_RCC_TIM5_EN
#  if PPS_CHAN == 1 && defined(GPIO_TIM5_CH1IN)
#    define PPS_GPIO      GPIO_TIM5_CH1IN
#  elif PPS_CHAN == 2 && defined(GPIO_TIM5_CH2IN)
#    define PPS_GPIO      GPIO_TIM5_CH2IN
#  elif PPS_CHAN == 3 && defined(GPIO_TIM5_CH3IN)
#    define PPS_GPIO      GPIO_TIM5_CH3IN
#  elif PPS_CHAN == 4 && defined(GPIO_TIM5_CH4IN)
#    define PPS_GPIO      GPIO_TIM5_CH4IN
#  endif
#else
#  error "No timer selected for STM32_PTP_PPS"
#endif

#ifndef PPS_GPIO
#  error "The board must define GPIO_TIMx_CHyIN for the PPS input pin"
#endif

/* Registers and bits of the capture channel */

#define PPS_REG(off)      (PPS_TIM_BASE + (off))
#define PPS_CCMR_OFFSET   (PPS_CHAN <= 2 ? STM32_GTIM_CCMR1_OFFSET : \
                                           STM32_GTIM_CCMR2_OFFSET)
#define PPS_CCMR_SHIFT    (((PPS_CHAN - 1) & 1) * 8)
#define PPS_CCMR_MASK     (0xffu << PPS_CCMR_SHIFT)
#define PPS_CCMR_VAL      ((GTIM_CCMR_CCS_CCIN1 |                       \
                            (CONFIG_STM32_PTP_PPS_FILTER << 4))         \
                           << PPS_CCMR_SHIFT)
#define PPS_CCR_OFFSET    (STM32_GTIM_CCR1_OFFSET + (PPS_CHAN - 1) * 4)
#define PPS_CCER_SHIFT    ((PPS_CHAN - 1) * 4)
#define PPS_CCER_E        (GTIM_CCER_CC1E << PPS_CCER_SHIFT)
#define PPS_CCER_P        (GTIM_CCER_CC1P << PPS_CCER_SHIFT)
#define PPS_CCER_NP       (GTIM_CCER_CC1NP << PPS_CCER_SHIFT)
#define PPS_CCER_MASK     (PPS_CCER_E | PPS_CCER_P | PPS_CCER_NP)
#define PPS_DIER_IE       (GTIM_DIER_CC1IE << (PPS_CHAN - 1))
#define PPS_SR_IF         (GTIM_SR_CC1IF << (PPS_CHAN - 1))
#define PPS_SR_OF         (GTIM_SR_CC1OF << (PPS_CHAN - 1))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_ptp_pps_s
{
  struct ptp_pps_lowerhalf_s lower; /* Must be first */
  unsigned int               edge;  /* Edges being captured */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint32_t stm32_ptp_pps_read(FAR struct ptp_pps_lowerhalf_s *lower);
static int stm32_ptp_pps_enable(FAR struct ptp_pps_lowerhalf_s *lower,
                                unsigned int index, unsigned int flags,
                                bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ptp_pps_ops_s g_stm32_ptp_pps_ops =
{
  stm32_ptp_pps_read,   /* read */
  stm32_ptp_pps_enable, /* enable */
};

static struct stm32_ptp_pps_s g_stm32_ptp_pps;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ptp_pps_read
 ****************************************************************************/

static uint32_t stm32_ptp_pps_read(FAR struct ptp_pps_lowerhalf_s *lower)
{
  return getreg32(PPS_REG(STM32_GTIM_CNT_OFFSET));
}

/****************************************************************************
 * Name: stm32_ptp_pps_enable
 ****************************************************************************/

static int stm32_ptp_pps_enable(FAR struct ptp_pps_lowerhalf_s *lower,
                                unsigned int index, unsigned int flags,
                                bool on)
{
  FAR struct stm32_ptp_pps_s *priv = (FAR struct stm32_ptp_pps_s *)lower;
  irqstate_t irqflags;
  uint32_t ccer;

  if (index != 0)
    {
      return -EINVAL;
    }

  irqflags = enter_critical_section();

  /* Stop the capture while the channel is reconfigured */

  modifyreg32(PPS_REG(STM32_GTIM_DIER_OFFSET), PPS_DIER_IE, 0);
  modifyreg32(PPS_REG(STM32_GTIM_CCER_OFFSET), PPS_CCER_E, 0);

  if (on)
    {
      /* CCxS = 01: capture TIx, ICxPSC = 0: every edge, ICxF: filter */

      modifyreg32(PPS_REG(PPS_CCMR_OFFSET), PPS_CCMR_MASK, PPS_CCMR_VAL);

      /* Edge: CCxNP/CCxP = 00 rising, 01 falling, 11 both */

      if ((flags & PTP_EXTTS_EDGES) == PTP_EXTTS_EDGES)
        {
          ccer = PPS_CCER_P | PPS_CCER_NP;
        }
      else if (flags & PTP_FALLING_EDGE)
        {
          ccer = PPS_CCER_P;
        }
      else
        {
          ccer = 0;
          flags = PTP_RISING_EDGE;
        }

      priv->edge = flags & PTP_EXTTS_EDGES;

      modifyreg32(PPS_REG(STM32_GTIM_CCER_OFFSET), PPS_CCER_MASK,
                  ccer | PPS_CCER_E);

      /* Clear stale flags (rc_w0 bits), then enable the interrupt */

      putreg32(~(PPS_SR_IF | PPS_SR_OF), PPS_REG(STM32_GTIM_SR_OFFSET));
      modifyreg32(PPS_REG(STM32_GTIM_DIER_OFFSET), 0, PPS_DIER_IE);
    }

  leave_critical_section(irqflags);
  return OK;
}

/****************************************************************************
 * Name: stm32_ptp_pps_interrupt
 *
 * Description:
 *   Capture interrupt.  The counter was latched by the hardware at the
 *   edge, so the latency of this handler does not matter.
 *
 ****************************************************************************/

static int stm32_ptp_pps_interrupt(int irq, FAR void *context,
                                   FAR void *arg)
{
  FAR struct stm32_ptp_pps_s *priv = arg;
  uint32_t sr = getreg32(PPS_REG(STM32_GTIM_SR_OFFSET));
  uint32_t ccr;

  if ((sr & PPS_SR_IF) == 0)
    {
      return OK;
    }

  /* Reading CCRx clears CCxIF */

  ccr = getreg32(PPS_REG(PPS_CCR_OFFSET));

  if (sr & PPS_SR_OF)
    {
      /* Another edge came before the previous one was read: CCRx holds
       * the latest one.  Not expected with a 1 Hz PPS.
       */

      putreg32(~PPS_SR_OF, PPS_REG(STM32_GTIM_SR_OFFSET));
      ptpwarn("WARNING: PPS overcapture\n");
    }

  ptp_pps_capture(&priv->lower, 0,
                  priv->edge == PTP_EXTTS_EDGES ? 0 : priv->edge, ccr);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ptp_pps_initialize
 *
 * Description:
 *   Start the timer and register the PPS disciplined PTP clock as
 *   /dev/ptp<devno>.
 *
 ****************************************************************************/

int stm32_ptp_pps_initialize(int devno)
{
  FAR struct stm32_ptp_pps_s *priv = &g_stm32_ptp_pps;
  int ret;

  /* Clock the timer and connect the PPS pin */

  modifyreg32(PPS_RCC_EN_REG, 0, PPS_RCC_EN);
  stm32_configgpio(PPS_GPIO);

  /* Free running up counter at the full timer clock: PSC = 0,
   * ARR = 0xffffffff.  UG loads the prescaler and clears the counter.
   */

  putreg32(0, PPS_REG(STM32_GTIM_CR1_OFFSET));
  putreg32(0, PPS_REG(STM32_GTIM_DIER_OFFSET));
  putreg32(0, PPS_REG(STM32_GTIM_PSC_OFFSET));
  putreg32(UINT32_MAX, PPS_REG(STM32_GTIM_ARR_OFFSET));
  putreg32(GTIM_EGR_UG, PPS_REG(STM32_GTIM_EGR_OFFSET));
  putreg32(0, PPS_REG(STM32_GTIM_SR_OFFSET));
  putreg32(GTIM_CR1_CEN, PPS_REG(STM32_GTIM_CR1_OFFSET));

  ret = irq_attach(PPS_TIM_IRQ, stm32_ptp_pps_interrupt, priv);
  if (ret < 0)
    {
      return ret;
    }

  /* The default priority is enough: the edge is latched by hardware, the
   * handler only has to run before the next edge.
   */

  up_enable_irq(PPS_TIM_IRQ);

  priv->lower.ops      = &g_stm32_ptp_pps_ops;
  priv->lower.freq     = PPS_TIM_CLKIN;
  priv->lower.n_ext_ts = 1;

  ret = ptp_pps_register(&priv->lower, devno);
  if (ret < 0)
    {
      up_disable_irq(PPS_TIM_IRQ);
      irq_detach(PPS_TIM_IRQ);
      return ret;
    }

  ptpinfo("PTP PPS clock on TIM%d CH%d, %lu Hz\n",
          PPS_TIM_BASE == STM32_TIM2_BASE ? 2 : 5, PPS_CHAN,
          (unsigned long)PPS_TIM_CLKIN);
  return OK;
}

#endif /* CONFIG_STM32_PTP_PPS */
