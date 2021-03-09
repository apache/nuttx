/****************************************************************************
 * arch/arm/src/stm32f7/stm32_capture.c
 *
 *   Copyright (C) 2015 Bouteville Pierre-Noel. All rights reserved.
 *   Author: Bouteville Pierre-Noel <pnb990@gmail.com>
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "arm_arch.h"

#include "stm32_gpio.h"
#include "stm32_capture.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Configuration ************************************************************/

#if defined(GPIO_TIM1_CH1IN) || defined(GPIO_TIM2_CH1IN) || \
    defined(GPIO_TIM3_CH1IN) || defined(GPIO_TIM4_CH1IN) || \
    defined(GPIO_TIM5_CH1IN) || defined(GPIO_TIM8_CH1IN) || \
    defined(GPIO_TIM9_CH1IN) || defined(GPIO_TIM10_CH1IN) || \
    defined(GPIO_TIM11_CH1IN) || defined(GPIO_TIM12_CH1IN) || \
    defined(GPIO_TIM13_CH1IN) || defined(GPIO_TIM14_CH1IN)
#  define HAVE_CH1IN 1
#endif

#if defined(GPIO_TIM1_CH2IN) || defined(GPIO_TIM2_CH2IN) || \
    defined(GPIO_TIM3_CH2IN) || defined(GPIO_TIM4_CH2IN) || \
    defined(GPIO_TIM5_CH2IN) || defined(GPIO_TIM8_CH2IN) || \
    defined(GPIO_TIM9_CH2IN) || defined(GPIO_TIM12_CH2IN)
#  define HAVE_CH2IN 1
#endif

#if defined(GPIO_TIM1_CH3IN) || defined(GPIO_TIM2_CH3IN) || \
    defined(GPIO_TIM3_CH3IN) || defined(GPIO_TIM4_CH3IN) || \
    defined(GPIO_TIM5_CH3IN) || defined(GPIO_TIM8_CH3IN)
#  define HAVE_CH3IN 1
#endif

#if defined(GPIO_TIM1_CH4IN) || defined(GPIO_TIM2_CH4IN) || \
    defined(GPIO_TIM3_CH4IN) || defined(GPIO_TIM4_CH4IN) || \
    defined(GPIO_TIM5_CH4IN) || defined(GPIO_TIM8_CH4IN)
#  define HAVE_CH4IN 1
#endif

#if defined(CONFIG_STM32_TIM1_CAP) || defined(CONFIG_STM32_TIM8_CAP)
#define USE_ADVENCED_TIM 1
#endif

#if defined(GPIO_TIM1_EXT_CLK_IN) || defined(GPIO_TIM2_EXT_CLK_IN) || \
    defined(GPIO_TIM3_EXT_CLK_IN) || defined(GPIO_TIM4_EXT_CLK_IN) || \
    defined(GPIO_TIM5_EXT_CLK_IN) || defined(GPIO_TIM8_EXT_CLK_IN) || \
    defined(GPIO_TIM9_EXT_CLK_IN) || defined(GPIO_TIM12_EXT_CLK_IN)
#  define USE_EXT_CLOCK 1
#endif

/* This module then only compiles if there are enabled timers that are not
 * intended for some other purpose.
 */

#if defined(CONFIG_STM32F7_TIM1_CAP)  || \
    defined(CONFIG_STM32F7_TIM2_CAP)  || \
    defined(CONFIG_STM32F7_TIM3_CAP)  || \
    defined(CONFIG_STM32F7_TIM4_CAP)  || \
    defined(CONFIG_STM32F7_TIM5_CAP)  || \
    defined(CONFIG_STM32F7_TIM8_CAP)  || \
    defined(CONFIG_STM32F7_TIM9_CAP)  || \
    defined(CONFIG_STM32F7_TIM10_CAP) || \
    defined(CONFIG_STM32F7_TIM11_CAP) || \
    defined(CONFIG_STM32F7_TIM12_CAP) || \
    defined(CONFIG_STM32F7_TIM13_CAP) || \
    defined(CONFIG_STM32F7_TIM14_CAP)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TIM Device Structure */

struct stm32_cap_priv_s
{
  const struct stm32_cap_ops_s *ops;
  const uint32_t base;      /* TIMn base address */
#ifdef USE_EXT_CLOCK
  const uint32_t gpio_clk;  /* TIMn base address */
#endif
  const int irq;            /* irq vector */
#ifdef USE_ADVENCED_TIM
  const int irq_of;         /* irq timer overflow is deferent in advanced timer */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Get a 16-bit register value by offset */

static inline uint16_t
stm32_getreg16(FAR const struct stm32_cap_priv_s *priv, uint8_t offset)
{
  return getreg16(priv->base + offset);
}

/* Put a 16-bit register value by offset */

static inline void stm32_putreg16(FAR const struct stm32_cap_priv_s *priv,
                                  uint8_t offset, uint16_t value)
{
  putreg16(value, priv->base + offset);
}

/* Modify a 16-bit register value by offset */

static inline void stm32_modifyreg16(FAR const struct stm32_cap_priv_s *priv,
                                     uint8_t offset, uint16_t clearbits,
                                     uint16_t setbits)
{
  modifyreg16(priv->base + offset, clearbits, setbits);
}

/* Get a 32-bit register value by offset.  This applies only for the STM32 F7
 * 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2 and TIM5.
 */

static inline uint32_t
stm32_getreg32(FAR const struct stm32_cap_priv_s *priv, uint8_t offset)
{
  return getreg32(priv->base + offset);
}

/* Put a 32-bit register value by offset.  This applies only for the STM32 F7
 * 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2 and TIM5.
 */

static inline void stm32_putreg32(FAR const struct stm32_cap_priv_s *priv,
                                  uint8_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * gpio Functions
 ****************************************************************************/

static inline uint32_t
stm32_cap_gpio(FAR const struct stm32_cap_priv_s *priv, int channel)
{
  switch (priv->base)
    {
#ifdef CONFIG_STM32F7_TIM1_CAP
      case STM32_TIM1_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM1_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM1_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM1_CH1IN
            case 1:
              return GPIO_TIM1_CH1IN;
#endif
#ifdef GPIO_TIM1_CH2IN
            case 2:
              return GPIO_TIM1_CH2IN;
#endif
#ifdef GPIO_TIM1_CH3IN
            case 3:
              return GPIO_TIM1_CH3IN;
#endif
#ifdef GPIO_TIM1_CH4IN
            case 4:
              return GPIO_TIM1_CH4IN;
#endif
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM2_CAP
      case STM32_TIM2_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM2_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM2_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM2_CH1IN
            case 1:
              return GPIO_TIM2_CH1IN;
#endif
#ifdef GPIO_TIM2_CH2IN
            case 2:
              return GPIO_TIM2_CH2IN;
#endif
#ifdef GPIO_TIM2_CH3IN
            case 3:
              return GPIO_TIM2_CH3IN;
#endif
#ifdef GPIO_TIM2_CH4IN
            case 4:
              return GPIO_TIM2_CH4IN;
#endif
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM3_CAP
      case STM32_TIM3_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM3_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM3_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM3_CH1IN
            case 1:
              return GPIO_TIM3_CH1IN;
#endif
#ifdef GPIO_TIM3_CH2IN
            case 2:
              return GPIO_TIM3_CH2IN;
#endif
#ifdef GPIO_TIM3_CH3IN
            case 3:
              return GPIO_TIM3_CH3IN;
#endif
#ifdef GPIO_TIM3_CH4IN
            case 4:
              return GPIO_TIM3_CH4IN;
#endif
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM4_CAP
      case STM32_TIM4_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM4_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM4_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM4_CH1IN
            case 1:
              return GPIO_TIM4_CH1IN;
#endif
#ifdef GPIO_TIM4_CH2IN
            case 2:
              return GPIO_TIM4_CH2IN;
#endif
#ifdef GPIO_TIM4_CH3IN
            case 3:
              return GPIO_TIM4_CH3IN;
#endif
#ifdef GPIO_TIM4_CH4IN
            case 4:
              return GPIO_TIM4_CH4IN;
#endif
          }
        break;
#endif
#ifdef CONFIG_STM32F7_TIM5_CAP
      case STM32_TIM5_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM5_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM5_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM5_CH1IN
            case 1:
              return GPIO_TIM5_CH1IN;
#endif
#ifdef GPIO_TIM5_CH2IN
            case 2:
              return GPIO_TIM5_CH2IN;
#endif
#ifdef GPIO_TIM5_CH3IN
            case 3:
              return GPIO_TIM5_CH3IN;
#endif
#ifdef GPIO_TIM5_CH4IN
            case 4:
              return GPIO_TIM5_CH4IN;
#endif
          }
        break;
#endif

      /* TIM6 and TIM7 cannot be used in capture */

#ifdef CONFIG_STM32F7_TIM8_CAP
      case STM32_TIM8_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM8_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM8_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM8_CH1IN
            case 1:
              return GPIO_TIM8_CH1IN;
#endif
#ifdef GPIO_TIM8_CH2IN
            case 2:
              return GPIO_TIM8_CH2IN;
#endif
#ifdef GPIO_TIM8_CH3IN
            case 3:
              return GPIO_TIM8_CH3IN;
#endif
#ifdef GPIO_TIM8_CH4IN
            case 4:
              return GPIO_TIM8_CH4IN;
#endif
          }
        break;
#endif

#ifdef CONFIG_STM32F7_TIM9_CAP
      case STM32_TIM9_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM9_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM9_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM9_CH1IN
            case 1:
              return GPIO_TIM9_CH1IN;
#endif
#ifdef GPIO_TIM9_CH2IN
            case 2:
              return GPIO_TIM9_CH2IN;
#endif
#ifdef GPIO_TIM9_CH3IN
            case 3:
              return GPIO_TIM9_CH3IN;
#endif
#ifdef GPIO_TIM9_CH4IN
            case 4:
              return GPIO_TIM9_CH4IN;
#endif
          }
        break;
#endif

#ifdef CONFIG_STM32F7_TIM10_CAP
      case STM32_TIM10_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM10_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM10_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM10_CH1IN
            case 1:
              return GPIO_TIM10_CH1IN;
#endif
#ifdef GPIO_TIM10_CH2IN
            case 2:
              return GPIO_TIM10_CH2IN;
#endif
#ifdef GPIO_TIM10_CH4IN
            case 3:
              return GPIO_TIM10_CH4IN;
#endif
#ifdef GPIO_TIM10_CH5IN
            case 4:
              return GPIO_TIM10_CH5IN;
#endif
          }
        break;
#endif

#ifdef CONFIG_STM32F7_TIM11_CAP
      case STM32_TIM11_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM11_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM11_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM11_CH1IN
            case 1:
              return GPIO_TIM11_CH1IN;
#endif
#ifdef GPIO_TIM11_CH2IN
            case 2:
              return GPIO_TIM11_CH2IN;
#endif
#ifdef GPIO_TIM11_CH4IN
            case 3:
              return GPIO_TIM11_CH4IN;
#endif
#ifdef GPIO_TIM11_CH5IN
            case 4:
              return GPIO_TIM11_CH5IN;
#endif
          }
        break;
#endif

#ifdef CONFIG_STM32F7_TIM12_CAP
      case STM32_TIM12_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM12_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM12_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM12_CH1IN
            case 1:
              return GPIO_TIM12_CH1IN;
#endif
#ifdef GPIO_TIM12_CH2IN
            case 2:
              return GPIO_TIM12_CH2IN;
#endif
#ifdef GPIO_TIM12_CH4IN
            case 3:
              return GPIO_TIM12_CH4IN;
#endif
#ifdef GPIO_TIM12_CH5IN
            case 4:
              return GPIO_TIM12_CH5IN;
#endif
          }
        break;
#endif

#ifdef CONFIG_STM32F7_TIM13_CAP
      case STM32_TIM13_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM13_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM13_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM13_CH1IN
            case 1:
              return GPIO_TIM13_CH1IN;
#endif
#ifdef GPIO_TIM13_CH2IN
            case 2:
              return GPIO_TIM13_CH2IN;
#endif
#ifdef GPIO_TIM13_CH4IN
            case 3:
              return GPIO_TIM13_CH4IN;
#endif
#ifdef GPIO_TIM13_CH5IN
            case 4:
              return GPIO_TIM13_CH5IN;
#endif
          }
        break;
#endif

#ifdef CONFIG_STM32F7_TIM14_CAP
      case STM32_TIM14_BASE:
        switch (channel)
          {
#ifdef GPIO_TIM14_EXT_CLK_IN
            case STM32_CAP_CHANNEL_COUNTER:
              return GPIO_TIM14_EXT_CLK_IN;
#endif
#ifdef GPIO_TIM14_CH1IN
            case 1:
              return GPIO_TIM14_CH1IN;
#endif
#ifdef GPIO_TIM14_CH2IN
            case 2:
              return GPIO_TIM14_CH2IN;
#endif
#ifdef GPIO_TIM14_CH4IN
            case 3:
              return GPIO_TIM14_CH4IN;
#endif
#ifdef GPIO_TIM14_CH5IN
            case 4:
              return GPIO_TIM14_CH5IN;
#endif
          }
        break;
#endif
    }

  return 0;
}

static inline int stm32_cap_set_rcc(FAR const struct stm32_cap_priv_s *priv,
                                    bool on)
{
  uint32_t offset = 0;
  uint32_t mask   = 0;

  switch (priv->base)
    {
#ifdef CONFIG_STM32F7_TIM1_CAP
      case STM32_TIM1_BASE:
        offset = STM32_RCC_APB2ENR;
        mask   = RCC_APB2ENR_TIM1EN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM2_CAP
      case STM32_TIM2_BASE:
        offset = STM32_RCC_APB1ENR;
        mask   = RCC_APB1ENR_TIM2EN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM3_CAP
      case STM32_TIM3_BASE:
        offset = STM32_RCC_APB1ENR;
        mask   = RCC_APB1ENR_TIM3EN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM4_CAP
      case STM32_TIM4_BASE:
        offset = STM32_RCC_APB1ENR;
        mask   = RCC_APB1ENR_TIM4EN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM5_CAP
      case STM32_TIM5_BASE:
        offset = STM32_RCC_APB1ENR;
        mask   = RCC_APB1ENR_TIM5EN;
        break;
#endif

      /* TIM6 and TIM7 cannot be used in capture */

#ifdef CONFIG_STM32F7_TIM8_CAP
      case STM32_TIM8_BASE:
        offset = STM32_RCC_APB2ENR;
        mask   = RCC_APB2ENR_TIM8EN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM9_CAP
      case STM32_TIM9_BASE:
        offset = STM32_RCC_APB2ENR;
        mask   = RCC_APB2ENR_TIM9EN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM10_CAP
      case STM32_TIM10_BASE:
        offset = STM32_RCC_APB2ENR;
        mask   = RCC_APB2ENR_TIM10EN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM11_CAP
      case STM32_TIM11_BASE:
        offset = STM32_RCC_APB2ENR;
        mask   = RCC_APB2ENR_TIM11EN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM12_CAP
      case STM32_TIM12_BASE:
        offset = STM32_RCC_APB1ENR;
        mask   = RCC_APB1ENR_TIM12EN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM13_CAP
      case STM32_TIM13_BASE:
        offset = STM32_RCC_APB1ENR;
        mask   = RCC_APB1ENR_TIM13EN;
        break;
#endif
#ifdef CONFIG_STM32F7_TIM14_CAP
      case STM32_TIM14_BASE:
        offset = STM32_RCC_APB1ENR;
        mask   = RCC_APB1ENR_TIM14EN;
        break;
#endif
    }

  if (mask == 0)
    {
      return ERROR;
    }

  if (on)
    {
      modifyreg32(offset, 0, mask);
    }
  else
    {
      modifyreg32(offset, mask, 0);
    }

  return OK;
}

/****************************************************************************
 * Basic Functions
 ****************************************************************************/

static int stm32_cap_setclock(FAR struct stm32_cap_dev_s *dev,
                              stm32_cap_clk_t clk, uint32_t prescaler,
                              uint32_t max)
{
  const struct stm32_cap_priv_s *priv = (const struct stm32_cap_priv_s *)dev;
  uint16_t regval = 0;

  if (prescaler == 0)
    {
      /* Disable Timer */

      stm32_modifyreg16(priv, STM32_BTIM_CR1_OFFSET, ATIM_CR1_CEN, 0);
      return 0;
    }

  /* We need to decrement value for '1', but only, if we are allowed to
   * not to cause underflow. Check for overflow.
   */

  if (prescaler > 0)
    {
      prescaler--;
    }

  if (prescaler > 0xffff)
    {
      prescaler = 0xffff;
    }

  switch (clk)
    {
      case STM32_CAP_CLK_INT:
          regval = GTIM_SMCR_DISAB;
          break;

      case STM32_CAP_CLK_EXT:
          regval = GTIM_SMCR_EXTCLK1;
          break;

      /* TODO: Add other case */

      default:
        return ERROR;
    }

  stm32_modifyreg16(priv, STM32_BTIM_EGR_OFFSET, GTIM_SMCR_SMS_MASK, regval);

  /* Set Maximum */

  stm32_putreg32(priv, STM32_BTIM_ARR_OFFSET, max);

  /* Set prescaler */

  stm32_putreg16(priv, STM32_BTIM_PSC_OFFSET, prescaler);

  /* Reset counter timer */

  stm32_modifyreg16(priv, STM32_BTIM_EGR_OFFSET, 0, BTIM_EGR_UG);

  /* Enable timer */

  stm32_modifyreg16(priv, STM32_BTIM_CR1_OFFSET, 0, BTIM_CR1_CEN);

#ifdef USE_ADVENCED_TIM
  /* Advanced registers require Main Output Enable */

  if (priv->base == STM32_TIM1_BASE || priv->base == STM32_TIM8_BASE)
    {
      stm32_modifyreg16(priv, STM32_ATIM_BDTR_OFFSET, 0, ATIM_BDTR_MOE);
    }
#endif

  return prescaler;
}

static int stm32_cap_setisr(FAR struct stm32_cap_dev_s *dev, xcpt_t handler,
                            void *arg)
{
  const struct stm32_cap_priv_s *priv = (const struct stm32_cap_priv_s *)dev;
  int irq;
#ifdef USE_ADVENCED_TIM
  int irq_of;
#endif

  DEBUGASSERT(dev != NULL);

  irq = priv->irq;
#ifdef USE_ADVENCED_TIM
  irq_of = priv->irq_of;
#endif

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(irq);
      irq_detach(irq);

#ifdef USE_ADVENCED_TIM
      if (priv->irq_of)
        {
          up_disable_irq(irq_of);
          irq_detach(irq_of);
        }
#endif

      return OK;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(irq, handler, arg);
  up_enable_irq(irq);

#ifdef USE_ADVENCED_TIM
  if (priv->irq_of)
    {
      irq_attach(priv->irq_of, handler, arg);
      up_enable_irq(priv->irq_of);
    }
#endif

  return OK;
}

static void stm32_cap_enableint(FAR struct stm32_cap_dev_s *dev,
                                stm32_cap_flags_t src, bool on)
{
  const struct stm32_cap_priv_s *priv = (const struct stm32_cap_priv_s *)dev;
  uint16_t mask = 0;

  DEBUGASSERT(dev != NULL);

  if (src & STM32_CAP_FLAG_IRQ_COUNTER)
    {
      mask |= ATIM_DIER_UIE;
    }

  if (src & STM32_CAP_FLAG_IRQ_CH_1)
    {
      mask |= ATIM_DIER_CC1IE;
    }

  if (src & STM32_CAP_FLAG_IRQ_CH_2)
    {
      mask |= ATIM_DIER_CC2IE;
    }

  if (src & STM32_CAP_FLAG_IRQ_CH_3)
    {
      mask |= ATIM_DIER_CC3IE;
    }

  if (src & STM32_CAP_FLAG_IRQ_CH_4)
    {
      mask |= ATIM_DIER_CC4IE;
    }

  /* Not IRQ on channel overflow */

  if (on)
    {
      stm32_modifyreg16(priv, STM32_BTIM_DIER_OFFSET, 0, mask);
    }
  else
    {
      stm32_modifyreg16(priv, STM32_BTIM_DIER_OFFSET, mask, 0);
    }
}

static void stm32_cap_ackflags(FAR struct stm32_cap_dev_s *dev, int flags)
{
  const struct stm32_cap_priv_s *priv = (const struct stm32_cap_priv_s *)dev;
  uint16_t mask = 0;

  if (flags & STM32_CAP_FLAG_IRQ_COUNTER)
    {
      mask |= ATIM_SR_UIF;
    }

  if (flags & STM32_CAP_FLAG_IRQ_CH_1)
    {
      mask |= ATIM_SR_CC1IF;
    }

  if (flags & STM32_CAP_FLAG_IRQ_CH_2)
    {
      mask |= ATIM_SR_CC2IF;
    }

  if (flags & STM32_CAP_FLAG_IRQ_CH_3)
    {
      mask |= ATIM_SR_CC3IF;
    }

  if (flags & STM32_CAP_FLAG_IRQ_CH_4)
    {
      mask |= ATIM_SR_CC4IF;
    }

  if (flags & STM32_CAP_FLAG_OF_CH_1)
    {
      mask |= ATIM_SR_CC1OF;
    }

  if (flags & STM32_CAP_FLAG_OF_CH_2)
    {
      mask |= ATIM_SR_CC2OF;
    }

  if (flags & STM32_CAP_FLAG_OF_CH_3)
    {
      mask |= ATIM_SR_CC3OF;
    }

  if (flags & STM32_CAP_FLAG_OF_CH_4)
    {
      mask |= ATIM_SR_CC4OF;
    }

  stm32_putreg16(priv, STM32_BTIM_SR_OFFSET, ~mask);
}

static stm32_cap_flags_t stm32_cap_getflags(FAR struct stm32_cap_dev_s *dev)
{
  const struct stm32_cap_priv_s *priv = (const struct stm32_cap_priv_s *)dev;
  uint16_t regval = 0;
  stm32_cap_flags_t flags = 0;

  regval = stm32_getreg16(priv, STM32_BTIM_SR_OFFSET);

  if (regval & ATIM_SR_UIF)
    {
      flags |= STM32_CAP_FLAG_IRQ_COUNTER;
    }

  if (regval & ATIM_SR_CC1IF)
    {
      flags |= STM32_CAP_FLAG_IRQ_CH_1;
    }

  if (regval & ATIM_SR_CC2IF)
    {
      flags |= STM32_CAP_FLAG_IRQ_CH_2;
    }

  if (regval & ATIM_SR_CC3IF)
    {
      flags |= STM32_CAP_FLAG_IRQ_CH_3;
    }

  if (regval & ATIM_SR_CC4IF)
    {
      flags |= STM32_CAP_FLAG_IRQ_CH_4;
    }

  if (regval & ATIM_SR_CC1OF)
    {
      flags |= STM32_CAP_FLAG_OF_CH_1;
    }

  if (regval & ATIM_SR_CC2OF)
    {
      flags |= STM32_CAP_FLAG_OF_CH_2;
    }

  if (regval & ATIM_SR_CC3OF)
    {
      flags |= STM32_CAP_FLAG_OF_CH_3;
    }

  if (regval & ATIM_SR_CC4OF)
    {
      flags |= STM32_CAP_FLAG_OF_CH_4;
    }

  return flags;
}

/****************************************************************************
 * General Functions
 ****************************************************************************/

static int stm32_cap_setchannel(FAR struct stm32_cap_dev_s *dev,
                                uint8_t channel, stm32_cap_ch_cfg_t cfg)
{
  const struct stm32_cap_priv_s *priv = (const struct stm32_cap_priv_s *)dev;
  uint32_t gpio = 0;
  uint16_t mask;
  uint16_t regval;
  uint16_t ccer_en_bit;

  DEBUGASSERT(dev != NULL);

  gpio = stm32_cap_gpio(priv, channel);

  if (gpio == 0)
    {
      return ERROR;
    }

  if ((cfg & STM32_CAP_MAPPED_MASK) == 0)
    {
      return ERROR; /* MAPPED not selected */
    }

  /* Change to zero base index */

  channel--;

  /* Set ccer :
   *
   * GTIM_CCER_CCxE Is written latter to allow writing CCxS bits.
   *
   */

  switch (cfg & STM32_CAP_EDGE_MASK)
    {
      case STM32_CAP_EDGE_DISABLED:
        regval = 0;
        ccer_en_bit = 0;
        break;

      case STM32_CAP_EDGE_RISING:
        ccer_en_bit = GTIM_CCER_CC1E;
        regval      = 0;
        break;

      case STM32_CAP_EDGE_FALLING:
        ccer_en_bit = GTIM_CCER_CC1E;
        regval      = GTIM_CCER_CC1P;
        break;

      case STM32_CAP_EDGE_BOTH:
        ccer_en_bit = GTIM_CCER_CC1E;
        regval      = GTIM_CCER_CC1P | GTIM_CCER_CC1NP;
        break;

      default:
        return ERROR;
    }

  /* Shift all CCER bits to corresponding channel */

  mask = (GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC1NP);
  mask          <<= (channel << 2);
  regval        <<= (channel << 2);
  ccer_en_bit   <<= (channel << 2);

  stm32_modifyreg16(priv, STM32_GTIM_CCER_OFFSET, mask, regval);

  /* Set ccmr */

  regval = cfg;
  mask = (GTIM_CCMR1_IC1F_MASK | GTIM_CCMR1_IC1PSC_MASK |
          GTIM_CCMR1_CC1S_MASK);
  regval &= mask;

  if (channel & 1)
    {
      regval <<= 8;
      mask   <<= 8;
    }

  if (channel < 2)
    {
      stm32_modifyreg16(priv, STM32_GTIM_CCMR1_OFFSET, mask, regval);
    }
  else
    {
      stm32_modifyreg16(priv, STM32_GTIM_CCMR2_OFFSET, mask, regval);
    }

  /* Set GPIO */

  if ((cfg & STM32_CAP_EDGE_MASK) == STM32_CAP_EDGE_DISABLED)
    {
      stm32_unconfiggpio(gpio);
    }
  else
    {
      stm32_configgpio(gpio);
    }

  /* Enable this channel timer */

  stm32_modifyreg16(priv, STM32_GTIM_CCER_OFFSET, 0, ccer_en_bit);
  return OK;
}

static uint32_t stm32_cap_getcapture(FAR struct stm32_cap_dev_s *dev,
                                     uint8_t channel)
{
  const struct stm32_cap_priv_s *priv = (const struct stm32_cap_priv_s *)dev;
  uint32_t offset;

  DEBUGASSERT(dev != NULL);

  switch (channel)
    {
      case STM32_CAP_CHANNEL_COUNTER:
        offset = STM32_GTIM_CNT_OFFSET;
        break;
#ifdef HAVE_CH1IN
      case 1:
        offset = STM32_GTIM_CCR1_OFFSET;
        break;
#endif
#ifdef HAVE_CH2IN
      case 2:
        offset = STM32_GTIM_CCR2_OFFSET;
        break;
#endif
#ifdef HAVE_CH3IN
      case 3:
        offset = STM32_GTIM_CCR3_OFFSET;
        break;
#endif
#ifdef HAVE_CH4IN
      case 4:
        offset = STM32_GTIM_CCR4_OFFSET;
        break;
#endif
      default:
        return ERROR;
    }

  if (priv->base == STM32_TIM2_BASE || priv->base == STM32_TIM5_BASE)
    {
      return stm32_getreg32(priv, offset);
    }

  return stm32_getreg16(priv, offset);
}

/****************************************************************************
 * Advanced Functions
 ****************************************************************************/

/* TODO: Advanced functions for the STM32_ATIM */

/****************************************************************************
 * Device Structures, Instantiation
 ****************************************************************************/

struct stm32_cap_ops_s stm32_cap_ops =
{
  .setclock     = &stm32_cap_setclock,
  .setchannel   = &stm32_cap_setchannel,
  .getcapture   = &stm32_cap_getcapture,
  .setisr       = &stm32_cap_setisr,
  .enableint    = &stm32_cap_enableint,
  .ackflags     = &stm32_cap_ackflags,
  .getflags     = &stm32_cap_getflags
};

#ifdef CONFIG_STM32F7_TIM1_CAP
const struct stm32_cap_priv_s stm32_tim1_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM1_BASE,
  .irq          = STM32_IRQ_TIM1CC,
#ifdef USE_ADVENCED_TIM
  .irq_of       = STM32_IRQ_TIM1UP,
#endif
};
#endif

#ifdef CONFIG_STM32F7_TIM2_CAP
const struct stm32_cap_priv_s stm32_tim2_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM2_BASE,
  .irq          = STM32_IRQ_TIM2,
#ifdef USE_ADVENCED_TIM
  .irq_of       = 0,
#endif
};
#endif

#ifdef CONFIG_STM32F7_TIM3_CAP
const struct stm32_cap_priv_s stm32_tim3_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM3_BASE,
  .irq          = STM32_IRQ_TIM3,
#ifdef USE_ADVENCED_TIM
  .irq_of       = 0,
#endif
};
#endif

#ifdef CONFIG_STM32F7_TIM4_CAP
const struct stm32_cap_priv_s stm32_tim4_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM4_BASE,
  .irq          = STM32_IRQ_TIM4,
#ifdef USE_ADVENCED_TIM
  .irq_of       = 0,
#endif
};
#endif

#ifdef CONFIG_STM32F7_TIM5_CAP
const struct stm32_cap_priv_s stm32_tim5_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM5_BASE,
  .irq          = STM32_IRQ_TIM5,
#ifdef USE_ADVENCED_TIM
  .irq_of       = 0,
#endif
};
#endif

/* TIM6 and TIM7 cannot be used in capture */

#ifdef CONFIG_STM32F7_TIM8_CAP
const struct stm32_cap_priv_s stm32_tim8_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM8_BASE,
  .irq          = STM32_IRQ_TIM8CC,
#ifdef USE_ADVENCED_TIM
  .irq_of       = STM32_IRQ_TIM8UP,
#endif
};
#endif

#ifdef CONFIG_STM32F7_TIM9_CAP
const struct stm32_cap_priv_s stm32_tim9_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM9_BASE,
  .irq          = STM32_IRQ_TIM9,
#ifdef USE_ADVENCED_TIM
  .irq_of       = 0,
#endif
};
#endif

#ifdef CONFIG_STM32F7_TIM10_CAP
const struct stm32_cap_priv_s stm32_tim10_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM10_BASE,
  .irq          = STM32_IRQ_TIM10,
#ifdef USE_ADVENCED_TIM
  .irq_of       = 0,
#endif
};
#endif

#ifdef CONFIG_STM32F7_TIM11_CAP
const struct stm32_cap_priv_s stm32_tim11_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM11_BASE,
  .irq          = STM32_IRQ_TIM11,
#ifdef USE_ADVENCED_TIM
  .irq_of       = 0,
#endif
};
#endif

#ifdef CONFIG_STM32F7_TIM12_CAP
const struct stm32_cap_priv_s stm32_tim12_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM12_BASE,
  .irq          = STM32_IRQ_TIM12,
#ifdef USE_ADVENCED_TIM
  .irq_of       = 0,
#endif
};
#endif

#ifdef CONFIG_STM32F7_TIM13_CAP
const struct stm32_cap_priv_s stm32_tim13_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM13_BASE,
  .irq          = STM32_IRQ_TIM13,
#ifdef USE_ADVENCED_TIM
  .irq_of       = 0,
#endif
};
#endif

#ifdef CONFIG_STM32F7_TIM14_CAP
const struct stm32_cap_priv_s stm32_tim14_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM14_BASE,
  .irq          = STM32_IRQ_TIM14,
#ifdef USE_ADVENCED_TIM
  .irq_of       = 0,
#endif
};
#endif

static inline const struct stm32_cap_priv_s * stm32_cap_get_priv(int timer)
{
  switch (timer)
    {
#ifdef CONFIG_STM32F7_TIM1_CAP
      case 1:
        return &stm32_tim1_priv;
#endif
#ifdef CONFIG_STM32F7_TIM2_CAP
      case 2:
        return &stm32_tim2_priv;
#endif
#ifdef CONFIG_STM32F7_TIM3_CAP
      case 3:
        return &stm32_tim3_priv;
#endif
#ifdef CONFIG_STM32F7_TIM4_CAP
      case 4:
        return &stm32_tim4_priv;
#endif
#ifdef CONFIG_STM32F7_TIM5_CAP
      case 5:
        return &stm32_tim5_priv;
#endif

      /* TIM6 and TIM7 cannot be used in capture */

#ifdef CONFIG_STM32F7_TIM8_CAP
      case 8:
        return &stm32_tim8_priv;
#endif
#ifdef CONFIG_STM32F7_TIM9_CAP
      case 9:
        return &stm32_tim9_priv;
#endif
#ifdef CONFIG_STM32F7_TIM10_CAP
      case 10:
        return &stm32_tim10_priv;
#endif
#ifdef CONFIG_STM32F7_TIM11_CAP
      case 11:
        return &stm32_tim11_priv;
#endif
#ifdef CONFIG_STM32F7_TIM12_CAP
      case 12:
        return &stm32_tim12_priv;
#endif
#ifdef CONFIG_STM32F7_TIM13_CAP
      case 13:
        return &stm32_tim13_priv;
#endif
#ifdef CONFIG_STM32F7_TIM14_CAP
      case 14:
        return &stm32_tim14_priv;
#endif
    }

  return NULL;
}

/****************************************************************************
 * Public Function - Initialization
 ****************************************************************************/

FAR struct stm32_cap_dev_s *stm32_cap_init(int timer)
{
  const struct stm32_cap_priv_s *priv = stm32_cap_get_priv(timer);
  uint32_t gpio;

  if (priv)
    {
      stm32_cap_set_rcc(priv, true);

      gpio = stm32_cap_gpio(priv, STM32_CAP_CHANNEL_COUNTER);
      if (gpio)
        {
          stm32_configgpio(gpio);
        }

      /* Disable timer while is not configured */

      stm32_modifyreg16(priv, STM32_BTIM_CR1_OFFSET, ATIM_CR1_CEN, 0);
    }

  return (struct stm32_cap_dev_s *)priv;
}

int stm32_cap_deinit(FAR struct stm32_cap_dev_s * dev)
{
  const struct stm32_cap_priv_s *priv = (struct stm32_cap_priv_s *)dev;
  uint32_t gpio;

  DEBUGASSERT(dev != NULL);

  /* Disable timer while is not configured */

  stm32_modifyreg16(priv, STM32_BTIM_CR1_OFFSET, ATIM_CR1_CEN, 0);

  gpio = stm32_cap_gpio(priv, STM32_CAP_CHANNEL_COUNTER);
  if (gpio)
    {
      stm32_unconfiggpio(gpio);
    }

  stm32_cap_set_rcc(priv, false);
  return OK;
}

#endif /* defined(CONFIG_STM32F7_TIM1 || ... || TIM14) */
