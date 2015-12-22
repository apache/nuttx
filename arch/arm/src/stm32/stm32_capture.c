/************************************************************************************
 * arm/arm/src/stm32/stm32_capture.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_capture.h"

/************************************************************************************
 * Private Types
 ************************************************************************************/
/* Configuration ********************************************************************/

#if defined(GPIO_TIM1_CH1IN) || defined(GPIO_TIM2_CH1IN) || defined(GPIO_TIM3_CH1IN) || \
    defined(GPIO_TIM4_CH1IN) || defined(GPIO_TIM5_CH1IN) || defined(GPIO_TIM8_CH1IN) || \
    defined(GPIO_TIM9_CH1IN) || defined(GPIO_TIM10_CH1IN) || defined(GPIO_TIM11_CH1IN) || \
    defined(GPIO_TIM12_CH1IN) || defined(GPIO_TIM13_CH1IN) || defined(GPIO_TIM14_CH1IN)
#  define HAVE_CH1IN 1
#endif

#if defined(GPIO_TIM1_CH2IN) || defined(GPIO_TIM2_CH2IN) || defined(GPIO_TIM3_CH2IN) || \
    defined(GPIO_TIM4_CH2IN) || defined(GPIO_TIM5_CH2IN) || defined(GPIO_TIM8_CH2IN) || \
    defined(GPIO_TIM9_CH2IN) || defined(GPIO_TIM12_CH2IN)
#  define HAVE_CH2IN 1
#endif

#if defined(GPIO_TIM1_CH3IN) || defined(GPIO_TIM2_CH3IN) || defined(GPIO_TIM3_CH3IN) || \
    defined(GPIO_TIM4_CH3IN) || defined(GPIO_TIM5_CH3IN) || defined(GPIO_TIM8_CH3IN)
#  define HAVE_CH3IN 1
#endif

#if defined(GPIO_TIM1_CH4IN) || defined(GPIO_TIM2_CH4IN) || defined(GPIO_TIM3_CH4IN) || \
    defined(GPIO_TIM4_CH4IN) || defined(GPIO_TIM5_CH4IN) || defined(GPIO_TIM8_CH4IN)
#  define HAVE_CH4IN 1
#endif

#if defined(CONFIG_STM32_TIM1_CAP) || defined(CONFIG_STM32_TIM1_CAP) 
#define HAVE_ADANCED_TIM 1
#endif

/* This module then only compiles if there are enabled timers that are not intended for
 * some other purpose.
 */

#if defined(CONFIG_STM32_TIM1_CAP) || defined(CONFIG_STM32_TIM2_CAP) || defined(CONFIG_STM32_TIM3_CAP) || \
    defined(CONFIG_STM32_TIM4_CAP) || defined(CONFIG_STM32_TIM5_CAP) || defined(CONFIG_STM32_TIM8_CAP) || \
    defined(CONFIG_STM32_TIM9_CAP) || defined(CONFIG_STM32_TIM10_CAP) || defined(CONFIG_STM32_TIM11_CAP) || \
    defined(CONFIG_STM32_TIM12_CAP) || defined(CONFIG_STM32_TIM13_CAP) || defined(CONFIG_STM32_TIM14_CAP)


/************************************************************************************
 * Private Types
 ************************************************************************************/

/* TIM Device Structure */

struct stm32_cap_priv_s
{
  const struct stm32_cap_ops_s *ops;
  const uint32_t base;      /* TIMn base address */
  const int irq;            /* irq vector */
#define HAVE_ADANCED_TIM 1
  const int irq_of;         /* irq timer overflow is deferent in advanced timer */
#endif
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/* Get a 16-bit register value by offset */

static inline uint16_t stm32_getreg16(FAR struct stm32_cap_priv_s *priv,
                                      uint8_t offset)
{
  return getreg16(priv->base + offset);
}

/* Put a 16-bit register value by offset */

static inline void stm32_putreg16(FAR struct stm32_cap_priv_s *priv, uint8_t offset,
                                  uint16_t value)
{
  putreg16(value, priv->base + offset);
}

/* Modify a 16-bit register value by offset */

static inline void stm32_modifyreg16(FAR struct stm32_cap_priv_s *priv,
                                     uint8_t offset, uint16_t clearbits,
                                     uint16_t setbits)
{
  modifyreg16(priv->base + offset, clearbits, setbits);
}

/* Get a 32-bit register value by offset.  This applies only for the STM32 F4
 * 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 */

static inline uint32_t stm32_getreg32(FAR struct stm32_cap_priv_s *priv,
                                      uint8_t offset)
{
  return getreg32(priv->base + offset);
}

/* Put a 32-bit register value by offset.  This applies only for the STM32 F4
 * 32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 */

static inline void stm32_putreg32(FAR struct stm32_cap_priv_s *priv, uint8_t offset,
                                  uint32_t value)
{
  putreg32(value, priv->base + offset);
}


/************************************************************************************
 * gpio Functions
 ************************************************************************************/

static inline uint32_t stm32_cap_gpio(FAR struct stm32_cap_priv_s *priv, int channel)
{

  switch(priv->base)
    {
#ifdef CONFIG_STM32_TIM1
      case STM32_TIM1_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM1_CH1IN)
            case 0: return GPIO_TIM1_CH1IN;
#endif
#if defined(GPIO_TIM1_CH2IN)
            case 1: return GPIO_TIM1_CH2IN;
#endif
#if defined(GPIO_TIM1_CH3IN)
            case 2: return GPIO_TIM1_CH3IN;
#endif
#if defined(GPIO_TIM1_CH4IN)
            case 3: return GPIO_TIM1_CH4IN;
#endif
          }
        break;
#endif
#ifdef CONFIG_STM32_TIM2
      case STM32_TIM2_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM2_CH1IN)
            case 0: return GPIO_TIM2_CH1IN;
#endif
#if defined(GPIO_TIM2_CH2IN)
            case 1: return GPIO_TIM2_CH2IN;
#endif
#if defined(GPIO_TIM2_CH3IN)
            case 2: return GPIO_TIM2_CH3IN;
#endif
#if defined(GPIO_TIM2_CH4IN)
            case 3: return GPIO_TIM2_CH4IN;
#endif
          }
        break;
#endif
#ifdef CONFIG_STM32_TIM3
      case STM32_TIM3_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM3_CH1IN)
            case 0: return GPIO_TIM3_CH1IN;
#endif
#if defined(GPIO_TIM3_CH2IN)
            case 1: return GPIO_TIM3_CH2IN;
#endif
#if defined(GPIO_TIM3_CH3IN)
            case 2: return GPIO_TIM3_CH3IN;
#endif
#if defined(GPIO_TIM3_CH4IN)
            case 3: return GPIO_TIM3_CH4IN;
#endif
          }
        break;
#endif
#ifdef CONFIG_STM32_TIM4
      case STM32_TIM4_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM4_CH1IN)
            case 0: return GPIO_TIM4_CH1IN;
#endif
#if defined(GPIO_TIM4_CH2IN)
            case 1: return GPIO_TIM4_CH2IN;
#endif
#if defined(GPIO_TIM4_CH3IN)
            case 2: return GPIO_TIM4_CH3IN;
#endif
#if defined(GPIO_TIM4_CH4IN)
            case 3: return GPIO_TIM4_CH4IN;
#endif
          }
        break;
#endif
#ifdef CONFIG_STM32_TIM5
      case STM32_TIM5_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM5_CH1IN)
            case 0: return GPIO_TIM5_CH1IN;
#endif
#if defined(GPIO_TIM5_CH2IN)
            case 1: return GPIO_TIM5_CH2IN;
#endif
#if defined(GPIO_TIM5_CH3IN)
            case 2: return GPIO_TIM5_CH3IN;
#endif
#if defined(GPIO_TIM5_CH4IN)
            case 3: return GPIO_TIM5_CH4IN;
#endif
          }
        break;
#endif

#ifdef CONFIG_STM32_TIM8
      case STM32_TIM8_BASE:
        switch (channel)
          {
#if defined(GPIO_TIM8_CH1IN)
            case 0: return GPIO_TIM8_CH1OUIN ;
#endif
#if defined(GPIO_TIM8_CH2IN)
            case 1: return GPIO_TIM8_CH2OUIN ;
#endif
#if defined(GPIO_TIM8_CH3IN)
            case 2: return GPIO_TIM8_CH3OUIN ;
#endif
#if defined(GPIO_TIM8_CH4IN)
            case 3: return GPIO_TIM8_CH4OUIN ;
#endif
          }
        break;
#endif
    }
  return gpio;
}
/************************************************************************************
 * Basic Functions
 ************************************************************************************/

static int stm32_cap_setclock(FAR struct stm32_cap_dev_s *dev, stm32_cap_clk_src_t clk_src, 
                              uint32_t prescaler)
{
  struct stm32_cap_priv_s *priv = (struct stm32_cap_priv_s *)dev;
  uint16_t regval = 0;

  if (prescaler == 0)
    {
      //disable Timer
      stm32_modifyreg16(priv, STM32_BTIM_CR1_OFFSET,ATIM_CR1_CEN,0);
      return 0;
    }

  /* We need to decrement value for '1', but only, if we are allowed to
   * not to cause underflow. Check for overflow.
   */

  if (prescaler > 0)
      prescaler--;

  if (prescaler > 0xffff)
      prescaler = 0xffff;


  switch(clk_src)
    {
      case STM32_CAP_CLK_INT:
          regval = GTIM_SMCR_DISAB;
          break;

      case STM32_CAP_CLK_EXT:
          regval = GTIM_SMCR_EXTCLK1
          break;

    /* TODO: Add other case */

      default:
        return ERROR;
    }

  stm32_modifyreg16(priv, STM32_BTIM_EGR_OFFSET, GTIM_SMCR_SMS_MASK, regval );

  // Set Maximum
  stm32_putreg32(priv, STM32_BTIM_ARR_OFFSET, period);

  // Set prescaler
  stm32_putreg16(priv, STM32_BTIM_PSC_OFFSET, prescaler);

  //reset counter timer
  stm32_modifyreg16(priv, STM32_BTIM_EGR_OFFSET,0,BTIM_EGR_UG);

  //enable timer
  stm32_modifyreg16(priv, STM32_BTIM_CR1_OFFSET,0,BTIM_CR1_CEN);

#ifdef HAVE_ADANCED_TIM 
  /* Advanced registers require Main Output Enable */
  if ((priv->base == STM32_TIM1_BASE) || (priv->base == STM32_TIM8_BASE))
    {
      stm32_modifyreg16(priv, STM32_ATIM_BDTR_OFFSET, 0, ATIM_BDTR_MOE);
    }
#endif 

  return prescaler;
}

static int stm32_cap_setisr(FAR struct stm32_cap_dev_s *dev, xcpt_t handler);
{
  struct stm32_cap_priv_s *priv = (struct stm32_cap_priv_s *)dev;
  int irq;
#ifdef HAVE_ADANCED_TIM 
  int irq_of;
#endif

  ASSERT(dev);

  irq = priv->irq;
#ifdef HAVE_ADANCED_TIM 
  irq_of = priv->irq_of;
#endif

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(irq);
      irq_detach(irq);
#ifdef HAVE_ADANCED_TIM 
      if (priv->irq_of)
        {
          up_disable_irq(irq_of);
          irq_detach(irq_of);
        }
#endif
      return OK;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(irq, handler);
  up_enable_irq(irq);

#ifdef HAVE_ADANCED_TIM 
  if (priv->irq_of)
    {
      irq_attach(priv->irq_of, handler);
      up_enable_irq(priv->irq_of);
    }
#endif

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set the interrupt priority */

  up_prioritize_irq(irq, NVIC_SYSH_PRIORITY_DEFAULT);

#  ifdef HAVE_ADANCED_TIM 
  if (priv->irq_of)
    {
      up_prioritize_irq(irq_of, NVIC_SYSH_PRIORITY_DEFAULT);
    }
#  endif

#endif

  return OK;
}


static void stm32_cap_enableint(FAR struct stm32_cap_dev_s *dev, 
                                stm32_cap_flags_t src, bool on)
{
  struct stm32_cap_priv_s *priv = (struct stm32_cap_priv_s *)dev;
  uint16_t mask;
  ASSERT(dev);

  if (src & STM32_TIM_FLAG_IRQ_OVERFLOW)
      regval |= ATIM_DIER_UIE;
  if (src & STM32_TIM_FLAG_IRQ_CAPTURE_1)
      regval |= ATIM_DIER_CC1IE;
  if (src & STM32_TIM_FLAG_IRQ_CAPTURE_2)
      regval |= ATIM_DIER_CC1IE;
  if (src & STM32_TIM_FLAG_IRQ_CAPTURE_3)
      regval |= ATIM_DIER_CC1IE;
  if (src & STM32_TIM_FLAG_IRQ_CAPTURE_4)
      regval |= ATIM_DIER_CC1IE;

  /* Not IRQ on channel overflow */

  if (on)
    stm32_modifyreg16(priv, STM32_BTIM_DIER_OFFSET,0,mask);
  else
    stm32_modifyreg16(priv, STM32_BTIM_DIER_OFFSET,mask,0);

}

static void stm32_cap_ackflags(FAR struct stm32_cap_dev_s *dev, int src)
{
  struct stm32_cap_priv_s *priv = (struct stm32_cap_priv_s *)dev;
  uint16_t mask = 0;

  if (src & STM32_TIM_FLAG_IRQ_OVERFLOW)
      regval |= ATIM_SR_UIF;

  if (src & STM32_TIM_FLAG_IRQ_CAPTURE_1)
      regval |= ATIM_SR_CC1IF;
  if (src & STM32_TIM_FLAG_IRQ_CAPTURE_2)
      regval |= ATIM_SR_CC2IF;
  if (src & STM32_TIM_FLAG_IRQ_CAPTURE_3)
      regval |= ATIM_SR_CC3IF;
  if (src & STM32_TIM_FLAG_IRQ_CAPTURE_4)
      regval |= ATIM_SR_CC4IF;

  if (src & STM32_TIM_FLAG_OF_CAPTURE_1)
      regval |= ATIM_SR_CC1OF;
  if (src & STM32_TIM_FLAG_OF_CAPTURE_2)
      regval |= ATIM_SR_CC2OF;
  if (src & STM32_TIM_FLAG_OF_CAPTURE_3)
      regval |= ATIM_SR_CC3OF;
  if (src & STM32_TIM_FLAG_OF_CAPTURE_4)
      regval |= ATIM_SR_CC4OF;

  stm32_putreg16(priv, STM32_BTIM_SR_OFFSET, ~mask);

}

static stm32_cap_flags_t stm32_cap_getint(FAR struct stm32_cap_dev_s *dev)
{
  struct stm32_cap_priv_s *priv = (struct stm32_cap_priv_s *)dev;
  uint16_t regval = 0;
  stm32_cap_flags_t src = 0;

  regval = stm32_getreg16(priv, STM32_BTIM_SR_OFFSET);

  if (regval & ATIM_SR_UIF)
      src |= STM32_TIM_FLAG_IRQ_OVERFLOW;

  if (regval & ATIM_SR_CC1IF)
      src |= STM32_TIM_FLAG_IRQ_CAPTURE_1;
  if (regval & ATIM_SR_CC2IF)
      src |= STM32_TIM_FLAG_IRQ_CAPTURE_2;
  if (regval & ATIM_SR_CC3IF)
      src |= STM32_TIM_FLAG_IRQ_CAPTURE_3;
  if (regval & ATIM_SR_CC4IF)
      src |= STM32_TIM_FLAG_IRQ_CAPTURE_4;

  if (regval & ATIM_SR_CC1OF)
      src |= STM32_TIM_FLAG_OF_CAPTURE_1;
  if (regval & ATIM_SR_CC2OF)
      src |= STM32_TIM_FLAG_OF_CAPTURE_2;
  if (regval & ATIM_SR_CC3OF)
      src |= STM32_TIM_FLAG_OF_CAPTURE_3;
  if (regval & ATIM_SR_CC4OF)
      src |= STM32_TIM_FLAG_OF_CAPTURE_4;

  return src;

}

/************************************************************************************
 * General Functions
 ************************************************************************************/

static int stm32_cap_setchannel(FAR struct stm32_cap_dev_s *dev, uint8_t channel,
                                stm32_cap_ch_cfg_t edge)
{
  int i;
  struct stm32_cap_priv_s *priv = (struct stm32_cap_priv_s *)dev;
  uint32_t gpio = 0;
  uint16_t mask;
  uint16_t regval;

  ASSERT(dev);

  gpio = stm32_cap_gpio(priv,channel);

  if ( gpio == 0 ) 
      return ERROR;

  /* change to zero base index */
  channel--;

  /* Set ccer */
  switch (cfg & STM32_CAP_EDGE_MASK)
    {
      case STM32_CAP_EDGE_DISABLED:
        regval = 0;
        break;
      case STM32_CAP_EDGE_RISING:
        regval = GTIM_CCER_CC1E;
        break;
      case STM32_CAP_EDGE_FALLING:
        regval = GTIM_CCER_CC1E | GTIM_CCER_CC1P;
        break;
      case STM32_CAP_EDGE_BOTH:
        regval = GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC1NP;
        break;
      default:
        return ERROR;
    }

  mask = (GTIM_CCER_CC1E | GTIM_CCER_CC1P | GTIM_CCER_CC1NP) 
  mask   <<= (channel << 2);
  regval <<= (channel << 2);
  stm32_modifyreg16(priv,mask,regval);

  /* Set ccmr */

  ccmr_val = cfg;

  if ( (ccmr_val & GTIM_CCMR1_CC1S_MASK ) == 0 )
      return ERROR; /* configured as output */

  if ( (ccmr_val & GTIM_CCMR1_CC1S_MASK ) == 3 )
      return ERROR; /* Not implemented */

  /* Define its position (shift) and get register offset */

  mask = (GTIM_CCMR1_IC1F_MASK | GTIM_CCMR1_IC1PSC_MASK | GTIM_CCMR1_CC1S_MASK)
  ccmr_val &= mask

  if (channel & 1)
    {
      ccmr_val  <<= 8;
      ccmr_mask <<= 8;
    }

  if (channel < 2)
      stm32_modifyreg16(priv,STM32_GTIM_CCMR1_OFFSET,ccmr_mask,ccmr_val);
  else
      stm32_modifyreg16(priv,STM32_GTIM_CCMR2_OFFSET,ccmr_mask,ccmr_val);

  /* set GPIO */
  
  if ( (cfg & STM32_CAP_EDGE_MASK) == STM32_CAP_EDGE_DISABLED)
      stm32_unconfiggpio(gpio);
  else
      stm32_configgpio(gpio);

  return OK;
}


static int stm32_cap_getcapture(FAR struct stm32_cap_dev_s *dev, uint8_t channel)
{
  struct stm32_cap_priv_s *priv = (struct stm32_cap_priv_s *)dev;
  ASSERT(dev);

  switch (channel)
    {
#ifdef HAVE_CH1IN
      case 1:
        return stm32_getreg32(dev, STM32_GTIM_CCR1_OFFSET);
#endif
#ifdef HAVE_CH1IN
      case 2:
        return stm32_getreg32(dev, STM32_GTIM_CCR2_OFFSET);
#endif
#ifdef HAVE_CH1IN
      case 3:
        return stm32_getreg32(dev, STM32_GTIM_CCR3_OFFSET);
#endif
#ifdef HAVE_CH1IN
      case 4:
        return stm32_getreg32(dev, STM32_GTIM_CCR4_OFFSET);
#endif
    }

  return ERROR;
}


/************************************************************************************
 * Advanced Functions
 ************************************************************************************/

/* TODO: Advanced functions for the STM32_ATIM */

/************************************************************************************
 * Device Structures, Instantiation
 ************************************************************************************/

struct stm32_cap_ops_s stm32_cap_ops =
{
  .setclock       = &stm32_cap_setclock,
  .setchannel     = &stm32_cap_setchannel,
  .getcapture     = &stm32_cap_getcapture,
  .setisr         = &stm32_cap_setisr,
  .enableint      = &stm32_cap_enableint,
  .ackflags       = &stm32_cap_ackflags,
  .getflags       = &stm32_cap_getflags
};

#ifdef CONFIG_STM32_TIM2_CAP
const struct stm32_cap_priv_s stm32_tim2_priv =
{
  .ops          = &stm32_cap_ops,
  .base         = STM32_TIM2_BASE,
  .irg          = STM32_IRQ_TIM2,
#define HAVE_ADANCED_TIM 1
  .irg_of       = -1,
#endif
  .gpio_clk     = GPIO_TIM2_CLKIN;
  .channels     = {
#if defined(GPIO_TIM2_CH1IN)
                .gpio   = GPIO_TIM2_CH1IN;
#endif
#if defined(GPIO_TIM2_CH2IN)
                .gpio   = GPIO_TIM2_CH2IN;
                .ccmr = ( GPIO_TIM2_CH2_ICF   << ATIM_CCMR1_IC1F_SHIFT    )|\
                        ( GPIO_TIM2_CH2_ICPSC << ATIM_CCMR1_IC1PSC_SHIFT  );
#endif
#if defined(GPIO_TIM2_CH3IN)
                .gpio   = GPIO_TIM2_CH3IN;
                .ccmr = ( GPIO_TIM2_CH3_ICF   << ATIM_CCMR1_IC1F_SHIFT    )|\
                        ( GPIO_TIM2_CH3_ICPSC << ATIM_CCMR1_IC1PSC_SHIFT  );
#endif
#if defined(GPIO_TIM2_CH4IN)
                .gpio   = GPIO_TIM2_CH4IN;
                .ccmr = ( GPIO_TIM2_CH4_ICF   << ATIM_CCMR1_IC1F_SHIFT    )|\
                        ( GPIO_TIM2_CH4_ICPSC << ATIM_CCMR1_IC1PSC_SHIFT  );
#endif
  }
};
#endif


/************************************************************************************
 * Public Function - Initialization
 ************************************************************************************/

FAR struct stm32_cap_dev_s *stm32_cap_init(int timer)
{
  struct stm32_cap_priv_s *priv = NULL;

  /* Get structure and enable power */

  switch (timer)
    {
#ifdef CONFIG_STM32_TIM1_CAP
      case 1:
        priv = &stm32_tim1_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);
        break;
#endif
#ifdef CONFIG_STM32_TIM2_CAP
      case 2:
        priv = &stm32_tim2_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM2EN);
        break;
#endif
#ifdef CONFIG_STM32_TIM3_CAP
      case 3:
        priv = &stm32_tim3_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM3EN);
        break;
#endif
#ifdef CONFIG_STM32_TIM4_CAP
      case 4:
        priv = &stm32_tim4_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM4EN);
        break;
#endif
#ifdef CONFIG_STM32_TIM5_CAP
      case 5:
        priv = &stm32_tim5_priv;
        modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM5EN);
        break;
#endif
/* TIM6 and TIM7 cannot be used in capture */
#ifdef CONFIG_STM32_TIM8_CAP
      case 8:
        priv = &stm32_tim8_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM8EN);
        break;
#endif
#ifdef CONFIG_STM32_TIM9_CAP
      case 9:
        priv = &stm32_tim9_priv;
        modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM8EN);
        break;
#endif
      default:
        return NULL;
    }

  if (priv->gpio_clk)
      stm32_configgpio(priv->gpio_clk)

  // disable timer while is not configured 
  stm32_modifyreg16(priv, STM32_BTIM_CR1_OFFSET, ATIM_CR1_CEN, 0);

  return (struct stm32_cap_dev_s *)priv;
}


int stm32_cap_deinit(FAR struct stm32_cap_dev_s * dev)
{
  struct stm32_cap_priv_s *priv = (struct stm32_cap_priv_s *)dev;
  ASSERT(dev);

  // disable timer while is not configured 
  stm32_modifyreg16(dev, STM32_BTIM_CR1_OFFSET, ATIM_CR1_CEN, 0);

  if (priv->gpio_clk)
      stm32_unconfiggpio(priv->gpio_clk)

  switch (priv->base)
    {
#ifdef CONFIG_STM32_TIM2_CAP
      case STM32_TIM2_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM2EN, 0);
        break;
#endif
#ifdef CONFIG_STM32_TIM3_CAP
      case STM32_TIM3_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM3EN, 0);
        break;
#endif
#ifdef CONFIG_STM32_TIM4_CAP
      case STM32_TIM4_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM4EN, 0);
        break;
#endif
#ifdef CONFIG_STM32_TIM5_CAP
      case STM32_TIM5_BASE:
        modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM5EN, 0);
        break;
#endif
#ifdef CONFIG_STM32_TIM1_CAP
      case STM32_TIM1_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM1EN, 0);
        break;
#endif
#ifdef CONFIG_STM32_TIM8_CAP
      case STM32_TIM8_BASE:
        modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM8EN, 0);
        break;
#endif
      default:
        return ERROR;
    }

  return OK;
}

#endif /* defined(CONFIG_STM32_TIM1 || ... || TIM8) */
