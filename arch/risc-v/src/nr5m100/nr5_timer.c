/************************************************************************************
 * arm/risc-v/src/nr5m100/nr5_timer.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Modified for RISC-V:
 *
 *   Copyright (C) 2016 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "chip.h"
#include "riscv_internal.h"
#include "riscv_arch.h"

#include "nr5.h"

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* Configuration ********************************************************************/

/* This module then only compiles if there are enabled timers that are not intended
 * for some other purpose.
 */

#if defined(CONFIG_NR5_TIMER0) || defined(CONFIG_NR5_TIMER1) || defined(CONFIG_NR5_TIMER2) || \
    defined(CONFIG_NR5_TIMER3) || defined(CONFIG_NR5_TIMER4) || defined(CONFIG_NR5_TIMER5)

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* Timer Device Structure */

struct nr5_timer_priv_s
{
  struct nr5_timer_ops_s *ops;
  nr5_timer_mode_t        mode;
  uint32_t                base;   /* TIMERn base address */
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/* Get a 16-bit register value by offset */

static inline uint16_t nr5_getreg16(FAR struct nr5_timer_dev_s *dev,
                                      uint8_t offset)
{
  uint16_t  *p16 = (uint16_t *) (((struct nr5_timer_priv_s *)dev)->base + offset);
  return *p16;
}

/* Put a 16-bit register value by offset */

static inline void nr5_putreg16(FAR struct nr5_timer_dev_s *dev, uint8_t offset,
                                  uint16_t value)
{
  uint16_t  *p16 = (uint16_t *) (((struct nr5_timer_priv_s *)dev)->base + offset);
  *p16 = value;
}

/* Modify a 16-bit register value by offset */

static inline void nr5_modifyreg16(FAR struct nr5_timer_dev_s *dev,
                                     uint8_t offset, uint16_t clearbits,
                                     uint16_t setbits)
{
  modifyreg16(((struct nr5_timer_priv_s *)dev)->base + offset, clearbits, setbits);
}

/* Clear the TAR counter */

static void nr5_timer_clear_counter(FAR struct nr5_timer_dev_s *dev)
{
  uint16_t val = nr5_getreg16(dev, NR5_TIMERA_TACTL_OFFSET);
  val |= TIMERA_TACTL_TACLR;
  nr5_putreg16(dev, NR5_TIMERA_TACTL_OFFSET, val);
}

static void nr5_timer_enable(FAR struct nr5_timer_dev_s *dev)
{
  uint16_t val = nr5_getreg16(dev, NR5_BTIM_CR1_OFFSET);
  nr5_timer_clear_counter(dev);
  val |= ATIM_CR1_CEN;
  nr5_putreg16(dev, NR5_BTIM_CR1_OFFSET, val);
}

/* Disable the timer by setting the mode to STOP */

static void nr5_timer_disable(FAR struct nr5_timer_dev_s *dev)
{
  uint16_t val = nr5_getreg16(dev, NR5_TIMERA_TACTL_OFFSET);
  val &= ~TIMERA_TACTL_MC_MASK;
  nr5_putreg16(dev, NR5_TIMERA_TACTL_OFFSET, val);
}

/* Reset timer into system default state, but do not affect output/input pins */

static void nr5_timer_reset(FAR struct nr5_timer_dev_s *dev)
{
  ((struct nr5_timer_priv_s *)dev)->mode = NR5_TIMER_MODE_DISABLED;
  nr5_timer_disable(dev);
}

/************************************************************************************
 * Basic Functions
 ************************************************************************************/

static int nr5_timer_setclock(FAR struct nr5_timer_dev_s *dev, uint32_t freq)
{
  int prescaler;

  DEBUGASSERT(dev);

  /* Disable Timer? */

  if (freq == 0)
    {
      nr5_timer_disable(dev);
      return 0;
    }

#if NR5_NATIM > 0
  if (((struct nr5_timer_priv_s *)dev)->base == NR5_TIM1_BASE ||
      ((struct nr5_timer_priv_s *)dev)->base == NR5_TIM8_BASE)
    {
      prescaler = NR5_TIM18_FREQUENCY / freq;
    }
  else
#endif
    {
      prescaler = NR5_TIM27_FREQUENCY / freq;
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

  nr5_putreg16(dev, NR5_BTIM_PSC_OFFSET, prescaler);
  nr5_timer_enable(dev);

  return prescaler;
}

static void nr5_timer_setperiod(FAR struct nr5_timer_dev_s *dev,
                                uint32_t period)
{
  DEBUGASSERT(dev);
  nr5_putreg32(dev, NR5_BTIM_ARR_OFFSET, period);
}

static int nr5_timer_setisr(FAR struct nr5_timer_dev_s *dev,
                            xcpt_t handler, void * arg, int source)
{
  int vectorno;

  DEBUGASSERT(dev);
  DEBUGASSERT(source == 0);

  switch (((struct nr5_timer_priv_s *)dev)->base)
    {
#ifdef CONFIG_NR5_TIM2
      case NR5_TIM2_BASE:
        vectorno = NR5_IRQ_TIM2;
        break;
#endif
#ifdef CONFIG_NR5_TIM3
      case NR5_TIM3_BASE:
        vectorno = NR5_IRQ_TIM3;
        break;
#endif
#ifdef CONFIG_NR5_TIM4
      case NR5_TIM4_BASE:
        vectorno = NR5_IRQ_TIM4;
        break;
#endif
#ifdef CONFIG_NR5_TIM5
      case NR5_TIM5_BASE:
        vectorno = NR5_IRQ_TIM5;
        break;
#endif
#if NR5_NBTIM > 0
#ifdef CONFIG_NR5_TIM6
      case NR5_TIM6_BASE:
        vectorno = NR5_IRQ_TIM6;
        break;
#endif
#endif
#if NR5_NBTIM > 1
#ifdef CONFIG_NR5_TIM7
      case NR5_TIM7_BASE:
        vectorno = NR5_IRQ_TIM7;
        break;
#endif
#endif
#if NR5_NATIM > 0
      /* TODO: add support for multiple sources and callbacks */

#ifdef CONFIG_NR5_TIM1
      case NR5_TIM1_BASE:
        vectorno = NR5_IRQ_TIM1UP;
        break;
#endif
#ifdef CONFIG_NR5_TIM8
      case NR5_TIM8_BASE:
        vectorno = NR5_IRQ_TIM8UP;
        break;
#endif
#endif
      default:
        return ERROR;
    }

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(vectorno);
      irq_detach(vectorno);
      return OK;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(vectorno, handler, arg);
  up_enable_irq(vectorno);

#ifdef CONFIG_ARCH_IRQPRIO
  /* Set the interrupt priority */

  up_prioritize_irq(vectorno, NVIC_SYSH_PRIORITY_DEFAULT);
#endif

  return OK;
}

static void nr5_timer_enableint(FAR struct nr5_timer_dev_s *dev, int source)
{
  DEBUGASSERT(dev);
  nr5_modifyreg16(dev, NR5_BTIM_DIER_OFFSET, 0, ATIM_DIER_UIE);
}

static void nr5_timer_disableint(FAR struct nr5_timer_dev_s *dev, int source)
{
  DEBUGASSERT(dev);
  nr5_modifyreg16(dev, NR5_BTIM_DIER_OFFSET, ATIM_DIER_UIE, 0);
}

static void nr5_timer_ackint(FAR struct nr5_timer_dev_s *dev, int source)
{
  nr5_putreg16(dev, NR5_BTIM_SR_OFFSET, ~ATIM_SR_UIF);
}

/************************************************************************************
 * General Functions
 ************************************************************************************/

static int nr5_timer_setmode(FAR struct nr5_timer_dev_s *dev, nr5_timer_mode_t mode)
{
  uint16_t val = ATIM_CR1_CEN | ATIM_CR1_ARPE;

  DEBUGASSERT(dev);

  /* Decode operational modes */

  switch (mode & NR5_TIMER_MODE_MASK)
    {
      case NR5_TIMER_MODE_DISABLED:
        val = 0;
        break;

      case NR5_TIMER_MODE_DOWN:
        val |= ATIM_CR1_DIR;

      case NR5_TIMER_MODE_UP:
        break;

      case NR5_TIMER_MODE_UPDOWN:
        val |= ATIM_CR1_CENTER1;

        /* Our default: Interrupts are generated on compare, when counting
         * down
         */

        break;

      case NR5_TIMER_MODE_PULSE:
        val |= ATIM_CR1_OPM;
        break;

      default: return ERROR;
    }

  nr5_timer_clear_counter(dev);
  nr5_putreg16(dev, NR5_BTIM_CR1_OFFSET, val);

  return OK;
}

/************************************************************************************
 * Device Structures, Instantiation
 ************************************************************************************/

struct nr5_timer_ops_s nr5_timer_ops =
{
  .setmode        = &nr5_timer_setmode,
  .setclock       = &nr5_timer_setclock,
  .setperiod      = &nr5_timer_setperiod,
  .setisr         = &nr5_timer_setisr,
  .enableint      = &nr5_timer_enableint,
  .disableint     = &nr5_timer_disableint,
  .ackint         = &nr5_timer_ackint
};

#ifdef CONFIG_NR5_TIMER1
struct nr5_timer_priv_s nr5_timer1_priv =
{
  .ops        = &nr5_timer_ops,
  .mode       = NR5_TIMER_MODE_UNUSED,
  .base       = NR5_TIMER1_BASE,
};
#endif

#ifdef CONFIG_NR5_TIMER2
struct nr5_timer_priv_s nr5_timer2_priv =
{
  .ops        = &nr5_timer_ops,
  .mode       = NR5_TIMER_MODE_UNUSED,
  .base       = NR5_TIMER2_BASE,
};
#endif

#ifdef CONFIG_NR5_TIMER3
struct nr5_timer_priv_s nr5_timer3_priv =
{
  .ops        = &nr5_timer_ops,
  .mode       = NR5_TIMER_MODE_UNUSED,
  .base       = NR5_TIMER3_BASE,
};
#endif

#ifdef CONFIG_NR5_TIMER4
struct nr5_timer_priv_s nr5_timer4_priv =
{
  .ops        = &nr5_timer_ops,
  .mode       = NR5_TIMER_MODE_UNUSED,
  .base       = NR5_TIMER4_BASE,
};
#endif

#ifdef CONFIG_NR5_TIMER5
struct nr5_timer_priv_s nr5_timer5_priv =
{
  .ops        = &nr5_timer_ops,
  .mode       = NR5_TIMER_MODE_UNUSED,
  .base       = NR5_TIMER5_BASE,
};
#endif

/************************************************************************************
 * Public Function - Initialization
 ************************************************************************************/

FAR struct nr5_timer_dev_s *nr5_timer_init(int timer)
{
  struct nr5_timer_dev_s *dev = NULL;

  /* Get structure pointer */

  switch (timer)
    {
#ifdef CONFIG_NR5_TIMER1
      case 1:
        dev = (struct nr5_timer_dev_s *)&nr5_timer1_priv;
        break;
#endif
#ifdef CONFIG_NR5_TIMER2
      case 2:
        dev = (struct nr5_timer_dev_s *)&nr5_timer2_priv;
        break;
#endif
#ifdef CONFIG_NR5_TIMER3
      case 3:
        dev = (struct nr5_timer_dev_s *)&nr5_timer3_priv;
        break;
#endif
#ifdef CONFIG_NR5_TIMER4
      case 4:
        dev = (struct nr5_timer_dev_s *)&nr5_timer4_priv;
        break;
#endif
#ifdef CONFIG_NR5_TIMER5
      case 5:
        dev = (struct nr5_timer_dev_s *)&nr5_timer5_priv;
        break;
#endif

      default:
        return NULL;
    }

  /* Is device already allocated */

  if (((struct nr5_timer_priv_s *)dev)->mode != NR5_TIMER_MODE_UNUSED)
    {
      return NULL;
    }

  nr5_timer_reset(dev);

  return dev;
}

/* TODO: Detach interrupts, and close down all TIM Channels */

int nr5_timer_deinit(FAR struct nr5_timer_dev_s * dev)
{
  DEBUGASSERT(dev);

  /* Set timer mode to STOP */

  switch (((struct nr5_timer_priv_s *)dev)->base)
    {
#if NR5_NATIM > 0
#ifdef CONFIG_NR5_TIM1
      case NR5_TIMER1_BASE:
        modifyreg32(NR5_RCC_APB2ENR, RCC_APB2ENR_TIM1EN, 0);
        break;
#endif
#ifdef CONFIG_NR5_TIM2
      case NR5_TIMER2_BASE:
        modifyreg32(NR5_RCC_APB1ENR, RCC_APB1ENR_TIM2EN, 0);
        break;
#endif
#ifdef CONFIG_NR5_TIM3
      case NR5_TIMER3_BASE:
        modifyreg32(NR5_RCC_APB1ENR, RCC_APB1ENR_TIM3EN, 0);
        break;
#endif
#ifdef CONFIG_NR5_TIM4
      case NR5_TIMER4_BASE:
        modifyreg32(NR5_RCC_APB1ENR, RCC_APB1ENR_TIM4EN, 0);
        break;
#endif
#ifdef CONFIG_NR5_TIM5
      case NR5_TIMER5_BASE:
        modifyreg32(NR5_RCC_APB1ENR, RCC_APB1ENR_TIM5EN, 0);
        break;
#endif
#endif

      default:
        return ERROR;
    }

  /* Mark it as free */

  ((struct nr5_timer_priv_s *)dev)->mode = NR5_TIMER_MODE_UNUSED;

  return OK;
}

#endif /* defined(CONFIG_NR5_TIM1 || ... || TIM8) */
