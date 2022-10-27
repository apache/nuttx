/****************************************************************************
 * arch/sparc/src/s698pm/s698pm_tim.c
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_internal.h"
#include "s698pm.h"
#include "s698pm_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This module then only compiles if there are enabled timers that are not
 * intended for some other purpose.
 */

#if defined(CONFIG_S698PM_TIM1) || defined(CONFIG_S698PM_TIM2) || \
    defined(CONFIG_S698PM_TIM3) || defined(CONFIG_S698PM_TIM4)
/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TIM Device Structure */

struct s698pm_tim_priv_s
{
  const struct s698pm_tim_ops_s *ops;
  enum s698pm_tim_mode_e mode;
  uint32_t base;                      /* TIMn base address */
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

/* Register helpers */

static inline uint16_t s698pm_getreg16(struct s698pm_tim_dev_s *dev,
                                        uint8_t offset);
static inline void s698pm_putreg16(struct s698pm_tim_dev_s *dev,
                                    uint8_t offset, uint16_t value);
static inline void s698pm_modifyreg32(struct s698pm_tim_dev_s *dev,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits);
static inline uint32_t s698pm_getreg32(struct s698pm_tim_dev_s *dev,
                                        uint8_t offset);
static inline void s698pm_putreg32(struct s698pm_tim_dev_s *dev,
                                    uint8_t offset, uint32_t value);

/* Timer helpers */

static void s698pm_tim_reload_counter(struct s698pm_tim_dev_s *dev);
static void s698pm_tim_enable(struct s698pm_tim_dev_s *dev);
static void s698pm_tim_disable(struct s698pm_tim_dev_s *dev);
static void s698pm_tim_reset(struct s698pm_tim_dev_s *dev);

/* Timer methods */

static int s698pm_tim_setmode(struct s698pm_tim_dev_s *dev,
                               enum s698pm_tim_mode_e mode);

static int s698pm_tim_setclock(struct s698pm_tim_dev_s *dev,
                                uint32_t freq);
static uint32_t  s698pm_tim_getclock(struct s698pm_tim_dev_s *dev);
static void s698pm_tim_setperiod(struct s698pm_tim_dev_s *dev,
                                  uint32_t period);
static uint32_t s698pm_tim_getperiod(struct s698pm_tim_dev_s *dev);
static uint32_t s698pm_tim_getcounter(struct s698pm_tim_dev_s *dev);

static int s698pm_tim_setisr(struct s698pm_tim_dev_s *dev,
                              xcpt_t handler, void *arg, int source);

static int s698pm_tim_checkint(struct s698pm_tim_dev_s *dev, int source);
static void s698pm_tim_clrint(struct s698pm_tim_dev_s *dev, int source);
/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct s698pm_tim_ops_s s698pm_tim_ops =
{
  .setmode    = s698pm_tim_setmode,
  .setclock   = s698pm_tim_setclock,
  .getclock   = s698pm_tim_getclock,
  .setperiod  = s698pm_tim_setperiod,
  .getperiod  = s698pm_tim_getperiod,
  .getcounter = s698pm_tim_getcounter,
  .setisr     = s698pm_tim_setisr,
  .clrint     = s698pm_tim_clrint,
  .checkint   = s698pm_tim_checkint,
};

#ifdef CONFIG_S698PM_TIM1
struct s698pm_tim_priv_s s698pm_tim1_priv =
{
  .ops        = &s698pm_tim_ops,
  .mode       = S698PM_TIM_MODE_UNUSED,
  .base       = S698PM_TIM1_BASE,
};
#endif
#ifdef CONFIG_S698PM_TIM2
struct s698pm_tim_priv_s s698pm_tim2_priv =
{
  .ops        = &s698pm_tim_ops,
  .mode       = S698PM_TIM_MODE_UNUSED,
  .base       = S698PM_TIM2_BASE,
};
#endif
#ifdef CONFIG_S698PM_TIM3
struct s698pm_tim_priv_s s698pm_tim3_priv =
{
  .ops        = &s698pm_tim_ops,
  .mode       = S698PM_TIM_MODE_UNUSED,
  .base       = S698PM_TIM3_BASE,
};
#endif
#ifdef CONFIG_S698PM_TIM4
struct s698pm_tim_priv_s s698pm_tim4_priv =
{
  .ops        = &s698pm_tim_ops,
  .mode       = S698PM_TIM_MODE_UNUSED,
  .base       = S698PM_TIM4_BASE,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s698pm_getreg16
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t s698pm_getreg16(struct s698pm_tim_dev_s *dev,
                                        uint8_t offset)
{
  return getreg16(((struct s698pm_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: s698pm_putreg16
 *
 * Description:
 *   Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void s698pm_putreg16(struct s698pm_tim_dev_s *dev,
                                    uint8_t offset, uint16_t value)
{
  putreg16(value, ((struct s698pm_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: s698pm_modifyreg32
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void s698pm_modifyreg32(struct s698pm_tim_dev_s *dev,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits)
{
  modifyreg32(((struct s698pm_tim_priv_s *)dev)->base + offset, clearbits,
              setbits);
}

/****************************************************************************
 * Name: s698pm_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset.  This applies only for the s698pm
 *   32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 *
 ****************************************************************************/

static inline uint32_t s698pm_getreg32(struct s698pm_tim_dev_s *dev,
                                        uint8_t offset)
{
  return getreg32(((struct s698pm_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: s698pm_putreg32
 *
 * Description:
 *   Put a 32-bit register value by offset.  This applies only for the s698pm
 *   32-bit registers (CNT, ARR, CRR1-4) in the 32-bit timers TIM2-5.
 *
 ****************************************************************************/

static inline void s698pm_putreg32(struct s698pm_tim_dev_s *dev,
                                    uint8_t offset, uint32_t value)
{
  putreg32(value, ((struct s698pm_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: s698pm_tim_reload_counter
 ****************************************************************************/

static void s698pm_tim_reload_counter(struct s698pm_tim_dev_s *dev)
{
  uint32_t val = s698pm_getreg32(dev, S698PM_TIM_CR_OFFSET);
  val |= TIMER_LOADCOUNT;
  s698pm_putreg32(dev, S698PM_TIM_CR_OFFSET, val);
}

/****************************************************************************
 * Name: s698pm_tim_enable
 ****************************************************************************/

static void s698pm_tim_enable(struct s698pm_tim_dev_s *dev)
{
  uint32_t val = s698pm_getreg32(dev, S698PM_TIM_CR_OFFSET);
  val |= TIMER_ENABLE | TIMER_RELOADCOUNT;
  s698pm_tim_reload_counter(dev);
  s698pm_putreg32(dev, S698PM_TIM_CR_OFFSET, val);
}

/****************************************************************************
 * Name: s698pm_tim_disable
 ****************************************************************************/

static void s698pm_tim_disable(struct s698pm_tim_dev_s *dev)
{
  uint32_t val = s698pm_getreg32(dev, S698PM_TIM_CR_OFFSET);
  val &= ~TIMER_ENABLE;
  s698pm_putreg32(dev, S698PM_TIM_CR_OFFSET, val);
}

/****************************************************************************
 * Name: s698pm_tim_reset
 *
 * Description:
 * Reset timer into system default state, but do not affect output/input pins
 *
 ****************************************************************************/

static void s698pm_tim_reset(struct s698pm_tim_dev_s *dev)
{
  ((struct s698pm_tim_priv_s *)dev)->mode = S698PM_TIM_MODE_DISABLED;
  s698pm_tim_disable(dev);
}

/****************************************************************************
 * Name: s698pm_tim_setmode
 ****************************************************************************/

static int s698pm_tim_setmode(struct s698pm_tim_dev_s *dev,
                               enum s698pm_tim_mode_e mode)
{
  uint32_t val = s698pm_getreg32(dev, S698PM_TIM_CR_OFFSET);

  DEBUGASSERT(dev != NULL);

  /* Decode operational modes */

  switch (mode & S698PM_TIM_MODE_MASK)
    {
      case S698PM_TIM_MODE_DISABLED:
        val &= (~TIMER_LOADCOUNT) & (~TIMER_RELOADCOUNT) & (~TIMER_ENABLE);
        break;

      case S698PM_TIM_MODE_DOWN:
        val |= (TIMER_LOADCOUNT | TIMER_RELOADCOUNT | TIMER_ENABLE);
        break;

      default:
        return -EINVAL;
    }

  s698pm_tim_reload_counter(dev);
  s698pm_putreg32(dev, S698PM_TIM_CR_OFFSET, val);

  return OK;
}

/****************************************************************************
 * Name: s698pm_tim_setclock
 ****************************************************************************/

static int s698pm_tim_setclock(struct s698pm_tim_dev_s *dev,
                                uint32_t freq)
{
  uint32_t freqin;
  int prescaler;

  DEBUGASSERT(dev != NULL);

  /* Disable Timer? */

  if (freq == 0)
    {
      s698pm_tim_disable(dev);
      return 0;
    }

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (((struct s698pm_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_S698PM_TIM1
      case S698PM_TIM1_BASE:
        freqin = BOARD_TIM1_FREQUENCY;
        break;
#endif

#ifdef CONFIG_S698PM_TIM2
      case S698PM_TIM2_BASE:
        freqin = BOARD_TIM2_FREQUENCY;
        break;
#endif

#ifdef CONFIG_S698PM_TIM3
      case S698PM_TIM3_BASE:
        freqin = BOARD_TIM3_FREQUENCY;
        break;
#endif

#ifdef CONFIG_S698PM_TIM4
      case S698PM_TIM4_BASE:
        freqin = BOARD_TIM4_FREQUENCY;
        break;
#endif

      default:
        return -EINVAL;
    }

  /* Select a pre-scaler value for this timer using the input clock
   * frequency.
   */

  prescaler = freqin / freq;

  /* We need to decrement value for '1', but only, if that will not to
   * cause underflow.
   */

  if (prescaler > 0)
    {
      prescaler--;
    }

  /* Check for overflow as well. */

  if (prescaler > 0x3ff)
    {
      prescaler = 0x3ff;
    }

  putreg32(prescaler, S698PM_TIMPRE_BASE + S698PM_TIM_PSCLOAD_OFFSET);
  putreg32(prescaler, S698PM_TIMPRE_BASE + S698PM_TIM_PSCCONT_OFFSET);
  s698pm_tim_enable(dev);

  return prescaler;
}

/****************************************************************************
 * Name: s698pm_tim_getclock
 ****************************************************************************/

static uint32_t s698pm_tim_getclock(struct s698pm_tim_dev_s *dev)
{
  uint32_t freqin;
  uint32_t clock;
  DEBUGASSERT(dev != NULL);

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (((struct s698pm_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_S698PM_TIM1
      case S698PM_TIM1_BASE:
        freqin = BOARD_TIM1_FREQUENCY;
        break;
#endif

#ifdef CONFIG_S698PM_TIM2
      case S698PM_TIM2_BASE:
        freqin = BOARD_TIM2_FREQUENCY;
        break;
#endif

#ifdef CONFIG_S698PM_TIM3
      case S698PM_TIM3_BASE:
        freqin = BOARD_TIM3_FREQUENCY;
        break;
#endif

#ifdef CONFIG_S698PM_TIM4
      case S698PM_TIM4_BASE:
        freqin = BOARD_TIM4_FREQUENCY;
        break;
#endif

      default:
        return -EINVAL;
    }

  /* From chip datasheet, at page 1179. */

  clock = freqin / ((0x3ff & getreg32(S698PM_TIMPRE_BASE +
          S698PM_TIM_PSCLOAD_OFFSET)) + 1);

  return clock;
}

/****************************************************************************
 * Name: s698pm_tim_setperiod
 ****************************************************************************/

static void s698pm_tim_setperiod(struct s698pm_tim_dev_s *dev,
                                uint32_t period)
{
  DEBUGASSERT(dev != NULL);
  s698pm_putreg32(dev, S698PM_TIM_ARR_OFFSET, period);
  s698pm_putreg32(dev, S698PM_TIM_CNT_OFFSET, period);
}

/****************************************************************************
 * Name: s698pm_tim_getperiod
 ****************************************************************************/

static uint32_t s698pm_tim_getperiod (struct s698pm_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  return s698pm_getreg32(dev, S698PM_TIM_ARR_OFFSET);
}

/****************************************************************************
 * Name: s698pm_tim_getcounter
 ****************************************************************************/

static uint32_t s698pm_tim_getcounter(struct s698pm_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  uint32_t counter = s698pm_getreg32(dev, S698PM_TIM_CNT_OFFSET);

  return counter;
}

/****************************************************************************
 * Name: s698pm_tim_setisr
 ****************************************************************************/

static int s698pm_tim_setisr(struct s698pm_tim_dev_s *dev,
                              xcpt_t handler, void *arg, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (((struct s698pm_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_S698PM_TIM1
      case S698PM_TIM1_BASE:
        vectorno = S698PM_IRQ_TIMER1;
        break;
#endif

#ifdef CONFIG_S698PM_TIM2
      case S698PM_TIM2_BASE:
        vectorno = S698PM_IRQ_TIMER2;
        break;
#endif

#ifdef CONFIG_S698PM_TIM3
      case S698PM_TIM3_BASE:
        vectorno = S698PM_IRQ_TIMER3;
        break;
#endif

#ifdef CONFIG_S698PM_TIM4
      case S698PM_TIM4_BASE:
        vectorno = S698PM_IRQ_TIMER4;
        break;
#endif

      default:
        vectorno = S698PM_IRQ_TIMER1;
        return -EINVAL;
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

  return OK;
}

/****************************************************************************
 * Name: s698pm_tim_clrint
 ****************************************************************************/

static void s698pm_tim_clrint(struct s698pm_tim_dev_s *dev, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (((struct s698pm_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_S698PM_TIM1
      case S698PM_TIM1_BASE:
        vectorno = S698PM_IRQ_TIMER1;
        break;
#endif

#ifdef CONFIG_S698PM_TIM2
      case S698PM_TIM2_BASE:
        vectorno = S698PM_IRQ_TIMER2;
        break;
#endif

#ifdef CONFIG_S698PM_TIM3
      case S698PM_TIM3_BASE:
        vectorno = S698PM_IRQ_TIMER3;
        break;
#endif

#ifdef CONFIG_S698PM_TIM4
      case S698PM_TIM4_BASE:
        vectorno = S698PM_IRQ_TIMER4;
        break;
#endif

      default:
        break;
    }

  up_clrpend_irq(vectorno);
}

/****************************************************************************
 * Name: s698pm_tim_checkint
 ****************************************************************************/

static int s698pm_tim_checkint(struct s698pm_tim_dev_s *dev, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (((struct s698pm_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_S698PM_TIM1
      case S698PM_TIM1_BASE:
        vectorno = S698PM_IRQ_TIMER1;
        break;
#endif

#ifdef CONFIG_S698PM_TIM2
      case S698PM_TIM2_BASE:
        vectorno = S698PM_IRQ_TIMER2;
        break;
#endif

#ifdef CONFIG_S698PM_TIM3
      case S698PM_TIM3_BASE:
        vectorno = S698PM_IRQ_TIMER3;
        break;
#endif

#ifdef CONFIG_S698PM_TIM4
      case S698PM_TIM4_BASE:
        vectorno = S698PM_IRQ_TIMER4;
        break;
#endif

      default:
        return -EINVAL;
    }

  return up_pending_irq(vectorno);
}

/****************************************************************************
 * Pubic Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s698pm_tim_init
 ****************************************************************************/

struct s698pm_tim_dev_s *s698pm_tim_init(int timer)
{
  struct s698pm_tim_dev_s *dev = NULL;

  /* Get structure and enable power */

  switch (timer)
    {
#ifdef CONFIG_S698PM_TIM1
      case 1:
        dev = (struct s698pm_tim_dev_s *)&s698pm_tim1_priv;

        break;
#endif

#ifdef CONFIG_S698PM_TIM2
      case 2:
        dev = (struct s698pm_tim_dev_s *)&s698pm_tim2_priv;

        break;
#endif

#ifdef CONFIG_S698PM_TIM3
      case 3:
        dev = (struct s698pm_tim_dev_s *)&s698pm_tim3_priv;

        break;
#endif

#ifdef CONFIG_S698PM_TIM4
      case 4:
        dev = (struct s698pm_tim_dev_s *)&s698pm_tim4_priv;

        break;
#endif

      default:
        return NULL;
    }

  /* Is device already allocated */

  if (((struct s698pm_tim_priv_s *)dev)->mode != S698PM_TIM_MODE_UNUSED)
    {
      return NULL;
    }

  s698pm_tim_reset(dev);

  return dev;
}

/****************************************************************************
 * Name: s698pm_tim_deinit
 *
 * TODO: Detach interrupts, and close down all TIM Channels
 *
 ****************************************************************************/

int s698pm_tim_deinit(struct s698pm_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  /* Disable power */

  switch (((struct s698pm_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_S698PM_TIM1
      case S698PM_TIM1_BASE:

        break;
#endif

#ifdef CONFIG_S698PM_TIM2
      case S698PM_TIM2_BASE:

        break;
#endif

#ifdef CONFIG_S698PM_TIM3
      case S698PM_TIM3_BASE:

        break;
#endif

#ifdef CONFIG_S698PM_TIM4
      case S698PM_TIM4_BASE:

        break;
#endif

      default:
        return -EINVAL;
    }

  /* Mark it as free */

  ((struct s698pm_tim_priv_s *)dev)->mode = S698PM_TIM_MODE_UNUSED;

  return OK;
}

#endif /* defined(CONFIG_S698PM_TIM1 ||...|| CONFIG_S698PM_TIM4) */
