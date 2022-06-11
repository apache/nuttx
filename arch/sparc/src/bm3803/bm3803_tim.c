/****************************************************************************
 * arch/sparc/src/bm3803/bm3803_tim.c
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
#include "bm3803.h"
#include "bm3803_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This module then only compiles if there are enabled timers that are not
 * intended for some other purpose.
 */

#if defined(CONFIG_BM3803_TIM1)  || defined(CONFIG_BM3803_TIM2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TIM Device Structure */

struct bm3803_tim_priv_s
{
  const struct bm3803_tim_ops_s *ops;
  enum bm3803_tim_mode_e mode;
  uint32_t base;                      /* TIMn base address */
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

/* Register helpers */

static inline uint16_t bm3803_getreg16(struct bm3803_tim_dev_s *dev,
                                        uint8_t offset);
static inline void bm3803_putreg16(struct bm3803_tim_dev_s *dev,
                                    uint8_t offset, uint16_t value);
static inline void bm3803_modifyreg32(struct bm3803_tim_dev_s *dev,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits);
static inline uint32_t bm3803_getreg32(struct bm3803_tim_dev_s *dev,
                                        uint8_t offset);
static inline void bm3803_putreg32(struct bm3803_tim_dev_s *dev,
                                    uint8_t offset, uint32_t value);

/* Timer helpers */

static void bm3803_tim_reload_counter(struct bm3803_tim_dev_s *dev);
static void bm3803_tim_enable(struct bm3803_tim_dev_s *dev);
static void bm3803_tim_disable(struct bm3803_tim_dev_s *dev);
static void bm3803_tim_reset(struct bm3803_tim_dev_s *dev);

/* Timer methods */

static int bm3803_tim_setmode(struct bm3803_tim_dev_s *dev,
                               enum bm3803_tim_mode_e mode);

static int bm3803_tim_setclock(struct bm3803_tim_dev_s *dev,
                                uint32_t freq);
static uint32_t  bm3803_tim_getclock(struct bm3803_tim_dev_s *dev);
static void bm3803_tim_setperiod(struct bm3803_tim_dev_s *dev,
                                  uint32_t period);
static uint32_t bm3803_tim_getperiod(struct bm3803_tim_dev_s *dev);
static uint32_t bm3803_tim_getcounter(struct bm3803_tim_dev_s *dev);

static int bm3803_tim_setisr(struct bm3803_tim_dev_s *dev,
                              xcpt_t handler, void *arg, int source);

static int bm3803_tim_checkint(struct bm3803_tim_dev_s *dev, int source);
static void bm3803_tim_clrint(struct bm3803_tim_dev_s *dev, int source);
/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct bm3803_tim_ops_s bm3803_tim_ops =
{
  .setmode    = bm3803_tim_setmode,
  .setclock   = bm3803_tim_setclock,
  .getclock   = bm3803_tim_getclock,
  .setperiod  = bm3803_tim_setperiod,
  .getperiod  = bm3803_tim_getperiod,
  .getcounter = bm3803_tim_getcounter,
  .setisr     = bm3803_tim_setisr,
  .clrint     = bm3803_tim_clrint,
  .checkint   = bm3803_tim_checkint,
};

#ifdef CONFIG_BM3803_TIM1
struct bm3803_tim_priv_s bm3803_tim1_priv =
{
  .ops        = &bm3803_tim_ops,
  .mode       = BM3803_TIM_MODE_UNUSED,
  .base       = BM3803_TIM1_BASE,
};
#endif
#ifdef CONFIG_BM3803_TIM2
struct bm3803_tim_priv_s bm3803_tim2_priv =
{
  .ops        = &bm3803_tim_ops,
  .mode       = BM3803_TIM_MODE_UNUSED,
  .base       = BM3803_TIM2_BASE,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm3803_getreg16
 *
 * Description:
 *   Get a 16-bit register value by offset
 *
 ****************************************************************************/

static inline uint16_t bm3803_getreg16(struct bm3803_tim_dev_s *dev,
                                        uint8_t offset)
{
  return getreg16(((struct bm3803_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: bm3803_putreg16
 *
 * Description:
 *   Put a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void bm3803_putreg16(struct bm3803_tim_dev_s *dev,
                                    uint8_t offset, uint16_t value)
{
  putreg16(value, ((struct bm3803_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: bm3803_modifyreg32
 *
 * Description:
 *   Modify a 16-bit register value by offset
 *
 ****************************************************************************/

static inline void bm3803_modifyreg32(struct bm3803_tim_dev_s *dev,
                                       uint8_t offset, uint32_t clearbits,
                                       uint32_t setbits)
{
  modifyreg32(((struct bm3803_tim_priv_s *)dev)->base + offset, clearbits,
              setbits);
}

/****************************************************************************
 * Name: bm3803_getreg32
 *
 * Description:
 *   Get a 24-bit register value by offset.  This applies only for the bm3803
 *   24-bit registers (CNT, ARR, CRR1-4) in the 24-bit timers TIM2-5.
 *
 ****************************************************************************/

static inline uint32_t bm3803_getreg32(struct bm3803_tim_dev_s *dev,
                                        uint8_t offset)
{
  return getreg32(((struct bm3803_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: bm3803_putreg32
 *
 * Description:
 *   Put a 24-bit register value by offset.  This applies only for the bm3803
 *   24-bit registers (CNT, ARR, CRR1-4) in the 24-bit timers TIM2-5.
 *
 ****************************************************************************/

static inline void bm3803_putreg32(struct bm3803_tim_dev_s *dev,
                                    uint8_t offset, uint32_t value)
{
  putreg32(value, ((struct bm3803_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: bm3803_tim_reload_counter
 ****************************************************************************/

static void bm3803_tim_reload_counter(struct bm3803_tim_dev_s *dev)
{
  uint32_t val = bm3803_getreg32(dev, BM3803_TIM_CR_OFFSET);
  val |= TIMER_LOADCOUNT;
  bm3803_putreg32(dev, BM3803_TIM_CR_OFFSET, val);
}

/****************************************************************************
 * Name: bm3803_tim_enable
 ****************************************************************************/

static void bm3803_tim_enable(struct bm3803_tim_dev_s *dev)
{
  uint32_t val = bm3803_getreg32(dev, BM3803_TIM_CR_OFFSET);
  val |= TIMER_ENABLE | TIMER_RELOADCOUNT;
  bm3803_tim_reload_counter(dev);
  bm3803_putreg32(dev, BM3803_TIM_CR_OFFSET, val);
}

/****************************************************************************
 * Name: bm3803_tim_disable
 ****************************************************************************/

static void bm3803_tim_disable(struct bm3803_tim_dev_s *dev)
{
  uint32_t val = bm3803_getreg32(dev, BM3803_TIM_CR_OFFSET);
  val &= ~TIMER_ENABLE;
  bm3803_putreg32(dev, BM3803_TIM_CR_OFFSET, val);
}

/****************************************************************************
 * Name: bm3803_tim_reset
 *
 * Description:
 * Reset timer into system default state, but do not affect output/input pins
 *
 ****************************************************************************/

static void bm3803_tim_reset(struct bm3803_tim_dev_s *dev)
{
  ((struct bm3803_tim_priv_s *)dev)->mode = BM3803_TIM_MODE_DISABLED;
  bm3803_tim_disable(dev);
}

/****************************************************************************
 * Name: bm3803_tim_setmode
 ****************************************************************************/

static int bm3803_tim_setmode(struct bm3803_tim_dev_s *dev,
                               enum bm3803_tim_mode_e mode)
{
  uint32_t val = bm3803_getreg32(dev, BM3803_TIM_CR_OFFSET);

  DEBUGASSERT(dev != NULL);

  /* Decode operational modes */

  switch (mode & BM3803_TIM_MODE_MASK)
    {
      case BM3803_TIM_MODE_DISABLED:
        val &= (~TIMER_LOADCOUNT) & (~TIMER_RELOADCOUNT) & (~TIMER_ENABLE);
        break;

      case BM3803_TIM_MODE_DOWN:
        val |= (TIMER_LOADCOUNT | TIMER_RELOADCOUNT | TIMER_ENABLE);
        break;

      default:
        return -EINVAL;
    }

  bm3803_tim_reload_counter(dev);
  bm3803_putreg32(dev, BM3803_TIM_CR_OFFSET, val);

  return OK;
}

/****************************************************************************
 * Name: bm3803_tim_setclock
 ****************************************************************************/

static int bm3803_tim_setclock(struct bm3803_tim_dev_s *dev,
                                uint32_t freq)
{
  uint32_t freqin;
  int prescaler;

  DEBUGASSERT(dev != NULL);

  /* Disable Timer? */

  if (freq == 0)
    {
      bm3803_tim_disable(dev);
      return 0;
    }

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (((struct bm3803_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_BM3803_TIM1
      case BM3803_TIM1_BASE:
        freqin = BOARD_TIM1_FREQUENCY;
        break;
#endif

#ifdef CONFIG_BM3803_TIM2
      case BM3803_TIM2_BASE:
        freqin = BOARD_TIM2_FREQUENCY;
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

  putreg32(prescaler, BM3803_TIM12PRE_BASE + BM3803_TIM_PSCLOAD_OFFSET);
  putreg32(prescaler, BM3803_TIM12PRE_BASE + BM3803_TIM_PSCCONT_OFFSET);
  bm3803_tim_enable(dev);

  return prescaler;
}

/****************************************************************************
 * Name: bm3803_tim_getclock
 ****************************************************************************/

static uint32_t bm3803_tim_getclock(struct bm3803_tim_dev_s *dev)
{
  uint32_t freqin;
  uint32_t clock;
  DEBUGASSERT(dev != NULL);

  /* Get the input clock frequency for this timer.  These vary with
   * different timer clock sources, MCU-specific timer configuration, and
   * board-specific clock configuration.  The correct input clock frequency
   * must be defined in the board.h header file.
   */

  switch (((struct bm3803_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_BM3803_TIM1
      case BM3803_TIM1_BASE:
        freqin = BOARD_TIM1_FREQUENCY;
        break;
#endif

#ifdef CONFIG_BM3803_TIM2
      case BM3803_TIM2_BASE:
        freqin = BOARD_TIM2_FREQUENCY;
        break;
#endif
      default:
        return -EINVAL;
    }

  /* From chip datasheet, at page 1179. */

  clock = freqin / ((0x3ff & getreg32(BM3803_TIM12PRE_BASE +
          BM3803_TIM_PSCLOAD_OFFSET)) + 1);

  return clock;
}

/****************************************************************************
 * Name: bm3803_tim_setperiod
 ****************************************************************************/

static void bm3803_tim_setperiod(struct bm3803_tim_dev_s *dev,
                                uint32_t period)
{
  DEBUGASSERT(dev != NULL);
  bm3803_putreg32(dev, BM3803_TIM_ARR_OFFSET, period);
  bm3803_putreg32(dev, BM3803_TIM_CNT_OFFSET, period);
}

/****************************************************************************
 * Name: bm3803_tim_getperiod
 ****************************************************************************/

static uint32_t bm3803_tim_getperiod (struct bm3803_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  return 0xffffff & bm3803_getreg32(dev, BM3803_TIM_ARR_OFFSET);
}

/****************************************************************************
 * Name: bm3803_tim_getcounter
 ****************************************************************************/

static uint32_t bm3803_tim_getcounter(struct bm3803_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);
  uint32_t counter = bm3803_getreg32(dev, BM3803_TIM_CNT_OFFSET);

  return counter & 0x00ffffff;
}

/****************************************************************************
 * Name: bm3803_tim_setisr
 ****************************************************************************/

static int bm3803_tim_setisr(struct bm3803_tim_dev_s *dev,
                              xcpt_t handler, void *arg, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (((struct bm3803_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_BM3803_TIM1
      case BM3803_TIM1_BASE:
        vectorno = BM3803_IRQ_TIMER1;
        break;
#endif

#ifdef CONFIG_BM3803_TIM2
      case BM3803_TIM2_BASE:
        vectorno = BM3803_IRQ_TIMER2;
        break;
#endif

      default:
        vectorno = BM3803_IRQ_TIMER1;
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
 * Name: bm3803_tim_clrint
 ****************************************************************************/

static void bm3803_tim_clrint(struct bm3803_tim_dev_s *dev, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (((struct bm3803_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_BM3803_TIM1
      case BM3803_TIM1_BASE:
        vectorno = BM3803_IRQ_TIMER1;
        break;
#endif

#ifdef CONFIG_BM3803_TIM2
      case BM3803_TIM2_BASE:
        vectorno = BM3803_IRQ_TIMER2;
        break;
#endif

      default:
        break;
    }

  up_clrpend_irq(vectorno);
}

/****************************************************************************
 * Name: bm3803_tim_checkint
 ****************************************************************************/

static int bm3803_tim_checkint(struct bm3803_tim_dev_s *dev, int source)
{
  int vectorno;

  DEBUGASSERT(dev != NULL);
  DEBUGASSERT(source == 0);

  switch (((struct bm3803_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_BM3803_TIM1
      case BM3803_TIM1_BASE:
        vectorno = BM3803_IRQ_TIMER1;
        break;
#endif

#ifdef CONFIG_BM3803_TIM2
      case BM3803_TIM2_BASE:
        vectorno = BM3803_IRQ_TIMER2;
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
 * Name: bm3803_tim_init
 ****************************************************************************/

struct bm3803_tim_dev_s *bm3803_tim_init(int timer)
{
  struct bm3803_tim_dev_s *dev = NULL;

  /* Get structure and enable power */

  switch (timer)
    {
#ifdef CONFIG_BM3803_TIM1
      case 1:
        dev = (struct bm3803_tim_dev_s *)&bm3803_tim1_priv;

        break;
#endif

#ifdef CONFIG_BM3803_TIM2
      case 2:
        dev = (struct bm3803_tim_dev_s *)&bm3803_tim2_priv;

        break;
#endif

      default:
        return NULL;
    }

  /* Is device already allocated */

  if (((struct bm3803_tim_priv_s *)dev)->mode != BM3803_TIM_MODE_UNUSED)
    {
      return NULL;
    }

  bm3803_tim_reset(dev);

  return dev;
}

/****************************************************************************
 * Name: bm3803_tim_deinit
 *
 * TODO: Detach interrupts, and close down all TIM Channels
 *
 ****************************************************************************/

int bm3803_tim_deinit(struct bm3803_tim_dev_s *dev)
{
  DEBUGASSERT(dev != NULL);

  /* Disable power */

  switch (((struct bm3803_tim_priv_s *)dev)->base)
    {
#ifdef CONFIG_BM3803_TIM1
      case BM3803_TIM1_BASE:

        break;
#endif

#ifdef CONFIG_BM3803_TIM2
      case BM3803_TIM2_BASE:

        break;
#endif
      default:
        return -EINVAL;
    }

  /* Mark it as free */

  ((struct bm3803_tim_priv_s *)dev)->mode = BM3803_TIM_MODE_UNUSED;

  return OK;
}

#endif /* defined(CONFIG_BM3803_TIM1 || CONFIG_BM3803_TIM2) */
