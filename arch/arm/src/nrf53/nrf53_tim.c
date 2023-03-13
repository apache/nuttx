/****************************************************************************
 * arch/arm/src/nrf53/nrf53_tim.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/nrf53_tim.h"

#include "nrf53_tim.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf53_tim_priv_s
{
  struct nrf53_tim_ops_s *ops;
  uint32_t                base;
  uint32_t                irq;
  uint8_t                 chan;
  bool                    inuse;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TIM registers access *****************************************************/

static uint32_t nrf53_tim_getreg(struct nrf53_tim_dev_s *dev,
                                 uint32_t offset);
static void nrf53_tim_putreg(struct nrf53_tim_dev_s *dev,
                             uint32_t offset,
                             uint32_t value);

/* TIM helpers **************************************************************/

static uint32_t nrf53_tim_irq2reg(struct nrf53_tim_dev_s *dev,
                                  uint8_t s);

/* TIM operations ***********************************************************/

static int nrf53_tim_start(struct nrf53_tim_dev_s *dev);
static int nrf53_tim_stop(struct nrf53_tim_dev_s *dev);
static int nrf53_tim_clear(struct nrf53_tim_dev_s *dev);
static int nrf53_tim_configure(struct nrf53_tim_dev_s *dev, uint8_t mode,
                               uint8_t width);
static int nrf53_tim_shorts(struct nrf53_tim_dev_s *dev, uint8_t s,
                            uint8_t i, bool en);
static int nrf53_tim_count(struct nrf53_tim_dev_s *dev);
static int nrf53_tim_setcc(struct nrf53_tim_dev_s *dev, uint8_t i,
                           uint32_t cc);
static int nrf53_tim_getcc(struct nrf53_tim_dev_s *dev, uint8_t i,
                           uint32_t *cc);
static int nrf53_tim_setpre(struct nrf53_tim_dev_s *dev, uint8_t pre);
static int nrf53_tim_setisr(struct nrf53_tim_dev_s *dev, xcpt_t handler,
                            void * arg);
static int nrf53_tim_enableint(struct nrf53_tim_dev_s *dev, uint8_t s);
static int nrf53_tim_disableint(struct nrf53_tim_dev_s *dev, uint8_t s);
static int nrf53_tim_checkint(struct nrf53_tim_dev_s *dev, uint8_t s);
static int nrf53_tim_ackint(struct nrf53_tim_dev_s *dev, uint8_t s);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NRF53 TIM ops */

struct nrf53_tim_ops_s nrf53_tim_ops =
{
  .start      = nrf53_tim_start,
  .stop       = nrf53_tim_stop,
  .clear      = nrf53_tim_clear,
  .configure  = nrf53_tim_configure,
  .shorts     = nrf53_tim_shorts,
  .count      = nrf53_tim_count,
  .setcc      = nrf53_tim_setcc,
  .getcc      = nrf53_tim_getcc,
  .setpre     = nrf53_tim_setpre,
  .setisr     = nrf53_tim_setisr,
  .enableint  = nrf53_tim_enableint,
  .disableint = nrf53_tim_disableint,
  .checkint   = nrf53_tim_checkint,
  .ackint     = nrf53_tim_ackint
};

#ifdef CONFIG_NRF53_TIMER0
/* TIMER0 */

struct nrf53_tim_priv_s g_nrf53_tim0_priv =
{
  .ops   = &nrf53_tim_ops,
  .base  = NRF53_TIMER0_BASE,
  .irq   = NRF53_IRQ_TIMER0,
  .chan  = 4,
  .inuse = false,
};
#endif

#ifdef CONFIG_NRF53_TIMER1
/* TIMER1 */

struct nrf53_tim_priv_s g_nrf53_tim1_priv =
{
  .ops   = &nrf53_tim_ops,
  .base  = NRF53_TIMER1_BASE,
  .irq   = NRF53_IRQ_TIMER1,
  .chan  = 4,
  .inuse = false,
};
#endif

#ifdef CONFIG_NRF53_TIMER2
/* TIMER2 */

struct nrf53_tim_priv_s g_nrf53_tim2_priv =
{
  .ops   = &nrf53_tim_ops,
  .base  = NRF53_TIMER2_BASE,
  .irq   = NRF53_IRQ_TIMER2,
  .chan  = 4,
  .inuse = false,
};
#endif

#ifdef CONFIG_NRF53_TIMER3
/* TIMER3 */

struct nrf53_tim_priv_s g_nrf53_tim3_priv =
{
  .ops   = &nrf53_tim_ops,
  .base  = NRF53_TIMER3_BASE,
  .irq   = NRF53_IRQ_TIMER3,
  .chan  = 6,
  .inuse = false,
};
#endif

#ifdef CONFIG_NRF53_TIMER4
/* TIMER4 */

struct nrf53_tim_priv_s g_nrf53_tim4_priv =
{
  .ops   = &nrf53_tim_ops,
  .base  = NRF53_TIMER4_BASE,
  .irq   = NRF53_IRQ_TIMER4,
  .chan  = 6,
  .inuse = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_tim_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static uint32_t nrf53_tim_getreg(struct nrf53_tim_dev_s *dev,
                                 uint32_t offset)
{
  DEBUGASSERT(dev);

  return getreg32(((struct nrf53_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: nrf53_tim_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static void nrf53_tim_putreg(struct nrf53_tim_dev_s *dev,
                             uint32_t offset,
                             uint32_t value)
{
  DEBUGASSERT(dev);

  putreg32(value, ((struct nrf53_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: nrf53_tim_irq2reg
 *
 * Description:
 *   Get the value of the interrupt register corresponding to the given
 *   interrupt source
 *
 ****************************************************************************/

static uint32_t nrf53_tim_irq2reg(struct nrf53_tim_dev_s *dev, uint8_t s)
{
  uint32_t regval = 0;

  switch (s)
    {
      case NRF53_TIM_INT_COMPARE0:
        {
          regval = TIM_INT_COMPARE(0);
          break;
        }

      case NRF53_TIM_INT_COMPARE1:
        {
          regval = TIM_INT_COMPARE(1);
          break;
        }

      case NRF53_TIM_INT_COMPARE2:
        {
          regval = TIM_INT_COMPARE(2);
          break;
        }

      case NRF53_TIM_INT_COMPARE3:
        {
          regval = TIM_INT_COMPARE(3);
          break;
        }

      case NRF53_TIM_INT_COMPARE4:
        {
          regval = TIM_INT_COMPARE(4);
          break;
        }

      case NRF53_TIM_INT_COMPARE5:
        {
          regval = TIM_INT_COMPARE(5);
          break;
        }

      default:
        {
          tmrerr("ERROR: unsupported IRQ source %d\n", s);
          regval = 0;
          goto errout;
        }
    }

errout:
  return regval;
}

/****************************************************************************
 * Name: nrf53_tim_start
 ****************************************************************************/

static int nrf53_tim_start(struct nrf53_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf53_tim_putreg(dev, NRF53_TIM_TASKS_START_OFFSET, TIM_TASKS_START);

  return OK;
}

/****************************************************************************
 * Name: nrf53_tim_stop
 ****************************************************************************/

static int nrf53_tim_stop(struct nrf53_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf53_tim_putreg(dev, NRF53_TIM_TASKS_STOP_OFFSET, TIM_TASKS_STOP);

  return OK;
}

/****************************************************************************
 * Name: nrf53_tim_clear
 ****************************************************************************/

static int nrf53_tim_clear(struct nrf53_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf53_tim_putreg(dev, NRF53_TIM_TASKS_CLEAR_OFFSET, TIM_TASKS_CLEAR);

  return OK;
}

/****************************************************************************
 * Name: nrf53_tim_configure
 ****************************************************************************/

static int nrf53_tim_configure(struct nrf53_tim_dev_s *dev, uint8_t mode,
                               uint8_t width)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Configure TIMER mode */

  switch (mode)
    {
      case NRF53_TIM_MODE_UNUSED:
        {
          regval = 0;
          break;
        }

      case NRF53_TIM_MODE_TIMER:
        {
          regval = TIM_MODE_TIMER;
          break;
        }

      case NRF53_TIM_MODE_COUNTER:
        {
          regval = TIM_MODE_COUNTER;
          break;
        }

      case NRF53_TIM_MODE_LOWPOWER:
        {
          regval = TIM_MODE_LPCOUNTER;
          break;
        }

      default:
        {
          tmrerr("ERROR: unsupported TIMER mode %d\n", mode);
          ret = -EINVAL;
          goto errout;
        }
    }

  nrf53_tim_putreg(dev, NRF53_TIM_MODE_OFFSET, regval);

  /* Configure TIMER width */

  switch (width)
    {
      case NRF53_TIM_WIDTH_16B:
        {
          regval = TIM_BITMODE_16B;
          break;
        }

      case NRF53_TIM_WIDTH_8B:
        {
          regval = TIM_BITMODE_8B;
          break;
        }

      case NRF53_TIM_WIDTH_24B:
        {
          regval = TIM_BITMODE_24B;
          break;
        }

      case NRF53_TIM_WIDTH_32B:
        {
          regval = TIM_BITMODE_32B;
          break;
        }

      default:
        {
          tmrerr("ERROR: unsupported TIMER width %d\n", width);
          ret = -EINVAL;
          goto errout;
        }
    }

  nrf53_tim_putreg(dev, NRF53_TIM_BITMODE_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf53_tim_shorts
 ****************************************************************************/

static int nrf53_tim_shorts(struct nrf53_tim_dev_s *dev, uint8_t s,
                            uint8_t i, bool en)
{
  uint32_t regval = 0;
  uint32_t val    = 0;
  int      ret    = OK;

  regval = nrf53_tim_getreg(dev, NRF53_TIM_SHORTS_OFFSET);

  switch (s)
    {
      case NRF53_TIM_SHORT_COMPARE_CLEAR:
        {
          val = TIM_SHORTS_COMPARE_CLEAR(i);
          break;
        }

      case NRF53_TIM_SHORT_COMPARE_STOP:
        {
          val = TIM_SHORTS_COMPARE_STOP(i);
          break;
        }

      default:
        {
          tmrerr("ERROR: unsupported SHORT %d\n", s);
          ret = -EINVAL;
          goto errout;
        }
    }

  if (en == true)
    {
      regval |= val;
    }
  else
    {
      regval &= ~val;
    }

  nrf53_tim_putreg(dev, NRF53_TIM_SHORTS_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf53_tim_count
 ****************************************************************************/

static int nrf53_tim_count(struct nrf53_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  nrf53_tim_putreg(dev, NRF53_TIM_TASKS_COUNT_OFFSET, TIM_TASKS_COUNT);

  return OK;
}

/****************************************************************************
 * Name: nrf53_tim_setcc
 ****************************************************************************/

static int nrf53_tim_setcc(struct nrf53_tim_dev_s *dev, uint8_t i,
                           uint32_t cc)
{
  struct nrf53_tim_priv_s *tim = NULL;
  int ret = OK;

  DEBUGASSERT(dev);

  tim = (struct nrf53_tim_priv_s *)dev;

  /* Is the channel supported? */

  if (i > tim->chan)
    {
      tmrerr("ERROR: unsupported TIMER channel %d\n", i);
      ret = -EINVAL;
      goto errout;
    }

  nrf53_tim_putreg(dev, NRF53_TIM_CC_OFFSET(i), cc);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf53_tim_getcc
 ****************************************************************************/

static int nrf53_tim_getcc(struct nrf53_tim_dev_s *dev, uint8_t i,
                           uint32_t *cc)
{
  struct nrf53_tim_priv_s *tim = NULL;
  int ret = OK;

  DEBUGASSERT(dev);
  DEBUGASSERT(cc);

  tim = (struct nrf53_tim_priv_s *)dev;

  /* Is the channel supported? */

  if (i > tim->chan)
    {
      tmrerr("ERROR: unsupported TIMER channel %d\n", i);
      ret = -EINVAL;
      goto errout;
    }

  *cc = nrf53_tim_getreg(dev, NRF53_TIM_CC_OFFSET(i));

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf53_tim_setpre
 ****************************************************************************/

static int nrf53_tim_setpre(struct nrf53_tim_dev_s *dev, uint8_t pre)
{
  int ret = OK;

  DEBUGASSERT(dev);

  if (pre < NRF53_TIM_PRE_16000000 || pre > NRF53_TIM_PRE_31250)
    {
      tmrerr("ERROR: unsupported TIMER prescaler %d\n", pre);
      ret = -EINVAL;
      goto errout;
    }

  nrf53_tim_putreg(dev, NRF53_TIM_PRESCALER_OFFSET, pre);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf53_tim_setisr
 ****************************************************************************/

static int nrf53_tim_setisr(struct nrf53_tim_dev_s *dev, xcpt_t handler,
                            void *arg)
{
  struct nrf53_tim_priv_s *tim = NULL;
  int ret = OK;

  DEBUGASSERT(dev);

  tim = (struct nrf53_tim_priv_s *)dev;

  /* Disable interrupt when callback is removed */

  if (!handler)
    {
      up_disable_irq(tim->irq);
      irq_detach(tim->irq);
      ret = OK;
      goto errout;
    }

  /* Otherwise set callback and enable interrupt */

  irq_attach(tim->irq, handler, arg);
  up_enable_irq(tim->irq);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf53_tim_enableint
 ****************************************************************************/

static int nrf53_tim_enableint(struct nrf53_tim_dev_s *dev, uint8_t s)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Get register value for given interrupt source */

  regval = nrf53_tim_irq2reg(dev, s);
  if (regval == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf53_tim_putreg(dev, NRF53_TIM_INTENSET_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf53_tim_disableint
 ****************************************************************************/

static int nrf53_tim_disableint(struct nrf53_tim_dev_s *dev, uint8_t s)
{
  uint32_t regval = 0;
  int      ret    = OK;

  DEBUGASSERT(dev);

  /* Get register value for given interrupt source */

  regval = nrf53_tim_irq2reg(dev, s);
  if (regval == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  nrf53_tim_putreg(dev, NRF53_TIM_INTCLR_OFFSET, regval);

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf53_tim_checkint
 ****************************************************************************/

static int nrf53_tim_checkint(struct nrf53_tim_dev_s *dev, uint8_t s)
{
  int ret = 0;

  DEBUGASSERT(dev);

  switch (s)
    {
      case NRF53_TIM_INT_COMPARE0:
        {
          ret = nrf53_tim_getreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(0));
          break;
        }

      case NRF53_TIM_INT_COMPARE1:
        {
          ret = nrf53_tim_getreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(0));
          break;
        }

      case NRF53_TIM_INT_COMPARE2:
        {
          ret = nrf53_tim_getreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(2));
          break;
        }

      case NRF53_TIM_INT_COMPARE3:
        {
          ret = nrf53_tim_getreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(3));
          break;
        }

      case NRF53_TIM_INT_COMPARE4:
        {
          ret = nrf53_tim_getreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(4));
          break;
        }

      case NRF53_TIM_INT_COMPARE5:
        {
          ret = nrf53_tim_getreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(5));
          break;
        }

      default:
        {
          tmrerr("ERROR: unsupported IRQ source %d\n", s);
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: nrf53_tim_ackint
 ****************************************************************************/

static int nrf53_tim_ackint(struct nrf53_tim_dev_s *dev, uint8_t s)
{
  int ret = 0;

  DEBUGASSERT(dev);

  switch (s)
    {
      case NRF53_TIM_INT_COMPARE0:
        {
          nrf53_tim_putreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(0), 0);
          break;
        }

      case NRF53_TIM_INT_COMPARE1:
        {
          nrf53_tim_putreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(1), 0);
          break;
        }

      case NRF53_TIM_INT_COMPARE2:
        {
          nrf53_tim_putreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(2), 0);
          break;
        }

      case NRF53_TIM_INT_COMPARE3:
        {
          nrf53_tim_putreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(3), 0);
          break;
        }

      case NRF53_TIM_INT_COMPARE4:
        {
          nrf53_tim_putreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(4), 0);
          break;
        }

      case NRF53_TIM_INT_COMPARE5:
        {
          nrf53_tim_putreg(dev, NRF53_TIM_EVENTS_COMPARE_OFFSET(5), 0);
          break;
        }

      default:
        {
          tmrerr("ERROR: unsupported IRQ source %d\n", s);
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_tim_init
 *
 * Description:
 *   Initialize TIMER device
 *
 ****************************************************************************/

struct nrf53_tim_dev_s *nrf53_tim_init(int timer)
{
  struct nrf53_tim_priv_s *tim = NULL;

  /* Get timer instance */

  switch (timer)
    {
#ifdef CONFIG_NRF53_TIMER0
      case 0:
        {
          tim = &g_nrf53_tim0_priv;
          break;
        }
#endif

#ifdef CONFIG_NRF53_TIMER1
      case 1:
        {
          tim = &g_nrf53_tim1_priv;
          break;
        }
#endif

#ifdef CONFIG_NRF53_TIMER2
      case 2:
        {
          tim = &g_nrf53_tim2_priv;
          break;
        }
#endif

#ifdef CONFIG_NRF53_TIMER3
      case 3:
        {
          tim = &g_nrf53_tim3_priv;
          break;
        }
#endif

#ifdef CONFIG_NRF53_TIMER4
      case 4:
        {
          tim = &g_nrf53_tim4_priv;
          break;
        }
#endif

      default:
        {
          tmrerr("ERROR: unsupported TIMER %d\n", timer);
          goto errout;
        }
    }

  if (tim->inuse != false)
    {
      /* Timer already in use */

      tim = NULL;
    }

errout:
  return (struct nrf53_tim_dev_s *)tim;
}

/****************************************************************************
 * Name: nrf53_tim_deinit
 *
 * Description:
 *   Deinit TIMER device
 *
 ****************************************************************************/

int nrf53_tim_deinit(struct nrf53_tim_dev_s *dev)
{
  struct nrf53_tim_priv_s *tim = NULL;

  DEBUGASSERT(dev);

  tim = (struct nrf53_tim_priv_s *)dev;

  tim->inuse = false;

  return OK;
}
