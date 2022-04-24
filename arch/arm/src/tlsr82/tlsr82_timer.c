/****************************************************************************
 * arch/arm/src/tlsr82/tlsr82_timer.c
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

#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include "tlsr82_timer.h"

#include "hardware/tlsr82_irq.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tlsr82_timer_priv_s
{
  struct tlsr82_timer_ops_s *ops;
  enum tlsr82_timer_inst_e   inst;    /* Timer instance */
  uint8_t                    periph;  /* Peripheral ID */
  int                        irq;     /* Interrupt ID */
  bool                       inuse;   /* Flag indicating if the timer is in use */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TIM operations ***********************************************************/

static void tlsr82_timer_start(struct tlsr82_timer_dev_s *dev);
static void tlsr82_timer_stop(struct tlsr82_timer_dev_s *dev);
static void tlsr82_timer_clear(struct tlsr82_timer_dev_s *dev);
static void tlsr82_timer_setmode(struct tlsr82_timer_dev_s *dev,
                                 enum tlsr82_timer_mode_e mode);
static void tlsr82_timer_getcounter(struct tlsr82_timer_dev_s *dev,
                                    uint32_t *value);
static void tlsr82_timer_setcounter(struct tlsr82_timer_dev_s *dev,
                                    uint32_t value);
static void tlsr82_timer_getcapture(struct tlsr82_timer_dev_s *dev,
                                    uint32_t *value);
static void tlsr82_timer_setcapture(struct tlsr82_timer_dev_s *dev,
                                    uint32_t value);
static void tlsr82_timer_getclock(struct tlsr82_timer_dev_s *dev,
                                  uint32_t *value);
static int  tlsr82_timer_setisr(struct tlsr82_timer_dev_s *dev,
                                xcpt_t handler, void * arg);
static void tlsr82_timer_enableint(struct tlsr82_timer_dev_s *dev);
static void tlsr82_timer_disableint(struct tlsr82_timer_dev_s *dev);
static void tlsr82_timer_ackint(struct tlsr82_timer_dev_s *dev);
static int  tlsr82_timer_checkint(struct tlsr82_timer_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* TLSR82 TIMER ops */

struct tlsr82_timer_ops_s tlsr82_timer_ops =
{
  .start         = tlsr82_timer_start,
  .stop          = tlsr82_timer_stop,
  .clear         = tlsr82_timer_clear,
  .setmode       = tlsr82_timer_setmode,
  .getcounter    = tlsr82_timer_getcounter,
  .setcounter    = tlsr82_timer_setcounter,
  .getcapture    = tlsr82_timer_getcapture,
  .setcapture    = tlsr82_timer_setcapture,
  .getclock      = tlsr82_timer_getclock,
  .setisr        = tlsr82_timer_setisr,
  .enableint     = tlsr82_timer_enableint,
  .disableint    = tlsr82_timer_disableint,
  .ackint        = tlsr82_timer_ackint,
  .checkint      = tlsr82_timer_checkint
};

struct tlsr82_timer_priv_s g_tlsr82_timers_priv[] =
{
#ifdef CONFIG_TLSR82_TIMER1
  {
    .ops    = &tlsr82_timer_ops,
    .inst   = TLSR82_INST_TIMER1,
    .irq    = NR_TIMER1_IRQ,          /* Interrupt ID */
    .inuse  = false,
  },
#endif
#ifdef CONFIG_TLSR82_TIMER2
  {
    .ops    = &tlsr82_timer_ops,
    .inst   = TLSR82_INST_TIMER2,
    .irq    = NR_TIMER2_IRQ,          /* Interrupt ID */
    .inuse  = false,
  },
#endif
};

/* TIMER0 is used for os tick */

#ifdef CONFIG_TLSR82_TIMER1
/* TIMER1 */

struct tlsr82_timer_priv_s g_tlsr82_timer1_priv =
{
  .ops    = &tlsr82_timer_ops,
  .inst   = TLSR82_INST_TIMER1,
  .irq    = NR_TIMER1_IRQ,          /* Interrupt ID */
  .inuse  = false,
};
#endif

#ifdef CONFIG_TLSR82_TIMER2
/* TIMER2 */

struct tlsr82_timer_priv_s g_tlsr82_timer2_priv =
{
  .ops    = &tlsr82_timer_ops,
  .inst   = TLSR82_INST_TIMER2,
  .irq    = NR_TIMER2_IRQ,          /* Interrupt ID */
  .inuse  = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_timer_start
 *
 * Description:
 *   Release the counter.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tlsr82_timer_start(struct tlsr82_timer_dev_s *dev)
{
  struct tlsr82_timer_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (struct tlsr82_timer_priv_s *)dev;
  if (priv->inst == TLSR82_INST_TIMER1)
    {
      BM_SET(TIMER_CTRL_REG, TIMER_CTRL_T1_ENABLE);
    }
  else if (priv->inst == TLSR82_INST_TIMER2)
    {
      BM_SET(TIMER_CTRL_REG, TIMER_CTRL_T2_ENABLE);
    }
  else
    {
      tmrerr("Timer instance %d is not existed.\n", (int)priv->inst);
    }
}

/****************************************************************************
 * Name: tlsr82_timer_stop
 *
 * Description:
 *   Halt the counter.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tlsr82_timer_stop(struct tlsr82_timer_dev_s *dev)
{
  struct tlsr82_timer_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (struct tlsr82_timer_priv_s *)dev;
  if (priv->inst == TLSR82_INST_TIMER1)
    {
      BM_CLR(TIMER_CTRL_REG, TIMER_CTRL_T1_ENABLE);
    }
  else if (priv->inst == TLSR82_INST_TIMER2)
    {
      BM_CLR(TIMER_CTRL_REG, TIMER_CTRL_T2_ENABLE);
    }
  else
    {
      tmrerr("Timer instance %d is not existed.\n", (int)priv->inst);
    }
}

/****************************************************************************
 * Name: tlsr82_timer_clear
 *
 * Description:
 *   Set the counter to zero instantly.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tlsr82_timer_clear(struct tlsr82_timer_dev_s *dev)
{
  DEBUGASSERT(dev);
  tlsr82_timer_setcounter(dev, 0);
}

/****************************************************************************
 * Name: tlsr82_timer_setmode
 *
 * Description:
 *   Set counter mode.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   mode          - Variable indicating the counting direction (up/down).
 *
 ****************************************************************************/

static void tlsr82_timer_setmode(struct tlsr82_timer_dev_s *dev,
                                 enum tlsr82_timer_mode_e mode)
{
  struct tlsr82_timer_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (struct tlsr82_timer_priv_s *)dev;

  if (mode >= TLSR82_TIMER_MODE_NUM)
    {
      tmrerr("Timer mode error, mode: %d\n", (int)mode);
      return;
    }

  if (priv->inst == TLSR82_INST_TIMER1)
    {
      BM_CLR(TIMER_CTRL_REG, TIMER_CTRL_T1_MODE);
      BM_SET(TIMER_CTRL_REG, ((uint32_t)mode << TIMER_CTRL_T1_MODE_SHIFT));
    }
  else if (priv->inst == TLSR82_INST_TIMER2)
    {
      BM_CLR(TIMER_CTRL_REG, TIMER_CTRL_T2_MODE);
      BM_SET(TIMER_CTRL_REG, ((uint32_t)mode << TIMER_CTRL_T2_MODE_SHIFT));
    }
  else
    {
      tmrerr("Timer instance %d is not existed.\n", (int)priv->inst);
    }
}

/****************************************************************************
 * Name: tlsr82_timer_getcounter
 *
 * Description:
 *   Get the current counter value.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - A pointer to a variable to store the current read
 *                   value from counter.
 *
 ****************************************************************************/

static void tlsr82_timer_getcounter(struct tlsr82_timer_dev_s *dev,
                                    uint32_t *value)
{
  struct tlsr82_timer_priv_s *priv;
  DEBUGASSERT(dev);
  DEBUGASSERT(value);
  priv = (struct tlsr82_timer_priv_s *)dev;
  if (priv->inst == TLSR82_INST_TIMER1)
    {
      *value = TIMER_TICK1_REG;
    }
  else if (priv->inst == TLSR82_INST_TIMER2)
    {
      *value = TIMER_TICK2_REG;
    }
  else
    {
      *value = 0;
      tmrerr("Timer instance %d is not existed.\n", (int)priv->inst);
    }
}

/****************************************************************************
 * Name: tlsr82_timer_setcounter
 *
 * Description:
 *   Set the value to be loaded to the counter.
 *   If you want the counter to be loaded at an alarm, enable the alarm and
 *   the auto-reload before.
 *   If you want the counter to be loaded instantly, call
 *   tlsr82_timer_reload_now() after this function.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - The value to be loaded the counter.
 *
 ****************************************************************************/

static void tlsr82_timer_setcounter(struct tlsr82_timer_dev_s *dev,
                                    uint32_t value)
{
  struct tlsr82_timer_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (struct tlsr82_timer_priv_s *)dev;

  if (priv->inst == TLSR82_INST_TIMER1)
    {
      TIMER_TICK1_REG = value;
    }
  else if (priv->inst == TLSR82_INST_TIMER2)
    {
      TIMER_TICK2_REG = value;
    }
  else
    {
      tmrerr("Timer instance %d is not existed.\n", (int)priv->inst);
    }
}

/****************************************************************************
 * Name: tlsr82_timer_getcapture
 *
 * Description:
 *   Get the alarm value.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - Pointer to retrieve the current configured alarm value.
 *
 ****************************************************************************/

static void tlsr82_timer_getcapture(struct tlsr82_timer_dev_s *dev,
                                    uint32_t *value)
{
  struct tlsr82_timer_priv_s *priv;
  DEBUGASSERT(dev);
  DEBUGASSERT(value);
  priv = (struct tlsr82_timer_priv_s *)dev;
  if (priv->inst == TLSR82_INST_TIMER1)
    {
      *value = TIMER_CAPT1_REG;
    }
  else if (priv->inst == TLSR82_INST_TIMER2)
    {
      *value = TIMER_CAPT2_REG;
    }
  else
    {
      tmrerr("Timer instance %d is not existed.\n", (int)priv->inst);
    }
}

/****************************************************************************
 * Name: tlsr82_timer_setcapture
 *
 * Description:
 *   Set the value that will trigger an alarm when the
 *   counter value matches this value.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - The alarm value.
 *
 ****************************************************************************/

static void tlsr82_timer_setcapture(struct tlsr82_timer_dev_s *dev,
                                    uint32_t value)
{
  struct tlsr82_timer_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (struct tlsr82_timer_priv_s *)dev;
  if (priv->inst == TLSR82_INST_TIMER1)
    {
      TIMER_CAPT1_REG = value;
    }
  else if (priv->inst == TLSR82_INST_TIMER2)
    {
      TIMER_CAPT2_REG = value;
    }
  else
    {
      tmrerr("Timer instance %d is not existed.\n", (int)priv->inst);
    }
}

/****************************************************************************
 * Name: tlsr82_timer_getclock
 *
 * Description:
 *   Get the timer clock.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - Pointer to clock value (MHz).
 *
 ****************************************************************************/

static void tlsr82_timer_getclock(struct tlsr82_timer_dev_s *dev,
                                  uint32_t *value)
{
  DEBUGASSERT(dev);
  DEBUGASSERT(value);

  /* Timer clock always using the system clock */

  *value = CONFIG_TLSR82_CPU_CLK_MHZ;
}

/****************************************************************************
 * Name: tlsr82_timer_setisr
 *
 * Description:
 *   Allocate a CPU Interrupt, connect the peripheral source to this
 *   Interrupt, register the callback and enable CPU the Interruption.
 *   In case a NULL handler is provided, deallocate the interrupt and
 *   unregister the previously provided handler.
 *
 * Parameters:
 *   dev           - Pointer to the driver state structure.
 *   handler       - Callback to be invoked on timer interrupt.
 *   arg           - Argument to be passed to the handler callback.
 *
 * Returned Values:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

static int tlsr82_timer_setisr(struct tlsr82_timer_dev_s *dev,
                               xcpt_t handler, void *arg)
{
  struct tlsr82_timer_priv_s *priv = NULL;
  int ret = OK;

  DEBUGASSERT(dev);

  priv = (struct tlsr82_timer_priv_s *)dev;

  /* Disable interrupt when callback is removed. */

  if (handler == NULL)
    {
      if (priv->irq < NR_IRQS)
        {
          /* Disable cpu interrupt */

          up_disable_irq(priv->irq);

          /* Dissociate the IRQ from the ISR */

          irq_detach(priv->irq);
        }
    }

  /* Otherwise set callback and enable interrupt */

  else
    {
      if (priv->irq < NR_IRQS)
        {
          /* Disable the provided CPU interrupt to configure it. */

          up_disable_irq(priv->irq);

          /* Associate an IRQ Number (from the timer) to an ISR */

          ret = irq_attach(priv->irq, handler, arg);
          if (ret != OK)
            {
              tmrerr("ERROR: Failed to associate an IRQ Number to and ISR");
              goto errout;
            }

          /* Enable the CPU Interrupt that is linked to the timer */

          up_enable_irq(priv->irq);
        }
      else
        {
          tmrerr("Timer isr can't attach, irq number=%d is invalid\n",
                 priv->irq);
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: tlsr82_timer_enableint
 *
 * Description:
 *   Enable a level Interrupt at the alarm if it is set.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tlsr82_timer_enableint(struct tlsr82_timer_dev_s *dev)
{
  struct tlsr82_timer_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (struct tlsr82_timer_priv_s *)dev;
  if (priv->inst == TLSR82_INST_TIMER1)
    {
      up_enable_irq(NR_TIMER1_IRQ);
    }
  else if (priv->inst == TLSR82_INST_TIMER2)
    {
      up_enable_irq(NR_TIMER2_IRQ);
    }
  else
    {
      tmrerr("Timer instance %d is not existed.\n", (int)priv->inst);
    }
}

/****************************************************************************
 * Name: tlsr82_timer_disableint
 *
 * Description:
 *   Disable a level Interrupt at the alarm if it is set.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tlsr82_timer_disableint(struct tlsr82_timer_dev_s *dev)
{
  struct tlsr82_timer_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (struct tlsr82_timer_priv_s *)dev;
  if (priv->inst == TLSR82_INST_TIMER1)
    {
      up_disable_irq(NR_TIMER1_IRQ);
    }
  else if (priv->inst == TLSR82_INST_TIMER2)
    {
      up_disable_irq(NR_TIMER2_IRQ);
    }
  else
    {
      tmrerr("Timer instance %d is not existed.\n", (int)priv->inst);
    }
}

/****************************************************************************
 * Name: tlsr82_timer_ackint
 *
 * Description:
 *   Acknowledge an interrupt, that means, clear the interrupt.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tlsr82_timer_ackint(struct tlsr82_timer_dev_s *dev)
{
  struct tlsr82_timer_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (struct tlsr82_timer_priv_s *)dev;
  if (priv->inst == TLSR82_INST_TIMER1)
    {
      BM_SET(TIMER_STATUS_REG, TIMER_STATUS_T1_CLR);
    }
  else if (priv->inst == TLSR82_INST_TIMER2)
    {
      BM_SET(TIMER_STATUS_REG, TIMER_STATUS_T2_CLR);
    }
  else
    {
      tmrerr("Timer instance %d is not existed.\n", (int)priv->inst);
    }
}

/****************************************************************************
 * Name: tlsr82_timer_checkint
 *
 * Description:
 *   Check the interrupt status bit.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 * Returned Values:
 *  Return 1 in case of an interrupt is triggered, otherwise 0.
 *
 ****************************************************************************/

static int tlsr82_timer_checkint(struct tlsr82_timer_dev_s *dev)
{
  struct tlsr82_timer_priv_s *priv = (struct tlsr82_timer_priv_s *)dev;
  int ret = 0;

  DEBUGASSERT(dev != NULL);

  if (priv->inst == TLSR82_INST_TIMER1)
    {
      if (BM_IS_SET(IRQ_SRC_REG, 1 << NR_TIMER1_IRQ))
        {
          ret = 1;
        }
    }
  else if (priv->inst == TLSR82_INST_TIMER2)
    {
      if (BM_IS_SET(IRQ_SRC_REG, 1 << NR_TIMER2_IRQ))
        {
          ret = 1;
        }
    }
  else
    {
      tmrerr("Timer instance %d is not existed.\n", (int)priv->inst);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlsr82_timer_init
 *
 * Description:
 *   Initialize TIMER device.
 *
 * Parameters:
 *   timer           - Timer instance to be initialized.
 *                     Valid values: 1 or 2.
 *
 * Returned Values:
 *   If the initialization is successful, return a pointer to the timer
 *   driver struct associated to that timer instance.
 *   In case it fails, return NULL.
 *
 ****************************************************************************/

struct tlsr82_timer_dev_s *tlsr82_timer_init(int timer)
{
  struct tlsr82_timer_priv_s *tim = NULL;

  /* First, take the data structure associated with the timer instance */

  switch (timer)
    {
#ifdef CONFIG_TLSR82_TIMER1
      case 1:
        {
          tim = &g_tlsr82_timer1_priv;
          break;
        }
#endif

#ifdef CONFIG_TLSR82_TIMER2
      case 2:
        {
          tim = &g_tlsr82_timer2_priv;
          break;
        }
#endif

      default:
        {
          tmrerr("ERROR: unsupported TIMER %d\n", timer);
          goto errout;
        }
    }

  /* Verify if it is in use */

  if (tim->inuse == false)
    {
      tim->inuse = true;  /* If it was not, now it is */
    }
  else
    {
      tmrerr("ERROR: TIMER %d is already in use\n", timer);
      tim = NULL;
    }

errout:
  return (struct tlsr82_timer_dev_s *)tim;
}

/****************************************************************************
 * Name: tlsr82_timer_deinit
 *
 * Description:
 *   Deinit TIMER device.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

void tlsr82_timer_deinit(struct tlsr82_timer_dev_s *dev)
{
  struct tlsr82_timer_priv_s *tim = NULL;

  DEBUGASSERT(dev);

  tim = (struct tlsr82_timer_priv_s *)dev;
  tim->inuse = false;
}
