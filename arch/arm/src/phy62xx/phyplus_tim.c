/****************************************************************************
 * arch/arm/src/phy62xx/phyplus_tim.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "phyplus_tim.h"

#include "mcu_phy_bumbee.h"
#include "arm_arch.h"

struct phyplus_tim_priv_s
{
  const struct phyplus_tim_ops_s *ops;
  uint32_t  base;  /* Timer register base address */
  bool      inuse; /* Flag indicating if the timer is in use */

  /* AP_TIM_TypeDef *timer; */
};

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

/* static int  phyplus_tim_checkint(FAR struct phyplus_tim_dev_s *dev,
 *     int source);
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct phyplus_tim_ops_s phyplus_tim_ops =
{
  .start      = phyplus_tim_start,
  .stop       = phyplus_tim_stop,
  .clear      = phyplus_tim_clear,
  .setmode    = phyplus_tim_setmode,

  .getcounter = phyplus_tim_getcounter,
  .setcounter = phyplus_tim_setcounter,

  .setisr     = phyplus_tim_setisr,
  .enableint  = phyplus_tim_enableint,
  .disableint = phyplus_tim_disableint,
  .ackint     = phyplus_tim_ackint,

  /* .checkint   = phyplus_tim_checkint */
};

/* #ifdef CONFIG_PHYPLUS_TIM1 */

struct phyplus_tim_priv_s phyplus_tim1_priv =
{
  .ops                 = &phyplus_tim_ops,
  .base                =  (long unsigned int)AP_TIM1,
  .inuse               = false,
};

/* #endif */

/* #ifdef CONFIG_PHYPLUS_TIM2 */

struct phyplus_tim_priv_s phyplus_tim2_priv =
{
  .ops                 = &phyplus_tim_ops,
  .base                =  (long unsigned int)AP_TIM2,
  .inuse               = false,
};

/* #endif */

/* #ifdef CONFIG_PHYPLUS_TIM3 */

struct phyplus_tim_priv_s phyplus_tim3_priv =
{
  .ops                 = &phyplus_tim_ops,
  .base                =  (long unsigned int)AP_TIM3,
  .inuse               = false,
};

/* #endif */

/* #ifdef CONFIG_PHYPLUS_TIM4 */

struct phyplus_tim_priv_s phyplus_tim4_priv =
{
  .ops                 = &phyplus_tim_ops,
  .base                =  (long unsigned int)AP_TIM4,
  .inuse               = false,
};

/* #endif */

/* #ifdef CONFIG_PHYPLUS_TIM5 */

struct phyplus_tim_priv_s phyplus_tim5_priv =
{
  .ops                 = &phyplus_tim_ops,
  .base                =  (long unsigned int)AP_TIM5,
  .inuse               = false,
};

/* #endif */

/* #ifdef CONFIG_PHYPLUS_TIM6 */

struct phyplus_tim_priv_s phyplus_tim6_priv =
{
  .ops                 = &phyplus_tim_ops,
  .base                =  (long unsigned int)AP_TIM6,
  .inuse               = false,
};

/* #endif */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: phyplus_tim_getreg32
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

uint32_t phyplus_tim_getreg32(FAR struct phyplus_tim_dev_s *dev,
                                 uint32_t offset)
{
  DEBUGASSERT(dev);

  return getreg32(((struct phyplus_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: phyplus_tim_putreg32
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

void phyplus_tim_putreg32(FAR struct phyplus_tim_dev_s *dev,
                             uint32_t offset,
                             uint32_t value)
{
  DEBUGASSERT(dev);

  putreg32(value, ((struct phyplus_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: phyplus_tim_modifyreg32
 *
 * Description:
 *   Modify a reg of 32 bits
 *
 ****************************************************************************/

void phyplus_tim_modifyreg32(FAR struct phyplus_tim_dev_s *dev,
                                  uint32_t offset,
                                  uint32_t clearbits,
                                  uint32_t setbits)
{
  DEBUGASSERT(dev);

  modifyreg32(((struct phyplus_tim_priv_s *)dev)->base + offset,
                clearbits, setbits);
}

/****************************************************************************
 * Name: phyplus_tim_start
 *
 * Description:
 *   Releases the counter
 *
 ****************************************************************************/

void phyplus_tim_start(FAR struct phyplus_tim_dev_s *dev)
{
  syslog(LOG_ERR, "timer_enable\n");
  DEBUGASSERT(dev);
  phyplus_tim_modifyreg32(dev, TIM_CONTROLREG_OFFSET, 0, TIM_ENABLE);
}

/****************************************************************************
 * Name: phyplus_tim_stop
 *
 * Description:
 *   Halts the counter
 *
 ****************************************************************************/

void phyplus_tim_stop(FAR struct phyplus_tim_dev_s *dev)
{
  syslog(LOG_ERR, "timer_disable\n");
  DEBUGASSERT(dev);
  phyplus_tim_modifyreg32(dev, TIM_CONTROLREG_OFFSET, TIM_ENABLE, 0);
}

/****************************************************************************
 * Name: phyplus_tim_clear
 *
 * Description:
 *   Set the counter to zero instantly
 *
 ****************************************************************************/

void phyplus_tim_clear(FAR struct phyplus_tim_dev_s *dev)
{
  uint32_t clear_value = 0;
  DEBUGASSERT(dev);
  phyplus_tim_setcounter(dev, clear_value);
  phyplus_tim_getreg32(dev, TIM_EOI_OFFSET);  /* clear the timer. */
}

/****************************************************************************
 * Name: phyplus_tim_setmode
 *
 * Description:
 *   Set counter mode (up/down)
 *
 ****************************************************************************/

void phyplus_tim_setmode(FAR struct phyplus_tim_dev_s *dev,
    phyplus_tim_mode_t mode)
{
  DEBUGASSERT(dev);

  if (mode == PHYPLUS_TIM_FREERUN)
    {
      phyplus_tim_modifyreg32(dev, TIM_CONTROLREG_OFFSET, TIM_MODE, 0);
    }
  else if (mode == PHYPLUS_TIM_COUNT)
    {
      phyplus_tim_modifyreg32(dev, TIM_CONTROLREG_OFFSET, 0, TIM_MODE);
    }
}

/****************************************************************************
 * Name: phyplus_tim_getcounter
 *
 * Description:
 *   Get the current counter value
 *
 ****************************************************************************/

void phyplus_tim_getcounter(FAR struct phyplus_tim_dev_s *dev,
    uint32_t *value)
{
  DEBUGASSERT(dev);

  *value = phyplus_tim_getreg32(dev, TIM_COUNT_OFFSET);
}

/****************************************************************************
 * Name: phyplus_tim_setcounter
 *
 * Description:
 *   Set the value to be loaded to the counter
 *   If you want the counter to be loaded at an alarm, enable the alarm and
 *   the auto-reload before.
 *
 ****************************************************************************/

void phyplus_tim_setcounter(FAR struct phyplus_tim_dev_s *dev,
    uint32_t value)
{
  DEBUGASSERT(dev);

  phyplus_tim_putreg32(dev, TIM_COUNT_OFFSET, (uint32_t)value);
}

void phyplus_tim_getcurrent(FAR struct phyplus_tim_dev_s *dev,
    uint32_t *value)
{
  DEBUGASSERT(dev);

  *value = phyplus_tim_getreg32(dev, TIM_CURRENT_OFFSET);
}

void phyplus_tim_getcontrolreg(FAR struct phyplus_tim_dev_s *dev,
    uint32_t *value)
{
  DEBUGASSERT(dev);

  *value = phyplus_tim_getreg32(dev, TIM_CONTROLREG_OFFSET);
}

/****************************************************************************
 * Name: phyplus_tim_setisr
 *
 * Description:
 *   Allocates a level CPU Interrupt, connects the peripheral source to this
 *   Interrupt, register the callback and enables the Interruption. It does
 *   opposite if the handler and arg are NULL.
 *
 ****************************************************************************/

int phyplus_tim_setisr(FAR struct phyplus_tim_dev_s *dev, xcpt_t handler,
    void *arg)
{
  FAR struct phyplus_tim_priv_s *tim = NULL;
  int vectorno;

  DEBUGASSERT(dev);

  tim = (FAR struct phyplus_tim_priv_s *)dev;

  if ((long unsigned int)AP_TIM1 == ((struct phyplus_tim_priv_s *)dev)->base)
    {
      vectorno = PHY62XX_IRQ_TIM1_IRQn;
    }
  else if ((long unsigned int)AP_TIM2 ==
    ((struct phyplus_tim_priv_s *)dev)->base)
    {
      vectorno = PHY62XX_IRQ_TIM2_IRQn;
    }
  else if ((long unsigned int)AP_TIM3 ==
    ((struct phyplus_tim_priv_s *)dev)->base)
    {
      vectorno = PHY62XX_IRQ_TIM3_IRQn;
    }
  else if ((long unsigned int)AP_TIM4 ==
    ((struct phyplus_tim_priv_s *)dev)->base)
    {
      vectorno = PHY62XX_IRQ_TIM4_IRQn;
    }
  else if ((long unsigned int)AP_TIM5 ==
    ((struct phyplus_tim_priv_s *)dev)->base)
    {
      vectorno = PHY62XX_IRQ_TIM5_IRQn;
    }
  else if ((long unsigned int)AP_TIM6 ==
    ((struct phyplus_tim_priv_s *)dev)->base)
    {
      vectorno = PHY62XX_IRQ_TIM6_IRQn;
    }
  else
    {
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
 * Name: phyplus_tim_enableint
 *
 * Description:
 *   Enables a level Interrupt at the alarm if it is set.
 *
 ****************************************************************************/

void phyplus_tim_enableint(FAR struct phyplus_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Set the level interrupt bit */

  phyplus_tim_modifyreg32(dev, TIM_CURRENT_OFFSET, 0, TIM_MASK);
}

/****************************************************************************
 * Name: phyplus_tim_disableint
 *
 * Description:
 *   Disables a level Interrupt at the alarm if it is set.
 *
 ****************************************************************************/

void phyplus_tim_disableint(FAR struct phyplus_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Clear the level interrupt bit */

  phyplus_tim_modifyreg32(dev, TIM_CURRENT_OFFSET, TIM_MASK, 0);
}

/****************************************************************************
 * Name: phyplus_tim_ackint
 *
 * Description:
 *
 ****************************************************************************/

void phyplus_tim_ackint(FAR struct phyplus_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Clear the level interrupt bit */

  phyplus_tim_getreg32(dev, TIM_EOI_OFFSET);
}

/****************************************************************************
 * Name: phyplus_tim_init
 *
 * Description:
 *   Initialize TIMER device.
 *
 ****************************************************************************/

FAR struct phyplus_tim_dev_s *phyplus_tim_init(int timer)
{
  FAR struct phyplus_tim_priv_s *tim = NULL;

  /* First, take the data structure associated with the timer instance */

  syslog(LOG_ERR, "phyplus_tim_init 1\n");
  switch (timer)
    {
      /* #ifdef CONFIG_PHYPLUS_TIM1 */

      case 1:
        {
          tim = &phyplus_tim1_priv;
          break;
        }

      /* #endif */

      /* #ifdef CONFIG_PHYPLUS_TIM2 */

      case 2:
        {
          tim = &phyplus_tim2_priv;
          break;
        }

      /* #endif */

      /* #ifdef CONFIG_PHYPLUS_TIM3 */

      case 3:
        {
          tim = &phyplus_tim3_priv;
          break;
        }

      /* #endif */

      /* #ifdef CONFIG_PHYPLUS_TIM4 */

      case 4:
        {
          tim = &phyplus_tim4_priv;
          break;
        }

      /* #endif */

      /* #ifdef CONFIG_PHYPLUS_TIM5 */

      case 5:
        {
          tim = &phyplus_tim5_priv;
          break;
        }

      /* #endif */

      /* #ifdef CONFIG_PHYPLUS_TIM6 */

      case 6:
        {
          tim = &phyplus_tim6_priv;
          break;
        }

      /* #endif */

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

  syslog(LOG_ERR, "phyplus_tim_init 2\n");
  errout:
  return (FAR struct phyplus_tim_dev_s *)tim;
}

/****************************************************************************
 * Name: esp32_tim_deinit
 *
 * Description:
 *   Deinit TIMER device
 *
 ****************************************************************************/

void phyplus_tim_deinit(FAR struct phyplus_tim_dev_s *dev)
{
  FAR struct phyplus_tim_priv_s *tim = NULL;

  DEBUGASSERT(dev);

  tim = (FAR struct phyplus_tim_priv_s *)dev;

  tim->inuse = false;
}
