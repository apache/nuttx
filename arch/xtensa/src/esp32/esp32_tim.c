/****************************************************************************
 * arch/xtensa/src/esp32/esp32_tim.c
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

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>

#include "xtensa.h"

#include "hardware/esp32_tim.h"

#include "esp32_tim.h"
#include "esp32_cpuint.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32_tim_priv_s
{
  FAR struct esp32_tim_ops_s *ops;
  uint32_t                    base;    /* Timer register base address */
  uint8_t                     periph;  /* Peripheral ID */
  uint8_t                     irq;     /* Interrupt ID */
  int                         cpuint;  /* CPU interrupt assigned to this timer */
  bool                        inuse;   /* Flag indicating if the timer is in use */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TIM registers access *****************************************************/

static uint32_t esp32_tim_getreg(FAR struct esp32_tim_dev_s *dev,
                                 uint32_t offset);
static void esp32_tim_putreg(FAR struct esp32_tim_dev_s *dev,
                             uint32_t offset,
                             uint32_t value);
static void esp32_tim_modifyreg32(FAR struct esp32_tim_dev_s *dev,
                                  uint32_t offset,
                                  uint32_t clearbits,
                                  uint32_t setbits);

/* TIM helpers **************************************************************/

/* TIM operations ***********************************************************/

static int esp32_tim_start(FAR struct esp32_tim_dev_s *dev);
static int esp32_tim_stop(FAR struct esp32_tim_dev_s *dev);
static int esp32_tim_clear(FAR struct esp32_tim_dev_s *dev);
static int esp32_tim_configure(FAR struct esp32_tim_dev_s *dev, uint16_t pre,
                               uint8_t mode, uint64_t counter_value,
                               uint64_t alarm_value, bool alarm,
                               bool autoreload);
static int esp32_tim_setmode(FAR struct esp32_tim_dev_s *dev, uint8_t mode);
static int esp32_tim_setpre(FAR struct esp32_tim_dev_s *dev, uint16_t pre);
static int esp32_tim_getconfig(FAR struct esp32_tim_dev_s *dev,
                               uint32_t *value);
static int esp32_tim_getcounter(FAR struct esp32_tim_dev_s *dev,
                                uint64_t *value);
static int esp32_tim_setcounter(FAR struct esp32_tim_dev_s *dev,
                                uint64_t value);
static int esp32_tim_getalarmvalue(FAR struct esp32_tim_dev_s *dev,
                                   uint64_t *value);
static int esp32_tim_setalarmvalue(FAR struct esp32_tim_dev_s *dev,
                                   uint64_t value);
static int esp32_tim_setalarm(FAR struct esp32_tim_dev_s *dev, bool enable);
static int esp32_tim_setautoreload(FAR struct esp32_tim_dev_s *dev,
                                   bool enable);
static int esp32_tim_setisr(FAR struct esp32_tim_dev_s *dev, xcpt_t handler,
                            FAR void * arg);
static int esp32_tim_enableint(FAR struct esp32_tim_dev_s *dev);
static int esp32_tim_disableint(FAR struct esp32_tim_dev_s *dev);
static int esp32_tim_checkint(FAR struct esp32_tim_dev_s *dev);
static int esp32_tim_ackint(FAR struct esp32_tim_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ESP32 TIM ops */

struct esp32_tim_ops_s esp32_tim_ops =
{
  .start         = esp32_tim_start,
  .stop          = esp32_tim_stop,
  .clear         = esp32_tim_clear,
  .configure     = esp32_tim_configure,
  .setmode       = esp32_tim_setmode,
  .getcounter    = esp32_tim_getcounter,
  .setpre        = esp32_tim_setpre,
  .getconfig     = esp32_tim_getconfig,
  .setcounter    = esp32_tim_setcounter,
  .getalarmvalue = esp32_tim_getalarmvalue,
  .setalarmvalue = esp32_tim_setalarmvalue,
  .setalarm      = esp32_tim_setalarm,
  .setautoreload = esp32_tim_setautoreload,
  .setisr        = esp32_tim_setisr,
  .enableint     = esp32_tim_enableint,
  .disableint    = esp32_tim_disableint,
  .checkint      = esp32_tim_checkint,
  .ackint        = esp32_tim_ackint
};

#ifdef CONFIG_ESP32_TIMER0
/* TIMER0 */

struct esp32_tim_priv_s g_esp32_tim0_priv =
{
  .ops    = &esp32_tim_ops,
  .base   = TIMG_T0CONFIG_REG(0),
  .periph = ESP32_PERIPH_TG_T0_LEVEL,   /* Peripheral ID */
  .irq    = ESP32_IRQ_TG_T0_LEVEL,      /* Interrupt ID */
  .cpuint = -ENOMEM,                    /* CPU interrupt assigned to this timer */
  .inuse = false,
};
#endif

#ifdef CONFIG_ESP32_TIMER1
/* TIMER1 */

struct esp32_tim_priv_s g_esp32_tim1_priv =
{
  .ops   = &esp32_tim_ops,
  .base  = TIMG_T1CONFIG_REG(0),
  .periph = ESP32_PERIPH_TG_T1_LEVEL,   /* Peripheral ID */
  .irq    = ESP32_IRQ_TG_T1_LEVEL,      /* Interrupt ID */
  .cpuint = -ENOMEM,                    /* CPU interrupt assigned to this timer */
  .inuse = false,
};
#endif

#ifdef CONFIG_ESP32_TIMER2
/* TIMER2 */

struct esp32_tim_priv_s g_esp32_tim2_priv =
{
  .ops   = &esp32_tim_ops,
  .base  = TIMG_T0CONFIG_REG(1),
  .periph = ESP32_PERIPH_TG1_T0_LEVEL,   /* Peripheral ID */
  .irq    = ESP32_IRQ_TG1_T0_LEVEL,      /* Interrupt ID */
  .cpuint = -ENOMEM,                     /* CPU interrupt assigned to this timer */
  .inuse = false,
};
#endif

#ifdef CONFIG_ESP32_TIMER3
/* TIMER3 */

struct esp32_tim_priv_s g_esp32_tim3_priv =
{
  .ops   = &esp32_tim_ops,
  .base  = TIMG_T1CONFIG_REG(1),
  .periph = ESP32_PERIPH_TG1_T1_LEVEL,   /* Peripheral ID */
  .irq    = ESP32_IRQ_TG1_T1_LEVEL,      /* Interrupt ID */
  .cpuint = -ENOMEM,                     /* CPU interrupt assigned to this timer */
  .inuse = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_tim_getreg
 *
 * Description:
 *   Get a 32-bit register value by offset
 *
 ****************************************************************************/

static uint32_t esp32_tim_getreg(FAR struct esp32_tim_dev_s *dev,
                                 uint32_t offset)
{
  DEBUGASSERT(dev);

  return getreg32(((struct esp32_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: esp32_tim_putreg
 *
 * Description:
 *   Put a 32-bit register value by offset
 *
 ****************************************************************************/

static void esp32_tim_putreg(FAR struct esp32_tim_dev_s *dev,
                             uint32_t offset,
                             uint32_t value)
{
  DEBUGASSERT(dev);

  putreg32(value, ((struct esp32_tim_priv_s *)dev)->base + offset);
}

/****************************************************************************
 * Name: esp32_tim_modifyreg32
 *
 * Description:
 *   Modify a reg of 32 bits
 *
 ****************************************************************************/

static void esp32_tim_modifyreg32(FAR struct esp32_tim_dev_s *dev,
                                  uint32_t offset,
                                  uint32_t clearbits,
                                  uint32_t setbits)
{
  DEBUGASSERT(dev);

  modifyreg32(((struct esp32_tim_priv_s *)dev)->base + offset,
                clearbits, setbits);
}

/****************************************************************************
 * Name: esp32_tim_start
 *
 * Description:
 *   Releases the counter
 *
 ****************************************************************************/

static int esp32_tim_start(FAR struct esp32_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  esp32_tim_modifyreg32(dev, TIM_CONFIG_OFFSET, 0, TIMG_T0_EN);

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_stop
 *
 * Description:
 *   Halts the counter
 *
 ****************************************************************************/

static int esp32_tim_stop(FAR struct esp32_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  esp32_tim_modifyreg32(dev, TIM_CONFIG_OFFSET, TIMG_T0_EN, 0);

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_clear
 *
 * Description:
 *   Set the counter to zero instantly
 *
 ****************************************************************************/

static int esp32_tim_clear(FAR struct esp32_tim_dev_s *dev)
{
  uint32_t clear_value = 0;

  DEBUGASSERT(dev);

  esp32_tim_putreg(dev, TIM_LOAD_LO_OFFSET, clear_value);
  esp32_tim_putreg(dev, TIM_LOAD_HI_OFFSET, clear_value);

  /* Dummy value to trigger reload the counter with the previous value */

  esp32_tim_putreg(dev, TIM_LOAD_OFFSET, BIT(0));

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_configure
 *
 * Description:
 *   Configure prescaler divider, counter mode (up/down), counter value,
 * Reload Event
 *   and alarm trigger value.
 *
 ****************************************************************************/

static int esp32_tim_configure(FAR struct esp32_tim_dev_s *dev, uint16_t pre,
                               uint8_t mode, uint64_t counter_value,
                               uint64_t alarm_value, bool alarm,
                               bool autoreload)
{
  int ret = OK;

  DEBUGASSERT(dev);

  /* Change the prescaler divider with the timer enabled can lead to
   * unpredictable results, so it is disabled before configuring
   */

  ret = esp32_tim_stop(dev);
  if (ret != OK)
    {
      goto errout;
    }

  /* Configure TIMER prescaler */

  ret = esp32_tim_setpre(dev, pre);
  if (ret != OK)
    {
      goto errout;
    }

  /* Configure TIMER mode */

  ret = esp32_tim_setmode(dev, mode);
  if (ret != OK)
    {
      goto errout;
    }

  /* Configure TIMER counter value */

  ret = esp32_tim_setcounter(dev, counter_value);
  if (ret != OK)
    {
      goto errout;
    }

  /* Configure TIMER alarm value */

  ret = esp32_tim_setalarmvalue(dev, counter_value);
  if (ret != OK)
    {
      goto errout;
    }

  /* Enable TIMER alarm */

  ret = esp32_tim_setalarm(dev, alarm);
  if (ret != OK)
    {
      goto errout;
    }

  /* Configure TIMER auto-reload */

  ret = esp32_tim_setautoreload(dev, autoreload);
  if (ret != OK)
    {
      goto errout;
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: esp32_tim_setmode
 *
 * Description:
 *   Set counter mode (up/down)
 *
 ****************************************************************************/

static int esp32_tim_setmode(FAR struct esp32_tim_dev_s *dev, uint8_t mode)
{
  int ret = OK;

  DEBUGASSERT(dev);

  switch (mode)
    {
      case ESP32_TIM_MODE_DOWN:
        {
          esp32_tim_modifyreg32(dev, TIM_CONFIG_OFFSET, TIMG_T0_INCREASE, 0);
          break;
        }

      case ESP32_TIM_MODE_UP:
        {
          esp32_tim_modifyreg32(dev, TIM_CONFIG_OFFSET, 0, TIMG_T0_INCREASE);
          break;
        }

      default:
        {
          tmrerr("ERROR: unsupported TIMER mode %d\n", mode);
          ret = -EINVAL;
          goto errout;
        }
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: esp32_tim_setpre
 *
 * Description:
 *   Set prescaler divider (2 - 65356)
 *   0         = 65536
 *   1,2       = 2
 *
 ****************************************************************************/

static int esp32_tim_setpre(FAR struct esp32_tim_dev_s *dev, uint16_t pre)
{
  uint32_t mask = (uint32_t)pre << TIMG_T0_DIVIDER_S;

  DEBUGASSERT(dev);

  esp32_tim_modifyreg32(dev, TIM_CONFIG_OFFSET, TIMG_T0_DIVIDER_M, mask);

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_getconfig
 *
 * Description:
 *   Get the current configuration from config reg
 *
 ****************************************************************************/

static int esp32_tim_getconfig(FAR struct esp32_tim_dev_s *dev,
                               uint32_t *value)
{
  DEBUGASSERT(dev);

  /* Read value */

  *value = esp32_tim_getreg(dev, TIM_CONFIG_OFFSET); /* High 32 bits */

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_getcounter
 *
 * Description:
 *   Get the current counter value
 *
 ****************************************************************************/

static int esp32_tim_getcounter(FAR struct esp32_tim_dev_s *dev,
                                uint64_t *value)
{
  uint32_t value_32;

  DEBUGASSERT(dev);

  *value = 0;

  /* Dummy value to latch the counter value to read it */

  esp32_tim_putreg(dev, TIM_UPDATE_OFFSET, BIT(0));

  /* Read value */

  value_32 = esp32_tim_getreg(dev, TIM_HI_OFFSET); /* High 32 bits */

  *value |= (uint64_t)value_32;

  *value <<= SHIFT_32;

  value_32 = esp32_tim_getreg(dev, TIM_LO_OFFSET); /* Low 32 bits */

  *value |= (uint64_t)value_32;

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_setcounter
 *
 * Description:
 *   Set the value to be loaded to the counter
 *   It may be loaded at an alarm or instantly
 *
 ****************************************************************************/

static int esp32_tim_setcounter(FAR struct esp32_tim_dev_s *dev,
                                uint64_t value)
{
  uint64_t low_64 = value & LOW_32_MASK;
  uint64_t high_64 = (value >> SHIFT_32) & LOW_32_MASK;

  DEBUGASSERT(dev);

  /* Set the counter value */

  esp32_tim_putreg(dev, TIM_LOAD_LO_OFFSET, (uint32_t)low_64);
  esp32_tim_putreg(dev, TIM_LOAD_HI_OFFSET, (uint32_t)high_64);

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_getalarmvalue
 *
 * Description:
 *   Get the alarm value.
 *
 ****************************************************************************/

static int esp32_tim_getalarmvalue(FAR struct esp32_tim_dev_s *dev,
                                   uint64_t *value)
{
  uint32_t value_32;

  DEBUGASSERT(dev);

  *value = 0;

  /* Read value */

  value_32 = esp32_tim_getreg(dev, TIMG_ALARM_HI_OFFSET); /* High 32 bits */

  *value |= (uint64_t)value_32;

  *value <<= SHIFT_32;

  value_32 = esp32_tim_getreg(dev, TIMG_ALARM_LO_OFFSET); /* Low 32 bits */

  *value |= (uint64_t)value_32;

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_setalarmvalue
 *
 * Description:
 *   Set the value that will trigger an alarm when the
 *   counter value matches this value.
 *
 ****************************************************************************/

static int esp32_tim_setalarmvalue(FAR struct esp32_tim_dev_s *dev,
                                   uint64_t value)
{
  uint64_t low_64 = value & LOW_32_MASK;
  uint64_t high_64 = (value >> SHIFT_32) & LOW_32_MASK;

  DEBUGASSERT(dev);

  /* Set an alarm value */

  esp32_tim_putreg(dev, TIMG_ALARM_LO_OFFSET, (uint32_t)low_64);
  esp32_tim_putreg(dev, TIMG_ALARM_HI_OFFSET, (uint32_t)high_64);

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_setalarm
 *
 * Description:
 *   Enables/Disables the alarm.
 *
 ****************************************************************************/

static int esp32_tim_setalarm(FAR struct esp32_tim_dev_s *dev, bool enable)
{
  DEBUGASSERT(dev);

  if (enable)
    {
      esp32_tim_modifyreg32(dev, TIM_CONFIG_OFFSET, 0, TIMG_T0_ALARM_EN);
    }
  else
    {
      esp32_tim_modifyreg32(dev, TIM_CONFIG_OFFSET, TIMG_T0_ALARM_EN, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_setautoreload
 *
 * Description:
 *   Enable or disabales the auto reload. If is set the counter auto
 * reloads when it matches the alarm value, otherwise, the counter
 * continues to increment or decrement after the alarm. The alarm also
 * needs to be enabled to trigger a reload event.
 *
 ****************************************************************************/

static int esp32_tim_setautoreload(FAR struct esp32_tim_dev_s *dev,
                                   bool enable)
{
  DEBUGASSERT(dev);

  if (enable)
    {
      esp32_tim_modifyreg32(dev, TIM_CONFIG_OFFSET, 0, TIMG_T0_AUTORELOAD);
    }
  else
    {
      esp32_tim_modifyreg32(dev, TIM_CONFIG_OFFSET, TIMG_T0_AUTORELOAD, 0);
    }

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_setisr
 *
 * Description:
 *   Allocates an Edge CPU Interrupt, connects the peripheral source to this
 *   Interrupt, register the callback and enables the Interruption. It does
 *   opposite if the handler and arg are NULL.
 *
 ****************************************************************************/

static int esp32_tim_setisr(FAR struct esp32_tim_dev_s *dev, xcpt_t handler,
                            FAR void *arg)
{
  FAR struct esp32_tim_priv_s *tim = NULL;
  int ret = OK;
  uint8_t cpu;

  DEBUGASSERT(dev);

  tim = (FAR struct esp32_tim_priv_s *)dev;

  /* Disable interrupt when callback is removed */

  if (handler == NULL)
    {
      /* If a CPU Interrupt was previously allocated, then deallocate it */

      if (tim->cpuint >= 0)
        {
          /* Disable CPU Interrupt, free a previously allocated
           * CPU Interrupt
           */

          up_disable_irq(tim->cpuint);
          cpu = up_cpu_index();
          esp32_detach_peripheral(cpu, tim->periph, tim->cpuint);
          esp32_free_cpuint(tim->cpuint);
          irq_detach(tim->irq);
        }

      ret = OK;
      goto errout;
    }

  /* Otherwise set callback and enable interrupt */

  /* Verify the available CPU Interrupt */

  tim->cpuint = esp32_alloc_levelint(1);
  if (tim->cpuint < 0)
    {
      tmrerr("ERROR: No CPU Interrupt available");
      ret = tim->cpuint;
      goto errout;
    }

  /* Disable the provided CPU Interrupt to configure it */

  up_disable_irq(tim->cpuint);

  /* Attach a peripheral interrupt to the available CPU interrupt in
   * the current core
   */

  cpu = up_cpu_index();
  esp32_attach_peripheral(cpu, tim->periph, tim->cpuint);

  /* Associate an IRQ Number (from the timer) to an ISR */

  ret = irq_attach(tim->irq, handler, arg);

  if (ret != OK)
    {
      esp32_detach_peripheral(cpu, tim->periph, tim->cpuint);
      esp32_free_cpuint(tim->cpuint);
      tmrerr("ERROR: Failed to associate an IRQ Number");
      goto errout;
    }

  /* Enable the CPU Interrupt that is linked to the timer */

  up_enable_irq(tim->cpuint);

errout:
  return ret;
}

/****************************************************************************
 * Name: esp32_tim_enableint
 *
 * Description:
 *   Enables an Edge Interrupt at the alarm if it is set.
 *
 ****************************************************************************/

static int esp32_tim_enableint(FAR struct esp32_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Set the edge interrupt bit */

  esp32_tim_modifyreg32(dev, TIM_CONFIG_OFFSET, 0, TIMG_T0_LEVEL_INT_EN);

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_disableint
 *
 * Description:
 *   Disables an Edge Interrupt at the alarm if it is set.
 *
 ****************************************************************************/

static int esp32_tim_disableint(FAR struct esp32_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  esp32_tim_modifyreg32(dev, TIM_CONFIG_OFFSET, TIMG_T0_LEVEL_INT_EN, 0);

  return OK;
}

/****************************************************************************
 * Name: esp32_tim_checkint
 *
 * Description:
 *   Check the interrupt status bit.
 *
 ****************************************************************************/

static int esp32_tim_checkint(FAR struct esp32_tim_dev_s *dev)
{
  int ret = 0;
  uint32_t reg_value;

  DEBUGASSERT(dev);

  /* Timer 0 from group 0 or 1 */

  if (((struct esp32_tim_priv_s *)dev)->base == TIMG_T0CONFIG_REG(0) ||
    ((struct esp32_tim_priv_s *)dev)->base == TIMG_T0CONFIG_REG(1))
    {
      reg_value = esp32_tim_getreg(dev, TIM0_INT_ST_OFFSET);
      if (reg_value & TIMG_T0_INT_ST)
        {
          ret = 1;
        }
    }

  /* Timer 1 from group 0 or 1 */

  else
    {
      reg_value = esp32_tim_getreg(dev, TIM1_INT_ST_OFFSET);
      if (reg_value & TIMG_T1_INT_ST)
        {
          ret = 1;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: esp32_tim_ackint
 *
 *   Description:
 *   Acknowledges an interrupt
 *
 ****************************************************************************/

static int esp32_tim_ackint(FAR struct esp32_tim_dev_s *dev)
{
  DEBUGASSERT(dev);

  /* Timer 0 from group 0 or 1 */

  if (((struct esp32_tim_priv_s *)dev)->base == TIMG_T0CONFIG_REG(0) ||
    ((struct esp32_tim_priv_s *)dev)->base == TIMG_T0CONFIG_REG(1))
    {
        esp32_tim_putreg(dev, TIM0_CLR_OFFSET, TIMG_T0_INT_CLR);
    }

  /* Timer 1 from group 0 or 1 */

  else
    {
      esp32_tim_putreg(dev, TIM1_CLR_OFFSET, TIMG_T1_INT_CLR);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_tim_init
 *
 * Description:
 *   Initialize TIMER device
 *
 ****************************************************************************/

FAR struct esp32_tim_dev_s *esp32_tim_init(int timer)
{
  FAR struct esp32_tim_priv_s *tim = NULL;

  /* Get timer instance */

  switch (timer)
    {
#ifdef CONFIG_ESP32_TIMER0
      case 0:
        {
          tim = &g_esp32_tim0_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32_TIMER1
      case 1:
        {
          tim = &g_esp32_tim1_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32_TIMER2
      case 2:
        {
          tim = &g_esp32_tim2_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32_TIMER3
      case 3:
        {
          tim = &g_esp32_tim3_priv;
          break;
        }
#endif

      default:
        {
          tmrerr("ERROR: unsupported TIMER %d\n", timer);
          goto errout;
        }
    }

  if (tim->inuse == true)
    {
      tmrerr("ERROR: TIMER %d is already in use\n", timer);
      tim = NULL;
    }

  errout:
  return (FAR struct esp32_tim_dev_s *)tim;
}

/****************************************************************************
 * Name: esp32_tim_deinit
 *
 * Description:
 *   Deinit TIMER device
 *
 ****************************************************************************/

int esp32_tim_deinit(FAR struct esp32_tim_dev_s *dev)
{
  FAR struct esp32_tim_priv_s *tim = NULL;

  DEBUGASSERT(dev);

  tim = (FAR struct esp32_tim_priv_s *)dev;

  tim->inuse = false;

  return OK;
}
