/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_tim.c
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

#include <stdbool.h>
#include <debug.h>

#include "riscv_arch.h"
#include "hardware/esp32c3_tim.h"

#include "esp32c3_tim.h"
#include "esp32c3_irq.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32c3_tim_priv_s
{
  FAR struct esp32c3_tim_ops_s *ops;
  uint8_t                       id;      /* Timer instance */
  uint8_t                       periph;  /* Peripheral ID */
  uint8_t                       irq;     /* Interrupt ID */
  int                           cpuint;  /* CPU interrupt assigned to this timer */
  bool                          inuse;   /* Flag indicating if the timer is in use */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TIM operations ***********************************************************/

static void esp32c3_tim_start(FAR struct esp32c3_tim_dev_s *dev);
static void esp32c3_tim_stop(FAR struct esp32c3_tim_dev_s *dev);
static void esp32c3_tim_clear(FAR struct esp32c3_tim_dev_s *dev);
static void esp32c3_tim_setmode(FAR struct esp32c3_tim_dev_s *dev,
                                enum esp32c3_tim_mode_e mode);
static void esp32c3_tim_setclksrc(FAR struct esp32c3_tim_dev_s *dev,
                                  enum esp32c3_tim_clksrc_e src);
static void esp32c3_tim_setpre(FAR struct esp32c3_tim_dev_s *dev,
                               uint16_t pre);
static void esp32c3_tim_getcounter(FAR struct esp32c3_tim_dev_s *dev,
                                   uint64_t *value);
static void esp32c3_tim_setcounter(FAR struct esp32c3_tim_dev_s *dev,
                                   uint64_t value);
static void esp32c3_tim_reload_now(FAR struct esp32c3_tim_dev_s *dev);
static void esp32c3_tim_getalarmvalue(FAR struct esp32c3_tim_dev_s *dev,
                                      uint64_t *value);
static void esp32c3_tim_setalarmvalue(FAR struct esp32c3_tim_dev_s *dev,
                                      uint64_t value);
static void esp32c3_tim_setalarm(FAR struct esp32c3_tim_dev_s *dev,
                                 bool enable);
static void esp32c3_tim_setautoreload(FAR struct esp32c3_tim_dev_s *dev,
                                      bool enable);
static int esp32c3_tim_setisr(FAR struct esp32c3_tim_dev_s *dev,
                               xcpt_t handler, FAR void * arg);
static void esp32c3_tim_enableint(FAR struct esp32c3_tim_dev_s *dev);
static void esp32c3_tim_disableint(FAR struct esp32c3_tim_dev_s *dev);
static void esp32c3_tim_ackint(FAR struct esp32c3_tim_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ESP32C3 TIM ops */

struct esp32c3_tim_ops_s esp32c3_tim_ops =
{
  .start         = esp32c3_tim_start,
  .stop          = esp32c3_tim_stop,
  .clear         = esp32c3_tim_clear,
  .setmode       = esp32c3_tim_setmode,
  .getcounter    = esp32c3_tim_getcounter,
  .setclksrc     = esp32c3_tim_setclksrc,
  .setpre        = esp32c3_tim_setpre,
  .setcounter    = esp32c3_tim_setcounter,
  .reloadnow     = esp32c3_tim_reload_now,
  .getalarmvalue = esp32c3_tim_getalarmvalue,
  .setalarmvalue = esp32c3_tim_setalarmvalue,
  .setalarm      = esp32c3_tim_setalarm,
  .setautoreload = esp32c3_tim_setautoreload,
  .setisr        = esp32c3_tim_setisr,
  .enableint     = esp32c3_tim_enableint,
  .disableint    = esp32c3_tim_disableint,
  .ackint        = esp32c3_tim_ackint
};

#ifdef CONFIG_ESP32C3_TIMER0
/* TIMER0 */

struct esp32c3_tim_priv_s g_esp32c3_tim0_priv =
{
  .ops    = &esp32c3_tim_ops,
  .id     = ESP32C3_TIMER0,
  .periph = ESP32C3_PERIPH_TG0_T0,   /* Peripheral ID */
  .irq    = ESP32C3_IRQ_TG0_T0,      /* Interrupt ID */
  .cpuint = -ENOMEM,                 /* CPU interrupt assigned to this timer */
  .inuse = false,
};
#endif

#ifdef CONFIG_ESP32C3_TIMER1
/* TIMER1 */

struct esp32c3_tim_priv_s g_esp32c3_tim1_priv =
{
  .ops   = &esp32c3_tim_ops,
  .id     = ESP32C3_TIMER1,
  .periph = ESP32C3_PERIPH_TG1_T0,   /* Peripheral ID */
  .irq    = ESP32C3_IRQ_TG1_T0,      /* Interrupt ID */
  .cpuint = -ENOMEM,                 /* CPU interrupt assigned to this timer */
  .inuse = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_tim_start
 *
 * Description:
 *   Release the counter.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32c3_tim_start(FAR struct esp32c3_tim_dev_s *dev)
{
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;
  modifyreg32(TIMG_T0CONFIG_REG(priv->id), TIMG_T0_EN_M, TIMG_T0_EN_M);
}

/****************************************************************************
 * Name: esp32c3_tim_stop
 *
 * Description:
 *   Halt the counter.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32c3_tim_stop(FAR struct esp32c3_tim_dev_s *dev)
{
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;
  modifyreg32(TIMG_T0CONFIG_REG(priv->id), TIMG_T0_EN_M, 0);
}

/****************************************************************************
 * Name: esp32c3_tim_clear
 *
 * Description:
 *   Set the counter to zero instantly.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32c3_tim_clear(FAR struct esp32c3_tim_dev_s *dev)
{
  uint64_t clear_value = 0;
  DEBUGASSERT(dev);
  esp32c3_tim_setcounter(dev, clear_value);
  esp32c3_tim_reload_now(dev);
}

/****************************************************************************
 * Name: esp32c3_tim_setmode
 *
 * Description:
 *   Set counter mode.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   mode          - Variable indicating the counting direction (up/down).
 *
 ****************************************************************************/

static void esp32c3_tim_setmode(FAR struct esp32c3_tim_dev_s *dev,
                               enum esp32c3_tim_mode_e mode)
{
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;
  if (mode == ESP32C3_TIM_MODE_DOWN)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->id), TIMG_T0_INCREASE_M, 0);
    }
  else if (mode == ESP32C3_TIM_MODE_UP)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->id), 0, TIMG_T0_INCREASE_M);
    }
}

/****************************************************************************
 * Name: esp32c3_tim_setclksrc
 *
 * Description:
 *   Set CLK source.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   src           - The source, it may be APB_CLK or XTAL_CLK.
 *
 ****************************************************************************/

static void esp32c3_tim_setclksrc(FAR struct esp32c3_tim_dev_s *dev,
                                 enum esp32c3_tim_clksrc_e src)
{
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;
  if (src == ESP32C3_TIM_APB_CLK)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->id), TIMG_T0_USE_XTAL_M, 0);
    }
  else if(src == ESP32C3_TIM_XTAL_CLK)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->id), 0, TIMG_T0_USE_XTAL_M);
    }
}

/****************************************************************************
 * Name: esp32c3_tim_setpre
 *
 * Description:
 *    Set the prescaler.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   pre           - This is the division factor. This variable accepts
 *                   values from 0 to 65535. If pre = 0, the division factor
 *                   is 65536, if pre = 1 or 2, the division factor is 2.
 *
 ****************************************************************************/

static void esp32c3_tim_setpre(FAR struct esp32c3_tim_dev_s *dev,
                               uint16_t pre)
{
  uint32_t mask = (uint32_t)pre << TIMG_T0_DIVIDER_S;
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;
  modifyreg32(TIMG_T0CONFIG_REG(priv->id), TIMG_T0_DIVIDER_M, mask);
}

/****************************************************************************
 * Name: esp32c3_tim_getcounter
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

static void esp32c3_tim_getcounter(FAR struct esp32c3_tim_dev_s *dev,
                                uint64_t *value)
{
  uint32_t value_32;
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;
  *value = 0;

  /* Dummy value (0 or 1) to latch the counter value to read it */

  putreg32(BIT(0), TIMG_T0UPDATE_REG(priv->id));

  /* Read value */

  value_32  = getreg32(TIMG_T0HI_REG(priv->id)); /* High 32 bits */

  /* Discard the top 10 bits */

  value_32 &= LOW_22_MASK;
  *value   |= (uint64_t)value_32;
  *value  <<= SHIFT_32;
  value_32  = getreg32(TIMG_T0LO_REG(priv->id)); /* Low 32 bits */
  *value   |= (uint64_t)value_32;
}

/****************************************************************************
 * Name: esp32c3_tim_setcounter
 *
 * Description:
 *   Set the value to be loaded to the counter.
 *   If you want the counter to be loaded at an alarm, enable the alarm and
 *   the auto-reload before.
 *   If you want the counter to be loaded instantly, call
 *   esp32c3_tim_reload_now() after this function.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - The value to be loaded the counter.
 *
 ****************************************************************************/

static void esp32c3_tim_setcounter(FAR struct esp32c3_tim_dev_s *dev,
                                  uint64_t value)
{
  uint64_t low_64 = value & LOW_32_MASK;

  /* Get only the low 22 bits. */

  uint64_t high_64 = (value >> SHIFT_32) & LOW_22_MASK;
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;

  /* Set the counter value */

  putreg32((uint32_t)low_64, TIMG_T0LOADLO_REG(priv->id));
  putreg32((uint32_t)high_64, TIMG_T0LOADHI_REG(priv->id));
}

/****************************************************************************
 * Name: esp32c3_tim_reload_now
 *
 * Description:
 *   Reload the counter instantly. It may be called after
 *   esp32c3_tim_setcounter().
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32c3_tim_reload_now(FAR struct esp32c3_tim_dev_s *dev)
{
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;

  /* Dummy value to trigger reloading  */

  putreg32(BIT(0), TIMG_T0LOAD_REG(priv->id));
}

/****************************************************************************
 * Name: esp32c3_tim_getalarmvalue
 *
 * Description:
 *   Get the alarm value.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - Pointer to retrieve the current configured alarm value.
 *
 ****************************************************************************/

static void esp32c3_tim_getalarmvalue(FAR struct esp32c3_tim_dev_s *dev,
                                   uint64_t *value)
{
  uint32_t value_32;
  struct esp32c3_tim_priv_s *priv;

  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;
  *value = 0;

  /* Read value */

  value_32  = getreg32(TIMG_T0ALARMHI_REG(priv->id)); /* High 32 bits */

  /* Get only the 22 low bits. */

  value_32 &= LOW_22_MASK;
  *value   |= (uint64_t)value_32;
  *value  <<= SHIFT_32;
  value_32  = getreg32(TIMG_T0ALARMLO_REG(priv->id)); /* Low 32 bits */
  *value   |= (uint64_t)value_32;
}

/****************************************************************************
 * Name: esp32c3_tim_setalarmvalue
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

static void esp32c3_tim_setalarmvalue(FAR struct esp32c3_tim_dev_s *dev,
                                   uint64_t value)
{
  struct esp32c3_tim_priv_s *priv;
  uint64_t low_64  = value & LOW_32_MASK;
  uint64_t high_64 = (value >> SHIFT_32) & LOW_22_MASK;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;

  /* Set an alarm value */

  putreg32((uint32_t)low_64, TIMG_T0ALARMLO_REG(priv->id));
  putreg32((uint32_t)high_64, TIMG_T0ALARMHI_REG(priv->id));
}

/****************************************************************************
 * Name: esp32c3_tim_setalarm
 *
 * Description:
 *   Enable/Disable the alarm.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   enable        - A variable to indicate the action. If true, enable
 *                   the alarm, if false, disable it.
 *
 ****************************************************************************/

static void esp32c3_tim_setalarm(FAR struct esp32c3_tim_dev_s *dev,
                                 bool enable)
{
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;

  if (enable)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->id), 0, TIMG_T0_ALARM_EN_M);
    }
  else
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->id), TIMG_T0_ALARM_EN_M, 0);
    }
}

/****************************************************************************
 * Name: esp32c3_tim_setautoreload
 *
 * Description:
 *   Enable or disable the auto reload.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   enable        - A variable to indicate the action. If it is true,
 *                   enable the auto reload, if false,
 *                   disable auto reload.
 *
 ****************************************************************************/

static void esp32c3_tim_setautoreload(FAR struct esp32c3_tim_dev_s *dev,
                                   bool enable)
{
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;

  if (enable)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->id), 0, TIMG_T0_AUTORELOAD_M);
    }
  else
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->id), TIMG_T0_AUTORELOAD_M, 0);
    }
}

/****************************************************************************
 * Name: esp32c3_tim_setisr
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

static int esp32c3_tim_setisr(FAR struct esp32c3_tim_dev_s *dev,
                               xcpt_t handler, FAR void *arg)
{
  FAR struct esp32c3_tim_priv_s *priv = NULL;
  int ret = OK;

  DEBUGASSERT(dev);

  priv = (FAR struct esp32c3_tim_priv_s *)dev;

  /* Disable interrupt when callback is removed. */

  if (handler == NULL)
    {
      if (priv->cpuint != -ENOMEM)
        {
          /* Disable cpu interrupt */

          up_disable_irq(priv->cpuint);

          /* Dissociate the IRQ from the ISR */

          irq_detach(priv->irq);

          /* Free cpu interrupt that is attached to this peripheral */

          esp32c3_free_cpuint(priv->periph);
          priv->cpuint = -ENOMEM;
        }
    }

  /* Otherwise set callback and enable interrupt */

  else
    {
      if (priv->cpuint != -ENOMEM)
        {
          /* Disable the provided CPU interrupt to configure it. */

          up_disable_irq(priv->cpuint);

          /* Free cpu interrupt that is attached to this peripheral
           * because we will get another from esp32c3_request_irq()
           */

          esp32c3_free_cpuint(priv->periph);
        }

      priv->cpuint = esp32c3_request_irq(priv->periph,
                                        ESP32C3_INT_PRIO_DEF,
                                        ESP32C3_INT_LEVEL);

      if (priv->cpuint < 0)
        {
          tmrerr("ERROR: Failed to get a CPU interrupt");
          ret = priv->cpuint;
          goto errout;
        }

      /* Associate an IRQ Number (from the timer) to an ISR */

      ret = irq_attach(priv->irq, handler, arg);
      if (ret != OK)
        {
          tmrerr("ERROR: Failed to associate an IRQ Number to and ISR");
          esp32c3_free_cpuint(priv->periph);
          goto errout;
        }

      /* Enable the CPU Interrupt that is linked to the timer */

      up_enable_irq(priv->cpuint);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: esp32c3_tim_enableint
 *
 * Description:
 *   Enable a level Interrupt at the alarm if it is set.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32c3_tim_enableint(FAR struct esp32c3_tim_dev_s *dev)
{
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;
  modifyreg32(TIMG_INT_ENA_TIMERS_REG(priv->id), 0, TIMG_T0_INT_ENA_M);
}

/****************************************************************************
 * Name: esp32c3_tim_disableint
 *
 * Description:
 *   Disable a level Interrupt at the alarm if it is set.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32c3_tim_disableint(FAR struct esp32c3_tim_dev_s *dev)
{
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;
  modifyreg32(TIMG_INT_ENA_TIMERS_REG(priv->id), TIMG_T0_INT_ENA_M, 0);
}

/****************************************************************************
 * Name: esp32c3_tim_ackint
 *
 * Description:
 *   Acknowledge an interrupt, that means, clear the interrupt.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32c3_tim_ackint(FAR struct esp32c3_tim_dev_s *dev)
{
  struct esp32c3_tim_priv_s *priv;
  DEBUGASSERT(dev);
  priv = (FAR struct esp32c3_tim_priv_s *)dev;
  modifyreg32(TIMG_INT_CLR_TIMERS_REG(priv->id), 0, TIMG_T0_INT_CLR_M);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_tim_init
 *
 * Description:
 *   Initialize TIMER device, if software real-time timer
 *   (CONFIG_ESP32C3_RT_TIMER) is enabled, then timer0 can't
 *   be initialized by this function directly.
 *
 * Parameters:
 *   timer           - Timer instance to be initialized.
 *                     Valid values: 0 or 1.
 *
 * Returned Values:
 *   If the initialization is successful, return a pointer to the timer
 *   driver struct associated to that timer instance.
 *   In case it fails, return NULL.
 *
 ****************************************************************************/

FAR struct esp32c3_tim_dev_s *esp32c3_tim_init(int timer)
{
  FAR struct esp32c3_tim_priv_s *tim = NULL;

  /* Get timer instance */

  switch (timer)
    {
#if defined(CONFIG_ESP32C3_TIMER0) && !defined(CONFIG_ESP32C3_RT_TIMER)
      case 0:
        {
          tim = &g_esp32c3_tim0_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32C3_TIMER1
      case 1:
        {
          tim = &g_esp32c3_tim1_priv;
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
  return (FAR struct esp32c3_tim_dev_s *)tim;
}

/****************************************************************************
 * Name: esp32c3_tim_deinit
 *
 * Description:
 *   Deinit TIMER device.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

void esp32c3_tim_deinit(FAR struct esp32c3_tim_dev_s *dev)
{
  FAR struct esp32c3_tim_priv_s *tim = NULL;

  DEBUGASSERT(dev);

  tim = (FAR struct esp32c3_tim_priv_s *)dev;
  tim->inuse = false;
}

/****************************************************************************
 * Name: esp32c3_tim0_init
 *
 * Description:
 *   Initialize TIMER0 device, if software real-time timer
 *   (CONFIG_ESP32C3_RT_TIMER) is enabled.
 *
 * Parameters:
 *   None
 *
 * Returned Values:
 *   If the initialization is successful, return a pointer to the timer
 *   driver struct associated to that timer instance.
 *   In case it fails, return NULL.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32C3_RT_TIMER

FAR struct esp32c3_tim_dev_s *esp32c3_tim0_init(void)
{
  FAR struct esp32c3_tim_priv_s *tim = &g_esp32c3_tim0_priv;

  if (tim->inuse == true)
    {
      tmrerr("ERROR: TIMER0 is already in use\n");
      tim = NULL;
    }

  return (FAR struct esp32c3_tim_dev_s *)tim;
}

#endif
