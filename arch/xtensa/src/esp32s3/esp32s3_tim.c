/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_tim.c
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
#include <assert.h>
#include <debug.h>

#include "xtensa.h"
#include "hardware/esp32s3_tim.h"

#include "esp32s3_tim.h"
#include "esp32s3_irq.h"
#include "esp32s3_gpio.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Timer Group IDs */

enum esp32s3_tim_gid_e
{
  ESP32S3_TIM_GROUP0 = 0,  /* Timer Group 0 */
  ESP32S3_TIM_GROUP1,      /* Timer Group 1 */
};

/* Timer IDs */

enum esp32s3_tim_tid_e
{
  ESP32S3_TIM_TIMER0 = 0,  /* Timer 0 */
  ESP32S3_TIM_TIMER1,      /* Timer 1 */
};

struct esp32s3_tim_priv_s
{
  struct esp32s3_tim_ops_s *ops;
  enum esp32s3_tim_gid_e    gid;      /* Timer Group ID */
  enum esp32s3_tim_tid_e    tid;      /* Timer ID */
  uint8_t                   periph;   /* Peripheral ID */
  uint8_t                   irq;      /* Interrupt ID */
  int                       cpuint;   /* CPU interrupt assigned to this timer */
  int                       core;     /* Core that is taking care of the timer ints */
  bool                      inuse;    /* Flag indicating if the timer is in use */
  uint8_t                   priority; /* Interrupt priority */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TIM operations ***********************************************************/

static void tim_start(struct esp32s3_tim_dev_s *dev);
static void tim_stop(struct esp32s3_tim_dev_s *dev);
static void tim_clear(struct esp32s3_tim_dev_s *dev);
static void tim_setmode(struct esp32s3_tim_dev_s *dev,
                        enum esp32s3_tim_mode_e mode);
static void tim_setclksrc(struct esp32s3_tim_dev_s *dev,
                          enum esp32s3_tim_clksrc_e src);
static void tim_setpre(struct esp32s3_tim_dev_s *dev, uint16_t pre);
static void tim_getcounter(struct esp32s3_tim_dev_s *dev, uint64_t *value);
static void tim_setcounter(struct esp32s3_tim_dev_s *dev, uint64_t value);
static void tim_reload_now(struct esp32s3_tim_dev_s *dev);
static void tim_getalarmvalue(struct esp32s3_tim_dev_s *dev,
                              uint64_t *value);
static void tim_setalarmvalue(struct esp32s3_tim_dev_s *dev, uint64_t value);
static void tim_setalarm(struct esp32s3_tim_dev_s *dev, bool enable);
static void tim_setautoreload(struct esp32s3_tim_dev_s *dev, bool enable);
static int  tim_setisr(struct esp32s3_tim_dev_s *dev, xcpt_t handler,
                       void *arg);
static void tim_enableint(struct esp32s3_tim_dev_s *dev);
static void tim_disableint(struct esp32s3_tim_dev_s *dev);
static void tim_ackint(struct esp32s3_tim_dev_s *dev);
static int  tim_checkint(struct esp32s3_tim_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ESP32-S3 TIM ops */

struct esp32s3_tim_ops_s esp32s3_tim_ops =
{
  .start         = tim_start,
  .stop          = tim_stop,
  .clear         = tim_clear,
  .setmode       = tim_setmode,
  .getcounter    = tim_getcounter,
  .setclksrc     = tim_setclksrc,
  .setpre        = tim_setpre,
  .setcounter    = tim_setcounter,
  .reloadnow     = tim_reload_now,
  .getalarmvalue = tim_getalarmvalue,
  .setalarmvalue = tim_setalarmvalue,
  .setalarm      = tim_setalarm,
  .setautoreload = tim_setautoreload,
  .setisr        = tim_setisr,
  .enableint     = tim_enableint,
  .disableint    = tim_disableint,
  .ackint        = tim_ackint,
  .checkint      = tim_checkint
};

#ifdef CONFIG_ESP32S3_TIMER0
/* TIMER0 */

struct esp32s3_tim_priv_s g_esp32s3_tim0_priv =
{
  .ops      = &esp32s3_tim_ops,
  .gid      = ESP32S3_TIM_GROUP0,
  .tid      = ESP32S3_TIM_TIMER0,
  .periph   = ESP32S3_PERIPH_TG_T0_LEVEL,
  .irq      = ESP32S3_IRQ_TG_T0_LEVEL,
  .cpuint   = -ENOMEM,
  .core     = -ENODEV,
  .inuse    = false,
  .priority = 1,
};
#endif

#ifdef CONFIG_ESP32S3_TIMER1
/* TIMER1 */

struct esp32s3_tim_priv_s g_esp32s3_tim1_priv =
{
  .ops      = &esp32s3_tim_ops,
  .gid      = ESP32S3_TIM_GROUP0,
  .tid      = ESP32S3_TIM_TIMER1,
  .periph   = ESP32S3_PERIPH_TG_T1_LEVEL,
  .irq      = ESP32S3_IRQ_TG_T1_LEVEL,
  .cpuint   = -ENOMEM,
  .core     = -ENODEV,
  .inuse    = false,
  .priority = 1,
};
#endif

#ifdef CONFIG_ESP32S3_TIMER2
/* TIMER2 */

struct esp32s3_tim_priv_s g_esp32s3_tim2_priv =
{
  .ops      = &esp32s3_tim_ops,
  .gid      = ESP32S3_TIM_GROUP1,
  .tid      = ESP32S3_TIM_TIMER0,
  .periph   = ESP32S3_PERIPH_TG1_T0_LEVEL,
  .irq      = ESP32S3_IRQ_TG1_T0_LEVEL,
  .cpuint   = -ENOMEM,
  .core     = -ENODEV,
  .inuse    = false,
  .priority = 1,
};
#endif

#ifdef CONFIG_ESP32S3_TIMER3
/* TIMER3 */

struct esp32s3_tim_priv_s g_esp32s3_tim3_priv =
{
  .ops      = &esp32s3_tim_ops,
  .gid      = ESP32S3_TIM_GROUP1,
  .tid      = ESP32S3_TIM_TIMER1,
  .periph   = ESP32S3_PERIPH_TG1_T1_LEVEL,
  .irq      = ESP32S3_IRQ_TG1_T1_LEVEL,
  .cpuint   = -ENOMEM,
  .core     = -ENODEV,
  .inuse    = false,
  .priority = 1,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tim_start
 *
 * Description:
 *   Release the counter.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tim_start(struct esp32s3_tim_dev_s *dev)
{
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->gid), TIMG_T0_EN_M, TIMG_T0_EN_M);
    }
  else
    {
      modifyreg32(TIMG_T1CONFIG_REG(priv->gid), TIMG_T1_EN_M, TIMG_T1_EN_M);
    }
}

/****************************************************************************
 * Name: tim_stop
 *
 * Description:
 *   Halt the counter.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tim_stop(struct esp32s3_tim_dev_s *dev)
{
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->gid), TIMG_T0_EN_M, 0);
    }
  else
    {
      modifyreg32(TIMG_T1CONFIG_REG(priv->gid), TIMG_T1_EN_M, 0);
    }
}

/****************************************************************************
 * Name: tim_clear
 *
 * Description:
 *   Set the counter to zero instantly.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tim_clear(struct esp32s3_tim_dev_s *dev)
{
  uint64_t clear_value = 0;

  DEBUGASSERT(dev != NULL);

  tim_setcounter(dev, clear_value);
  tim_reload_now(dev);
}

/****************************************************************************
 * Name: tim_setmode
 *
 * Description:
 *   Set counter mode.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   mode          - Variable indicating the counting direction (up/down).
 *
 ****************************************************************************/

static void tim_setmode(struct esp32s3_tim_dev_s *dev,
                        enum esp32s3_tim_mode_e mode)
{
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  if (mode == ESP32S3_TIM_MODE_DOWN)
    {
      if (priv->tid == ESP32S3_TIM_TIMER0)
        {
          modifyreg32(TIMG_T0CONFIG_REG(priv->gid), TIMG_T0_INCREASE_M, 0);
        }
      else
        {
          modifyreg32(TIMG_T1CONFIG_REG(priv->gid), TIMG_T1_INCREASE_M, 0);
        }
    }
  else if (mode == ESP32S3_TIM_MODE_UP)
    {
      if (priv->tid == ESP32S3_TIM_TIMER0)
        {
          modifyreg32(TIMG_T0CONFIG_REG(priv->gid), 0, TIMG_T0_INCREASE_M);
        }
      else
        {
          modifyreg32(TIMG_T1CONFIG_REG(priv->gid), 0, TIMG_T1_INCREASE_M);
        }
    }
}

/****************************************************************************
 * Name: tim_setclksrc
 *
 * Description:
 *   Set CLK source.
 *   NOTE: It's not necessary to configure each timer's register for clock,
 *   because it doesn't matter which timer is configured, the clock
 *   configuration will apply to the timer group.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   src           - The source, it may be APB_CLK or XTAL_CLK.
 *
 ****************************************************************************/

static void tim_setclksrc(struct esp32s3_tim_dev_s *dev,
                          enum esp32s3_tim_clksrc_e src)
{
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  if (src == ESP32S3_TIM_APB_CLK)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->gid), TIMG_T0_USE_XTAL_M, 0);
    }
  else if (src == ESP32S3_TIM_XTAL_CLK)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->gid), 0, TIMG_T0_USE_XTAL_M);
    }
}

/****************************************************************************
 * Name: tim_setpre
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

static void tim_setpre(struct esp32s3_tim_dev_s *dev, uint16_t pre)
{
  uint32_t mask = (uint32_t)pre << TIMG_T0_DIVIDER_S;
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->gid), TIMG_T0_DIVIDER_M, mask);
    }
  else
    {
      modifyreg32(TIMG_T1CONFIG_REG(priv->gid), TIMG_T1_DIVIDER_M, mask);
    }
}

/****************************************************************************
 * Name: tim_getcounter
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

static void tim_getcounter(struct esp32s3_tim_dev_s *dev, uint64_t *value)
{
  uint32_t value_32;
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;
  *value = 0;

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      /* Dummy value (0 or 1) to latch the counter value to read it */

      putreg32(BIT(0), TIMG_T0UPDATE_REG(priv->gid));

      /* Read value */

      value_32  = getreg32(TIMG_T0HI_REG(priv->gid)); /* High 32 bits */
      *value   |= (uint64_t)value_32;
      *value  <<= SHIFT_32;
      value_32  = getreg32(TIMG_T0LO_REG(priv->gid)); /* Low 32 bits */
      *value   |= (uint64_t)value_32;
    }
  else
    {
      /* Dummy value (0 or 1) to latch the counter value to read it */

      putreg32(BIT(0), TIMG_T1UPDATE_REG(priv->gid));

      /* Read value */

      value_32  = getreg32(TIMG_T1HI_REG(priv->gid)); /* High 32 bits */
      *value   |= (uint64_t)value_32;
      *value  <<= SHIFT_32;
      value_32  = getreg32(TIMG_T1LO_REG(priv->gid)); /* Low 32 bits */
      *value   |= (uint64_t)value_32;
    }
}

/****************************************************************************
 * Name: tim_setcounter
 *
 * Description:
 *   Set the value to be loaded to the counter.
 *   If you want the counter to be loaded at an alarm, enable the alarm and
 *   the auto-reload before.
 *   If you want the counter to be loaded instantly, call
 *   tim_reload_now() after this function.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - The value to be loaded the counter.
 *
 ****************************************************************************/

static void tim_setcounter(struct esp32s3_tim_dev_s *dev, uint64_t value)
{
  uint64_t low_64 = value & LOW_32_MASK;
  uint64_t high_64;
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  high_64 = (value >> SHIFT_32) & LOW_22_MASK;

  /* Set the counter value */

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      putreg32((uint32_t)low_64, TIMG_T0LOADLO_REG(priv->gid));
      putreg32((uint32_t)high_64, TIMG_T0LOADHI_REG(priv->gid));
    }
  else
    {
      putreg32((uint32_t)low_64, TIMG_T1LOADLO_REG(priv->gid));
      putreg32((uint32_t)high_64, TIMG_T1LOADHI_REG(priv->gid));
    }
}

/****************************************************************************
 * Name: tim_reload_now
 *
 * Description:
 *   Reload the counter instantly. It may be called after
 *   tim_setcounter().
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tim_reload_now(struct esp32s3_tim_dev_s *dev)
{
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  /* Dummy value to trigger reloading  */

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      putreg32(BIT(0), TIMG_T0LOAD_REG(priv->gid));
    }
  else
    {
      putreg32(BIT(0), TIMG_T1LOAD_REG(priv->gid));
    }
}

/****************************************************************************
 * Name: tim_getalarmvalue
 *
 * Description:
 *   Get the alarm value.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - Pointer to retrieve the current configured alarm value.
 *
 ****************************************************************************/

static void tim_getalarmvalue(struct esp32s3_tim_dev_s *dev, uint64_t *value)
{
  uint32_t value_32;
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;
  *value = 0;

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      value_32  = getreg32(TIMG_T0ALARMHI_REG(priv->gid)); /* High 32 bits */
      *value   |= (uint64_t)value_32;
      *value  <<= SHIFT_32;
      value_32  = getreg32(TIMG_T0ALARMLO_REG(priv->gid)); /* Low 32 bits */
      *value   |= (uint64_t)value_32;
    }
  else
    {
      value_32  = getreg32(TIMG_T1ALARMHI_REG(priv->gid)); /* High 32 bits */
      *value   |= (uint64_t)value_32;
      *value  <<= SHIFT_32;
      value_32  = getreg32(TIMG_T1ALARMLO_REG(priv->gid)); /* Low 32 bits */
      *value   |= (uint64_t)value_32;
    }
}

/****************************************************************************
 * Name: tim_setalarmvalue
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

static void tim_setalarmvalue(struct esp32s3_tim_dev_s *dev, uint64_t value)
{
  uint64_t low_64  = value & LOW_32_MASK;
  uint64_t high_64 = (value >> SHIFT_32) & LOW_22_MASK;
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  /* Set an alarm value */

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      putreg32((uint32_t)low_64, TIMG_T0ALARMLO_REG(priv->gid));
      putreg32((uint32_t)high_64, TIMG_T0ALARMHI_REG(priv->gid));
    }
  else
    {
      putreg32((uint32_t)low_64, TIMG_T1ALARMLO_REG(priv->gid));
      putreg32((uint32_t)high_64, TIMG_T1ALARMHI_REG(priv->gid));
    }
}

/****************************************************************************
 * Name: tim_setalarm
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

static void tim_setalarm(struct esp32s3_tim_dev_s *dev, bool enable)
{
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      if (enable)
        {
          modifyreg32(TIMG_T0CONFIG_REG(priv->gid), 0, TIMG_T0_ALARM_EN_M);
        }
      else
        {
          modifyreg32(TIMG_T0CONFIG_REG(priv->gid), TIMG_T0_ALARM_EN_M, 0);
        }
    }
  else
    {
      if (enable)
        {
          modifyreg32(TIMG_T1CONFIG_REG(priv->gid), 0, TIMG_T1_ALARM_EN_M);
        }
      else
        {
          modifyreg32(TIMG_T1CONFIG_REG(priv->gid), TIMG_T1_ALARM_EN_M, 0);
        }
    }
}

/****************************************************************************
 * Name: tim_setautoreload
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

static void tim_setautoreload(struct esp32s3_tim_dev_s *dev, bool enable)
{
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      if (enable)
        {
          modifyreg32(TIMG_T0CONFIG_REG(priv->gid), 0, TIMG_T0_AUTORELOAD_M);
        }
      else
        {
          modifyreg32(TIMG_T0CONFIG_REG(priv->gid), TIMG_T0_AUTORELOAD_M, 0);
        }
    }
  else
    {
      if (enable)
        {
          modifyreg32(TIMG_T1CONFIG_REG(priv->gid), 0, TIMG_T1_AUTORELOAD_M);
        }
      else
        {
          modifyreg32(TIMG_T1CONFIG_REG(priv->gid), TIMG_T1_AUTORELOAD_M, 0);
        }
    }
}

/****************************************************************************
 * Name: tim_setisr
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

static int tim_setisr(struct esp32s3_tim_dev_s *dev, xcpt_t handler,
                      void *arg)
{
  struct esp32s3_tim_priv_s *priv = NULL;
  int ret = OK;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  /* Disable interrupt when callback is removed. */

  if (handler == NULL)
    {
      /* If a CPU Interrupt was previously allocated, then deallocate it */

      if (priv->cpuint != -ENOMEM)
        {
          /* Disable CPU Interrupt, free a previously allocated
           * CPU Interrupt
           */

          up_disable_irq(priv->irq);
          esp32s3_teardown_irq(priv->core, priv->periph, priv->cpuint);
          irq_detach(priv->irq);

          priv->cpuint = -ENOMEM;
          priv->core   = -ENODEV;
        }
    }

  /* Otherwise set callback and enable interrupt */

  else
    {
      if (priv->cpuint != -ENOMEM)
        {
          /* Disable the previous IRQ */

          up_disable_irq(priv->irq);
        }

      /* Set up to receive peripheral interrupts on the current CPU */

      priv->core = up_cpu_index();
      priv->cpuint = esp32s3_setup_irq(priv->core, priv->periph,
                                       priv->priority, ESP32S3_CPUINT_LEVEL);
      if (priv->cpuint < 0)
        {
          tmrerr("ERROR: No CPU Interrupt available");
          ret = priv->cpuint;
          goto errout;
        }

      /* Associate an IRQ Number (from the timer) to an ISR */

      ret = irq_attach(priv->irq, handler, arg);
      if (ret != OK)
        {
          esp32s3_teardown_irq(priv->core, priv->periph, priv->cpuint);
          tmrerr("ERROR: Failed to associate an IRQ Number");
          goto errout;
        }

      /* Enable the CPU Interrupt that is linked to the timer */

      up_enable_irq(priv->irq);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: tim_enableint
 *
 * Description:
 *   Enable a level Interrupt at the alarm if it is set.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tim_enableint(struct esp32s3_tim_dev_s *dev)
{
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      modifyreg32(TIMG_INT_ENA_TIMERS_REG(priv->gid), 0, TIMG_T0_INT_ENA_M);
    }
  else
    {
      modifyreg32(TIMG_INT_ENA_TIMERS_REG(priv->gid), 0, TIMG_T1_INT_ENA_M);
    }
}

/****************************************************************************
 * Name: tim_disableint
 *
 * Description:
 *   Disable a level Interrupt at the alarm if it is set.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tim_disableint(struct esp32s3_tim_dev_s *dev)
{
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      modifyreg32(TIMG_INT_ENA_TIMERS_REG(priv->gid), TIMG_T0_INT_ENA_M, 0);
    }
  else
    {
      modifyreg32(TIMG_INT_ENA_TIMERS_REG(priv->gid), TIMG_T1_INT_ENA_M, 0);
    }
}

/****************************************************************************
 * Name: tim_ackint
 *
 * Description:
 *   Acknowledge an interrupt, that means, clear the interrupt.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void tim_ackint(struct esp32s3_tim_dev_s *dev)
{
  struct esp32s3_tim_priv_s *priv;

  DEBUGASSERT(dev != NULL);

  priv = (struct esp32s3_tim_priv_s *)dev;

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      modifyreg32(TIMG_INT_CLR_TIMERS_REG(priv->gid), 0, TIMG_T0_INT_CLR_M);
    }
  else
    {
      modifyreg32(TIMG_INT_CLR_TIMERS_REG(priv->gid), 0, TIMG_T1_INT_CLR_M);
    }
}

/****************************************************************************
 * Name: tim_checkint
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

static int tim_checkint(struct esp32s3_tim_dev_s *dev)
{
  struct esp32s3_tim_priv_s *priv = (struct esp32s3_tim_priv_s *)dev;
  uint32_t reg_value;
  int ret;

  DEBUGASSERT(dev != NULL);

  reg_value = getreg32(TIMG_INT_ST_TIMERS_REG(priv->gid));

  if (priv->tid == ESP32S3_TIM_TIMER0)
    {
      ret = REG_MASK(reg_value, TIMG_T0_INT_ST);
    }
  else
    {
      ret = REG_MASK(reg_value, TIMG_T1_INT_ST);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_tim_init
 *
 * Description:
 *   Initialize TIMER device.
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

struct esp32s3_tim_dev_s *esp32s3_tim_init(int timer)
{
  struct esp32s3_tim_priv_s *tim = NULL;

  /* First, take the data structure associated with the timer instance */

  switch (timer)
    {
#ifdef CONFIG_ESP32S3_TIMER0
      case ESP32S3_TIMER0:
        {
          tim = &g_esp32s3_tim0_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32S3_TIMER1
      case ESP32S3_TIMER1:
        {
          tim = &g_esp32s3_tim1_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32S3_TIMER2
      case ESP32S3_TIMER2:
        {
          tim = &g_esp32s3_tim2_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32S3_TIMER3
      case ESP32S3_TIMER3:
        {
          tim = &g_esp32s3_tim3_priv;
          break;
        }
#endif

      default:
        {
          tmrerr("Unsupported TIMER %d\n", timer);
          goto errout;
        }
    }

  /* Verify if it is in use */

  if (!tim->inuse)
    {
      tim->inuse = true;  /* If it was not, now it is */
    }
  else
    {
      tmrerr("TIMER %d is already in use\n", timer);
      tim = NULL;
    }

errout:
  return (struct esp32s3_tim_dev_s *)tim;
}

/****************************************************************************
 * Name: esp32s3_tim_deinit
 *
 * Description:
 *   Deinit TIMER device.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

void esp32s3_tim_deinit(struct esp32s3_tim_dev_s *dev)
{
  struct esp32s3_tim_priv_s *tim = NULL;

  DEBUGASSERT(dev != NULL);

  tim = (struct esp32s3_tim_priv_s *)dev;
  tim->inuse = false;
}
