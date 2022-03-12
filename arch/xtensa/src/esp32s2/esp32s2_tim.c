/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_tim.c
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
#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "xtensa.h"
#include "esp32s2_irq.h"
#include "esp32s2_tim.h"
#include "hardware/esp32s2_system.h"
#include "hardware/esp32s2_systimer.h"
#include "hardware/esp32s2_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GROUP0 0
#define GROUP1 1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32s2_tim_priv_s
{
  struct esp32s2_tim_ops_s    *ops;
  uint8_t                          gid;  /* Group instance */
  uint8_t                          tid;  /* Timer instance */
  uint8_t                      int_pri;
  uint8_t                       periph;  /* Peripheral ID */
  uint8_t                          irq;  /* Interrupt ID */
  int                           cpuint;  /* CPU interrupt assigned to this timer */
  bool                           inuse;  /* Flag indicating if the timer is in use */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* TIM operations ***********************************************************/

static void esp32s2_tim_start(struct esp32s2_tim_dev_s *dev);
static void esp32s2_tim_stop(struct esp32s2_tim_dev_s *dev);
static void esp32s2_tim_clear(struct esp32s2_tim_dev_s *dev);
static void esp32s2_tim_setmode(struct esp32s2_tim_dev_s *dev,
                                enum esp32s2_tim_mode_e mode);
static void esp32s2_tim_setclksrc(struct esp32s2_tim_dev_s *dev,
                                  enum esp32s2_tim_clksrc_e src);
static void esp32s2_tim_setpre(struct esp32s2_tim_dev_s *dev,
                               uint16_t pre);
static void esp32s2_tim_setstep(struct esp32s2_tim_dev_s *dev,
                                enum esp32s2_tim_clksrc_e src,
                                uint16_t ticks);
static void esp32s2_tim_getcounter(struct esp32s2_tim_dev_s *dev,
                                   uint64_t *value);
static void esp32s2_tim_setcounter(struct esp32s2_tim_dev_s *dev,
                                   uint64_t value);
static void esp32s2_tim_reload_now(struct esp32s2_tim_dev_s *dev);
static void esp32s2_tim_getalarmvalue(struct esp32s2_tim_dev_s *dev,
                                      uint64_t *value);
static void esp32s2_tim_getperiod(struct esp32s2_tim_dev_s *dev,
                                  uint32_t *value);
static void esp32s2_tim_setalarmvalue(struct esp32s2_tim_dev_s *dev,
                                      uint64_t value);
static void esp32s2_tim_setperiod(struct esp32s2_tim_dev_s *dev,
                                  uint32_t value);
static void esp32s2_tim_setworkmode(struct esp32s2_tim_dev_s *dev,
                                    enum esp32s2_tim_work_mode_e mode);
static void esp32s2_tim_setalarm(struct esp32s2_tim_dev_s *dev,
                                 bool enable);
static void esp32s2_tim_setautoreload(struct esp32s2_tim_dev_s *dev,
                                      bool enable);
static int esp32s2_tim_setisr(struct esp32s2_tim_dev_s *dev,
                              xcpt_t handler, void * arg);
static void esp32s2_tim_enableint(struct esp32s2_tim_dev_s *dev);
static void esp32s2_tim_disableint(struct esp32s2_tim_dev_s *dev);
static void esp32s2_tim_ackint(struct esp32s2_tim_dev_s *dev);
static int  esp32s2_tim_checkint(struct esp32s2_tim_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* ESP32-S2 TIM ops */

struct esp32s2_tim_ops_s esp32s2_tim_ops =
{
  .start         = esp32s2_tim_start,
  .stop          = esp32s2_tim_stop,
  .clear         = esp32s2_tim_clear,
  .setmode       = esp32s2_tim_setmode,
  .getcounter    = esp32s2_tim_getcounter,
  .setclksrc     = esp32s2_tim_setclksrc,
  .setpre        = esp32s2_tim_setpre,
  .setstep       = NULL,
  .setcounter    = esp32s2_tim_setcounter,
  .reloadnow     = esp32s2_tim_reload_now,
  .getalarmvalue = esp32s2_tim_getalarmvalue,
  .getperiod     = NULL,
  .setalarmvalue = esp32s2_tim_setalarmvalue,
  .setperiod     = NULL,
  .setworkmode   = NULL,
  .setalarm      = esp32s2_tim_setalarm,
  .setautoreload = esp32s2_tim_setautoreload,
  .setisr        = esp32s2_tim_setisr,
  .enableint     = esp32s2_tim_enableint,
  .disableint    = esp32s2_tim_disableint,
  .ackint        = esp32s2_tim_ackint,
  .checkint      = esp32s2_tim_checkint
};

/* ESP32S2 SYSTIMER ops */

struct esp32s2_tim_ops_s esp32s2_systim_ops =
{
  .start         = NULL,  /* Systimer doesn't support releasing counter */
  .stop          = NULL,  /* Systimer doesn't support halting counter */
  .clear         = esp32s2_tim_clear,
  .setmode       = NULL,
  .getcounter    = esp32s2_tim_getcounter,
  .setclksrc     = NULL,
  .setpre        = NULL,
  .setstep       = esp32s2_tim_setstep,
  .setcounter    = esp32s2_tim_setcounter,
  .reloadnow     = esp32s2_tim_reload_now,
  .getalarmvalue = esp32s2_tim_getalarmvalue,
  .getperiod     = esp32s2_tim_getperiod,
  .setalarmvalue = esp32s2_tim_setalarmvalue,
  .setperiod     = esp32s2_tim_setperiod,
  .setworkmode   = esp32s2_tim_setworkmode,
  .setalarm      = esp32s2_tim_setalarm,
  .setautoreload = NULL,
  .setisr        = esp32s2_tim_setisr,
  .enableint     = esp32s2_tim_enableint,
  .disableint    = esp32s2_tim_disableint,
  .ackint        = esp32s2_tim_ackint,
  .checkint      = esp32s2_tim_checkint
};

#ifdef CONFIG_ESP32S2_TIMER0

/* TIMER0 */

struct esp32s2_tim_priv_s g_esp32s2_tim0_priv =
{
  .ops        = &esp32s2_tim_ops,
  .gid        = GROUP0,
  .tid        = TIMER0,
  .int_pri    = ESP32S2_INT_PRIO_DEF,
  .periph     = ESP32S2_PERIPH_TG_T0_LEVEL, /* Peripheral ID */
  .irq        = ESP32S2_IRQ_TG_T0_LEVEL,    /* Interrupt ID */
  .cpuint     = -ENOMEM,                    /* CPU interrupt assigned to this timer */
  .inuse      = false,
};
#endif

#ifdef CONFIG_ESP32S2_TIMER1
/* TIMER1 */

struct esp32s2_tim_priv_s g_esp32s2_tim1_priv =
{
  .ops        = &esp32s2_tim_ops,
  .gid        = GROUP0,
  .tid        = TIMER1,
  .int_pri    = ESP32S2_INT_PRIO_DEF,
  .periph     = ESP32S2_PERIPH_TG_T1_LEVEL, /* Peripheral ID */
  .irq        = ESP32S2_IRQ_TG_T1_LEVEL,    /* Interrupt ID */
  .cpuint     = -ENOMEM,                    /* CPU interrupt assigned to this timer */
  .inuse      = false,
};
#endif

#ifdef CONFIG_ESP32S2_TIMER2
/* TIMER2 */

struct esp32s2_tim_priv_s g_esp32s2_tim2_priv =
{
  .ops        = &esp32s2_tim_ops,
  .gid        = GROUP1,
  .tid        = TIMER0,
  .int_pri    = ESP32S2_INT_PRIO_DEF,
  .periph     = ESP32S2_PERIPH_TG1_T0_LEVEL, /* Peripheral ID */
  .irq        = ESP32S2_IRQ_TG1_T0_LEVEL,    /* Interrupt ID */
  .cpuint     = -ENOMEM,                     /* CPU interrupt assigned to this timer */
  .inuse      = false,
};
#endif

#ifdef CONFIG_ESP32S2_TIMER3
/* TIMER3 */

struct esp32s2_tim_priv_s g_esp32s2_tim3_priv =
{
  .ops        = &esp32s2_tim_ops,
  .gid        = GROUP1,
  .tid        = TIMER1,
  .int_pri    = ESP32S2_INT_PRIO_DEF,
  .periph     = ESP32S2_PERIPH_TG1_T1_LEVEL, /* Peripheral ID */
  .irq        = ESP32S2_IRQ_TG1_T1_LEVEL,    /* Interrupt ID */
  .cpuint     = -ENOMEM,                     /* CPU interrupt assigned to this timer */
  .inuse      = false,
};
#endif

#ifdef CONFIG_ESP32S2_RT_TIMER
/* SYSTIMER */

struct esp32s2_tim_priv_s g_esp32s2_tim4_priv =
{
  .ops        = &esp32s2_systim_ops,
  .gid        = -ENODEV,                          /* There's no group in systimer */
  .tid        = SYSTIMER_COMP0,                   /* Systimer contains 1 counter and 3 comps */
  .int_pri    = ESP32S2_INT_PRIO_DEF,
  .periph     = ESP32S2_PERIPH_SYSTIMER_TARGET0,  /* Peripheral ID */
  .irq        = ESP32S2_IRQ_SYSTIMER_TARGET0,     /* Interrupt ID */
  .cpuint     = -ENOMEM,                          /* CPU interrupt assigned to this timer */
  .inuse      = false,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_tim_start
 *
 * Description:
 *   Release the counter.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32s2_tim_start(struct esp32s2_tim_dev_s *dev)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->tid == TIMER0)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->gid), 0, TIMG_T0_EN_M);
    }
  else
    {
      modifyreg32(TIMG_T1CONFIG_REG(priv->gid), 0, TIMG_T1_EN_M);
    }
}

/****************************************************************************
 * Name: esp32s2_tim_stop
 *
 * Description:
 *   Halt the counter.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32s2_tim_stop(struct esp32s2_tim_dev_s *dev)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->tid == TIMER0)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->gid), TIMG_T0_EN_M, 0);
    }
  else
    {
      modifyreg32(TIMG_T1CONFIG_REG(priv->gid), TIMG_T1_EN_M, 0);
    }
}

/****************************************************************************
 * Name: esp32s2_tim_clear
 *
 * Description:
 *   Set the counter to zero instantly.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32s2_tim_clear(struct esp32s2_tim_dev_s *dev)
{
  uint64_t clear_value = 0;

  DEBUGASSERT(dev);

  esp32s2_tim_setcounter(dev, clear_value);
  esp32s2_tim_reload_now(dev);
}

/****************************************************************************
 * Name: esp32s2_tim_setmode
 *
 * Description:
 *   Set counter mode.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   mode          - Variable indicating the counting direction (up/down).
 *
 ****************************************************************************/

static void esp32s2_tim_setmode(struct esp32s2_tim_dev_s *dev,
                                enum esp32s2_tim_mode_e mode)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  if (mode == ESP32S2_TIM_MODE_DOWN)
    {
      if (priv->tid == TIMER0)
        {
          modifyreg32(TIMG_T0CONFIG_REG(priv->gid), TIMG_T0_INCREASE_M, 0);
        }
      else
        {
          modifyreg32(TIMG_T1CONFIG_REG(priv->gid), TIMG_T1_INCREASE_M, 0);
        }
    }
  else if (mode == ESP32S2_TIM_MODE_UP)
    {
      if (priv->tid == TIMER0)
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
 * Name: esp32s2_tim_setclksrc
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

static void esp32s2_tim_setclksrc(struct esp32s2_tim_dev_s *dev,
                                  enum esp32s2_tim_clksrc_e src)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  if (src == ESP32S2_TIM_APB_CLK)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->gid), TIMG_T0_USE_XTAL_M, 0);
    }
  else if(src == ESP32S2_TIM_XTAL_CLK)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->gid), 0, TIMG_T0_USE_XTAL_M);
    }
}

/****************************************************************************
 * Name: esp32s2_tim_setpre
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

static void esp32s2_tim_setpre(struct esp32s2_tim_dev_s *dev,
                               uint16_t pre)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;
  uint32_t mask = (uint32_t)pre << TIMG_T0_DIVIDER_S;

  DEBUGASSERT(dev);

  if (priv->tid == TIMER0)
    {
      modifyreg32(TIMG_T0CONFIG_REG(priv->gid), TIMG_T0_DIVIDER_M, mask);
    }
  else
    {
      modifyreg32(TIMG_T1CONFIG_REG(priv->gid), TIMG_T1_DIVIDER_M, mask);
    }
}

/****************************************************************************
 * Name: esp32s2_tim_setstep
 *
 * Description:
 *    Set the timer increment step, i.e, defines the time interval in ticks
 *    used by the counter.
 *    NOTE: dev pointer is not used because this feature is not available
 *    for Generic Timers and it doesn't rely in any systimer comparator.
 *
 * Parameters:
 *   src           - Define the clock source for the ticks.
 *   ticks         - Number of ticks to define 1 increment step.
 *
 ****************************************************************************/

static void esp32s2_tim_setstep(struct esp32s2_tim_dev_s *dev,
                                enum esp32s2_tim_clksrc_e src,
                                uint16_t ticks)
{
  if (src == ESP32S2_TIM_PLL_CLK)
    {
      REG_SET_FIELD(SYSTIMER_STEP_REG, SYSTIMER_TIMER_PLL_STEP,
                    ticks & SYSTIMER_TIMER_PLL_STEP_V);
    }
  else if(src == ESP32S2_TIM_XTAL_CLK)
    {
      REG_SET_FIELD(SYSTIMER_STEP_REG, SYSTIMER_TIMER_XTAL_STEP,
                    ticks & SYSTIMER_TIMER_XTAL_STEP_V);
    }
}

/****************************************************************************
 * Name: esp32s2_tim_getcounter
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

static void esp32s2_tim_getcounter(struct esp32s2_tim_dev_s *dev,
                                   uint64_t *value)
{
  uint32_t value_32;
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  *value = 0;
  if (priv->tid == SYSTIMER_COMP0)
    {
      /* Latch counter */

      modifyreg32(SYSTIMER_UPDATE_REG, 0, SYSTIMER_TIMER_UPDATE);

      /* Wait until result is ready */

      while (!REG_GET_FIELD(SYSTIMER_UPDATE_REG,
             SYSTIMER_TIMER_VALUE_VALID));

      /* Read value */

      value_32  = getreg32(SYSTIMER_VALUE_HI_REG); /* High 32 bits */
      *value   |= (uint64_t)value_32;
      *value  <<= SHIFT_32;
      value_32  = getreg32(SYSTIMER_VALUE_LO_REG); /* Low 32 bits */
      *value   |= (uint64_t)value_32;
    }
  else
    {
      if (priv->tid == TIMER0)
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
}

/****************************************************************************
 * Name: esp32s2_tim_setcounter
 *
 * Description:
 *   Set the value to be loaded to the counter.
 *   If you want the counter to be loaded at an alarm, enable the alarm and
 *   the auto-reload before.
 *   If you want the counter to be loaded instantly, call
 *   esp32s2_tim_reload_now() after this function.
 *   NOTE: Systimer has 1 counter for 3 comparators.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - The value to be loaded the counter.
 *
 ****************************************************************************/

static void esp32s2_tim_setcounter(struct esp32s2_tim_dev_s *dev,
                                   uint64_t value)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;
  uint64_t low_64 = value & UINT32_MAX;
  uint64_t high_64 = (value >> SHIFT_32);

  DEBUGASSERT(dev);

  /* Set the counter value */

  if (priv->tid == SYSTIMER_COMP0)
    {
      putreg32((uint32_t)low_64, SYSTIMER_LOAD_LO_REG);
      putreg32((uint32_t)high_64, SYSTIMER_LOAD_HI_REG);
    }
  else
    {
      if (priv->tid == TIMER0)
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
}

/****************************************************************************
 * Name: esp32s2_tim_reload_now
 *
 * Description:
 *   Reload the counter instantly. It may be called after
 *   esp32s2_tim_setcounter().
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32s2_tim_reload_now(struct esp32s2_tim_dev_s *dev)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  /* Dummy value to trigger reloading  */

  if (priv->tid == SYSTIMER_COMP0)
    {
      putreg32(BIT(31), SYSTIMER_LOAD_REG);
    }
  else
    {
      if (priv->tid == TIMER0)
        {
          putreg32(BIT(0), TIMG_T0LOAD_REG(priv->gid));
        }
      else
        {
          putreg32(BIT(0), TIMG_T1LOAD_REG(priv->gid));
        }
    }
}

/****************************************************************************
 * Name: esp32s2_tim_getalarmvalue
 *
 * Description:
 *   Get the alarm value.
 *   NOTE: For systimer, the returned alarm value is the Time-Delay Alarm.
 *   See TRM for more details.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - Pointer to retrieve the current configured alarm value.
 *
 ****************************************************************************/

static void esp32s2_tim_getalarmvalue(struct esp32s2_tim_dev_s *dev,
                                      uint64_t *value)
{
  uint32_t value_32;
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  *value = 0;

  /* Read value */

  if (priv->tid == SYSTIMER_COMP0)
    {
      value_32  = getreg32(SYSTIMER_TARGET0_HI_REG); /* High 32 bits */
      *value   |= (uint64_t)value_32;
      *value  <<= SHIFT_32;
      value_32  = getreg32(SYSTIMER_TARGET0_LO_REG); /* Low 32 bits */
      *value   |= (uint64_t)value_32;
    }
  else
    {
      if (priv->tid == TIMER0)
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
}

/****************************************************************************
 * Name: esp32s2_tim_getperiod
 *
 * Description:
 *   This function is only available for Systimer. And it's intended to get
 *   the alarm when it's working on periodic mode.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - Pointer to retrieve the current configured alarm value.
 *
 ****************************************************************************/

static void esp32s2_tim_getperiod(struct esp32s2_tim_dev_s *dev,
                                  uint32_t *value)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  *value = 0;

  /* Read value */

  if (priv->tid == SYSTIMER_COMP0)
    {
      *value  = REG_GET_FIELD(SYSTIMER_TARGET0_CONF_REG,
                              SYSTIMER_TARGET0_PERIOD);
    }
}

/****************************************************************************
 * Name: esp32s2_tim_setalarmvalue
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

static void esp32s2_tim_setalarmvalue(struct esp32s2_tim_dev_s *dev,
                                      uint64_t value)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;
  uint64_t low_64  = value & UINT32_MAX;
  uint64_t high_64 = (value >> SHIFT_32);

  DEBUGASSERT(dev);

  /* Set an alarm value */

  if (priv->tid == SYSTIMER_COMP0)
    {
      putreg32((uint32_t)low_64, SYSTIMER_TARGET0_LO_REG);
      putreg32((uint32_t)high_64, SYSTIMER_TARGET0_HI_REG);
    }
  else
    {
      if (priv->tid == TIMER0)
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
}

/****************************************************************************
 * Name: esp32s2_tim_setperiod
 *
 * Description:
 *   This function is only available for Systimer. And it's intended to set
 *   the alarm when it's working on periodic mode.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - Period value.
 *
 ****************************************************************************/

static void esp32s2_tim_setperiod(struct esp32s2_tim_dev_s *dev,
                                  uint32_t value)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  /* Read value */

  if (priv->tid == SYSTIMER_COMP0)
    {
      REG_SET_FIELD(SYSTIMER_TARGET0_CONF_REG,
                    SYSTIMER_TARGET0_PERIOD, value);
    }
}

/****************************************************************************
 * Name: esp32s2_tim_setworkmode
 *
 * Description:
 *   This function is only available for Systimer. And it's intended to set
 *   the work mode: Time-Delay alarm mode or periodic alarm mode.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *   value         - Period value.
 *
 ****************************************************************************/

static void esp32s2_tim_setworkmode(struct esp32s2_tim_dev_s *dev,
                                    enum esp32s2_tim_work_mode_e mode)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  /* Read value */

  if (priv->tid == SYSTIMER_COMP0)
    {
      REG_SET_FIELD(SYSTIMER_TARGET0_CONF_REG,
                    SYSTIMER_TARGET0_PERIOD_MODE, mode);
    }
}

/****************************************************************************
 * Name: esp32s2_tim_setalarm
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

static void esp32s2_tim_setalarm(struct esp32s2_tim_dev_s *dev,
                                 bool enable)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->tid == SYSTIMER_COMP0)
    {
      if (enable)
        {
          modifyreg32(SYSTIMER_TARGET0_CONF_REG, 0,
                      SYSTIMER_TARGET0_WORK_EN);
        }
      else
        {
          modifyreg32(SYSTIMER_TARGET0_CONF_REG,
                      SYSTIMER_TARGET0_WORK_EN, 0);
        }
    }
  else
    {
      if (priv->tid == TIMER0)
        {
          if (enable)
            {
              modifyreg32(TIMG_T0CONFIG_REG(priv->gid), 0,
                          TIMG_T0_ALARM_EN_M);
            }
          else
            {
              modifyreg32(TIMG_T0CONFIG_REG(priv->gid),
                          TIMG_T0_ALARM_EN_M, 0);
            }
        }
      else
        {
          if (enable)
            {
              modifyreg32(TIMG_T1CONFIG_REG(priv->gid),
                          0, TIMG_T1_ALARM_EN_M);
            }
          else
            {
              modifyreg32(TIMG_T1CONFIG_REG(priv->gid),
                          TIMG_T1_ALARM_EN_M, 0);
            }
        }
    }
}

/****************************************************************************
 * Name: esp32s2_tim_setautoreload
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

static void esp32s2_tim_setautoreload(struct esp32s2_tim_dev_s *dev,
                                   bool enable)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->tid == TIMER0)
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
 * Name: esp32s2_tim_setisr
 *
 * Description:
 *   Allocate a CPU Interrupt, connect the peripheral source to this
 *   Interrupt, register the callback and enable the CPU Interruption.
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

static int esp32s2_tim_setisr(struct esp32s2_tim_dev_s *dev,
                              xcpt_t handler, void *arg)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;
  int ret = OK;

  DEBUGASSERT(dev);

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
          esp32s2_teardown_irq(priv->periph, priv->cpuint);
          irq_detach(priv->irq);

          priv->cpuint = -ENOMEM;
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

      if (priv->tid == SYSTIMER_COMP0)
        {
          priv->cpuint = esp32s2_setup_irq(priv->periph, priv->int_pri,
                                           ESP32S2_CPUINT_EDGE);
        }
      else
        {
          priv->cpuint = esp32s2_setup_irq(priv->periph, priv->int_pri,
                                           ESP32S2_CPUINT_LEVEL);
        }

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
          esp32s2_teardown_irq(priv->periph, priv->cpuint);
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
 * Name: esp32s2_tim_enableint
 *
 * Description:
 *   Enable Interrupt at the alarm if it is set.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32s2_tim_enableint(struct esp32s2_tim_dev_s *dev)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->tid == SYSTIMER_COMP0)
    {
      modifyreg32(SYSTIMER_INT_ENA_REG, 0, SYSTIMER_SYSTIMER_INT0_ENA_M);
    }
  else
    {
      if (priv->tid == TIMER0)
        {
          modifyreg32(TIMG_T0CONFIG_REG(priv->gid), 0,
                      TIMG_T0_LEVEL_INT_EN_M);
          modifyreg32(TIMG_INT_ENA_TIMERS_REG(priv->gid), 0,
                      TIMG_T0_INT_ENA_M);
        }
      else
        {
          modifyreg32(TIMG_T1CONFIG_REG(priv->gid), 0,
                      TIMG_T1_LEVEL_INT_EN_M);
          modifyreg32(TIMG_INT_ENA_TIMERS_REG(priv->gid), 0,
                      TIMG_T1_INT_ENA_M);
        }
    }
}

/****************************************************************************
 * Name: esp32s2_tim_disableint
 *
 * Description:
 *   Disable a Interrupt at the alarm if it is set.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32s2_tim_disableint(struct esp32s2_tim_dev_s *dev)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->tid == SYSTIMER_COMP0)
    {
      modifyreg32(SYSTIMER_INT_ENA_REG, SYSTIMER_SYSTIMER_INT0_ENA_M, 0);
    }
  else
    {
      if (priv->tid == TIMER0)
        {
          modifyreg32(TIMG_T0CONFIG_REG(priv->gid),
                      TIMG_T0_LEVEL_INT_EN_M, 0);
          modifyreg32(TIMG_INT_ENA_TIMERS_REG(priv->gid),
                      TIMG_T0_INT_ENA_M, 0);
        }
      else
        {
          modifyreg32(TIMG_T1CONFIG_REG(priv->gid),
                      TIMG_T1_LEVEL_INT_EN_M, 0);
          modifyreg32(TIMG_INT_ENA_TIMERS_REG(priv->gid),
                      TIMG_T1_INT_ENA_M, 0);
        }
    }
}

/****************************************************************************
 * Name: esp32s2_tim_ackint
 *
 * Description:
 *   Acknowledge an interrupt, that means, clear the interrupt.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

static void esp32s2_tim_ackint(struct esp32s2_tim_dev_s *dev)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;

  DEBUGASSERT(dev);

  if (priv->tid == SYSTIMER_COMP0)
    {
      modifyreg32(SYSTIMER_INT_CLR_REG, 0, SYSTIMER_SYSTIMER_INT0_CLR_M);
    }
  else
    {
      if (priv->tid == TIMER0)
        {
          modifyreg32(TIMG_INT_CLR_TIMERS_REG(priv->gid), 0,
                      TIMG_T0_INT_CLR_M);
        }
      else
        {
          modifyreg32(TIMG_INT_CLR_TIMERS_REG(priv->gid), 0,
                      TIMG_T1_INT_CLR_M);
        }
    }
}

/****************************************************************************
 * Name: esp32s2_tim_checkint
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

static int esp32s2_tim_checkint(struct esp32s2_tim_dev_s *dev)
{
  struct esp32s2_tim_priv_s *priv = (struct esp32s2_tim_priv_s *)dev;
  uint32_t reg_value;
  int ret;

  DEBUGASSERT(dev);

  if (priv->tid == SYSTIMER_COMP0)
    {
      reg_value = getreg32(SYSTIMER_INT_RAW_REG);
      ret = REG_MASK(reg_value, SYSTIMER_SYSTIMER_INT0_RAW);
    }
  else
    {
      if (priv->tid == TIMER0)
        {
          reg_value = getreg32(TIMG_INT_ST_TIMERS_REG(priv->gid));
          ret = REG_MASK(reg_value, TIMG_T0_INT_ST);
        }
      else
        {
          reg_value = getreg32(TIMG_INT_ST_TIMERS_REG(priv->gid));
          ret = REG_MASK(reg_value, TIMG_T1_INT_ST);
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_tim_init
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

struct esp32s2_tim_dev_s *esp32s2_tim_init(int timer)
{
  struct esp32s2_tim_priv_s *tim = NULL;

  /* First, take the data structure associated with the timer instance */

  switch (timer)
    {
#ifdef CONFIG_ESP32S2_TIMER0
      case TIMER0:
        {
          tim = &g_esp32s2_tim0_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32S2_TIMER1
      case TIMER1:
        {
          tim = &g_esp32s2_tim1_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32S2_TIMER2
      case TIMER2:
        {
          tim = &g_esp32s2_tim2_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32S2_TIMER3
      case TIMER3:
        {
          tim = &g_esp32s2_tim3_priv;
          break;
        }
#endif

#ifdef CONFIG_ESP32S2_RT_TIMER
      case SYSTIMER_COMP0:
        {
          tim = &g_esp32s2_tim4_priv;

          /* Enable Systimer peripheral clock and reset it */

          modifyreg32(SYSTEM_PERIP_CLK_EN0_REG, 0, SYSTEM_SYSTIMER_CLK_EN);
          modifyreg32(SYSTEM_PERIP_RST_EN0_REG, 0, SYSTEM_SYSTIMER_RST);
          modifyreg32(SYSTEM_PERIP_RST_EN0_REG, SYSTEM_SYSTIMER_RST, 0);
          modifyreg32(SYSTIMER_CONF_REG, 0, SYSTIMER_CLK_EN
                      | SYSTIMER_CLK_FO);
          break;
        }
#endif
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

  return (struct esp32s2_tim_dev_s *)tim;
}

/****************************************************************************
 * Name: esp32s2_tim_deinit
 *
 * Description:
 *   Deinit TIMER device.
 *
 * Parameters:
 *   dev           - Pointer to the timer driver struct.
 *
 ****************************************************************************/

void esp32s2_tim_deinit(struct esp32s2_tim_dev_s *dev)
{
  struct esp32s2_tim_priv_s *tim = NULL;

  DEBUGASSERT(dev);

  tim = (struct esp32s2_tim_priv_s *)dev;
  tim->inuse = false;
}
