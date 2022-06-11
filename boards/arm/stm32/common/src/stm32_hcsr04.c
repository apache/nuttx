/****************************************************************************
 * boards/arm/stm32/common/src/stm32_hcsr04.c
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

#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/sensors/hc_sr04.h>

#include "stm32.h"
#include "stm32_freerun.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_FREERUN
#error "This implementation requires support for free running timers"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_hcsr04config_s
{
  /* Configuration structure as seen by the HC-SR04 driver */

  struct hcsr04_config_s config;

  /* Additional private definitions only known to this driver */

  void *arg;    /* Argument to pass to the interrupt handler */
  xcpt_t isr;   /* ISR Handler */
  bool rising;  /* Rising edge enabled */
  bool falling; /* Falling edge enabled */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  hcsr04_irq_attach(struct hcsr04_config_s *state, xcpt_t isr,
                              void *arg);
static void hcsr04_irq_enable(const struct hcsr04_config_s *state,
                              bool enable);
static void hcsr04_irq_clear(const struct hcsr04_config_s *state);
static void hcsr04_irq_setmode(struct hcsr04_config_s *state,
                               bool rise_mode);
static void hcsr04_set_trigger(const struct hcsr04_config_s *state,
                               bool on);
static int64_t hcsr04_get_clock(const struct hcsr04_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the HC-SR04
 * driver.  This structure provides information about the configuration
 * of the HC-SR04 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or X plate resistance values.
 */

static struct stm32_hcsr04config_s g_hcsr04config =
{
  .config =
  {
    .irq_attach  = hcsr04_irq_attach,
    .irq_enable  = hcsr04_irq_enable,
    .irq_clear   = hcsr04_irq_clear,
    .irq_setmode = hcsr04_irq_setmode,
    .set_trigger = hcsr04_set_trigger,
    .get_clock   = hcsr04_get_clock,
  },
};

struct stm32_freerun_s g_freerun;
struct timespec ts;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Attach the HC-SR04 interrupt handler to the GPIO interrupt */

static int hcsr04_irq_attach(struct hcsr04_config_s *state, xcpt_t isr,
                             void *arg)
{
  struct stm32_hcsr04config_s *priv =
             (struct stm32_hcsr04config_s *)state;
  irqstate_t flags;

  sinfo("hcsr04_irq_attach\n");

  flags = enter_critical_section();

  priv->rising  = true;
  priv->falling = false;
  priv->isr     = isr;
  priv->arg     = arg;

  stm32_gpiosetevent(BOARD_HCSR04_GPIO_INT, priv->rising, priv->falling,
                     true, isr, arg);

  leave_critical_section(flags);

  return OK;
}

/* Setup the interruption mode: Rising or Falling */

static void hcsr04_irq_setmode(struct hcsr04_config_s *state,
                               bool rise_mode)
{
  struct stm32_hcsr04config_s *priv =
             (struct stm32_hcsr04config_s *)state;

  if (rise_mode)
    {
      priv->rising = true;
      priv->falling = false;
    }
  else
    {
      priv->rising = false;
      priv->falling = true;
    }
}

/* Enable or disable the GPIO interrupt */

static void hcsr04_irq_enable(const struct hcsr04_config_s *state,
                              bool enable)
{
  struct stm32_hcsr04config_s *priv =
             (struct stm32_hcsr04config_s *)state;

  iinfo("%d\n", enable);

  stm32_gpiosetevent(BOARD_HCSR04_GPIO_INT, priv->rising, priv->falling,
                     true, enable ? priv->isr : NULL, priv->arg);
}

/* Acknowledge/clear any pending GPIO interrupt */

static void hcsr04_irq_clear(const struct hcsr04_config_s *state)
{
  /* FIXME: Nothing to do ? */
}

/* Set the Trigger pin state */

static void hcsr04_set_trigger(const struct hcsr04_config_s *state,
                               bool on)
{
  stm32_gpiowrite(BOARD_HCSR04_GPIO_TRIG, on);
}

/* Return the current Free Running clock tick */

static int64_t hcsr04_get_clock(const struct hcsr04_config_s *state)
{
  /* Get the time from free running timer */

  stm32_freerun_counter(&g_freerun, &ts);

  /* Return time in microseconds */

  return ((ts.tv_sec * 1000000) + (ts.tv_nsec / 1000));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hcsr04_initialize
 *
 * Description:
 *   This function is called by application-specific, setup logic to
 *   configure the HC-SR04 sensor.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/distN
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_hcsr04_initialize(int devno)
{
  int ret;
  char devpath[12];

  /* Configure the PIO interrupt */

  stm32_configgpio(BOARD_HCSR04_GPIO_INT);

  /* Configure the Trigger pin */

  stm32_configgpio(BOARD_HCSR04_GPIO_TRIG);

  /* Initialize the free-running timer with 1uS resolution */

  ret = stm32_freerun_initialize(&g_freerun, BOARD_HCSR04_FRTIMER, 1);
  if (ret < 0)
    {
      serr("Failed to initialize the free running timer! Err = %d\n", ret);
      return -ENODEV;
    }

  snprintf(devpath, 12, "/dev/dist%d", devno);
  return hcsr04_register(devpath, &g_hcsr04config.config);
}
