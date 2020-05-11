/****************************************************************************
 * boards/arm/stm32/common/src/stm32_hcsr04.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

  FAR void *arg;  /* Argument to pass to the interrupt handler */
  FAR xcpt_t isr; /* ISR Handler */
  bool rising;    /* Rising edge enabled */
  bool falling;   /* Falling edge enabled */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  hcsr04_irq_attach(FAR struct hcsr04_config_s *state, xcpt_t isr,
                              FAR void *arg);
static void hcsr04_irq_enable(FAR const struct hcsr04_config_s *state,
                              bool enable);
static void hcsr04_irq_clear(FAR const struct hcsr04_config_s *state);
static void hcsr04_irq_setmode(FAR struct hcsr04_config_s *state,
                               bool rise_mode);
static void hcsr04_set_trigger(FAR const struct hcsr04_config_s *state,
                               bool on);
static int64_t hcsr04_get_clock(FAR const struct hcsr04_config_s *state);

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

static int hcsr04_irq_attach(FAR struct hcsr04_config_s *state, xcpt_t isr,
                             FAR void *arg)
{
  FAR struct stm32_hcsr04config_s *priv =
             (FAR struct stm32_hcsr04config_s *)state;
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

static void hcsr04_irq_setmode(FAR struct hcsr04_config_s *state,
                               bool rise_mode)
{
  FAR struct stm32_hcsr04config_s *priv =
             (FAR struct stm32_hcsr04config_s *)state;

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

static void hcsr04_irq_enable(FAR const struct hcsr04_config_s *state,
                                 bool enable)
{
  FAR struct stm32_hcsr04config_s *priv =
             (FAR struct stm32_hcsr04config_s *)state;

  iinfo("%d\n", enable);

  stm32_gpiosetevent(BOARD_HCSR04_GPIO_INT, priv->rising, priv->falling,
                     true, enable ? priv->isr : NULL, priv->arg);
}

/* Acknowledge/clear any pending GPIO interrupt */

static void hcsr04_irq_clear(FAR const struct hcsr04_config_s *state)
{
  /* FIXME: Nothing to do ? */
}

/* Set the Trigger pin state */

static void hcsr04_set_trigger(FAR const struct hcsr04_config_s *state,
                               bool on)
{
  stm32_gpiowrite(BOARD_HCSR04_GPIO_TRIG, on);
}

/* Return the current Free Running clock tick */

static int64_t hcsr04_get_clock(FAR const struct hcsr04_config_s *state)
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
