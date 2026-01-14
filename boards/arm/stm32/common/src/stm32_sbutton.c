/****************************************************************************
 * boards/arm/stm32/common/src/stm32_sbutton.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/input/sbutton.h>

#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_sbtnconfig_s
{
  /* Configuration structure as seen by the HC-SR04 driver */

  struct sbutton_config_s config;

  /* Additional private definitions only known to this driver */

  void *arg;    /* Argument to pass to the interrupt handler */
  xcpt_t isr;   /* ISR Handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  sbtn_irq_attach(const struct sbutton_config_s *config,
                            xcpt_t isr, void *arg);
static void sbtn_irq_enable(const struct sbutton_config_s *config,
                            bool enable);
static void sbtn_irq_clear(const struct sbutton_config_s *config);

static bool sbtn_pin_status(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* A reference to a structure of this type must be passed to the SButton
 * driver.  This structure provides information about the configuration
 * of the SButton and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active.
 */

static struct stm32_sbtnconfig_s g_sbtnconfig =
{
  .config =
  {
    .attach     = sbtn_irq_attach,
    .enable     = sbtn_irq_enable,
    .clear      = sbtn_irq_clear,
    .status     = sbtn_pin_status,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Attach the SBUTTON interrupt handler to the GPIO interrupt */

static int sbtn_irq_attach(const struct sbutton_config_s *state, xcpt_t isr,
                           void *arg)
{
  struct stm32_sbtnconfig_s *priv = (struct stm32_sbtnconfig_s *)state;
  irqstate_t flags;

  sinfo("sbtn_irq_attach\n");

  flags = enter_critical_section();

  priv->isr     = isr;
  priv->arg     = arg;

  stm32_gpiosetevent(BOARD_SBUTTON_GPIO_INT, true, true,
                     true, isr, arg);

  leave_critical_section(flags);

  return OK;
}

/* Enable or disable the GPIO interrupt */

static void sbtn_irq_enable(const struct sbutton_config_s *state,
                            bool enable)
{
  struct stm32_sbtnconfig_s *priv = (struct stm32_sbtnconfig_s *)state;

  iinfo("%d\n", enable);

  stm32_gpiosetevent(BOARD_SBUTTON_GPIO_INT, true, true,
                     true, enable ? priv->isr : NULL, priv->arg);
}

/* Acknowledge/clear any pending GPIO interrupt */

static void sbtn_irq_clear(const struct sbutton_config_s *state)
{
  /* FIXME: Nothing to do ? */
}

/* Read and return the status of button (PRESSED = true; RELEASED = false */

static bool sbtn_pin_status(void)
{
  /* STM32F4Discovery button B1 is high level when press
   * then just returned the status of the pin directly.
   */

  return stm32_gpioread(BOARD_SBUTTON_GPIO_INT);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_sbutton_initialize
 *
 * Description:
 *   This function is called by application-specific, setup logic to
 *   configure the Single Button Dual Action.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/kbdN
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_sbutton_initialize(int devno)
{
  /* Configure the Single Button Dual Action interrupt pin */

  stm32_configgpio(BOARD_SBUTTON_GPIO_INT);

  /* Register the Single Button with overlay config pointer */

  return sbutton_register(&g_sbtnconfig.config, devno);
}
