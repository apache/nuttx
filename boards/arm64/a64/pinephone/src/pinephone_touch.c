/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_touch.c
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

/* Reference:
 * "NuttX RTOS for PinePhone: Touch Panel"
 * https://lupyuen.github.io/articles/touch2
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/gt9xx.h>
#include "arm64_arch.h"
#include "arm64_gic.h"
#include "a64_pio.h"
#include "pinephone_touch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Goodix GT917S Touch Panel Interrupt at PH4 */

#define CTP_INT (PIO_EINT | PIO_PORT_PIOH | PIO_PIN4)

/* I2C Address for Touch Panel */

#define CTP_I2C_ADDR 0x5d

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int pinephone_gt9xx_irq_attach(const struct gt9xx_board_s *state,
                                      xcpt_t isr, void *arg);
static void pinephone_gt9xx_irq_enable(const struct gt9xx_board_s *state,
                                       bool enable);
static int pinephone_gt9xx_set_power(const struct gt9xx_board_s *state,
                                     bool on);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Callback for Board-Specific Operations */

static const struct gt9xx_board_s g_pinephone_gt9xx =
{
  .irq_attach = pinephone_gt9xx_irq_attach,
  .irq_enable = pinephone_gt9xx_irq_enable,
  .set_power  = pinephone_gt9xx_set_power
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_gt9xx_irq_attach
 *
 * Description:
 *   Attach the Interrupt Handler for Touch Panel.
 *
 * Input Parameters:
 *   state - Callback for Board-Specific Operations
 *   isr   - Interrupt Handler
 *   arg   - Argument for Interrupt Handler
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pinephone_gt9xx_irq_attach(const struct gt9xx_board_s *state,
                                      xcpt_t isr, void *arg)
{
  int ret;

  iinfo("\n");
  DEBUGASSERT(state != NULL && isr != NULL && arg != NULL);

  /* Attach the Interrupt Handler to Port PH */

  ret = irq_attach(A64_IRQ_PH_EINT, isr, arg);
  if (ret < 0)
    {
      ierr("Attach Interrupt Handler failed: %d\n", ret);
      return ret;
    }

  /* Set Interrupt Priority in Generic Interrupt Controller v2 */

  arm64_gic_irq_set_priority(A64_IRQ_PH_EINT, 0, IRQ_TYPE_EDGE);

  /* Enable Interrupts for Port PH */

  up_enable_irq(A64_IRQ_PH_EINT);

  return OK;
}

/****************************************************************************
 * Name: pinephone_gt9xx_irq_enable
 *
 * Description:
 *   Enable or disable Interrupts for the Touch Panel.
 *
 * Input Parameters:
 *   state  - Callback for Board-Specific Operations
 *   enable - True to enable interrupts; False to disable interrupts
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static void pinephone_gt9xx_irq_enable(const struct gt9xx_board_s *state,
                                       bool enable)
{
  int ret;

  iinfo("enable=%d\n", enable);
  DEBUGASSERT(state != NULL);

  if (enable)
    {
      /* Configure the Touch Panel Interrupt */

      ret = a64_pio_config(CTP_INT);
      if (ret < 0)
        {
          ierr("Configure Touch Panel Interrupt failed: %d\n", ret);
          return;
        }

      /* Enable the Touch Panel Interrupt */

      ret = a64_pio_irqenable(CTP_INT);
      if (ret < 0)
        {
          ierr("Enable Touch Panel Interrupt failed: %d\n", ret);
          return;
        }
    }
  else
    {
      /* Disable the Touch Panel Interrupt */

      ret = a64_pio_irqdisable(CTP_INT);
      if (ret < 0)
        {
          ierr("Disable Touch Panel Interrupt failed: %d\n", ret);
          return;
        }
    }
}

/****************************************************************************
 * Name: pinephone_gt9xx_set_power
 *
 * Description:
 *   Power on or off the Touch Panel.
 *
 * Input Parameters:
 *   state - Callback for Board-Specific Operations
 *   on    - True to power on; False to power off
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

static int pinephone_gt9xx_set_power(const struct gt9xx_board_s *state,
                                     bool on)
{
  /* Assume that Touch Panel is already powered on by pinephone_pmic_init() */

  iinfo("on=%d\n", on);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pinephone_touch_panel_register
 *
 * Description:
 *   Register the driver for Goodix GT9XX Touch Panel.  Attach the
 *   Interrupt Handler for the Touch Panel and disable Touch Interrupts.
 *
 * Input Parameters:
 *   devpath - Device Path (e.g. "/dev/input0")
 *   i2c     - I2C Bus
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int pinephone_touch_panel_register(const char *devpath,
                                   struct i2c_master_s *i2c)
{
  int ret;

  iinfo("devpath=%s\n", devpath);
  DEBUGASSERT(devpath != NULL && i2c != NULL);

  ret = gt9xx_register(devpath, i2c, CTP_I2C_ADDR, &g_pinephone_gt9xx);
  if (ret < 0)
    {
      ierr("Register Touch Input GT9xx failed: %d\n", ret);
      return ret;
    }

  return OK;
}
