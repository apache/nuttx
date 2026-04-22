/****************************************************************************
 * boards/arm/stm32/common/src/stm32_mpr121.c
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

#include <errno.h>
#include <nuttx/debug.h>
#include <stdio.h>

#include <nuttx/input/mpr121.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Device I2C Address of MPR121 Capacitive Keypad */

#define MPR121_I2C_ADDR      0x5a

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct stm32_mpr121config_s
{
  /* Configuration structure as seen by the MPR121 driver */

  struct mpr121_config_s config;

  /* Additional private definitions only known to this driver */

  void *arg;  /* Argument to pass to the interrupt handler */
  xcpt_t isr; /* ISR Handler */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  mpr121_irq_attach(const struct mpr121_config_s *state,
                              xcpt_t isr, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Keymap for 4x3 MPR121 Capacitive Keypad
 * Keys named 11 and 10 were replaced with 'B' a 'A'
 */

static const uint32_t g_mpr121_keymap[] =
{
  '0', '1', '2', '3',
  '4', '5', '6', '7',
  '8', '9', 'A', 'B',
};

/* A reference to a structure of this type must be passed to the MPR121
 * driver.  This structure provides information about the configuration
 * of the MPR121 and provides some board-specific hooks.
 *
 * Memory for this structure is provided by the caller.  It is not copied
 * by the driver and is presumed to persist while the driver is active. The
 * memory must be writable because, under certain circumstances, the driver
 * may modify frequency or other values.
 */

static struct stm32_mpr121config_s g_mpr121config =
{
  .config =
    {
      .irq_attach  = mpr121_irq_attach,
      .keymap      = g_mpr121_keymap,
    },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Attach the MPR121 interrupt handler to the GPIO interrupt */

static int mpr121_irq_attach(const struct mpr121_config_s *state,
                             xcpt_t isr, void *arg)
{
  irqstate_t flags;

  iinfo("mpr121_irq_attach\n");

  flags = enter_critical_section();

  /* Setup interrupt for Falling Edge */

  stm32_gpiosetevent(BOARD_MPR121_GPIO_INT, false, true, true, isr, arg);

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_mpr121_initialize
 *
 * Description:
 *   Initialize and register the MPR121 gesture sensor.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/gestN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_mpr121_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  char devpath[14];
  int ret;

  iinfo("Initializing MPR121!\n");

  /* Configure the GPIO interrupt */

  stm32_configgpio(BOARD_MPR121_GPIO_INT);

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* Save this i2c in the config */

  g_mpr121config.config.i2c_dev  = i2c;
  g_mpr121config.config.i2c_addr = MPR121_I2C_ADDR;

  /* Then register the capacitive keypad */

  snprintf(devpath, sizeof(devpath), "/dev/keypad%d", devno);
  ret = mpr121_register(&g_mpr121config.config, devpath);
  if (ret < 0)
    {
      ierr("ERROR: Failed registering APDS-9960!\n");
    }

  return ret;
}
