/****************************************************************************
 * boards/arm/stm32/common/src/stm32_drv8825.c
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

#include <nuttx/motor/drv8825.h>
#include <nuttx/motor/stepper.h>
#include <nuttx/config.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void drv8825_initialize(void);
static void drv8825_step(int level);
static void drv8825_direction(int level);
static void drv8825_microstepping(int ms1, int ms2, int ms3);
static void drv8825_enable(int level);
static void drv8825_idle(int level);
static int drv8825_fault(void);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct drv8825_ops_s g_drv8825_ops =
{
  drv8825_initialize,    /* initialize */
  drv8825_step,          /* step */
  drv8825_direction,     /* direction */
  drv8825_microstepping, /* microstepping */
  drv8825_enable,        /* enable */
  drv8825_idle,          /* idle */
  drv8825_fault          /* fault */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: drv8825_initialize
 *
 * Description:
 *  Initialize and configure the GPIOs.
 *
 ****************************************************************************/

static void drv8825_initialize(void)
{
  stm32_configgpio(GPIO_DIR);
  stm32_configgpio(GPIO_STEP);
  stm32_configgpio(GPIO_SLEEP);
  stm32_configgpio(GPIO_M1);
  stm32_configgpio(GPIO_M2);
  stm32_configgpio(GPIO_M3);
  stm32_configgpio(GPIO_RESET);
}

/****************************************************************************
 * Name: drv8825_step
 *
 * Description:
 *  Control the step output.
 *
 ****************************************************************************/

static void drv8825_step(int level)
{
  stm32_gpiowrite(GPIO_STEP, (bool)level);
}

/****************************************************************************
 * Name: drv8825_direction
 *
 * Description:
 *  Control the direction (clockwise or counterclockwise).
 *
 ****************************************************************************/

static void drv8825_direction(int level)
{
  stm32_gpiowrite(GPIO_DIR, (bool)level);
}

/****************************************************************************
 * Name: drv8825_microstepping
 *
 * Description:
 *  Configure the step resolution.
 *
 ****************************************************************************/

static void drv8825_microstepping(int ms1, int ms2, int ms3)
{
  stm32_gpiowrite(GPIO_M1, (bool)ms1);
  stm32_gpiowrite(GPIO_M2, (bool)ms2);
  stm32_gpiowrite(GPIO_M3, (bool)ms3);
}

/****************************************************************************
 * Name: drv8825_enable
 *
 * Description:
 *  Enable control.
 *
 ****************************************************************************/

static void drv8825_enable(int level)
{
  stm32_gpiowrite(GPIO_RESET, (bool)level);
}

/****************************************************************************
 * Name: drv8825_idle
 *
 * Description:
 *  Idle control.
 *
 ****************************************************************************/

static void drv8825_idle(int level)
{
  stm32_gpiowrite(GPIO_SLEEP, !level);
}

/****************************************************************************
 * Name: drv8825_fault
 *
 * Description:
 *  Fault fetch.
 *
 ****************************************************************************/

static int drv8825_fault(void)
{
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_drv8825_initialize
 *
 * Description:
 *  Initialize drv8825 and register the stepper motor driver.
 *
 ****************************************************************************/

int board_drv8825_initialize(int devno)
{
  char devname[15];

  drv8825_initialize();

  snprintf(devname, sizeof(devname), "/dev/stepper%d", devno);

  return drv8825_register(devname, &g_drv8825_ops);
}
