/****************************************************************************
 * boards/arm/stm32/common/src/stm32_dhtxx.c
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

#include <unistd.h>
#include <errno.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/board.h>
#include <nuttx/sensors/dhtxx.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_freerun.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void dhtxx_config_data_pin(struct dhtxx_config_s *state,
                                  bool mode);
static void dhtxx_set_data_pin(struct dhtxx_config_s *state, bool value);
static bool dhtxx_read_data_pin(struct dhtxx_config_s *state);
static int64_t dhtxx_get_clock(struct dhtxx_config_s *state);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dhtxx_config_s g_dhtxx_config =
{
  .config_data_pin  = dhtxx_config_data_pin,
  .set_data_pin     = dhtxx_set_data_pin,
  .read_data_pin    = dhtxx_read_data_pin,
  .get_clock        = dhtxx_get_clock,
  .type             = DHTXX_DHT11
};

struct stm32_freerun_s g_freerun;
struct timespec ts;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void dhtxx_config_data_pin(struct dhtxx_config_s *state,
                                  bool mode)
{
  if (mode)
    {
      stm32_configgpio(BOARD_DHTXX_GPIO_INPUT);
    }
  else
    {
      stm32_configgpio(BOARD_DHTXX_GPIO_OUTPUT);
    }
}

static void dhtxx_set_data_pin(struct dhtxx_config_s *state, bool value)
{
  stm32_gpiowrite(BOARD_DHTXX_GPIO_OUTPUT, value);
}

static bool dhtxx_read_data_pin(struct dhtxx_config_s *state)
{
  return stm32_gpioread(BOARD_DHTXX_GPIO_INPUT);
}

static int64_t dhtxx_get_clock(struct dhtxx_config_s *state)
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
 * Name: board_dhtxx_initialize
 *
 * Description:
 *   This function is called by application-specific, setup logic to
 *   configure the DHTxx sensor.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/humN.
 *
 * Returned Value:
 *   Zero (OK) returned on success;
 *   a negated errno value is returned on failure.
 *
 ****************************************************************************/

int board_dhtxx_initialize(int devno)
{
  int ret;
  char devpath[12];

  stm32_configgpio(BOARD_DHTXX_GPIO_OUTPUT);

  stm32_gpiowrite(BOARD_DHTXX_GPIO_OUTPUT, false);

  /* Initialize the free-running timer with 1uS resolution */

  ret = stm32_freerun_initialize(&g_freerun, BOARD_DHTXX_FRTIMER, 1);
  if (ret < 0)
    {
      snerr("Failed to initialize the free running timer! Err = %d\n", ret);
      return -ENODEV;
    }

  snprintf(devpath, 12, "/dev/hum%d", devno);
  ret = dhtxx_register(devpath, &g_dhtxx_config);
  if (ret < 0)
    {
      snerr("ERROR: Error registering Dhtxx\n");
    }

  return ret;
}
