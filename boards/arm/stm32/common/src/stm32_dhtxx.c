/****************************************************************************
 * boards/arm/stm32/olimex-stm32-p407/src/stm32_dhtxx.c
 *
 *   Copyright (C) 2014 Abdelatif GUETTOUCHE. All rights reserved.
 *   Author: Abdelatif GUETTOUCHE <abdelatif.guettouche@gmail.com>
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

static void dhtxx_config_data_pin(FAR struct dhtxx_config_s *state,
                                  bool mode);
static void dhtxx_set_data_pin(FAR struct dhtxx_config_s *state, bool value);
static bool dhtxx_read_data_pin(FAR struct dhtxx_config_s *state);
static int64_t dhtxx_get_clock(FAR struct dhtxx_config_s *state);

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

static void dhtxx_config_data_pin(FAR struct dhtxx_config_s *state,
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

static void dhtxx_set_data_pin(FAR struct dhtxx_config_s *state, bool value)
{
  stm32_gpiowrite(BOARD_DHTXX_GPIO_OUTPUT, value);
}

static bool dhtxx_read_data_pin(FAR struct dhtxx_config_s *state)
{
  return stm32_gpioread(BOARD_DHTXX_GPIO_INPUT);
}

static int64_t dhtxx_get_clock(FAR struct dhtxx_config_s *state)
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
