/****************************************************************************
 * boards/arm/stm32/stm32f411-minimum/src/stm32_hx711.c
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
#include <nuttx/analog/hx711.h>
#include <nuttx/compiler.h>
#include <arch/board/board.h>
#include <arch/stm32/chip.h>
#include <debug.h>

#include "stm32_gpio.h"
#include "stm32f411-minimum.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_hx711_clock_set(unsigned char minor, int value);
static int stm32_hx711_data_read(unsigned char minor);
static int stm32_hx711_data_irq(unsigned char minor,
                                xcpt_t handler, void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct hx711_lower_s g_lower =
{
  .data_read = stm32_hx711_data_read,
  .clock_set = stm32_hx711_clock_set,
  .data_irq  = stm32_hx711_data_irq,
  .cleanup = NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_hx711_clock_set(unsigned char minor, int value)
{
  UNUSED(minor);

  stm32_gpiowrite(HX711_CLK_PIN, value);
  return OK;
}

static int stm32_hx711_data_read(unsigned char minor)
{
  UNUSED(minor);

  return stm32_gpioread(HX711_DATA_PIN);
}

static int stm32_hx711_data_irq(unsigned char minor,
                                xcpt_t handler, void *arg)
{
  UNUSED(minor);

  return stm32_gpiosetevent(HX711_DATA_PIN, false, true, true, handler, arg);
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_hx711_initialize(void)
{
  int ret;

  stm32_configgpio(HX711_DATA_PIN);
  stm32_configgpio(HX711_CLK_PIN);

  ret = hx711_register(0, &g_lower);
  if (ret != 0)
    {
      aerr("ERROR: Failed to register hx711 device: %d\n", ret);
      return -1;
    }

  return OK;
}
