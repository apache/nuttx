/****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_adc.c
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

#include <debug.h>
#include <errno.h>

#include "chip.h"
#include "stm32_adc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_setup
 *
 * Description:
 *   Initialize ADC and register the ADC driver.
 *
 ****************************************************************************/

int stm32_adc_setup(void)
{
  static bool initialized = false;
  uint8_t channel[1] =
    {
      10
    };

  struct adc_dev_s *adc;
  int rv;

  if (initialized)
    {
      return OK;
    }

  ainfo("INFO: Initializing ADC12_IN10\n");
  stm32_configgpio(GPIO_ADC12_IN10);
  if ((adc = stm32_adcinitialize(1, channel, 1)) == NULL)
    {
      aerr("ERROR: Failed to get adc interface\n");
      return -ENODEV;
    }

  if ((rv = adc_register("/dev/adc0", adc)) < 0)
    {
      aerr("ERROR: adc_register failed: %d\n", rv);
      return rv;
    }

  initialized = true;
  ainfo("INFO: ADC12_IN10 initialized successfully\n");
  return OK;
}
