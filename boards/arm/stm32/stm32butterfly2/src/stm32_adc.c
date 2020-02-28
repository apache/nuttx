/*****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_adc.c
 *
 *   Copyright (C) 2016 Michał Łyszczek. All rights reserved.
 *   Author: Michał Łyszczek <michal.lyszczek@gmail.com>
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
 ****************************************************************************/

/*****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>
#include <errno.h>

#include "chip.h"
#include "stm32_adc.h"

/*****************************************************************************
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
  uint8_t channel[1] = {10};
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
