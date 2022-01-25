/****************************************************************************
 * boards/arm/samv7/same70-qmtech/src/sam_afec.c
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
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/adc.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"

#include "sam_afec.h"
#include "same70-qmtech.h"

#ifdef CONFIG_SAMV7_AFEC

#define ADC0_NCHANNELS 3
#define ADC1_NCHANNELS 4

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SAMV7_AFEC0
static const uint8_t g_chanlist0[ADC0_NCHANNELS] =
  {
    0, 6, 8
  };
#endif

#ifdef CONFIG_SAMV7_AFEC1
static const uint8_t g_chanlist1[ADC1_NCHANNELS] =
  {
    1, 3, 5, 6
  };
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_afecinitialize
 *
 * Description:
 *   Initialize and register the ADC driver.
 *
 ****************************************************************************/

int sam_afec_setup(void)
{
  struct adc_dev_s *adc;
  int ret;

  /* Call sam_adc_initialize() to get an instance of the ADC interface */

  #ifdef CONFIG_SAMV7_AFEC0
      adc = sam_afec_initialize(0, g_chanlist0, ADC0_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC1 interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc0" */

      ret = adc_register("/dev/adc0", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register adc0 failed: %d\n", ret);
          return ret;
        }

  #endif
  #ifdef CONFIG_SAMV7_AFEC1
      adc = sam_afec_initialize(1, g_chanlist1, ADC1_NCHANNELS);
      if (adc == NULL)
        {
          aerr("ERROR: Failed to get ADC2 interface\n");
          return -ENODEV;
        }

      /* Register the ADC driver at "/dev/adc1" */

      ret = adc_register("/dev/adc1", adc);
      if (ret < 0)
        {
          aerr("ERROR: adc_register adc1 failed: %d\n", ret);
          return ret;
        }
  #endif

  return OK;
}

#endif /* CONFIG_SAMV7_AFEC */
