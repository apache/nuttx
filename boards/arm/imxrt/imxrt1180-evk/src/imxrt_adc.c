/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_adc.c
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
#include <stdbool.h>

#include <nuttx/analog/adc.h>
#include <nuttx/board.h>

#include "imxrt_adc.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_IMXRT_ADC

int imxrt_adc_initialize(void)
{
  static bool initialized;
  struct adc_dev_s *adc;
  int ret;

  static const uint8_t chanlist[] =
    {
      0
    };

  if (initialized)
    {
      return OK;
    }

#ifdef CONFIG_IMXRT_ADC1
  adc = imxrt_adcinitialize(1, chanlist, 1);
  if (adc == NULL)
    {
      return -ENODEV;
    }

  ret = adc_register("/dev/adc1", adc);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_IMXRT_ADC2
  adc = imxrt_adcinitialize(2, chanlist, 1);
  if (adc == NULL)
    {
      return -ENODEV;
    }

  ret = adc_register("/dev/adc2", adc);
  if (ret < 0)
    {
      return ret;
    }
#endif

  initialized = true;
  return OK;
}

#endif
