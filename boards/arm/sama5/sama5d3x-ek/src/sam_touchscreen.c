/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/src/sam_touchscreen.c
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

#include <sys/types.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "sam_adc.h"
#include "sam_tsd.h"
#include "sama5d3x-ek.h"

#include <nuttx/board.h>

#ifdef CONFIG_SAMA5_TSD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_INPUT
#  error "Touchscreen support requires CONFIG_INPUT"
#endif

#ifndef CONFIG_SAMA5D3XEK_TSD_DEVMINOR
#  define CONFIG_SAMA5D3XEK_TSD_DEVMINOR 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_tsc_setup
 *
 * Description:
 *   This function is called by board-bringup logic to configure the
 *   touchscreen device.  This function will register the driver as
 *   /dev/inputN where N is the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int sam_tsc_setup(int minor)
{
  struct sam_adc_s *adc;
  int ret;

  iinfo("minor:%d\n", minor);
  DEBUGASSERT(minor == 0);

  /* Initialize the ADC driver */

  adc = (struct sam_adc_s *)sam_adc_initialize();
  if (!adc)
    {
      ierr("ERROR: Failed to initialize the ADC driver\n");
      return -ENODEV;
    }

  /* Initialize and register the SPI touchscreen device */

  ret = sam_tsd_register(adc, CONFIG_SAMA5D3XEK_TSD_DEVMINOR);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register touchscreen device /dev/input%d: %d\n",
           CONFIG_SAMA5D3XEK_TSD_DEVMINOR, ret);
      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_INPUT_ADS7843E */
