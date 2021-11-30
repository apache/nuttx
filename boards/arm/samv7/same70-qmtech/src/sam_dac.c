/****************************************************************************
 * boards/arm/samv7/same70-qmtech/src/sam_dac.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include "sam_dac.h"
#include "same70-qmtech.h"

#if defined(CONFIG_SAMV7_DAC0) || defined(CONFIG_SAMV7_DAC1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_dacdev_initialize
 *
 * Description:
 *   Called to configure DAC peripheral module and register DAC device driver
 ****************************************************************************/

int sam_dacdev_initialize(void)
{
  static bool initialized = false;
  struct dac_dev_s *dac;
  int ret;

  /* Check if we have already initialized */

  if (!initialized)
    {
#ifdef CONFIG_SAMV7_DAC0
      /* Get an instance of the DAC0 interface */

      dac = sam_dac_initialize(0);
      if (dac == NULL)
        {
          aerr("ERROR:  Failed to get DAC0 interface\n");
          return -ENODEV;
        }

      /* Register the DAC driver at "/dev/dac0" */

      ret = dac_register("/dev/dac0", dac);
      if (ret < 0)
        {
          aerr("ERROR: dac_register failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_SAMV7_DAC1
      /* Get an instance of the DAC1 interface */

      dac = sam_dac_initialize(1);
      if (dac == NULL)
        {
          aerr("ERROR:  Failed to get DAC1 interface\n");
          return -ENODEV;
        }

      /* Register the DAC driver at "/dev/dac1" */

      ret = dac_register("/dev/dac1", dac);
      if (ret < 0)
        {
          aerr("ERROR: dac_register failed: %d\n", ret);
          return ret;
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_SAMV7_DAC0 || CONFIG_SAMV7_DAC1 */
