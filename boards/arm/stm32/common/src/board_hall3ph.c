/****************************************************************************
 * boards/arm/stm32/common/src/board_hall3ph.c
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
#include <stdio.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sensors/hall3ph.h>
#include <arch/board/board.h>

#include "stm32_hall3ph.h"
#include "board_hall3ph.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_hall3ph_initialize
 *
 * Description:
 *   Initialize the 3-phase Hall effect sensor driver
 *
 ****************************************************************************/

int board_hall3ph_initialize(int devno, int pha, int phb, int phc)
{
  struct stm32_hall3ph_cfg_s cfg;
  char                       devpath[12];
  int                        ret = OK;

  /* Get configuration */

  cfg.gpio_pha = pha;
  cfg.gpio_phb = phb;
  cfg.gpio_phc = phc;
  cfg.samples  = CONFIG_BOARD_STM32_HALL3PHASE_SAMPLES;

  /* Initialize a Hall effect sensor interface. */

  snprintf(devpath, 12, "/dev/hall%d", devno);

  ret = stm32_hall3ph_initialize(devpath, &cfg);
  if (ret < 0)
    {
      snerr("ERROR: stm32_hall3ph_initialize failed: %d\n", ret);
    }

  return ret;
}
