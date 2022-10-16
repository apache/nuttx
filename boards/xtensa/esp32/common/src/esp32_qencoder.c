/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_qencoder.c
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

#include <nuttx/sensors/qencoder.h>
#include <arch/board/board.h>

#include "esp32_qencoder.h"
#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_qencoder_initialize
 *
 * Description:
 *   Initialize the quadrature encoder driver for the given timer
 *
 ****************************************************************************/

int board_qencoder_initialize(int devno, int pcntno)
{
  int ret;
  char devpath[12];

  /* Initialize a quadrature encoder interface. */

  sninfo("Initializing the quadrature encoder using PCNT%d\n", pcntno);
  snprintf(devpath, sizeof(devpath), "/dev/qe%d", devno);
  ret = esp32_qeinitialize(devpath, pcntno);
  if (ret < 0)
    {
      snerr("ERROR: esp32_qeinitialize failed: %d\n", ret);
    }

  return ret;
}
