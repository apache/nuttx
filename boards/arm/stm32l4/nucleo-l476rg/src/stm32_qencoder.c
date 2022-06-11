/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_qencoder.c
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

#include <nuttx/sensors/qencoder.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32l4_qencoder.h"
#include "nucleo-l476rg.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: qe_devinit
 *
 * Description:
 *   All STM32L4 architectures must provide the following interface to work
 *   with examples/qencoder.
 *
 ****************************************************************************/

int stm32l4_qencoder_initialize(const char *devpath, int timer)
{
  int ret;

  /* Initialize a quadrature encoder interface. */

  sninfo("Initializing the quadrature encoder using TIM%d\n", timer);
  ret = stm32l4_qeinitialize(devpath, timer);
  if (ret < 0)
    {
      snerr("ERROR: stm32l4_qeinitialize failed: %d\n", ret);
    }

  return ret;
}
