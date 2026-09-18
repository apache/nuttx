/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_bootloader.c
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

#include <stdlib.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/irq.h>

#include "imxrt118x_start_cm7.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define M7_ENTRY  0x28080000u

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bootloader_main
 *
 * Description:
 *   Release the Cortex-M7 and sleep forever.
 *
 ****************************************************************************/

int bootloader_main(int argc, char *argv[])
{
  /* Hand over LPUART1 to the M7:  disable the IRQ on the M33 NVIC and
   * detach its handler.
   */

  up_disable_irq(IMXRT_IRQ_LPUART1);
  irq_detach(IMXRT_IRQ_LPUART1);

  if (imxrt118x_release_cm7(M7_ENTRY) < 0)
    {
      return EXIT_FAILURE;
    }

  while (1)
    {
      usleep(10000);
    }

  return 0;
}
