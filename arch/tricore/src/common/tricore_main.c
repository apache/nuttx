/****************************************************************************
 * arch/tricore/src/common/tricore_main.c
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

#include <tricore_internal.h>
#include <nuttx/init.h>

#include "Ifx_Types.h"
#include "IfxScuWdt.h"
#include "IfxCpu.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void core_main(void)
{
  static IfxCpu_syncEvent g_sync_event = 0;

  /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
   * Enable the watchdogs and service them periodically if it is required
   */

  IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
  IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

  /* Wait for CPU sync event */

  IfxCpu_emitEvent(&g_sync_event);
  IfxCpu_waitEvent(&g_sync_event, 1);

  if (IfxCpu_getCoreIndex() == 0)
    {
      tricore_earlyserialinit();
      nx_start();
    }

  while (1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void core0_main(void)
{
  core_main();
}

void core1_main(void)
{
  core_main();
}

void core2_main(void)
{
  core_main();
}

void core3_main(void)
{
  core_main();
}

void core4_main(void)
{
  core_main();
}

void core5_main(void)
{
  core_main();
}
