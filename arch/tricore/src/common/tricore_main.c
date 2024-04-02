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

unsigned long g_cpu_data_size;

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

      /* Copy data/bss to each of CPUs */

#ifdef CONFIG_BMP
      int i;

      g_cpu_data_size = _data_size;

      for (i = 0; i < CONFIG_BMP_NCPUS - 1; i++)
        {
          memcpy((void *)(_edata_align + g_cpu_data_size * i),
                 _sbss, g_cpu_data_size);
        }
#endif

      nx_start();
    }
  else
    {
      while (g_nx_initstate != OSINIT_IDLELOOP);

      if (IfxCpu_getCoreIndex() < CONFIG_BMP_NCPUS)
        {
          nx_start();
        }
    }

  /* This would be an appropriate place to put some MCU-specific logic to
   * sleep in a reduced power mode until an interrupt occurs to save power
   */

  while (1)
    {
      Ifx_Ssw_infiniteLoop();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#define DEFINE_CPU_CORE_MAIN(cpu) \
void core##cpu##_main(void) { core_main(); }

DEFINE_CPU_CORE_MAIN(0)
DEFINE_CPU_CORE_MAIN(1)
DEFINE_CPU_CORE_MAIN(2)
DEFINE_CPU_CORE_MAIN(3)
DEFINE_CPU_CORE_MAIN(4)
DEFINE_CPU_CORE_MAIN(5)
