/****************************************************************************
 * arch/arm64/src/qemu/qemu_initialize.c
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
#include <nuttx/power/pm.h>
#include <stdbool.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm64_pminitialize(void)
{
  pm_initialize();
}

#ifdef CONFIG_SMP
static bool pm_idle_handler(int cpu,
                               enum pm_state_e cpu_state,
                               enum pm_state_e system_state)
{
  bool first = false;
  switch (cpu_state)
    {
      case PM_NORMAL:
      case PM_IDLE:
      case PM_STANDBY:
      case PM_SLEEP:

        /* do cpu domain pm enter operations */

        asm("NOP");

        if (system_state >= PM_NORMAL)
          {
            switch (system_state)
              {
                case PM_NORMAL:
                case PM_IDLE:
                case PM_STANDBY:
                case PM_SLEEP:

                  /* do system domain pm enter operations */

                  asm("NOP");

                  break;
                default:
                  break;
              }
          }

        pm_idle_unlock();

        /* do no cross-core relative operations */

        asm("WFI");

        first = pm_idle_lock(cpu);
        if (first)
          {
            /* do system domain pm leave operations */

            asm("NOP");
          }

        /* do cpu domain pm leave operations */

        asm("NOP");

        break;
      default:
        break;
    }

  return first;
}
#else

static void pm_idle_handler(enum pm_state_e state)
{
  switch (state)
    {
      default:
        asm("WFI");
        break;
    }
}

#endif

void up_idle(void)
{
  pm_idle(pm_idle_handler);
}
