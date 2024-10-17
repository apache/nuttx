/****************************************************************************
 * arch/arm/src/qemu/qemu_idle.c
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

#include <nuttx/arch.h>
#include "arm_internal.h"

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
#include "sm.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when there is no other
 *   ready-to-run task.  This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
extern volatile uint32_t g_ap_entry;
#endif

void up_idle(void)
{
#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
  if (g_ap_entry != 0)
    {
      up_irq_disable();
      arm_sm_boot_nsec(g_ap_entry);
      arm_sm_switch_nsec();
    }
#else
  #if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

    nxsched_process_timer();

  #else

  /* Sleep until an interrupt occurs to save power */

    asm("WFI");
  #endif
#endif
}
