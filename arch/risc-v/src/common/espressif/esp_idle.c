/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_idle.c
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

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spinlock.h>

#ifdef CONFIG_PM
#include "esp_sleep.h"
#include "esp_pm.h"
#include "esp_idle.h"
#endif

#ifdef CONFIG_RTC_DRIVER
#include "esp_hr_timer.h"
#endif

#ifdef CONFIG_SCHED_TICKLESS
#include "esp_tickless.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#ifndef MIN 
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif
#define EXPECTED_IDLE_TIME_US (10000)
#define EARLY_WAKEUP_US       (200)

#define DEBUG_AUTOSLEEP 0

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idle
 *
 * Description:
 *   up_idle() is the logic that will be executed when their is no other
 *   ready-to-run task. This is processor idle time and will continue until
 *   some interrupt occurs to cause a context switch from the idle task.
 *
 *   Processing in this state may be processor-specific. e.g., this is where
 *   power management operations might be performed.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_idlepm
 *
 * Description:
 *   Perform IDLE state power management.
 *
 ****************************************************************************/
#if defined(CONFIG_PM)

static void up_idlepm(void)
{ 
  irqstate_t flags;
  flags = spin_lock_irqsave(NULL);

  if ( esp_pm_lockstatus() == 0 )
    {   
      uint64_t sleep_us = up_get_idletime();
      if ( (sleep_us > EXPECTED_IDLE_TIME_US) )
        {
          sleep_us -= EARLY_WAKEUP_US;
          esp_wait_tx_done();
          esp_sleep_enable_timer_wakeup(sleep_us);
          esp_light_sleep_start();

          if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_GPIO)
            {
    #if DEBUG_AUTOSLEEP
              printf("Wake up from gpio\n");
              esp_wait_tx_done();
    #endif
            }

          else 
            {
    #if DEBUG_AUTOSLEEP
            printf("Wake up from timer\n"); 
            esp_wait_tx_done();
    #endif
            }
        }
    }
  spin_unlock_irqrestore(NULL, flags);
}

#else
#  define up_idlepm() 
#endif


void up_idle(void)
{
#if defined(CONFIG_SUPPRESS_INTERRUPTS) || defined(CONFIG_SUPPRESS_TIMER_INTS)
  /* If the system is idle and there are no timer interrupts, then process
   * "fake" timer interrupts. Hopefully, something will wake up.
   */

  nxsched_process_timer();
#else

  /* This would be an appropriate place to put some MCU-specific logic to
   * sleep in a reduced power mode until an interrupt occurs to save power
   */

  asm("WFI");
  
  /* Perform IDLE mode power management */
  up_idlepm();

#endif
}
