/****************************************************************************
 * arch/risc-v/src/esp32c6/esp_coex_adapter.c
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

#include <debug.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <pthread.h>

#include <nuttx/spinlock.h>
#include <irq/irq.h>
#include <nuttx/kmalloc.h>

#include "esp_hr_timer.h"
#include "esp_wlan.h"

#include "esp_attr.h"
#include "esp_timer.h"
#include "soc/rtc.h"
#include "esp_private/esp_clk.h"
#include "private/esp_coexist_adapter.h"
#include "rom/ets_sys.h"
#include "soc/soc_caps.h"
#include "private/esp_modem_wrapper.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OSI_FUNCS_TIME_BLOCKING  0xffffffff

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int64_t esp_coex_esp_timer_get_time_wrapper(void);
static int32_t esp_coex_semphr_take_from_isr_wrapper(void *semphr,
                                                     void *hptw);
static int32_t esp_coex_semphr_give_from_isr_wrapper(void *semphr,
                                                     void *hptw);
static int esp_coex_is_in_isr_wrapper(void);

/****************************************************************************
 * Public Data
 ****************************************************************************/

coex_adapter_funcs_t g_coex_adapter_funcs =
{
  ._version = COEX_ADAPTER_VERSION,
  ._task_yield_from_isr = esp_coex_common_task_yield_from_isr_wrapper,
  ._semphr_create = esp_coex_common_semphr_create_wrapper,
  ._semphr_delete = esp_coex_common_semphr_delete_wrapper,
  ._semphr_take_from_isr = esp_coex_semphr_take_from_isr_wrapper,
  ._semphr_give_from_isr = esp_coex_semphr_give_from_isr_wrapper,
  ._semphr_take = esp_coex_common_semphr_take_wrapper,
  ._semphr_give = esp_coex_common_semphr_give_wrapper,
  ._is_in_isr = esp_coex_is_in_isr_wrapper,
  ._malloc_internal = esp_coex_common_malloc_internal_wrapper,
  ._free = free,
  ._esp_timer_get_time = esp_coex_esp_timer_get_time_wrapper,
  ._env_is_chip = esp_coex_common_env_is_chip_wrapper,
  ._timer_disarm = esp_coex_common_timer_disarm_wrapper,
  ._timer_done = esp_coex_common_timer_done_wrapper,
  ._timer_setfn = esp_coex_common_timer_setfn_wrapper,
  ._timer_arm_us = esp_coex_common_timer_arm_us_wrapper,
  ._magic = COEX_ADAPTER_MAGIC,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_coex_esp_timer_get_time_wrapper
 *
 * Description:
 *   This function retrieves the current time of the High Resolution Timer
 *   in microseconds. It is a wrapper around the esp_hr_timer_time_us
 *   function, providing a consistent interface for the coexistence module.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The current time of the High Resolution Timer in microseconds, as a
 *   64-bit integer.
 *
 ****************************************************************************/

static IRAM_ATTR int64_t esp_coex_esp_timer_get_time_wrapper(void)
{
  return (int64_t)esp_hr_timer_time_us();
}

/****************************************************************************
 * Name: esp_coex_semphr_take_from_isr_wrapper
 *
 * Description:
 *   Take a semaphore from an ISR
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer.
 *   hptw   - Unused.
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t IRAM_ATTR esp_coex_semphr_take_from_isr_wrapper(void *semphr,
                                                               void *hptw)
{
  *(int *)hptw = 0;

  return nuttx_err_to_freertos(nxsem_trywait(semphr));
}

/****************************************************************************
 * Name: esp_coex_semphr_give_from_isr_wrapper
 *
 * Description:
 *   Post semaphore
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *   hptw   - Unused.
 *
 * Returned Value:
 *   True if success or false if fail
 *
 ****************************************************************************/

static int32_t IRAM_ATTR esp_coex_semphr_give_from_isr_wrapper(void *semphr,
                                                               void *hptw)
{
  *(int *)hptw = 0;

  return esp_coex_common_semphr_give_wrapper(semphr);
}

/****************************************************************************
 * Name: esp_coex_is_in_isr_wrapper
 *
 * Description:
 *   This function checks if the current context is an interrupt service
 *   routine (ISR). It is a wrapper around the NuttX up_interrupt_context
 *   function.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns 1 if the current context is an ISR, 0 otherwise.
 *
 ****************************************************************************/

static int IRAM_ATTR esp_coex_is_in_isr_wrapper(void)
{
  return (int)up_interrupt_context();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_coex_common_env_is_chip_wrapper
 *
 * Description:
 *   This function checks if the environment is a chip or FPGA.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns true if the environment is a chip, false if it's an FPGA.
 *
 ****************************************************************************/

bool IRAM_ATTR esp_coex_common_env_is_chip_wrapper(void)
{
#ifdef CONFIG_IDF_ENV_FPGA
  return false;
#else
  return true;
#endif
}

/****************************************************************************
 * Name: esp_coex_common_spin_lock_create_wrapper
 *
 * Description:
 *   Create spin lock in SMP mode
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Spin lock data pointer
 *
 ****************************************************************************/

void *esp_coex_common_spin_lock_create_wrapper(void)
{
  spinlock_t *lock;
  int tmp;

  tmp = sizeof(*lock);
  lock = kmm_malloc(tmp);
  if (!lock)
    {
      wlerr("Failed to alloc %d memory\n", tmp);
      DEBUGPANIC();
    }

  spin_lock_init(lock);

  return lock;
}

/****************************************************************************
 * Name: esp_coex_common_int_disable_wrapper
 *
 * Description:
 *   Enter critical section by disabling interrupts and taking the spin lock
 *   if in SMP mode.
 *
 * Input Parameters:
 *   wifi_int_mux - Spin lock data pointer
 *
 * Returned Value:
 *   CPU PS value.
 *
 ****************************************************************************/

uint32_t IRAM_ATTR esp_coex_common_int_disable_wrapper(void *wifi_int_mux)
{
  irqstate_t flags;

  flags = spin_lock_irqsave((spinlock_t *)wifi_int_mux);

  return (uint32_t)flags;
}

/****************************************************************************
 * Name: esp_coex_common_int_restore_wrapper
 *
 * Description:
 *   Exit from critical section by enabling interrupts and releasing the spin
 *   lock if in SMP mode.
 *
 * Input Parameters:
 *   wifi_int_mux - Spin lock data pointer
 *   tmp          - CPU PS value.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp_coex_common_int_restore_wrapper(void *wifi_int_mux,
                                                   uint32_t tmp)
{
  irqstate_t flags = (irqstate_t)tmp;

  spin_unlock_irqrestore((spinlock_t *)wifi_int_mux, flags);
}

/****************************************************************************
 * Name: esp_task_yield_from_isr
 *
 * Description:
 *   Perform a solicited context switch on FreeRTOS. Do nothing in NuttX.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp_coex_common_task_yield_from_isr_wrapper(void)
{
}

/****************************************************************************
 * Name: esp_coex_common_semphr_create_wrapper
 *
 * Description:
 *   Create and initialize semaphore
 *
 * Input Parameters:
 *   max  - No meanining for NuttX
 *   init - semaphore initialization value
 *
 * Returned Value:
 *   Semaphore data pointer
 *
 ****************************************************************************/

void *esp_coex_common_semphr_create_wrapper(uint32_t max, uint32_t init)
{
  int ret;
  sem_t *sem;
  int tmp;

  tmp = sizeof(sem_t);
  sem = kmm_malloc(tmp);
  if (!sem)
    {
      wlerr("Failed to alloc %d memory\n", tmp);
      return NULL;
    }

  ret = nxsem_init(sem, 0, init);
  if (ret)
    {
      wlerr("Failed to initialize sem error=%d\n", ret);
      kmm_free(sem);
      return NULL;
    }

  return sem;
}

/****************************************************************************
 * Name: esp_coex_common_semphr_delete_wrapper
 *
 * Description:
 *   Delete semaphore
 *
 * Input Parameters:
 *   semphr - Semaphore data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_coex_common_semphr_delete_wrapper(void *semphr)
{
  sem_t *sem = (sem_t *)semphr;

  nxsem_destroy(sem);
  kmm_free(sem);
}

/****************************************************************************
 * Name: esp_coex_common_semphr_take_wrapper
 *
 * Description:
 *   This function attempts to take (wait for) a semaphore within a certain
 *   period of time. It is a wrapper around the NuttX nxsem_wait and
 *   nxsem_tickwait functions, providing error handling and translation
 *   between NuttX and ESP-IDF error codes.
 *
 * Input Parameters:
 *   semphr          - Pointer to the semaphore data structure.
 *   block_time_tick - The maximum number of system ticks to wait.
 *
 * Returned Value:
 *   Returns 0 if the semaphore was successfully taken, or a negative error
 *   code if the operation failed or the timeout expired.
 *
 ****************************************************************************/

int32_t esp_coex_common_semphr_take_wrapper(void *semphr,
                                            uint32_t block_time_tick)
{
  int ret;
  sem_t *sem = (sem_t *)semphr;

  if (block_time_tick == OSI_FUNCS_TIME_BLOCKING)
    {
      ret = nxsem_wait(sem);
    }
  else
    {
      if (block_time_tick > 0)
        {
          ret = nxsem_tickwait(sem, block_time_tick);
        }
      else
        {
          ret = nxsem_trywait(sem);
        }
    }

  if (ret)
    {
      wlerr("ERROR: Failed to wait sem in %lu ticks. Error=%d\n",
            block_time_tick, ret);
    }

  return nuttx_err_to_freertos(ret);
}

/****************************************************************************
 * Name: esp_coex_common_semphr_give_wrapper
 *
 * Description:
 *   This function posts (releases) a semaphore. It is a wrapper around the
 *   NuttX nxsem_post function, providing error handling and translation
 *   between NuttX and ESP-IDF error codes.
 *
 * Input Parameters:
 *   semphr - Pointer to the semaphore data structure.
 *
 * Returned Value:
 *   Returns 0 if the semaphore was successfully posted, or a negative error
 *   code if the operation failed.
 *
 ****************************************************************************/

int32_t esp_coex_common_semphr_give_wrapper(void *semphr)
{
  int ret;
  sem_t *sem = (sem_t *)semphr;

  ret = nxsem_post(sem);
  if (ret)
    {
      wlerr("Failed to post sem error=%d\n", ret);
    }

  return nuttx_err_to_freertos(ret);
}

/****************************************************************************
 * Name: esp_coex_common_timer_disarm_wrapper
 *
 * Description:
 *   Disable timer
 *
 * Input Parameters:
 *   timer - timer data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp_coex_common_timer_disarm_wrapper(void *timer)
{
  ets_timer_disarm(timer);
}

/****************************************************************************
 * Name: esp_coex_common_timer_done_wrapper
 *
 * Description:
 *   Disable and free timer
 *
 * Input Parameters:
 *   timer - timer data pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_coex_common_timer_done_wrapper(void *timer)
{
  ets_timer_done(timer);
}

/****************************************************************************
 * Name: esp_coex_common_timer_setfn_wrapper
 *
 * Description:
 *   Set timer callback function and private data
 *
 * Input Parameters:
 *   ptimer    - Timer data pointer
 *   pfunction - Callback function
 *   parg      - Callback function private data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_coex_common_timer_setfn_wrapper(void *ptimer,
                                         void *pfunction,
                                         void *parg)
{
  ets_timer_setfn(ptimer, pfunction, parg);
}

/****************************************************************************
 * Name: esp_coex_common_timer_arm_us_wrapper
 *
 * Description:
 *   Set timer timeout period and repeat flag
 *
 * Input Parameters:
 *   ptimer - timer data pointer
 *   us     - micro seconds
 *   repeat - true: run cycle, false: run once
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void IRAM_ATTR esp_coex_common_timer_arm_us_wrapper(void *ptimer,
                                                    uint32_t us,
                                                    bool repeat)
{
  ets_timer_arm_us(ptimer, us, repeat);
}

/****************************************************************************
 * Name: esp_coex_common_clk_slowclk_cal_get_wrapper
 *
 * Description:
 *   Get the calibration value of RTC slow clock
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The calibration value obtained using rtc_clk_cal
 *
 ****************************************************************************/

uint32_t esp_coex_common_clk_slowclk_cal_get_wrapper(void)
{
  /* The bit width of WiFi light sleep clock calibration is 12 while the one
   * of system is 19. It should shift 19 - 12 = 7.
   */

  return (esp_clk_slowclk_cal_get() >>
          (RTC_CLK_CAL_FRACT - SOC_WIFI_LIGHT_SLEEP_CLK_WIDTH));
}

/****************************************************************************
 * Name: esp_coex_common_malloc_internal_wrapper
 *
 * Description:
 *   Drivers allocate a block of memory
 *
 * Input Parameters:
 *   size - memory size
 *
 * Returned Value:
 *   Memory pointer
 *
 ****************************************************************************/

IRAM_ATTR void *esp_coex_common_malloc_internal_wrapper(size_t size)
{
  return kmm_malloc(size);
}
