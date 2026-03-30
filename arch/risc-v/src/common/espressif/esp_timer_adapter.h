/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_timer_adapter.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TIMER_ADAPTER_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TIMER_ADAPTER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HR_TIMER_NOFLAGS    (0)         /* Timer supports no feature */
#define HR_TIMER_REPEAT     (1 << 0)    /* Timer supports repeat mode */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* HR Timer state (simplified for adapter) */

enum esp_hr_timer_state_e
{
  HR_TIMER_IDLE,                    /* Timer is not counting */
  HR_TIMER_READY,                   /* Timer is counting */
  HR_TIMER_TIMEOUT,                 /* Timer timed out */
  HR_TIMER_DELETE                   /* Timer is to be deleted */
};

/* Forward declaration - opaque type */

struct esp_hr_timer_s;

/* HR Timer creation arguments data structure */

struct esp_hr_timer_args_s
{
  void (*callback)(void *arg);      /* Callback function */
  void *arg;                        /* Private data */
  const char *name;                 /* Timer name */
  bool skip_unhandled_events;       /* Skip unhandled events for periodic timers */
};

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_hr_timer_create
 *
 * Description:
 *   Create a high-resolution timer instance and initialize it from the
 *   provided arguments.
 *
 * Input Parameters:
 *   args         - Timer creation arguments.
 *   timer_handle - Location where the created timer handle is returned.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp_hr_timer_create(const struct esp_hr_timer_args_s *args,
                        struct esp_hr_timer_s **timer_handle);

/****************************************************************************
 * Name: esp_hr_timer_start
 *
 * Description:
 *   Start a high-resolution timer in one-shot or periodic mode.
 *
 * Input Parameters:
 *   timer   - Timer instance to start.
 *   timeout - Timeout period in microseconds.
 *   repeat  - True for periodic mode, false for one-shot mode.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_start(struct esp_hr_timer_s *timer,
                       uint64_t timeout,
                       bool repeat);

/****************************************************************************
 * Name: esp_hr_timer_start_once
 *
 * Description:
 *   Start a high-resolution timer in one-shot mode.
 *
 * Input Parameters:
 *   timer   - Timer instance to start.
 *   timeout - One-shot timeout in microseconds.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_start_once(struct esp_hr_timer_s *timer, uint64_t timeout);

/****************************************************************************
 * Name: esp_hr_timer_start_periodic
 *
 * Description:
 *   Start a high-resolution timer in periodic mode.
 *
 * Input Parameters:
 *   timer   - Timer instance to start.
 *   timeout - Period interval in microseconds.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_start_periodic(struct esp_hr_timer_s *timer,
                                uint64_t timeout);

/****************************************************************************
 * Name: esp_hr_timer_stop
 *
 * Description:
 *   Stop a running high-resolution timer.
 *
 * Input Parameters:
 *   timer - Timer instance to stop.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_stop(struct esp_hr_timer_s *timer);

/****************************************************************************
 * Name: esp_hr_timer_stop_nolock
 *
 * Description:
 *   Stop a running high-resolution timer without taking any adapter lock.
 *
 * Input Parameters:
 *   timer - Timer instance to stop.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_stop_nolock(struct esp_hr_timer_s *timer);

/****************************************************************************
 * Name: esp_hr_timer_delete
 *
 * Description:
 *   Delete a timer instance and release associated resources.
 *
 * Input Parameters:
 *   timer - Timer instance to delete.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_delete(struct esp_hr_timer_s *timer);

/****************************************************************************
 * Name: esp_hr_timer_time_us
 *
 * Description:
 *   Return the current high-resolution timer time in microseconds.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Current timer time in microseconds.
 *
 ****************************************************************************/

uint64_t esp_hr_timer_time_us(void);

/****************************************************************************
 * Name: esp_hr_timer_get_alarm
 *
 * Description:
 *   Return the timestamp of the next scheduled timer alarm.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Absolute time in microseconds for the next alarm.
 *
 ****************************************************************************/

uint64_t esp_hr_timer_get_alarm(void);

/****************************************************************************
 * Name: esp_hr_timer_calibration
 *
 * Description:
 *   Apply a timer calibration adjustment.
 *   The ESP timer backend does not support calibration, so this operation
 *   is a no-op.
 *
 * Input Parameters:
 *   time_us - Calibration adjustment in microseconds.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_calibration(uint64_t time_us);

/****************************************************************************
 * Name: esp_hr_timer_set
 *
 * Description:
 *   Set the high-resolution timer counter to a specific timestamp.
 *
 * Input Parameters:
 *   new_us - New timer value in microseconds.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_set(uint64_t new_us);

/****************************************************************************
 * Name: esp_hr_timer_lock
 *
 * Description:
 *   Acquire the adapter lock used to serialize timer operations.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_lock(void);

/****************************************************************************
 * Name: esp_hr_timer_unlock
 *
 * Description:
 *   Release the adapter lock used to serialize timer operations.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_unlock(void);

/****************************************************************************
 * Name: esp_hr_timer_init
 *
 * Description:
 *   Initialize the timer adapter and the underlying ESP timer subsystem.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK on success; ERROR on failure.
 *
 ****************************************************************************/

int esp_hr_timer_init(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_TIMER_ADAPTER_H */
