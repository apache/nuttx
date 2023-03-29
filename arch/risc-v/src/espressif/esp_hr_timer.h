/****************************************************************************
 * arch/risc-v/src/espressif/esp_hr_timer.h
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

#ifndef __ARCH_RISCV_SRC_ESPRESSIF_ESP_HR_TIMER_H
#define __ARCH_RISCV_SRC_ESPRESSIF_ESP_HR_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/list.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HR_TIMER_NOFLAGS    (0)         /* Timer supports no feature */
#define HR_TIMER_REPEAT     (1 << 0)    /* Timer supports repeat mode */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* HR Timer state */

enum esp_hr_timer_state_e
{
  HR_TIMER_IDLE,                    /* Timer is not counting */
  HR_TIMER_READY,                   /* Timer is counting */
  HR_TIMER_TIMEOUT,                 /* Timer timed out */
  HR_TIMER_DELETE                   /* Timer is to be delete */
};

/* HR Timer data structure */

struct esp_hr_timer_s
{
  uint64_t timeout;                 /* Timeout value */
  uint64_t alarm;                   /* Timeout period */
  void (*callback)(void *arg);      /* Callback function */
  void *arg;                        /* Private data */
  uint16_t flags;                   /* Supported features */
  enum esp_hr_timer_state_e state;  /* Timer state */
  struct list_node list;            /* Working list */
};

/* HR Timer creation arguments data structure */

struct esp_hr_timer_args_s
{
  void (*callback)(void *arg);      /* Callback function */
  void *arg;                        /* Private data */
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
 *   Create a High Resolution Timer from the provided arguments.
 *
 * Input Parameters:
 *   args          - HR Timer creation arguments.
 *
 * Output Parameters:
 *   timer_handle  - HR Timer handle pointer.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_hr_timer_create(const struct esp_hr_timer_args_s *args,
                        struct esp_hr_timer_s **timer_handle);

/****************************************************************************
 * Name: esp_hr_timer_start
 *
 * Description:
 *   Start the High Resolution Timer.
 *
 * Input Parameters:
 *   timer         - HR Timer pointer.
 *   timeout       - Timeout value.
 *   repeat        - Repeat mode (true: enabled, false: disabled).
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_start(struct esp_hr_timer_s *timer,
                        uint64_t timeout,
                        bool repeat);

/****************************************************************************
 * Name: esp_hr_timer_stop
 *
 * Description:
 *   Stop the High Resolution Timer.
 *
 * Input Parameters:
 *   timer         - HR Timer pointer.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_stop(struct esp_hr_timer_s *timer);

/****************************************************************************
 * Name: esp_hr_timer_delete
 *
 * Description:
 *   Stop and delete the High Resolution Timer.
 *
 * Input Parameters:
 *   timer         - HR Timer pointer.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_delete(struct esp_hr_timer_s *timer);

/****************************************************************************
 * Name: esp_hr_timer_time_us
 *
 * Description:
 *   Get time of the High Resolution Timer in microseconds.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Time of the HR Timer in microseconds.
 *
 ****************************************************************************/

uint64_t esp_hr_timer_time_us(void);

/****************************************************************************
 * Name: esp_hr_timer_get_alarm
 *
 * Description:
 *   Get the timestamp when the next timeout is expected to occur.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Timestamp of the nearest timer event in microseconds.
 *
 ****************************************************************************/

uint64_t esp_hr_timer_get_alarm(void);

/****************************************************************************
 * Name: esp_hr_timer_calibration
 *
 * Description:
 *   Adjust current High Resolution Timer by a certain value.
 *
 * Input Parameters:
 *   time_us       - Adjustment to apply to the HR Timer in microseconds.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_hr_timer_calibration(uint64_t time_us);

/****************************************************************************
 * Name: esp_hr_timer_init
 *
 * Description:
 *   Initialize High Resolution Timer.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int esp_hr_timer_init(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ARCH_RISCV_SRC_ESPRESSIF_ESP_HR_TIMER_H */
