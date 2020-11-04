/****************************************************************************
 * arch/xtensa/src/esp32/esp32_rt_timer.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_RT_TIMER_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_RT_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <sys/types.h>
#include <nuttx/list.h>

#define RT_TIMER_NOFLAGS    (0)         /* Timer support no feature */
#define RT_TIMER_REPEAT     (1 << 0)    /* Timer is repeat */

/**
 * RT timer state
 */

enum rt_timer_state_e
{
  RT_TIMER_IDLE,            /* Timer is not counting */
  RT_TIMER_READY,           /* Timer is counting */
  RT_TIMER_TIMEOUT,         /* Timer is timeout */
  RT_TIMER_DELETE           /* Timer is to be delete */
};

/**
 * RT timer data structure
 */

struct rt_timer_s
{
  uint32_t timeout;             /* Timeout value */
  uint64_t alarm;               /* Timeout period */
  void (*callback)(void *arg);  /* Callback function */
  void *arg;                    /* Private data */
  uint16_t flags;               /* Support feature */
  enum rt_timer_state_e state;  /* Mark if timer is started */
  struct list_node list;        /* Working list */
};

/**
 * RT timer creation arguments data structure
 */

struct rt_timer_args_s
{
  void (*callback)(void *arg);  /* Callback function */
  void *arg;                    /* Private data */
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
 * Name: rt_timer_create
 *
 * Description:
 *   Create RT timer by into timer creation arguments
 *
 * Input Parameters:
 *   args         - Input RT timer creation arguments
 *   timer_handle - Output RT timer handle pointer
 *
 * Returned Value:
 *   0 is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int rt_timer_create(FAR const struct rt_timer_args_s *args,
                    FAR struct rt_timer_s **timer_handle);

/****************************************************************************
 * Name: rt_timer_start
 *
 * Description:
 *   Start RT timer.
 *
 * Input Parameters:
 *   timer   - RT timer pointer
 *   timeout - Timeout value
 *   repeat  - If the timer run repeat
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rt_timer_start(FAR struct rt_timer_s *timer,
                    uint32_t timeout,
                    bool repeat);

/****************************************************************************
 * Name: rt_timer_stop
 *
 * Description:
 *   Stop RT timer.
 *
 * Input Parameters:
 *   timer - RT timer pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rt_timer_stop(FAR struct rt_timer_s *timer);

/****************************************************************************
 * Name: rt_timer_delete
 *
 * Description:
 *   Stop and deleta RT timer.
 *
 * Input Parameters:
 *   timer - RT timer pointer
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void rt_timer_delete(FAR struct rt_timer_s *timer);

/****************************************************************************
 * Name: esp32_rt_timer_init
 *
 * Description:
 *   Initialize ESP32 RT timer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0 is returned on success. Otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

int esp32_rt_timer_init(void);

/****************************************************************************
 * Name: esp32_rt_timer_deinit
 *
 * Description:
 *   Deinitialize ESP32 RT timer.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32_rt_timer_deinit(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_RT_TIMER_H */
