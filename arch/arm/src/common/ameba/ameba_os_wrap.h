/****************************************************************************
 * arch/arm/src/common/ameba/ameba_os_wrap.h
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

#ifndef __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_OS_WRAP_H
#define __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_OS_WRAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/semaphore.h>
#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AMEBA_QOBJ_SEM      (0)   /* counting / binary semaphore  */
#define AMEBA_QOBJ_MUTEX    (1)   /* (non-recursive) mutex        */
#define AMEBA_QOBJ_RMUTEX   (2)   /* recursive mutex              */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Unified synchronisation object.
 *
 * Both the os_wrapper (rtos_sema_* / rtos_mutex_*) backend and the raw
 * FreeRTOS shim (xQueue* / xSemaphore*) hand the chip libraries a pointer to
 * one of these.  Because FreeRTOS represents semaphores and mutexes as the
 * same opaque QueueHandle_t, the generic FreeRTOS functions (e.g.
 * xQueueSemaphoreTake, xQueueGenericSend) may be invoked on a handle created
 * by the os_wrapper sema API, and vice-versa.  Using a single tagged
 * object for every flavour guarantees a handle is valid no matter which API
 * acts on it.
 */

struct ameba_qobj_s
{
  uint8_t tag;                /* AMEBA_QOBJ_*                      */
  union
  {
    sem_t    sem;             /* AMEBA_QOBJ_SEM                    */
    mutex_t  mutex;           /* AMEBA_QOBJ_MUTEX                  */
    rmutex_t rmutex;          /* AMEBA_QOBJ_RMUTEX                 */
  } u;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Unified sync-object helpers implemented in ameba_os_wrap.c (the rtos_*
 * mutex/semaphore backend is built on top of them).
 */

struct ameba_qobj_s *ameba_qobj_alloc(uint8_t tag);
void ameba_qobj_free(struct ameba_qobj_s *obj);

/* Blocking / polling take + give operating on the unified object.  ms uses
 * the SDK convention (0 == try, 0xffffffff == block forever).
 */

int ameba_qobj_take(struct ameba_qobj_s *obj, uint32_t ms);
int ameba_qobj_give(struct ameba_qobj_s *obj);

#endif /* __ARCH_ARM_SRC_COMMON_AMEBA_AMEBA_OS_WRAP_H */
