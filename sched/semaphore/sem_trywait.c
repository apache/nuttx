/****************************************************************************
 * sched/semaphore/sem_trywait.c
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

#include <stdbool.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/init.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_trywait_slow
 *
 * Description:
 *   This function locks the specified semaphore in slow mode.
 *
 * Input Parameters:
 *   sem - the semaphore descriptor
 *
 * Returned Value:
 *
 *     EINVAL - Invalid attempt to get the semaphore
 *     EAGAIN - The semaphore is not available.
 *
 * Assumptions:
 *
 ****************************************************************************/

int nxsem_trywait_slow(FAR sem_t *sem)
{
  irqstate_t flags;
  int ret = -EAGAIN;
  bool mutex = NXSEM_IS_MUTEX(sem);
  FAR atomic_t *val = mutex ? NXSEM_MHOLDER(sem) : NXSEM_COUNT(sem);
  int32_t old;
  int32_t new;

  /* The following operations must be performed with interrupts disabled
   * because sem_post() may be called from an interrupt handler.
   */

  flags = enter_critical_section();

  /* If the semaphore is available, give it to the requesting task */

  if (mutex)
    {
      new = nxsched_gettid();
    }

  old = atomic_read(val);
  do
    {
      if (mutex)
        {
          if (NXSEM_MACQUIRED(old))
            {
              goto out;
            }
        }
      else
        {
          if (old <= 0)
            {
              goto out;
            }

          new = old - 1;
        }
    }
  while (!atomic_try_cmpxchg_acquire(val, &old, new));

  /* It is, let the task take the semaphore */

  ret = nxsem_protect_wait(sem);
  if (ret < 0)
    {
      if (mutex)
        {
          atomic_set(NXSEM_MHOLDER(sem), NXSEM_NO_MHOLDER);
        }
      else
        {
          atomic_fetch_add(NXSEM_COUNT(sem), 1);
        }

      leave_critical_section(flags);
      goto out;
    }

  if (!mutex)
    {
      nxsem_add_holder(sem);
    }

out:

  /* Interrupts may now be enabled. */

  leave_critical_section(flags);
  return ret;
}
