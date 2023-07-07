/****************************************************************************
 * include/nuttx/rwlock.h
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

#ifndef __INCLUDE_NUTTX_RWLOCK_H
#define __INCLUDE_NUTTX_RWLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mutex.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LIBC_RWLOCK
#  define rwlock_t mutex_t
#  define nxrwlock_init nxmutex_init
#  define nxrwlock_destroy nxmutex_destroy
#  define nxrwlock_read_lock nxmutex_lock
#  define nxrwlock_read_unlock nxmutex_unlock
#  define nxrwlock_write_lock nxmutex_lock
#  define nxrwlock_write_unlock nxmutex_unlock
#else

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

typedef struct
{
  mutex_t read;

  mutex_t write;

  int reads;
} rwlock_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxrwlock_init
 *
 * Description:
 *   Initialize a rwlock object to it's initial, both write semaphore and
 *   read semaphore was set to 1, the num of readers  was set to 0
 *
 * Input Parameters:
 *   lock  - read-write-lock descriptor.
 *
 * Returned Value:
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxrwlock_init(FAR rwlock_t *lock);

/****************************************************************************
 * Name: nxrwlock_destroy
 *
 * Description:
 *   rwlock_destroy a rwlock object, both write semaphore and read semaphore
 *   was destroied, the num of readers  was set to 0.
 *
 * Input Parameters:
 *   lock  - read-write-lock descriptor.
 *
 * Returned Value:
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxrwlock_destroy(FAR rwlock_t *lock);

/****************************************************************************
 * Name: nxrwlock_read_lock
 *
 * Description:
 *   This function attempts to get the rwlock. If thers's none other write
 *   task hold the write semaphore of rwlock, the task can successful get
 *   rwlock, otherwise the task would be blocked.
 *
 * Input Parameters:
 *   lock - read-write-lock descriptor.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure. Possible returned errors:
 *   - EINVAL:  Invalid attempt to get the semaphore
 *   - EINTR:   The wait was interrupted by the receipt of a signal.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

int nxrwlock_read_lock(FAR rwlock_t *lock);

/****************************************************************************
 * Name: nxrwlock_read_unlock
 *
 * Description:
 *   This function attempts to give the rwlock. If thers's none other read
 *   task hold the rwlock, the task can successful give the rwlock by give
  *  the write semaphore of rwlock.
 *
 * Input Parameters:
 *   lock - read-write-lock descriptor.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   failure. Possible returned errors:
 *   - EINVAL:  Invalid attempt to get the semaphore
 *   - EINTR:   The wait was interrupted by the receipt of a signal.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

int nxrwlock_read_unlock(FAR rwlock_t *lock);

/****************************************************************************
 * Name: nxrwlock_write_lock
 *
 * Description:
 *   When a task try to write with a rwlock, it will call rwlock_write_lock.
 *   This function try to get write semaphore of rwlock. If the write
 *   semaphore does already hold by other task, this tasks will be blocked,
 *   which is uninterruptible.
 *
 * Input Parameters:
 *   lock - read-write-lock descriptor.
 *
 * Returned Value:
 *   Zero(OK)  - On success
 *   EINVAL    - Invalid attempt to get the write lock
 *   ECANCELED - May be returned if the thread is canceled while waiting.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

int nxrwlock_write_lock(FAR rwlock_t *lock);

/****************************************************************************
 * Name: nxrwlock_write_unlock
 *
 * Description:
 *   When a task has finished write with a rwlock, it will call
 *   rwlock_write_unlock(). This function unlocks the write semaphore
 *   referenced by rwlock by performing the semaphore unlock operation on
 *   write semaphore of rwlock.
 *
 *   If the semaphore value resulting from this operation is positive, then
 *   no tasks were blocked waiting for the semaphore to become unlocked; the
 *   semaphore is simply incremented.
 *
 *   If the value of the semaphore resulting from this operation is zero,
 *   then one of the tasks blocked waiting for the semaphore shall be
 *   allowed to return successfully from its call to nxsem_wait().
 *
 * Input Parameters:
 *   lock - read-write-lock descriptor.
 *
 * Returned Value:
 *   This function will return zero (OK) if successful. Otherwise,
 *   -1 (ERROR) is returned and the errno value is set appropriately.
 *
 * Assumptions:
 *   Not running at the interrupt level.
 *
 ****************************************************************************/

int nxrwlock_write_unlock(FAR rwlock_t *lock);

#endif /* CONFIG_LIBC_RWLOCK */

#endif /* __INCLUDE_NUTTX_RWLOCK_H */
