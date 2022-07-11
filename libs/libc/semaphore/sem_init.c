/****************************************************************************
 * libs/libc/semaphore/sem_init.c
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

#include <sys/types.h>
#include <limits.h>
#include <errno.h>

#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_init
 *
 * Description:
 *   This function initializes the UNNAMED semaphore sem. Following a
 *   successful call to nxsem_init(), the semaphore may be used in subsequent
 *   calls to nxsem_wait(), nxsem_post(), and nxsem_trywait().  The semaphore
 *   remains usable until it is destroyed.
 *
 *   Only sem itself may be used for performing synchronization. The result
 *   of referring to copies of sem in calls to nxsem_wait(), nxsem_trywait(),
 *   nxsem_post(), and nxsem_destroy() is undefined.
 *
 * Input Parameters:
 *   sem - Semaphore to be initialized
 *   pshared - Process sharing (not used)
 *   value - Semaphore initialization value
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_init(FAR sem_t *sem, int pshared, unsigned int value)
{
  UNUSED(pshared);

  /* Verify that a semaphore was provided and the count is within the valid
   * range.
   */

  if (sem != NULL && value <= SEM_VALUE_MAX)
    {
      /* Initialize the semaphore count */

      sem->semcount         = (int16_t)value;

      /* Initialize to support priority inheritance */

#ifdef CONFIG_PRIORITY_INHERITANCE
      sem->flags            = 0;
#  if CONFIG_SEM_PREALLOCHOLDERS > 0
      sem->hhead            = NULL;
#  else
      INITIALIZE_SEMHOLDER(&sem->holder[0]);
      INITIALIZE_SEMHOLDER(&sem->holder[1]);
#  endif
#endif
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: sem_init
 *
 * Description:
 *   This function initializes the UNNAMED semaphore sem. Following a
 *   successful call to sem_init(), the semaphore may be used in subsequent
 *   calls to sem_wait(), sem_post(), and sem_trywait().  The semaphore
 *   remains usable until it is destroyed.
 *
 *   Only sem itself may be used for performing synchronization. The result
 *   of referring to copies of sem in calls to sem_wait(), sem_trywait(),
 *   sem_post(), and sem_destroy() is undefined.
 *
 * Input Parameters:
 *   sem - Semaphore to be initialized
 *   pshared - Process sharing (not used)
 *   value - Semaphore initialization value
 *
 * Returned Value:
 *   This returns zero (OK) if successful.  Otherwise, -1 (ERROR) is
 *   returned and the errno value is set appropriately.
 *
 ****************************************************************************/

int sem_init(FAR sem_t *sem, int pshared, unsigned int value)
{
  int ret;

  ret = nxsem_init(sem, pshared, value);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
