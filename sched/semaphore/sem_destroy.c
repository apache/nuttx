/****************************************************************************
 * sched/semaphore/sem_destroy.c
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

#include <errno.h>

#include "semaphore/semaphore.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_destroy
 *
 * Description:
 *   This function is used to destroy the un-named semaphore indicated by
 *   'sem'.  Only a semaphore that was created using nxsem_init() may be
 *   destroyed using nxsem_destroy(); the effect of calling nxsem_destroy()
 *   with a named semaphore is undefined.  The effect of subsequent use of
 *   the semaphore sem is undefined until sem is re-initialized by another
 *   call to nxsem_init().
 *
 *   The effect of destroying a semaphore upon which other processes are
 *   currently blocked is undefined.
 *
 * Input Parameters:
 *   sem - Semaphore to be destroyed.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_destroy(FAR sem_t *sem)
{
  DEBUGASSERT(sem != NULL);

  /* There is really no particular action that we need
   * take to destroy a semaphore.  We will just reset
   * the count to some reasonable value (0) and release
   * ownership.
   *
   * Check if other threads are waiting on the semaphore.
   * In this case, the behavior is undefined.  We will:
   * leave the count unchanged but still return OK.
   */

  if (sem->semcount > 0)
    {
      sem->semcount = 0;
    }

  /* Release holders of the semaphore */

  nxsem_destroyholder(sem);
  return OK;
}

/****************************************************************************
 * Name: sem_destroy
 *
 * Description:
 *   This function is used to destroy the un-named semaphore indicated by
 *   'sem'.  Only a semaphore that was created using nxsem_init() may be
 *   destroyed using sem_destroy(); the effect of calling sem_destroy() with
 *   a named semaphore is undefined.  The effect of subsequent use of the
 *   semaphore sem is undefined until sem is re-initialized by another call
 *   to nxsem_init().
 *
 *   The effect of destroying a semaphore upon which other processes are
 *   currently blocked is undefined.
 *
 * Input Parameters:
 *   sem - Semaphore to be destroyed.
 *
 * Returned Value:
 *   This function is a application interface.  It returns zero (OK) if
 *   successful.  Otherwise, -1 (ERROR) is returned and the errno value is
 *   set appropriately.
 *
 ****************************************************************************/

int sem_destroy (FAR sem_t *sem)
{
  int ret;

  /* Assure a valid semaphore is specified */

  if (sem == NULL)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  ret = nxsem_destroy(sem);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
