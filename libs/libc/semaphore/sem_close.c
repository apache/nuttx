/****************************************************************************
 * libs/libc/semaphore/sem_close.c
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

#include <nuttx/semaphore.h>

#ifdef CONFIG_FS_NAMED_SEMAPHORES

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sem_close
 *
 * Description:
 *   This function is called to indicate that the calling task is finished
 *   with the specified named semaphore, 'sem'.  The sem_close() deallocates
 *   any system resources allocated by the system for this named semaphore.
 *
 *   If the semaphore has not been removed with a call to sem_unlink(), then
 *   sem_close() has no effect on the named semaphore.  However, when the
 *   named semaphore has been fully unlinked, the semaphore will vanish when
 *   the last task closes it.
 *
 * Input Parameters:
 *  sem - semaphore descriptor
 *
 * Returned Value:
 *  0 (OK), or -1 (ERROR) if unsuccessful.
 *
 * Assumptions:
 *   - Care must be taken to avoid risking the deletion of a semaphore that
 *     another calling task has already locked.
 *   - sem_close must not be called for an un-named semaphore
 *
 ****************************************************************************/

int sem_close(FAR sem_t *sem)
{
  int ret;

  ret = nxsem_close(sem);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return ret;
}

#endif /* CONFIG_FS_NAMED_SEMAPHORES */
