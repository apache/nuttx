/****************************************************************************
 * libs/libc/semaphore/sem_getvalue.c
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
#include <errno.h>

#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsem_get_value
 *
 * Description:
 *   This function updates the location referenced by 'sval' argument to
 *   have the value of the semaphore referenced by 'sem' without effecting
 *   the state of the semaphore.  The updated value represents the actual
 *   semaphore value that occurred at some unspecified time during the call,
 *   but may not reflect the actual value of the semaphore when it is
 *   returned to the calling task.
 *
 *   If 'sem' is locked, then the object to which 'sval' points shall either
 *   be zero or a negative number whose absolute value represents the number
 *   of tasks waiting for the semaphore.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor
 *   sval - Buffer by which the value is returned
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_get_value(FAR sem_t *sem, FAR int *sval)
{
  if (sem != NULL && sval != NULL)
    {
      *sval = sem->semcount;
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name:  sem_getvalue
 *
 * Description:
 *   This function updates the location referenced by 'sval' argument to
 *   have the value of the semaphore referenced by 'sem' without effecting
 *   the state of the semaphore.  The updated value represents the actual
 *   semaphore value that occurred at some unspecified time during the call,
 *   but may not reflect the actual value of the semaphore when it is
 *   returned to the calling task.
 *
 *   If 'sem' is locked, then the object to which 'sval' points shall either
 *   be zero or a negative number whose absolute value represents the number
 *   of tasks waiting for the semaphore.
 *
 * Input Parameters:
 *   sem - Semaphore descriptor
 *   sval - Buffer by which the value is returned
 *
 *   This returns zero (OK) if successful.  Otherwise, -1 (ERROR) is
 *   returned and the errno value is set appropriately.
 *
 ****************************************************************************/

int sem_getvalue(FAR sem_t *sem, FAR int *sval)
{
  int ret;

  ret = nxsem_get_value(sem, sval);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
