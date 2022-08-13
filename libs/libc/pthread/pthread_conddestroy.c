/****************************************************************************
 * libs/libc/pthread/pthread_conddestroy.c
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

#include <pthread.h>
#include <debug.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_cond_destroy
 *
 * Description:
 *   A thread can delete condition variables.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK (0) on success; a (non-negated) errno value on failure. The errno
 *   variable is not set. EBUSY is returned when the implementation has
 *   detected an attempt to destroy the object referenced by cond while
 *   it is referenced. EINVAL is returned when cond is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_cond_destroy(FAR pthread_cond_t *cond)
{
  int ret = OK;
  int sval = 0;

  sinfo("cond=%p\n", cond);

  if (!cond)
    {
      ret = EINVAL;
    }

  /* Destroy the semaphore contained in the structure */

  else
    {
      ret = sem_getvalue(&cond->sem, &sval);
      if (ret < 0)
        {
          ret = -ret;
        }
      else
        {
          if (sval < 0)
            {
              ret = EBUSY;
            }
          else if (sem_destroy(&cond->sem) != OK)
            {
              ret = get_errno();
            }
        }
    }

  sinfo("Returning %d\n", ret);
  return ret;
}
