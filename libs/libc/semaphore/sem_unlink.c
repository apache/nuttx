/****************************************************************************
 * libs/libc/semaphore/sem_unlink.c
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

#include <nuttx/semaphore.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sem_unlink
 *
 * Description:
 *   This function removes the semaphore named by the input parameter 'name.'
 *   If the semaphore named by 'name' is currently referenced by other task,
 *   the sem_unlink() will have no effect on the state of the semaphore.  If
 *   one or more processes have the semaphore open when sem_unlink() is
 *   called, destruction of the semaphore will be postponed until all
 *   references to the semaphore have been destroyed by calls of sem_close().
 *
 * Input Parameters:
 *   name - Semaphore name
 *
 * Returned Value:
 *  0 (OK), or -1 (ERROR) if unsuccessful.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sem_unlink(FAR const char *name)
{
  int ret;

  ret = nxsem_unlink(name);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return ret;
}
