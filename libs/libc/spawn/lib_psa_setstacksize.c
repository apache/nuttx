/****************************************************************************
 * libs/libc/spawn/lib_psa_setstacksize.c
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

#include <sched.h>
#include <spawn.h>
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_spawnattr_setstacksize
 *
 * Description:
 *   The task_spawnattr_setstacksize() function shall set the spawn-
 *   stacksize attribute in an initialized attributes object referenced
 *   by attr.
 *
 * Input Parameters:
 *   attr  - The address spawn attributes to be used.
 *   stacksize - The new stacksize to set.
 *
 * Returned Value:
 *   On success, these functions return 0; on failure they return an error
 *   number from <errno.h>.
 *
 ****************************************************************************/

int task_spawnattr_setstacksize(FAR posix_spawnattr_t *attr,
                                size_t stacksize)
{
  DEBUGASSERT(attr);
  attr->stacksize = stacksize;
  return OK;
}
