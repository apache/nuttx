/****************************************************************************
 * libs/libc/pthread/pthread_rwlockattr_init.c
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
#include <errno.h>
#include <debug.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_rwlockattr_init
 *
 * Description:
 *   The pthread_rwlockattr_init() function will initialize a rwlock
 *   attribute object attr with the default value for all of the attributes
 *   defined by the implementation.
 *
 * Input Parameters:
 *   attr - rwlock attributes to be initialized.
 *
 * Returned Value:
 *   0 (OK) on success or EINVAL if attr is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_rwlockattr_init(FAR pthread_rwlockattr_t *attr)
{
  int ret = OK;

  if (!attr)
    {
      ret = EINVAL;
    }
  else
    {
      attr->pshared = PTHREAD_PROCESS_PRIVATE;
    }

  return ret;
}
