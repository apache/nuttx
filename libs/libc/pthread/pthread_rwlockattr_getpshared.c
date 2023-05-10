/****************************************************************************
 * libs/libc/pthread/pthread_rwlockattr_getpshared.c
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
 * Name: pthread_rwlockattr_getpshared
 *
 * Description:
 *   The pthread_rwlockattr_getpshared() function will obtain the value
 *   of the process-shared attribute from the attributes object referenced
 *   by attr.
 *
 * Input Parameters:
 *   attr - rwlock attributes to be queried.
 *   pshared - the location to stored the current value of the
 *   pshared attribute.
 *
 * Returned Value:
 *   0 (OK) on success or EINVAL if either attr or pshared is invalid.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_rwlockattr_getpshared(FAR const pthread_rwlockattr_t *attr,
                                  FAR int *pshared)
{
  int ret = OK;

  if (!attr || !pshared)
    {
      ret = EINVAL;
    }
  else
    {
      *pshared = attr->pshared;
    }

  return ret;
}
