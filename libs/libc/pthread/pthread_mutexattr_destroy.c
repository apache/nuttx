/****************************************************************************
 * libs/libc/pthread/pthread_mutexattr_destroy.c
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
 * Name:  pthread_mutexattr_destroy
 *
 * Description:
 *    Destroy mutex attributes.
 *
 * Input Parameters:
 *    attr
 *
 * Returned Value:
 *   0 if successful.  Otherwise, an error code.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_mutexattr_destroy(FAR pthread_mutexattr_t *attr)
{
  int ret = OK;

  linfo("attr=%p\n", attr);

  if (!attr)
    {
      ret = EINVAL;
    }
  else
    {
      attr->pshared = 0;
    }

  linfo("Returning %d\n", ret);
  return ret;
}
