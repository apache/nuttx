/****************************************************************************
 * libs/libc/semaphore/sem_setmaxvalue.c
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
#include <nuttx/trace.h>

#include <nuttx/semaphore.h>

#ifdef CONFIG_CUSTOM_SEMAPHORE_MAXVALUE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsem_setmaxvalue
 *
 * Description:
 *  Set sem max allowed value
 *
 * Input Parameters:
 *   sem     - Semaphore object
 *   delay   - Max allowed semaphore value.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsem_setmaxvalue(FAR sem_t *sem, int32_t maxvalue)
{
  if (sem != NULL)
    {
      sem->maxvalue = maxvalue;
      return OK;
    }

  return -EINVAL;
}

#endif /* CONFIG_PRIORITY_INHERITANCE */
