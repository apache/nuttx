/****************************************************************************
 * libs/libc/pthread/pthread_concurrency.c
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

#include <pthread.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

static int g_pthread_concurrency_level = 0;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_setconcurrency
 *
 * Description:
 *   The pthread_setconcurrency() function informs the implementation of
 *   the application's desired concurrency level.
 *
 *   NuttX uses 1:1 threading model, so this function has no real effect
 *   on the scheduling behavior.
 *
 * Input Parameters:
 *  new_level - desired concurrency level
 *
 * Returned Value:
 *  Returns 0 on success, on error it returns a nonzero error number
 *
 ****************************************************************************/

int pthread_setconcurrency(int new_level)
{
  if (new_level < 0)
    {
      return EINVAL;
    }

  g_pthread_concurrency_level = new_level;

  return OK;
}

/****************************************************************************
 * Name: pthread_getconcurrency
 *
 * Description:
 *   The pthread_getconcurrency() returns the value previously set by
 *   pthread_setconcurrency().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Returns the current value of concurrency level.
 *
 ****************************************************************************/

int pthread_getconcurrency(void)
{
  return g_pthread_concurrency_level;
}
