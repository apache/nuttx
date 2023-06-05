/****************************************************************************
 * libs/libc/pthread/pthread_attr_getscope.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_attr_getscope
 *
 * Description:
 *   The function returns the contention scope attribute of the thread
 *   attributes object referred to by attr in the buffer pointed to
 *   by scope.
 *
 * Input Parameters:
 *   attr  - The pointer to pthread attr.
 *   scope - The pointer to scope
 *
 * Returned Value:
 *   On success, these functions return 0; on error, they return a nonzero
 *   error number.
 *
 ****************************************************************************/

int pthread_attr_getscope(FAR const pthread_attr_t *attr, FAR int *scope)
{
  *scope = PTHREAD_SCOPE_SYSTEM;
  return 0;
}
