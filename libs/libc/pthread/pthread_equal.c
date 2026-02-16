/****************************************************************************
 * libs/libc/pthread/pthread_equal.c
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

#include <pthread.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_equal
 *
 * Description:
 *  Detect whether the two pthreads is equal or not
 *
 * Input Parameters:
 *  t1 - the first pthread to compare
 *  t2 - the another pthread to compare
 *
 * Returned Value:
 *  1 (TRUE) if the two pthreads are equal, 0 (FALSE) otherwise.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_equal(pthread_t t1, pthread_t t2)
{
  return t1 == t2;
}
