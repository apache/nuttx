/****************************************************************************
 * libs/libc/pthread/pthread_cleanup.c
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
#include <nuttx/tls.h>
#include <nuttx/pthread.h>

#include <pthread.h>
#include <sched.h>
#include <assert.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_cleanup_push
 *
 * Description:
 *   The pthread_cleanup_push() function will push the specified cancellation
 *   cleanup handler routine onto the calling thread's cancellation cleanup
 *   stack. The cancellation cleanup handler will be popped from the
 *   cancellation cleanup stack and invoked with the argument arg when:
 *
 *   - The thread exits (that is, calls pthread_exit()).
 *   - The thread acts upon a cancellation request.
 *   - The thread calls pthread_cleanup_pop() with non-zero execute argument.
 *
 * Input Parameters:
 *   routine - The cleanup routine to be pushed on the cleanup stack.
 *   arg     - An argument that will accompany the callback.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pthread_cleanup_push(pthread_cleanup_t routine, FAR void *arg)
{
  tls_cleanup_push(tls_get_info(), routine, arg);
}

/****************************************************************************
 * Name: pthread_cleanup_pop
 *
 * Description:
 *   The pthread_cleanup_pop() function will remove the routine at the top
 *   of the calling thread's cancellation cleanup stack and optionally
 *   invoke it (if 'execute' is non-zero).
 *
 *   - The thread exits (that is, calls pthread_exit()).
 *   - The thread acts upon a cancellation request.
 *   - The thread calls pthread_cleanup_pop() with non-zero execute argument.
 *
 * Input Parameters:
 *   execute - Execute the popped cleanup function immediately.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pthread_cleanup_pop(int execute)
{
  tls_cleanup_pop(tls_get_info(), execute);
}
