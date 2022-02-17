/****************************************************************************
 * libs/libc/pthread/pthread_create.c
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

#include <assert.h>

#include <nuttx/pthread.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_startup
 *
 * Description:
 *   This function is the user space pthread startup function.  Its purpose
 *   is to catch the return from the pthread main function so that
 *   pthread_exit() can be called from user space
 *
 * Input Parameters:
 *   entry - The user-space address of the pthread entry point
 *   arg   - Standard argument for the pthread entry point
 *
 * Returned Value:
 *   None.  This function does not return.
 *
 ****************************************************************************/

static void pthread_startup(pthread_startroutine_t entry,
                            pthread_addr_t arg)
{
  DEBUGASSERT(entry != NULL);

  /* Pass control to the thread entry point.  Handle any returned value. */

  pthread_exit(entry(arg));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_create
 *
 * Description:
 *   This function creates and activates a new thread with specified
 *   attributes.  It is simply a wrapper around the nx_pthread_create system
 *   call.
 *
 * Input Parameters:
 *    thread
 *    attr
 *    pthread_entry
 *    arg
 *
 * Returned Value:
 *   OK (0) on success; a (non-negated) errno value on failure. The errno
 *   variable is not set.
 *
 ****************************************************************************/

int pthread_create(FAR pthread_t *thread, FAR const pthread_attr_t *attr,
                   pthread_startroutine_t pthread_entry, pthread_addr_t arg)
{
  return nx_pthread_create(pthread_startup, thread, attr, pthread_entry,
                           arg);
}
