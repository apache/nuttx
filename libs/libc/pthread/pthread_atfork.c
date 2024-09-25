/****************************************************************************
 * libs/libc/pthread/pthread_atfork.c
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

#include <nuttx/tls.h>
#include <nuttx/lib/lib.h>

#include <pthread.h>
#include <errno.h>
#include <stdlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  pthread_atfork
 *
 * Description:
 *    To register the methods that need to be executed when fork is called
 *    by any thread in a process
 *
 * Input Parameters:
 *   prepare - the method that is executed in the parent process before
 *             fork() processing is started
 *   parent  - the method that is executed in the parent process after fork()
 *             processing completes
 *   child   - the method that is executed in the child process after fork()
 *             processing completes
 *
 * Returned Value:
 *   On success, pthread_atfork() returns 0.
 *   On error, pthread_atfork() returns errno.
 *
 * Assumptions:
 *
 ****************************************************************************/

int pthread_atfork(CODE void (*prepare)(void),
                   CODE void (*parent)(void),
                   CODE void (*child)(void))
{
#ifdef CONFIG_PTHREAD_ATFORK
  FAR struct task_info_s *info = task_get_info();
  FAR struct list_node *list = &info->ta_atfork;
  FAR struct pthread_atfork_s *entry =
                      (FAR struct pthread_atfork_s *)
                      lib_malloc(sizeof(struct pthread_atfork_s));

  if (entry == NULL)
    {
      return ENOMEM;
    }

  list_initialize(&entry->node);
  entry->prepare = prepare;
  entry->parent = parent;
  entry->child = child;

  nxmutex_lock(&info->ta_lock);
  list_add_head(list, &entry->node);
  nxmutex_unlock(&info->ta_lock);
#endif

  return OK;
}
