/****************************************************************************
 * libs/libc/pthread/pthread_exit.c
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

#include <debug.h>
#include <sched.h>

#include <nuttx/pthread.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pthread_exit
 *
 * Description:
 *   Terminate execution of a thread started with pthread_create.
 *
 * Input Parameters:
 *   exit_valie
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void pthread_exit(FAR void *exit_value)
{
#ifdef CONFIG_PTHREAD_CLEANUP
  int cnt;
  struct pthread_cleanup_s cleanup[CONFIG_PTHREAD_CLEANUP_STACKSIZE];
  cnt = pthread_cleanup_poplist(cleanup);

  sched_lock();
  while (cnt-- > 0)
    {
      struct pthread_cleanup_s cp = cleanup[cnt];
      if (cp.pc_cleaner)
        cp.pc_cleaner(cp.pc_arg);
    }

  sched_unlock();
#endif

  nx_pthread_exit(exit_value);
  PANIC();
}
