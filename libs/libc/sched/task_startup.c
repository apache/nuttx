/****************************************************************************
 * libs/libc/sched/task_startup.c
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

#include <sched.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include "libc.h"

#ifndef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxtask_startup
 *
 * Description:
 *   This function is the user-space, task startup function.  It is called
 *   from up_task_start() in user-mode.
 *
 * Input Parameters:
 *   entrypt - The user-space address of the task entry point
 *   argc and argv - Standard arguments for the task entry point
 *
 * Returned Value:
 *   None.  This function does not return.
 *
 ****************************************************************************/

void nxtask_startup(main_t entrypt, int argc, FAR char *argv[])
{
  DEBUGASSERT(entrypt);

  /* If C++ initialization for static constructors is supported, then do
   * that first
   */

  lib_cxx_initialize();

  /* Call the 'main' entry point passing argc and argv, calling exit()
   * if/when the task returns.
   */

  exit(entrypt(argc, argv));
}

#endif /* !CONFIG_BUILD_KERNEL */
