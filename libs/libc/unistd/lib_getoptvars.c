/****************************************************************************
 * libs/libc/unistd/lib_getoptvars.c
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

#include "unistd.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
/* Data is naturally process-specific in the KERNEL build so no special
 * access to process-specific global data is needed.
 */

FAR struct getopt_s g_getopt_vars =
{
  NULL,
  0,
  1,
  '?',
  NULL,
  false
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getoptvars
 *
 * Description:
 *   Returns a pointer to to the thread-specific getopt() data.
 *
 ****************************************************************************/

FAR struct getopt_s *getoptvars(void)
{
#ifndef CONFIG_BUILD_KERNEL
  FAR struct task_info_s *info;

  /* Get the structure of getopt() variables using the key. */

  info = task_get_info();
  DEBUGASSERT(info != NULL);
  return &info->ta_getopt;
#else
  return &g_getopt_vars;
#endif
}
