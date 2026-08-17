/****************************************************************************
 * libs/libc/sched/task_startup.c
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

#include <sched.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <nuttx/debug.h>
#include <nuttx/sched.h>
#include <nuttx/tls.h>

#ifdef CONFIG_BINFMT_LOADABLE
#  include <nuttx/binfmt/binfmt.h>
#endif

#include "libc.h"

#ifndef CONFIG_BUILD_KERNEL

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_BINFMT_LOADABLE) && defined(CONFIG_LIBC_ELF)
static void nxtask_binfmt_initialize(void)
{
  FAR struct task_info_s *info = task_get_info();
  FAR struct binary_s *binp;
  FAR void (**array)(void);
  uint16_t ninit;
  int i;

  binp = info == NULL ? NULL : info->ta_bininfo;
  if (binp == NULL || binp->mod.initarr == 0 || binp->mod.ninit == 0)
    {
      return;
    }

  array = (FAR void (**)(void))binp->mod.initarr;
  ninit = binp->mod.ninit;
  binp->mod.ninit = 0;

  for (i = 0; i < ninit; i++)
    {
      array[i]();
    }
}

static void nxtask_binfmt_finalize(void)
{
  FAR struct task_info_s *info = task_get_info();
  FAR struct binary_s *binp;
  FAR void (**array)(void);
  uint16_t nfini;
  int i;

  binp = info == NULL ? NULL : info->ta_bininfo;
  if (binp == NULL || binp->mod.finiarr == 0 || binp->mod.nfini == 0)
    {
      return;
    }

  array = (FAR void (**)(void))binp->mod.finiarr;
  nfini = binp->mod.nfini;
  binp->mod.nfini = 0;

  for (i = nfini - 1; i >= 0; i--)
    {
      array[i]();
    }
}

#if CONFIG_LIBC_MAX_EXITFUNS > 0
static bool nxtask_binfmt_has_fini_array(void)
{
  FAR struct task_info_s *info = task_get_info();
  FAR struct binary_s *binp;

  binp = info == NULL ? NULL : info->ta_bininfo;
  return binp != NULL && binp->mod.finiarr != 0 && binp->mod.nfini > 0;
}
#endif
#else
#  define nxtask_binfmt_initialize()
#  define nxtask_binfmt_finalize()
#  define nxtask_binfmt_has_fini_array() false
#endif

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
  int ret;

  DEBUGASSERT(entrypt);

  /* If C++ initialization for static constructors is supported, then do
   * that first
   */

  lib_cxx_initialize();
  nxtask_binfmt_initialize();

#if defined(CONFIG_BINFMT_LOADABLE) && defined(CONFIG_LIBC_ELF) && \
    CONFIG_LIBC_MAX_EXITFUNS > 0
  if (nxtask_binfmt_has_fini_array())
    {
      atexit(nxtask_binfmt_finalize);
    }
#endif

  /* Call the 'main' entry point passing argc and argv, calling exit()
   * if/when the task returns.
   */

  ret = entrypt(argc, argv);
  nxtask_binfmt_finalize();

  exit(ret);
}

#endif /* !CONFIG_BUILD_KERNEL */
