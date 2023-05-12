/****************************************************************************
 * arch/sim/src/sim/posix/sim_hostmisc.c
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

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "sim_internal.h"

#ifdef CONFIG_HOST_MACOS
#include <sys/syslimits.h>
#include <mach-o/dyld.h>
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ARCH_COVERAGE
void __gcov_dump(void);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern uint64_t up_irq_save(void);
extern void up_irq_restore(uint64_t flags);
extern int backtrace(void **array, int size);

/****************************************************************************
 * Name: host_abort
 *
 * Description:
 *   Abort the simulation
 *
 * Input Parameters:
 *   status - Exit status to set
 ****************************************************************************/

void host_abort(int status)
{
  uint64_t flags = up_irq_save();

#ifdef CONFIG_ARCH_COVERAGE
  /* Dump gcov data. */

  __gcov_dump();
#endif

  /* exit the simulation */

  exit(status);

  up_irq_restore(flags);
}

/****************************************************************************
 * Name: host_backtrace
 *
 * Description:
 *   bcaktrace
 *
 * Input Parameters:
 *   array - return array, which backtrace will be stored
 *   size  - array size
 ****************************************************************************/

int host_backtrace(void** array, int size)
{
#ifdef CONFIG_WINDOWS_CYGWIN
  return 0;
#else
  uint64_t flags = up_irq_save();
  int ret;

  ret = backtrace(array, size);

  up_irq_restore(flags);
  return ret;
#endif
}

/****************************************************************************
 * Name: host_init_cwd
 ****************************************************************************/

#ifdef CONFIG_SIM_IMAGEPATH_AS_CWD
void host_init_cwd(void)
{
  char *name;
  char path[PATH_MAX];
  int len = PATH_MAX;

  /* Get the absolute path of the executable file */

#  ifdef CONFIG_HOST_LINUX
  len = readlink("/proc/self/exe", path, len);
  if (len < 0)
    {
      perror("readlink  failed");
      return;
    }
#  else
  if (_NSGetExecutablePath(path, &len) < 0)
    {
      perror("_NSGetExecutablePath failed");
      return;
    }
#  endif

  path[len] = '\0';
  name = strrchr(path, '/');
  *++name = '\0';
  chdir(path);
}
#endif
