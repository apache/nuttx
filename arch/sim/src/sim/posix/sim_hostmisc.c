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

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <spawn.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <sys/wait.h>
#include <sys/types.h>

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
 * Name: host_system
 *
 * Description:
 *   Execute the command and get the result.
 *
 * Input Parameters:
 *   buf - return massage, which return info will be stored
 *   len - buf length
 *   fmt - the format of parameters
 *   ... - variable parameters.
 *
 * Returned Value:
 *   A nonnegative integer is returned on success.  Otherwise,
 *   a negated errno value is returned to indicate the nature of the failure.
 ****************************************************************************/

int host_system(char *buf, size_t len, const char *fmt, ...)
{
  FILE *fp;
  int ret;
  uint64_t flags;
  char cmd[512];
  va_list vars;

  va_start(vars, fmt);
  ret = vsnprintf(cmd, sizeof(cmd), fmt, vars);
  va_end(vars);
  if (ret <= 0 || ret > sizeof(cmd))
    {
      return ret < 0 ? -errno : -EINVAL;
    }

  if (buf == NULL)
    {
      ret = system(cmd);
    }
  else
    {
      flags = up_irq_save();
      fp = popen(cmd, "r");
      if (fp == NULL)
        {
          up_irq_restore(flags);
          return -errno;
        }

      ret = fread(buf, sizeof(char), len, fp);
      pclose(fp);
      up_irq_restore(flags);
    }

  return ret < 0 ? -errno : ret;
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

/****************************************************************************
 * Name: host_posix_spawn
 ****************************************************************************/

pid_t host_posix_spawn(const char *path,
                       char *const argv[], char *const envp[])
{
  int ret;
  pid_t pid;
  char *default_argv[] =
  {
    NULL
  };

  if (!argv)
    {
      argv = default_argv;
    }

  ret = posix_spawn(&pid, path, NULL, NULL, argv, envp);
  return ret > 0 ? -ret : pid;
}

/****************************************************************************
 * Name: host_wait
 ****************************************************************************/

int host_waitpid(pid_t pid)
{
  int status;

  pid = waitpid(pid, &status, 0);
  return pid < 0 ? -errno : status;
}
