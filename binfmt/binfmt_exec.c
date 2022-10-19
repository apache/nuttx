/****************************************************************************
 * binfmt/binfmt_exec.c
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

#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/binfmt/binfmt.h>

#include "binfmt.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exec_spawn
 *
 * Description:
 *   exec() configurable version, delivery the spawn attribute if this
 *   process has special customization.
 *
 * Input Parameters:
 *   filename - The path to the program to be executed. If
 *              CONFIG_LIBC_ENVPATH is defined in the configuration, then
 *              this may be a relative path from the current working
 *              directory. Otherwise, path must be the absolute path to the
 *              program.
 *   argv     - A pointer to an array of string arguments. The end of the
 *              array is indicated with a NULL entry.
 *   envp     - A pointer to an array of environment strings. Terminated with
 *              a NULL entry.
 *   exports  - The address of the start of the caller-provided symbol
 *              table. This symbol table contains the addresses of symbols
 *              exported by the caller and made available for linking the
 *              module into the system.
 *   nexports - The number of symbols in the exports table.
 *   attr     - The spawn attributes.
 *
 * Returned Value:
 *   It returns the PID of the exec'ed module.  On failure, it returns
 *   the negative errno value appropriately.
 *
 ****************************************************************************/

int exec_spawn(FAR const char *filename, FAR char * const *argv,
               FAR char * const *envp, FAR const struct symtab_s *exports,
               int nexports, FAR const posix_spawnattr_t *attr)
{
  FAR struct binary_s *bin;
  int pid;
  int ret;

  /* Allocate the load information */

  bin = (FAR struct binary_s *)kmm_zalloc(sizeof(struct binary_s));
  if (!bin)
    {
      berr("ERROR: Failed to allocate binary_s\n");
      ret = -ENOMEM;
      goto errout;
    }

  /* Load the module into memory */

  ret = load_module(bin, filename, exports, nexports);
  if (ret < 0)
    {
      berr("ERROR: Failed to load program '%s': %d\n", filename, ret);
      goto errout_with_bin;
    }

  /* Update the spawn attribute */

  if (attr)
    {
      if (attr->priority > 0)
        {
          bin->priority = attr->priority;
        }

      if (attr->stacksize > 0)
        {
          bin->stacksize = attr->stacksize;
        }
    }

  /* Disable pre-emption so that the executed module does
   * not return until we get a chance to connect the on_exit
   * handler.
   */

  sched_lock();

  /* Then start the module */

  pid = exec_module(bin, filename, argv, envp);
  if (pid < 0)
    {
      ret = pid;
      berr("ERROR: Failed to execute program '%s': %d\n",
           filename, ret);
      goto errout_with_lock;
    }

#ifdef CONFIG_BINFMT_LOADABLE
  /* Set up to unload the module (and free the binary_s structure)
   * when the task exists.
   */

  ret = group_exitinfo(pid, bin);
  if (ret < 0)
    {
      berr("ERROR: Failed to schedule unload '%s': %d\n", filename, ret);
    }

#else
  /* Free the binary_s structure here */

  kmm_free(bin);

  /* TODO: How does the module get unloaded in this case? */

#endif

  sched_unlock();
  return pid;

errout_with_lock:
  sched_unlock();
  unload_module(bin);
errout_with_bin:
  kmm_free(bin);
errout:
  return ret;
}

/****************************************************************************
 * Name: exec
 *
 * Description:
 *   This is a convenience function that wraps load_ and exec_module into
 *   one call.  If CONFIG_BINFMT_LOADABLE is defined, this function will
 *   schedule to unload the module when task exits.
 *
 *   This non-standard, NuttX function is similar to execv() and
 *   posix_spawn() but differs in the following ways;
 *
 *   - Unlike execv() and posix_spawn() this function accepts symbol table
 *     information as input parameters. This means that the symbol table
 *     used to link the application prior to execution is provided by the
 *     caller, not by the system.
 *   - Unlike execv(), this function always returns.
 *
 *   This non-standard interface is included as a official NuttX API only
 *   because it is needed in certain build modes: exec() is probably the
 *   only way to load programs in the PROTECTED mode. Other file execution
 *   APIs rely on a symbol table provided by the OS. In the PROTECTED build
 *   mode, the OS cannot provide any meaningful symbolic information for
 *   execution of code in the user-space blob so that is the exec() function
 *   is really needed in that build case
 *
 *   The interface is available in the FLAT build mode although it is not
 *   really necessary in that case. It is currently used by some example
 *   code under the apps/ that that generate their own symbol tables for
 *   linking test programs. So although it is not necessary, it can still
 *   be useful.
 *
 *   The interface would be completely useless and will not be supported in
 *   in the KERNEL build mode where the contrary is true: An application
 *   process cannot provide any meaningful symbolic information for use in
 *   linking a different process.
 *
 *   NOTE: This function is flawed and useless without CONFIG_BINFMT_LOADABLE
 *   because without that features there is then no mechanism to unload the
 *   module once it exits.
 *
 * Input Parameters:
 *   filename - The path to the program to be executed. If
 *              CONFIG_LIBC_ENVPATH is defined in the configuration, then
 *              this may be a relative path from the current working
 *              directory. Otherwise, path must be the absolute path to the
 *              program.
 *   argv     - A pointer to an array of string arguments. The end of the
 *              array is indicated with a NULL entry.
 *   envp     - An array of character pointers to null-terminated strings
 *              that provide the environment for the new process image.
 *              The environment array is terminated by a null pointer.
 *   exports  - The address of the start of the caller-provided symbol
 *              table. This symbol table contains the addresses of symbols
 *              exported by the caller and made available for linking the
 *              module into the system.
 *   nexports - The number of symbols in the exports table.
 *
 * Returned Value:
 *   This is an end-user function, so it follows the normal convention:
 *   It returns the PID of the exec'ed module.  On failure, it returns
 *   -1 (ERROR) and sets errno appropriately.
 *
 ****************************************************************************/

int exec(FAR const char *filename, FAR char * const *argv,
         FAR char * const *envp, FAR const struct symtab_s *exports,
         int nexports)
{
  int ret;

  ret = exec_spawn(filename, argv, envp, exports, nexports, NULL);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

#endif /* !CONFIG_BINFMT_DISABLE */
