/****************************************************************************
 * binfmt/binfmt_exec.c
 *
 *   Copyright (C) 2009, 2013-2014, 2017-2018 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 *   only want to load programs in the PROTECTED mode. Other file execution
 *   APIs rely on a symbol table provided by the OS. In the PROTECTED build
 *   mode, the OS cannot provide any meaningful symbolic information for
 *   execution of code in the user-space blob so that is the exec() function
 *   is really needed in that build case
 *
 *   The interface is available in the FLAT build mode although it is not
 *   really necessary in that case. It is currently used by some example
 *   code under the apps/ that that generate their own symbol tables for
 *   linking test programs. So althought it is not necessary, it can still
 *   be useful.
 *
 *   The interface would be completely useless and will not be supported in
 *   in the KERNEL build mode where the contrary is true: An application
 *   process cannot provide any meaning symbolic information for use in
 *   linking a different process.
 *
 *   NOTE: This function is flawed and useless without CONFIG_BINFMT_LOADABLE
 *   because without that features there is then no mechanism to unload the
 *   module once it exits.
 *
 * Input Parameters:
 *   filename - The path to the program to be executed. If
 *              CONFIG_BINFMT_EXEPATH is defined in the configuration, then
 *              this may be a relative path from the current working
 *              directory. Otherwise, path must be the absolute path to the
 *              program.
 *   argv     - A pointer to an array of string arguments. The end of the
 *              array is indicated with a NULL entry.
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
         FAR const struct symtab_s *exports, int nexports)
{
  FAR struct binary_s *bin;
  int pid;
  int errcode;
  int ret;

  /* Allocate the load information */

  bin = (FAR struct binary_s *)kmm_zalloc(sizeof(struct binary_s));
  if (!bin)
    {
      berr("ERROR: Failed to allocate binary_s\n");
      errcode = ENOMEM;
      goto errout;
    }

  /* Initialize the binary structure */

  bin->filename = filename;
  bin->exports  = exports;
  bin->nexports = nexports;

  /* Copy the argv[] list */

  ret = binfmt_copyargv(bin, argv);
  if (ret < 0)
    {
      errcode = -ret;
      berr("ERROR: Failed to copy argv[]: %d\n", errcode);
      goto errout_with_bin;
    }

  /* Load the module into memory */

  ret = load_module(bin);
  if (ret < 0)
    {
      errcode = -ret;
      berr("ERROR: Failed to load program '%s': %d\n", filename, errcode);
      goto errout_with_argv;
    }

  /* Disable pre-emption so that the executed module does
   * not return until we get a chance to connect the on_exit
   * handler.
   */

  sched_lock();

  /* Then start the module */

  pid = exec_module(bin);
  if (pid < 0)
    {
      errcode = -pid;
      berr("ERROR: Failed to execute program '%s': %d\n",
           filename, errcode);
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

  binfmt_freeargv(bin);
  kmm_free(bin);

  /* TODO: How does the module get unloaded in this case? */
#endif

  sched_unlock();
  return pid;

errout_with_lock:
  sched_unlock();
  (void)unload_module(bin);
errout_with_argv:
  binfmt_freeargv(bin);
errout_with_bin:
  kmm_free(bin);
errout:
  set_errno(errcode);
  return ERROR;

}

#endif /* !CONFIG_BINFMT_DISABLE */
