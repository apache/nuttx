/****************************************************************************
 * binfmt/binfmt_exec.c
 *
 *   Copyright (C) 2009, 2013-2014 Gregory Nutt. All rights reserved.
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
#include <nuttx/binfmt/binfmt.h>

#include "binfmt_internal.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exec
 *
 * Description:
 *   This is a convenience function that wraps load_ and exec_module into
 *   one call.  If CONFIG_SCHED_ONEXIT and CONFIG_SCHED_HAVE_PARENT are
 *   also defined, this function will automatically call schedule_unload()
 *   to unload the module when task exits.
 *
 *   NOTE: This function is flawed and useless without CONFIG_SCHED_ONEXIT
 *   and CONFIG_SCHED_HAVE_PARENT because there is then no mechanism to
 *   unload the module once it exits.
 *
 * Input Parameter:
 *   filename - Full path to the binary to be loaded
 *   argv     - Argument list
 *   exports  - Table of exported symbols
 *   nexports - The number of symbols in exports
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
#if defined(CONFIG_SCHED_ONEXIT) && defined(CONFIG_SCHED_HAVE_PARENT)
  FAR struct binary_s *bin;
  int pid;
  int err;
  int ret;

  /* Allocate the load information */

  bin = (FAR struct binary_s *)kmm_zalloc(sizeof(struct binary_s));
  if (!bin)
    {
      bdbg("ERROR: Failed to allocate binary_s\n");
      err = ENOMEM;
      goto errout;
    }

  /* Load the module into memory */

  bin->filename = filename;
  bin->argv     = argv;
  bin->exports  = exports;
  bin->nexports = nexports;

  ret = load_module(bin);
  if (ret < 0)
    {
      err = get_errno();
      bdbg("ERROR: Failed to load program '%s': %d\n", filename, err);
      goto errout_with_bin;
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
      err = get_errno();
      bdbg("ERROR: Failed to execute program '%s': %d\n", filename, err);
      goto errout_with_lock;
    }

  /* Set up to unload the module (and free the binary_s structure)
   * when the task exists.
   */

  ret = schedule_unload(pid, bin);
  if (ret < 0)
    {
      err = get_errno();
      bdbg("ERROR: Failed to schedule unload '%s': %d\n", filename, err);
    }

  sched_unlock();
  return pid;

errout_with_lock:
  sched_unlock();
  unload_module(bin);
errout_with_bin:
  kmm_free(bin);
errout:
  set_errno(err);
  return ERROR;

#else
  struct binary_s bin;
  int err;
  int ret;

  /* Load the module into memory */

  memset(&bin, 0, sizeof(struct binary_s));
  bin.filename = filename;
  bin.exports  = exports;
  bin.nexports = nexports;

  ret = load_module(&bin);
  if (ret < 0)
    {
      err = get_errno();
      bdbg("ERROR: Failed to load program '%s': %d\n", filename, err);
      goto errout;
    }

  /* Then start the module */

  ret = exec_module(&bin);
  if (ret < 0)
    {
      err = get_errno();
      bdbg("ERROR: Failed to execute program '%s': %d\n", filename, err);
      goto errout_with_module;
    }

  /* TODO:  How does the module get unloaded in this case? */

  return ret;

errout_with_module:
  unload_module(&bin);
errout:
  set_errno(err);
  return ERROR;
#endif
}

#endif /* !CONFIG_BINFMT_DISABLE */

