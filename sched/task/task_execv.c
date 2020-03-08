/****************************************************************************
 * sched/task/lib_execv.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/binfmt/binfmt.h>
#include <nuttx/binfmt/symtab.h>

#ifdef CONFIG_LIBC_EXECFUNCS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: execv
 *
 * Description:
 *   The standard 'exec' family of functions will replace the current process
 *   image with a new process image. The new image will be constructed from a
 *   regular, executable file called the new process image file. There will
 *   be no return from a successful exec, because the calling process image
 *   is overlaid by the new process image.
 *
 *   Simplified 'execl()' and 'execv()' functions are provided by NuttX for
 *   compatibility.  NuttX is a tiny embedded RTOS that does not support
 *   processes and hence the concept of overlaying a tasks process image with
 *   a new process image does not make any sense.  In NuttX, these functions
 *   are wrapper functions that:
 *
 *     1. Call the non-standard binfmt function 'exec', and then
 *     2. exit(0).
 *
 *   Note the inefficiency when 'exec[l|v]()' is called in the normal, two-
 *   step process:  (1) first call vfork() to create a new thread, then (2)
 *   call 'exec[l|v]()' to replace the new thread with a program from the
 *   file system.  Since the new thread will be terminated by the
 *   'exec[l|v]()' call, it really served no purpose other than to support
 *   Unix compatility.
 *
 *   The non-standard binfmt function 'exec()' needs to have (1) a symbol
 *   table that provides the list of symbols exported by the base code, and
 *   (2) the number of symbols in that table.  This information is currently
 *   provided to 'exec()' from 'exec[l|v]()' via NuttX configuration
 *   settings:
 *
 *     CONFIG_LIBC_EXECFUNCS          : Enable exec[l|v] support
 *     CONFIG_EXECFUNCS_HAVE_SYMTAB   : Defined if there is a pre-defined
 *                                      symbol table
 *     CONFIG_EXECFUNCS_SYMTAB_ARRAY  : Symbol table name used by exec[l|v]
 *     CONFIG_EXECFUNCS_NSYMBOLS_VAR  : Variable holding number of symbols
 *                                      in the table
 *
 *   As a result of the above, the current implementations of 'execl()' and
 *   'execv()' suffer from some incompatibilities that may or may not be
 *   addressed in a future version of NuttX.  Other than just being an
 *   inefficient use of MCU resource, the most serious of these is that
 *   the exec'ed task will not have the same task ID as the vfork'ed
 *   function.  So the parent function cannot know the ID of the exec'ed
 *   task.
 *
 * Input Parameters:
 *   path - The path to the program to be executed.  If CONFIG_LIB_ENVPATH
 *     is defined in the configuration, then this may be a relative path
 *     from the current working directory.  Otherwise, path must be the
 *     absolute path to the program.
 *   argv - A pointer to an array of string arguments.  The end of the
 *     array is indicated with a NULL entry.
 *
 * Returned Value:
 *   This function does not return on success.  On failure, it will return
 *   -1 (ERROR) and will set the 'errno' value appropriately.
 *
 ****************************************************************************/

int execv(FAR const char *path, FAR char * const argv[])
{
  FAR const struct symtab_s *symtab;
  int nsymbols;
  int ret;

  /* Get the current symbol table selection */

  exec_getsymtab(&symtab, &nsymbols);

  /* Start the task */

  ret = exec(path, (FAR char * const *)argv, symtab, nsymbols);
  if (ret < 0)
    {
      serr("ERROR: exec failed: %d\n", errno);
      return ERROR;
    }

  /* Then exit */

  exit(0);

  /* We should not get here, but might be needed by some compilers.  Other,
   * smarter compilers might complain that this code is unreachable.  You
   * just can't win.
   */

  return ERROR;
}

#endif /* CONFIG_LIBC_EXECFUNCS */
