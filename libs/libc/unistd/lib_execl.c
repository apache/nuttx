/****************************************************************************
 * libs/libc/unistd/lib_execl.c
 *
 *   Copyright (C) 2013, 2015 Gregory Nutt. All rights reserved.
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

#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#ifdef CONFIG_LIBC_EXECFUNCS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* This is an artificial limit to detect error conditions where an argv[]
 * list is not properly terminated.
 */

#define MAX_EXECL_ARGS 256

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: execl
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
 *   provided to 'exec()' from 'exec[l|v]()' via NuttX configuration settings:
 *
 *     CONFIG_LIBC_EXECFUNCS         : Enable exec[l|v] support
 *     CONFIG_EXECFUNCS_SYMTAB_ARRAY : Symbol table name used by exec[l|v]
 *     CONFIG_EXECFUNCS_NSYMBOLS_VAR : Variable holding number of symbols in the table
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
 *   path - The path to the program to be executed.  If CONFIG_BINFMT_EXEPATH
 *     is defined in the configuration, then this may be a relative path
 *     from the current working directory.  Otherwise, path must be the
 *     absolute path to the program.
 *   ... - A list of the string arguments to be recevied by the
 *     program.  Zero indicates the end of the list.
 *
 * Returned Value:
 *   This function does not return on success.  On failure, it will return
 *   -1 (ERROR) and will set the 'errno' value appropriately.
 *
 ****************************************************************************/

int execl(FAR const char *path, ...)
{
  FAR char **argv = (FAR char **)NULL;
  FAR char *arg;
  size_t nargs;
  va_list ap;
  int argc;
  int ret;

  /* Count the number of arguments */

  va_start(ap, path);
  nargs = 0;
  do
    {
      /* Check if the next argument is present */

      arg = va_arg(ap, FAR char *);
      if (arg)
        {
          /* Yes.. increment the number of arguments.  Here is a sanity
           * check to prevent running away with an unterminated argv[] list.
           * MAX_EXECL_ARGS should be sufficiently large that this never
           * happens in normal usage.
           */

          if (++nargs > MAX_EXECL_ARGS)
            {
              set_errno(E2BIG);
              va_end(ap);
              return ERROR;
            }
        }
    }
  while (arg);

  va_end(ap);

  /* Allocate a temporary argv[] array */

  if (nargs > 0)
    {
      argv = (FAR char **)malloc((nargs + 1) * sizeof(FAR char *));
      if (argv == (FAR char **)NULL)
        {
          set_errno(ENOMEM);
          return ERROR;
        }

      /* Collect the arguments into the argv[] array */

      va_start(ap, path);
      for (argc = 0; argc < nargs; argc++)
        {
          argv[argc] = va_arg(ap, FAR char *);
        }

      argv[nargs] = NULL;
      va_end(ap);
    }

  /* Then let execv() do the real work */

  ret = execv(path, (FAR char * const *)argv);

  /* Free the allocated argv[] list */

  if (argv)
    {
      free(argv);
    }

  return ret;
}

#endif /* CONFIG_LIBC_EXECFUNCS */
