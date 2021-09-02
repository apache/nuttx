/****************************************************************************
 * libs/libc/unistd/lib_execle.c
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

#include <stdarg.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include "libc.h"

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
 * Name: execle
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
 *   provided to 'exec()' from 'exec[l|v]()' via NuttX configuration setting:
 *
 *     CONFIG_LIBC_EXECFUNCS         : Enable exec[l|v] support
 *     CONFIG_EXECFUNCS_SYMTAB_ARRAY : Symbol table name used by exec[l|v]
 *     CONFIG_EXECFUNCS_NSYMBOLS_VAR : Variable holding number of symbols in
 *                                     the table
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
 *   path - The path to the program to be executed.  If CONFIG_LIBC_ENVPATH
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

int execle(FAR const char *path, FAR const char *arg0, ...)
{
  FAR char *arg = (FAR char *)arg0;
  FAR char **argv;
  FAR char **envp;
  size_t nargs;
  va_list ap;
  int argc;
  int ret;

  /* Count the number of arguments */

  va_start(ap, arg0);
  nargs = 0;

  while (arg != NULL)
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

      arg = va_arg(ap, FAR char *);
    }

  envp = va_arg(ap, FAR char **);
  va_end(ap);

  /* Allocate a temporary argv[] array */

  argv = (FAR char **)lib_malloc((nargs + 1) * sizeof(FAR char *));
  if (argv == NULL)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  argv[0] = (FAR char *)arg0;

  /* Collect the arguments into the argv[] array */

  va_start(ap, arg0);
  for (argc = 1; argc <= nargs; argc++)
    {
      argv[argc] = va_arg(ap, FAR char *);
    }

  va_end(ap);

  /* Then let execve() do the real work */

  ret = execve(path, argv, envp);

  /* Free the allocated argv[] list */

  lib_free(argv);
  return ret;
}

#endif /* CONFIG_LIBC_EXECFUNCS */
