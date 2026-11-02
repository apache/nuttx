/****************************************************************************
 * libs/libc/unistd/lib_execvp.c
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

#include <errno.h>
#include <paths.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/envpath.h>

#include "libc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The shell that runs a file that is not a program.  With
 * CONFIG_LIBC_ENVPATH, PATH gives the location of "sh" (as for popen()).
 */

#ifdef CONFIG_LIBC_ENVPATH
#  define EXECVP_SHELL "sh"
#else
#  define EXECVP_SHELL _PATH_BSHELL
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: execvp_shell
 *
 * Description:
 *   Run file with the shell after execve() refused it with ENOEXEC (for
 *   example a script without "#!"), as POSIX specifies for execvp().
 *
 * Returned Value:
 *   This function does not return on success.  On failure, it returns -1
 *   (ERROR) and sets errno.
 *
 ****************************************************************************/

static int execvp_shell(FAR const char *file, FAR char * const argv[],
                        FAR char * const envp[])
{
  FAR char *fullpath = NULL;
  FAR const char **shargv;
  int argc;
  int ret;
  int i;

#ifdef CONFIG_LIBC_ENVPATH
  /* execve() found a bare file name in PATH: give the shell that path */

  if (strchr(file, '/') == NULL)
    {
      ENVPATH_HANDLE handle = envpath_init("PATH");

      if (handle != NULL)
        {
          fullpath = envpath_next(handle, file);
          envpath_release(handle);
        }

      if (fullpath == NULL)
        {
          set_errno(ENOEXEC);
          return ERROR;
        }

      file = fullpath;
    }
#endif

  /* The arguments of the shell: its name, the file, then argv[1] and the
   * arguments after it.
   */

  for (argc = 0; argv[argc] != NULL; argc++)
    {
    }

  if (argc == 0)
    {
      argc = 1;
    }

  shargv = lib_malloc((argc + 2) * sizeof(FAR const char *));
  if (shargv == NULL)
    {
      lib_free(fullpath);
      set_errno(ENOMEM);
      return ERROR;
    }

  shargv[0] = EXECVP_SHELL;
  shargv[1] = file;
  for (i = 1; i < argc; i++)
    {
      shargv[i + 1] = argv[i];
    }

  shargv[argc + 1] = NULL;

  /* execve() takes the strings as char *, but does not change them */

  ret = execve(EXECVP_SHELL, (FAR char * const *)shargv, envp);

  /* The shell did not start: free the memory, keep the errno of execve() */

  lib_free(shargv);
  lib_free(fullpath);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: execvp
 *
 * Description:
 *   The same as execv(), except that a file that execv() refuses with
 *   ENOEXEC is run with the shell.  NuttX has only one PATH behavior: with
 *   CONFIG_LIBC_ENVPATH, execv() also searches the PATH for file.
 *
 ****************************************************************************/

int execvp(FAR const char *file, FAR char * const argv[])
{
  return execvpe(file, argv, NULL);
}

/****************************************************************************
 * Name: execvpe
 *
 * Description:
 *   The same as execve(), except that a file that execve() refuses with
 *   ENOEXEC is run with the shell.  NuttX has only one PATH behavior: with
 *   CONFIG_LIBC_ENVPATH, execve() also searches the PATH for file.
 *
 ****************************************************************************/

int execvpe(FAR const char *file, FAR char * const argv[],
            FAR char * const envp[])
{
  int ret;

  ret = execve(file, argv, envp);
  if (ret < 0 && get_errno() == ENOEXEC)
    {
      ret = execvp_shell(file, argv, envp);
    }

  return ret;
}
