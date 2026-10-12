/****************************************************************************
 * binfmt/binfmt_copyargv.c
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

#include <string.h>
#include <nuttx/debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/binfmt.h>

#include "binfmt.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: binfmt_nargs
 *
 * Description:
 *   Count the arguments of the list that binfmt_copyargv() builds: all of
 *   ldargv, then argv without argv[0] if ldargv is not NULL, or all of argv
 *   if it is.
 *
 ****************************************************************************/

static int binfmt_nargs(FAR char * const *ldargv, FAR char * const *argv,
                        FAR int *nld)
{
  int nargs = 0;
  int i;

  *nld = 0;
  if (ldargv != NULL)
    {
      while (ldargv[*nld] != NULL)
        {
          (*nld)++;
        }
    }

  if (argv != NULL)
    {
      for (i = ldargv != NULL && argv[0] != NULL; argv[i] != NULL; i++)
        {
          if (++nargs > CONFIG_TASK_MAX_ARGS)
            {
              berr("ERROR: Too many arguments\n");
              return -E2BIG;
            }
        }
    }

  return *nld + nargs;
}

/****************************************************************************
 * Name: binfmt_arg
 *
 * Description:
 *   Return argument i of the list that binfmt_copyargv() builds.
 *
 ****************************************************************************/

static FAR char *binfmt_arg(FAR char * const *ldargv,
                            FAR char * const *argv, int nld, int i)
{
  if (i < nld)
    {
      return ldargv[i];
    }

  return argv[i - nld + (ldargv != NULL)];
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)

/****************************************************************************
 * Name: binfmt_copyargv
 *
 * Description:
 *   In the kernel build, the argv list will likely lie in the caller's
 *   address environment and, hence, be inaccessible when we switch to the
 *   address environment of the new process address environment.  So we
 *   do not have any real option other than to copy the callers argv[] list.
 *
 *   If ldargv is not NULL, it takes the place of argv[0]: a loader gives
 *   the program to run and its first arguments (for example, the
 *   interpreter of a script).
 *
 * Input Parameters:
 *   copy     - The copy of the argument list
 *   ldargv   - The argument list of the loader, or NULL
 *   argv     - Argument list
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int binfmt_copyargv(FAR char * const **copy, FAR char * const *ldargv,
                    FAR char * const *argv)
{
  FAR char **argvbuf = NULL;
  FAR const char *arg;
  FAR char *ptr;
  FAR char *end;
  size_t argvsize;
  size_t argsize = 0;
  size_t len;
  int nargs;
  int nld;
  int i;

  /* Get the number of arguments and the size of the argument list */

  nargs = binfmt_nargs(ldargv, argv, &nld);
  if (nargs < 0)
    {
      return nargs;
    }

  for (i = 0; i < nargs; i++)
    {
      argsize += strlen(binfmt_arg(ldargv, argv, nld, i)) + 1;
    }

  binfo("args=%d argsize=%zu\n", nargs, argsize);

  /* Allocate the argv array and an argument buffer */

  if (argsize > 0)
    {
      argvsize = (nargs + 1) * sizeof(FAR char *);
      ptr      = kmm_malloc(argvsize + argsize);
      if (!ptr)
        {
          berr("ERROR: Failed to allocate the argument buffer\n");
          return -ENOMEM;
        }

      /* Copy the argv list */

      argvbuf = (FAR char **)ptr;
      ptr    += argvsize;
      end     = ptr + argsize;
      for (i = 0; i < nargs; i++)
        {
          arg = binfmt_arg(ldargv, argv, nld, i);
          if (ptr == end || arg == NULL)
            {
              kmm_free(argvbuf);
              return -EFAULT;
            }

          len        = strnlen(arg, end - ptr - 1);
          argvbuf[i] = memcpy(ptr, arg, len);
          ptr[len]   = '\0';
          ptr       += len + 1;
        }

      /* Terminate the argv[] list */

      argvbuf[i] = NULL;
    }

  *copy = argvbuf;
  return OK;
}

/****************************************************************************
 * Name: binfmt_freeargv
 *
 * Description:
 *   Release the copied argv[] list.
 *
 * Input Parameters:
 *   argv     - Argument list
 *   ldargv   - The argument list of the loader that binfmt_copyargv() got
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void binfmt_freeargv(FAR char * const *argv, FAR char * const *ldargv)
{
  /* Is there an allocated argument buffer */

  UNUSED(ldargv);

  if (argv)
    {
      /* Free the argument buffer */

      kmm_free((FAR char **)argv);
    }
}

#else

/****************************************************************************
 * Name: binfmt_copyargv
 *
 * Description:
 *   All tasks share one address space, so the caller's argv[] list is used
 *   as it is.  If ldargv is not NULL, it takes the place of argv[0]: the
 *   new list of pointers is allocated, the strings are not copied.
 *
 * Input Parameters:
 *   copy     - The argument list to use
 *   ldargv   - The argument list of the loader, or NULL
 *   argv     - Argument list
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int binfmt_copyargv(FAR char * const **copy, FAR char * const *ldargv,
                    FAR char * const *argv)
{
  FAR char **argvbuf;
  int nargs;
  int nld;
  int i;

  if (ldargv == NULL)
    {
      *copy = argv;
      return OK;
    }

  nargs = binfmt_nargs(ldargv, argv, &nld);
  if (nargs < 0)
    {
      return nargs;
    }

  argvbuf = kmm_malloc((nargs + 1) * sizeof(FAR char *));
  if (argvbuf == NULL)
    {
      return -ENOMEM;
    }

  for (i = 0; i < nargs; i++)
    {
      argvbuf[i] = binfmt_arg(ldargv, argv, nld, i);
    }

  argvbuf[nargs] = NULL;
  *copy = argvbuf;
  return OK;
}

/****************************************************************************
 * Name: binfmt_freeargv
 *
 * Description:
 *   Release the list that binfmt_copyargv() allocated, if it did.
 *
 * Input Parameters:
 *   argv     - Argument list
 *   ldargv   - The argument list of the loader that binfmt_copyargv() got
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void binfmt_freeargv(FAR char * const *argv, FAR char * const *ldargv)
{
  if (ldargv != NULL)
    {
      kmm_free((FAR char **)argv);
    }
}

#endif
