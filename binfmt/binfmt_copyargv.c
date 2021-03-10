/****************************************************************************
 * binfmt/binfmt_copyargv.c
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
#include <nuttx/binfmt/binfmt.h>

#include "binfmt.h"

#ifndef CONFIG_BINFMT_DISABLE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is an artificial limit to detect error conditions where an argv[]
 * list is not properly terminated.
 */

#define MAX_EXEC_ARGS 256

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: binfmt_copyargv
 *
 * Description:
 *   In the kernel build, the argv list will likely lie in the caller's
 *   address environment and, hence, be inaccessible when we switch to the
 *   address environment of the new process address environment.  So we
 *   do not have any real option other than to copy the callers argv[] list.
 *
 * Input Parameters:
 *   bin      - Load structure
 *   argv     - Argument list
 *
 * Returned Value:
 *   Zero (OK) on success; a negated error value on failure.
 *
 ****************************************************************************/

int binfmt_copyargv(FAR struct binary_s *bin, FAR char * const *argv)
{
#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
  FAR char *ptr;
  size_t argvsize;
  size_t argsize;
  int nargs;
  int i;

  /* Get the number of arguments and the size of the argument list */

  bin->argv      = (FAR char **)NULL;
  bin->argbuffer = (FAR char *)NULL;

  if (argv)
    {
      argsize = 0;
      nargs   = 0;

      for (i = 0; argv[i]; i++)
        {
          /* Increment the size of the allocation with the size of the next
           * string
           */

          argsize += (strlen(argv[i]) + 1);
          nargs++;

          /* This is a sanity check to prevent running away with an
           * unterminated argv[] list.
           * MAX_EXEC_ARGS should be sufficiently large that this
           * never happens in normal usage.
           */

          if (nargs > MAX_EXEC_ARGS)
            {
              berr("ERROR: Too many arguments: %lu\n",
                   (unsigned long)argvsize);
              return -E2BIG;
            }
        }

      binfo("args=%d argsize=%lu\n", nargs, (unsigned long)argsize);

      /* Allocate the argv array and an argument buffer */

      if (argsize > 0)
        {
          argvsize  = (nargs + 1) * sizeof(FAR char *);
          bin->argbuffer = (FAR char *)kmm_malloc(argvsize + argsize);
          if (!bin->argbuffer)
            {
              berr("ERROR: Failed to allocate the argument buffer\n");
              return -ENOMEM;
            }

          /* Copy the argv list */

          bin->argv = (FAR char **)bin->argbuffer;
          ptr       = bin->argbuffer + argvsize;
          for (i = 0; argv[i]; i++)
            {
              bin->argv[i] = ptr;
              argsize      = strlen(argv[i]) + 1;
              memcpy(ptr, argv[i], argsize);
              ptr         += argsize;
            }

          /* Terminate the argv[] list */

          bin->argv[i] = (FAR char *)NULL;
        }
    }

  return OK;

#else
  /* Just save the caller's argv pointer */

  bin->argv = argv;
  return OK;
#endif
}

/****************************************************************************
 * Name: binfmt_freeargv
 *
 * Description:
 *   Release the copied argv[] list.
 *
 * Input Parameters:
 *   binp - Load structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
void binfmt_freeargv(FAR struct binary_s *binp)
{
  /* Is there an allocated argument buffer */

  if (binp->argbuffer)
    {
      /* Free the argument buffer */

      kmm_free(binp->argbuffer);
    }

  /* Nullify the allocated argv[] array and the argument buffer pointers */

  binp->argbuffer = (FAR char *)NULL;
  binp->argv      = (FAR char **)NULL;
}
#endif

#endif /* !CONFIG_BINFMT_DISABLE */
