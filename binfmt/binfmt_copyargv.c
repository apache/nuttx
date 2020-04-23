/****************************************************************************
 * binfmt/binfmt_copyargv.c
 *
 *   Copyright (C) 2009, 2013-2015, 2020 Gregory Nutt. All rights reserved.
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
 *   address environment and, hence, by inaccessible when we switch to the
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
