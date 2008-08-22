/****************************************************************************
 * lib/lib_chdir.c
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "lib_internal.h"

/****************************************************************************
 * Public Variables
 ****************************************************************************/

#if CONFIG_NFILE_DESCRIPTORS > 0
char *g_cwd     = NULL;
char *g_prevcwd = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: chdir
 *
 * Description:
 *   The chdir() function causes the directory named by the pathname pointed
 *   to by the 'path' argument to become the current working directory; that
 *   is, the starting point for path searches for pathnames not beginning
 *   with '/'.
 *
 * Input Parmeters:
 *   path - A pointer to a directory to use as the new current working
 *     directory
 *
 * Returned Value:
 *   0(OK) on success; -1(ERROR) on failure with errno set appropriately:
 *
 *   EACCES
 *     Search permission is denied for any component of the pathname.
 *   ELOOP
 *     A loop exists in symbolic links encountered during resolution of the
 *     'path' argument OR more that SYMLOOP_MAX symbolic links in the
 *     resolution of the 'path' argument.
 *   ENAMETOOLONG
 *     The length of the path argument exceeds PATH_MAX or a pathname component
 *     is longer than NAME_MAX.
 *   ENOENT
 *     A component of 'path' does not name an existing directory or path is
 *     an empty string.
 *   ENOTDIR
 *     A component of the pathname is not a directory.
 *
 ****************************************************************************/

int chdir(FAR const char *path)
{
  char *duppath;

  /* Verify the input parameters */

  if (!path)
    {
      errno = ENOENT;
      return ERROR;
    }

  /* Verify that 'path' refers to a directory */
  /* (To be provided) */

  /* Make a persistent copy of 'path' */

  duppath = strdup(path);

  /* Free any preceding cwd and set the previous to the cwd (this
   * is to support 'cd -' in NSH
   */

  cwd_semtake();
  if (g_prevcwd)
    {
      free(g_prevcwd);
    }
  g_prevcwd = g_cwd;

  /* Set the cwd to a persistent copy of the input 'path' */

  g_cwd = duppath;
  cwd_semgive();
  return OK;
}
#endif /* CONFIG_NFILE_DESCRIPTORS */
