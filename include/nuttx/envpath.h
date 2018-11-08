/****************************************************************************
 * include/nuttx/envpath.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_ENVPATH_H
#define __INCLUDE_NUTTX_ENVPATH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_LIB_ENVPATH

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* ENVPATH_HANDLE is an opaque handle used to traverse the absolute paths
 * assigned to the PATH environment variable.
 */

typedef FAR void *ENVPATH_HANDLE;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: envpath_init
 *
 * Description:
 *   Initialize for the traversal of each value in the PATH variable.  The
 *   usage is sequence is as follows:
 *
 * Input Parameters:
 *   name - The variable name of environment to searches path list.
 *
 *   1) Call envpath_init() to initialize for the traversal.  envpath_init()
 *      will return an opaque handle that can then be provided to
 *      envpath_next() and envpath_release().
 *   2) Call envpath_next() repeatedly to examine every file that lies
 *      in the directories of the PATH variable
 *   3) Call envpath_release() to free resources set aside by envpath_init().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, envpath_init() return a non-NULL, opaque handle that may
 *   subsequently be used in calls to envpath_next() and envpath_release().
 *   On error, a NULL handle value will be returned.  The most likely cause
 *   of an error would be that there is no value associated with the PATH
 *   variable.
 *
 ****************************************************************************/

ENVPATH_HANDLE envpath_init(FAR const char *name);

/****************************************************************************
 * Name: envpath_next
 *
 * Description:
 *   Traverse all possible values in the PATH variable in attempt to find
 *   the full path to an executable file when only a relative path is
 *   provided.
 *
 * Input Parameters:
 *   handle - The handle value returned by envpath_init
 *   relpath - The relative path to the file to be found.
 *
 * Returned Value:
 *   On success, a non-NULL pointer to a null-terminated string is provided.
 *   This is the full path to a file that exists in the file system.  This
 *   function will verify that the file exists (but will not verify that it
 *   is marked executable).
 *
 *   NOTE: The string pointer return in the success case points to allocated
 *   memory.  This memory must be freed by the called by calling kmm_free().
 *
 *   NULL is returned if no path is found to any file with the provided
 *   'relpath' from any absolute path in the PATH variable.  In this case,
 *   there is no point in calling envpath_next() further; envpath_release()
 *   must be called to release resources set aside by expath_init().
 *
 ****************************************************************************/

FAR char *envpath_next(ENVPATH_HANDLE handle, FAR const char *relpath);

/****************************************************************************
 * Name: envpath_release
 *
 * Description:
 *   Release all resources set aside by envpath_init() when the handle value
 *   was created.  The handle value is invalid on return from this function.
 *   Attempts to all envpath_next() or envpath_release() with such a 'stale'
 *   handle will result in undefined (i.e., not good) behavior.
 *
 * Input Parameters:
 *   handle - The handle value returned by envpath_init
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void envpath_release(ENVPATH_HANDLE handle);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LIB_ENVPATH */
#endif /* INCLUDE_NUTTX_ENVPATH_H */
