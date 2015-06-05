/****************************************************************************
 * include/nuttx/fs/unionfs.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_FS_UNIONFS_H
#define __INCLUDE_NUTTX_FS_UNIONFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_FS_UNIONFS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: unionfs_mount
 *
 * Description:
 *   Create and mount a union file system
 *
 * Input Parameters:
 *   fspath1 - The full path to the first file system mountpoint
 *   prefix1 - An optiona prefix that may be applied to make the first
 *             file system appear a some path below the unionfs mountpoint,
 *   fspath2 - The full path to the second file system mountpoint
 *   prefix2 - An optiona prefix that may be applied to make the first
 *             file system appear a some path below the unionfs mountpoint,
 *   mountpt - The full path to the mountpoint for the union file system
 *
 * Returned value:
 *   Zero (OK) is returned if the union file system was correctly created and
 *   mounted.  On any failure, a negated error value will be returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

int unionfs_mount(FAR const char *fspath1, FAR const char *prefix1,
                  FAR const char *fspath2, FAR const char *prefix2,
                  FAR const char *mountpt);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_FS_UNIONFS */
#endif /* __INCLUDE_NUTTX_FS_UNIONFS_H */
