/****************************************************************************
 * include/nuttx/fs/unionfs.h
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
 * Returned Value:
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
