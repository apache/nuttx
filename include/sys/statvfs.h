/****************************************************************************
 * include/sys/statvfs.h
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

#ifndef __INCLUDE_SYS_STATVFS_H
#define __INCLUDE_SYS_STATVFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Definitions for the flag in `f_flag'.  These definitions should be
 * kept in sync with the definitions in <sys/mount.h>.
 */

#define ST_RDONLY             0x0001 /* Mount read-only.  */
#define ST_NOSUID             0x0002 /* Ignore suid and sgid bits.  */

#if defined(CONFIG_FS_LARGEFILE) && defined(CONFIG_HAVE_LONG_LONG)
#  define statvfs64           statvfs
#  define fstatvfs64          fstatvfs
#endif

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

struct statvfs
{
  unsigned long f_bsize;   /* File system block size */
  unsigned long f_frsize;  /* Fundamental file system block size */
  fsblkcnt_t    f_blocks;  /* Total number of blocks on file system in
                            * units of f_frsize */
  fsblkcnt_t    f_bfree;   /* Total number of free blocks */
  fsblkcnt_t    f_bavail;  /* Number of free blocks available to
                            * non-privileged process */
  fsfilcnt_t    f_files;   /* Total number of file serial numbers */
  fsfilcnt_t    f_ffree;   /* Total number of free file serial numbers */
  fsfilcnt_t    f_favail;  /* Number of file serial numbers available to
                            * non-privileged process */
  unsigned long f_fsid;    /* File system ID */
  unsigned long f_flag;    /* Bit mask of f_flag values */
  unsigned long f_namemax; /* Maximum filename length */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int statvfs(FAR const char *path, FAR struct statvfs *buf);
int fstatvfs(int fd, FAR struct statvfs *buf);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_STATVFS_H */
