/****************************************************************************
 * include/mntent.h
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

#ifndef __INCLUDE_MNTENT_H
#define __INCLUDE_MNTENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The table of mounted file systems.  NuttX lists them in procfs, one line
 * per mount: "<mount point> type <file system type>".
 */

#define MOUNTED          "/proc/fs/mount"
#define _PATH_MOUNTED    MOUNTED

/* Generic mount options */

#define MNTOPT_DEFAULTS  "defaults"
#define MNTOPT_RO        "ro"
#define MNTOPT_RW        "rw"
#define MNTOPT_SUID      "suid"
#define MNTOPT_NOSUID    "nosuid"
#define MNTOPT_NOAUTO    "noauto"

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mntent
{
  FAR char *mnt_fsname;  /* Mounted device or file system name */
  FAR char *mnt_dir;     /* Mount point */
  FAR char *mnt_type;    /* File system type */
  FAR char *mnt_opts;    /* Comma-separated mount options */
  int mnt_freq;          /* Dump frequency in days */
  int mnt_passno;        /* Pass number for parallel fsck */
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

FAR FILE *setmntent(FAR const char *filename, FAR const char *type);
FAR struct mntent *getmntent(FAR FILE *stream);
FAR struct mntent *getmntent_r(FAR FILE *stream, FAR struct mntent *mntbuf,
                               FAR char *buf, int buflen);
int addmntent(FAR FILE *stream, FAR const struct mntent *mnt);
int endmntent(FAR FILE *stream);
FAR char *hasmntopt(FAR const struct mntent *mnt, FAR const char *opt);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_MNTENT_H */
