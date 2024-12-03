/****************************************************************************
 * arch/sim/src/sim/sim_checkhostfstypes.c
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

#include <assert.h>

/* include headers for NuttX types
 * eg. struct stat
 *
 * Note: This file is not HOSTSRCS. Thus, these files are not
 * the host OS version of them.
 */

#include <dirent.h>
#include <sys/statfs.h>
#include <sys/stat.h>

/* include nuttx/fs/hostfs.h for hostfs types
 * eg. struct nuttx_stat_s
 *
 * define __SIM__ to make them visible.
 *
 * Note: For HOSTSRCS, __SIM__ is automatically defined by make files.
 * However, this file is not HOSTSRCS. Thus we need to define it
 * by ourselves here.
 */

#define __SIM__
#include <nuttx/fs/hostfs.h>
#undef __SIM__

/* Here we make static assertions to ensure NuttX types (eg. struct stat)
 * match the corresponding hostfs types. (eg. struct nuttx_stat_s)
 *
 * TODO: check the alignment as well. (_Alignof)
 */

#define STATIC_ASSERT(c) static_assert(c, #c)
#define STATIC_ASSERT_FILED(a, b, f) \
    STATIC_ASSERT(offsetof(struct a, f) == offsetof(struct b, f)); \
    STATIC_ASSERT(sizeof(((struct a *)0)->f) == sizeof(((struct b *)0)->f))

/* dirent */

STATIC_ASSERT_FILED(nuttx_dirent_s, dirent, d_type);
STATIC_ASSERT_FILED(nuttx_dirent_s, dirent, d_name);

STATIC_ASSERT(sizeof(struct nuttx_dirent_s) == sizeof(struct dirent));

/* stat */

STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_dev);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_ino);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_mode);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_nlink);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_uid);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_gid);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_rdev);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_size);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_atim);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_mtim);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_ctim);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_blksize);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_blocks);
STATIC_ASSERT_FILED(nuttx_stat_s, stat, st_size);

STATIC_ASSERT(sizeof(struct nuttx_stat_s) == sizeof(struct stat));

/* statfs */

STATIC_ASSERT_FILED(nuttx_statfs_s, statfs, f_type);
STATIC_ASSERT_FILED(nuttx_statfs_s, statfs, f_namelen);
STATIC_ASSERT_FILED(nuttx_statfs_s, statfs, f_bsize);
STATIC_ASSERT_FILED(nuttx_statfs_s, statfs, f_blocks);
STATIC_ASSERT_FILED(nuttx_statfs_s, statfs, f_bfree);
STATIC_ASSERT_FILED(nuttx_statfs_s, statfs, f_bavail);
STATIC_ASSERT_FILED(nuttx_statfs_s, statfs, f_files);
STATIC_ASSERT_FILED(nuttx_statfs_s, statfs, f_ffree);
STATIC_ASSERT_FILED(nuttx_statfs_s, statfs, f_fsid);

STATIC_ASSERT(sizeof(struct nuttx_statfs_s) == sizeof(struct statfs));

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* dummy function to suppress nxstyle and compiler warnings */

void check_hostfs_types_dummy(void);

void check_hostfs_types_dummy(void)
{
}
