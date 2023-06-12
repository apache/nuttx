/****************************************************************************
 * include/sys/statfs.h
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

#ifndef __INCLUDE_SYS_STATFS_H
#define __INCLUDE_SYS_STATFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* struct statfs file system types. */

#define ADFS_SUPER_MAGIC      0xadf5
#define AFFS_SUPER_MAGIC      0xadff
#define BEFS_SUPER_MAGIC      0x42465331
#define BFS_MAGIC             0x1badface
#define CIFS_MAGIC_NUMBER     0xff534d42
#define CODA_SUPER_MAGIC      0x73757245
#define COH_SUPER_MAGIC       0x012ff7b7
#define CRAMFS_MAGIC          0x28cd3d45
#define DEVFS_SUPER_MAGIC     0x1373
#define EFS_SUPER_MAGIC       0x00414a53
#define EXT_SUPER_MAGIC       0x137d
#define EXT2_OLD_SUPER_MAGIC  0xef51
#define EXT2_SUPER_MAGIC      0xef53
#define EXT3_SUPER_MAGIC      0xef53
#define HFS_SUPER_MAGIC       0x4244
#define HPFS_SUPER_MAGIC      0xf995e849
#define HUGETLBFS_MAGIC       0x958458f6
#define ISOFS_SUPER_MAGIC     0x9660
#define JFFS2_SUPER_MAGIC     0x72b6
#define JFS_SUPER_MAGIC       0x3153464a
#define MINIX_SUPER_MAGIC     0x137f /* orig. minix */
#define MINIX_SUPER_MAGIC2    0x138f /* 30 char minix */
#define MINIX2_SUPER_MAGIC    0x2468 /* minix V2 */
#define MINIX2_SUPER_MAGIC2   0x2478 /* minix V2, 30 char names */
#define MSDOS_SUPER_MAGIC     0x4d44
#define NCP_SUPER_MAGIC       0x564c
#define NFS_SUPER_MAGIC       0x6969
#define NTFS_SB_MAGIC         0x5346544e
#define OPENPROM_SUPER_MAGIC  0x9fa1
#define PROC_SUPER_MAGIC      0x9fa0
#define QNX4_SUPER_MAGIC      0x002f
#define REISERFS_SUPER_MAGIC  0x52654973
#define ROMFS_MAGIC           0x7275
#define SMB_SUPER_MAGIC       0x517B
#define SYSV2_SUPER_MAGIC     0x012ff7b6
#define SYSV4_SUPER_MAGIC     0x012FF7B5
#define TMPFS_MAGIC           0x01021994
#define UDF_SUPER_MAGIC       0x15013346
#define UFS_MAGIC             0x00011954
#define USBDEVICE_SUPER_MAGIC 0x9fa2
#define VXFS_SUPER_MAGIC      0xa501fcf5
#define XENIX_SUPER_MAGIC     0x012ff7b4
#define XFS_SUPER_MAGIC       0x58465342
#define _XIAFS_SUPER_MAGIC    0x012fd16d
#define SPIFFS_SUPER_MAGIC    0x20090315
#define LITTLEFS_SUPER_MAGIC  0x0a732923

/* NuttX specific file-systems */

#define BINFS_MAGIC           0x4242
#define PROCFS_MAGIC          0x434f5250
#define NXFFS_MAGIC           0x4747
#define SMARTFS_MAGIC         0x54524D53
#define UNIONFS_MAGIC         0x53464e55
#define HOSTFS_MAGIC          0x54534f48
#define USERFS_MAGIC          0x52455355
#define CROMFS_MAGIC          0x4d4f5243
#define RPMSGFS_MAGIC         0x54534f47

#if defined(CONFIG_FS_LARGEFILE)
#  define statfs64            statfs
#  define fstatfs64           fstatfs
#endif

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

typedef struct fsid_s fsid_t;

struct statfs
{
  uint32_t   f_type;     /* Type of filesystem (see definitions above) */
  size_t     f_namelen;  /* Maximum length of filenames */
  size_t     f_bsize;    /* Optimal block size for transfers */
  fsblkcnt_t f_blocks;   /* Total data blocks in the file system of this size */
  fsblkcnt_t f_bfree;    /* Free blocks in the file system */
  fsblkcnt_t f_bavail;   /* Free blocks avail to non-superuser */
  fsfilcnt_t f_files;    /* Total file nodes in the file system */
  fsfilcnt_t f_ffree;    /* Free file nodes in the file system */
  fsid_t     f_fsid;     /* Encode device type, not yet in use */
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

/* Inspired by Linux statfs() which was, in turn, inspired by
 * the BSD statfs(). None of these implementations agree in the
 * form of the struct statfs.
 */

int statfs(FAR const char *path, FAR struct statfs *buf);
int fstatfs(int fd, FAR struct statfs *buf);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_STATFS_H */
