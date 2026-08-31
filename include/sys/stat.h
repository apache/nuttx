/****************************************************************************
 * include/sys/stat.h
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

#ifndef __INCLUDE_SYS_STAT_H
#define __INCLUDE_SYS_STAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* mode_t bit settings (most of these do not apply to NuttX). This assumes
 * that the full size of a mode_t is 16-bits. (However, mode_t must be size
 * 'int' because it is promoted to size int when passed in varargs).
 *
 *   TTTT ...U UUGG GOOO
 *
 *   Bits 0-2:   Permissions for others
 *   Bits 3-5:   Group permissions
 *   Bits 6-8:   Owner permissions
 *   Bits 9-11:  Not used
 *   Bits 12-15: File type bits
 */

#define S_IXOTH     (1 << 0)   /* Bits 0-2: Permissions for others: RWX */
#define S_IWOTH     (1 << 1)
#define S_IROTH     (1 << 2)
#define S_IRWXO     (7 << 0)

#define S_IXGRP     (1 << 3)   /* Bits 3-5: Group permissions: RWX */
#define S_IWGRP     (1 << 4)
#define S_IRGRP     (1 << 5)
#define S_IRWXG     (7 << 3)

#define S_IXUSR     (1 << 6)   /* Bits 6-8: Owner permissions: RWX */
#define S_IWUSR     (1 << 7)
#define S_IRUSR     (1 << 8)
#define S_IRWXU     (7 << 6)

#define S_IREAD     S_IRUSR    /* Obsolete synonym provided for BSD compatibility. */
#define S_IWRITE    S_IWUSR    /* Obsolete synonym provided for BSD compatibility. */
#define S_IEXEC     S_IXUSR    /* Obsolete synonym provided for BSD compatibility. */

#define S_ISVTX     (1 << 9)   /* "Sticky" bit (not used) */
#define S_ISGID     (1 << 10)  /* Set group ID bit (not used)*/
#define S_ISUID     (1 << 11)  /* Set UID bit (not used) */

#define S_IFIFO     (1 << 12)  /* Bits 12-15: File type bits (not all used) */
#define S_IFCHR     (2 << 12)
#define S_IFSEM     (3 << 12)
#define S_IFDIR     (4 << 12)
#define S_IFMQ      (5 << 12)
#define S_IFBLK     (6 << 12)
#define S_IFSHM     (7 << 12)
#define S_IFREG     (8 << 12)
#define S_IFMTD     (9 << 12)
#define S_IFLNK     (10 << 12)
#define S_IFSOCK    (12 << 12)
#define S_IFMT      (15 << 12)

/* File type macros that operate on an instance of mode_t */

#define S_ISFIFO(m) (((m) & S_IFMT) == S_IFIFO)
#define S_ISCHR(m)  (((m) & S_IFMT) == S_IFCHR)
#define S_ISSEM(m)  (((m) & S_IFMT) == S_IFSEM)
#define S_ISDIR(m)  (((m) & S_IFMT) == S_IFDIR)
#define S_ISMQ(m)   (((m) & S_IFMT) == S_IFMQ)
#define S_ISBLK(m)  (((m) & S_IFMT) == S_IFBLK)
#define S_ISSHM(m)  (((m) & S_IFMT) == S_IFSHM)
#define S_ISREG(m)  (((m) & S_IFMT) == S_IFREG)
#define S_ISMTD(m)  (((m) & S_IFMT) == S_IFMTD)
#define S_ISLNK(m)  (((m) & S_IFMT) == S_IFLNK)
#define S_ISSOCK(m) (((m) & S_IFMT) == S_IFSOCK)

/* These are from POSIX.1b. If the objects are not implemented using separate
 * distinct file types, the macros always will evaluate to zero.  Unlike the
 * other S_* macros the following three take a pointer to a `struct stat'
 * object as the argument.
 */

#define S_TYPEISSEM(buf) S_ISSEM((buf)->st_mode)
#define S_TYPEISMQ(buf)  S_ISMQ((buf)->st_mode)
#define S_TYPEISSHM(buf) S_ISSHM((buf)->st_mode)

/* Special value for tv_nsec field of timespec */

#define UTIME_NOW     ((1l << 30) - 1l)
#ifndef __cplusplus
#  define UTIME_OMIT  ((1l << 30) - 2l)
#endif

#define STATX_TYPE            0x000001u /* Want/got stx_mode & S_IFMT */
#define STATX_MODE            0x000002u /* Want/got stx_mode & ~S_IFMT */
#define STATX_NLINK           0x000004u /* Want/got stx_nlink */
#define STATX_UID             0x000008u /* Want/got stx_uid */
#define STATX_GID             0x000010u /* Want/got stx_gid */
#define STATX_ATIME           0x000020u /* Want/got stx_atime */
#define STATX_MTIME           0x000040u /* Want/got stx_mtime */
#define STATX_CTIME           0x000080u /* Want/got stx_ctime */
#define STATX_INO             0x000100u /* Want/got stx_ino */
#define STATX_SIZE            0x000200u /* Want/got stx_size */
#define STATX_BLOCKS          0x000400u /* Want/got stx_blocks */
#define STATX_BASIC_STATS     0x0007ffu /* All fields common with struct stat */
#define STATX_BTIME           0x000800u /* Want/got stx_btime */
#define STATX_ALL             0x000fffu /* All currently supported fields */
#define STATX_MNT_ID          0x001000u /* Want/got stx_mnt_id */
#define STATX_DIOALIGN        0x002000u /* Want/got direct I/O alignment info */

#define STATX_ATTR_COMPRESSED 0x000004u /* File is compressed by the fs */
#define STATX_ATTR_IMMUTABLE  0x000010u /* File is marked immutable */
#define STATX_ATTR_APPEND     0x000020u /* File is append-only */
#define STATX_ATTR_NODUMP     0x000040u /* File is not to be dumped */
#define STATX_ATTR_ENCRYPTED  0x000800u /* File requires key to decrypt */
#define STATX_ATTR_AUTOMOUNT  0x001000u /* Dir: automount trigger */
#define STATX_ATTR_MOUNT_ROOT 0x002000u /* Root of a mount point */
#define STATX_ATTR_VERITY     0x100000u /* fs-verity protected */
#define STATX_ATTR_DAX        0x200000u /* DAX file */

/* The following macros are required by POSIX to achieve backward
 * compatibility with earlier versions of struct stat.
 */

#define st_atime     st_atim.tv_sec
#define st_ctime     st_ctim.tv_sec
#define st_mtime     st_mtim.tv_sec

#if defined(CONFIG_FS_LARGEFILE)
#  define stat64     stat
#  define fstat64    fstat
#  define lstat64    lstat
#  define fstatat64  fstatat
#endif

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* This is the simplified struct stat as returned by stat() and fstat().
 * This structure provides information about a specific file or directory in
 * the file system.
 */

struct stat
{
  /* Required, standard fields */

  dev_t            st_dev;     /* Device ID of device containing file */
  ino_t            st_ino;     /* File serial number */
  mode_t           st_mode;    /* File type, attributes, and access mode bits */
  nlink_t          st_nlink;   /* Number of hard links to the file */
  uid_t            st_uid;     /* User ID of file */
  gid_t            st_gid;     /* Group ID of file */
  dev_t            st_rdev;    /* Device ID (if file is character or block special) */
  off_t            st_size;    /* Size of file/directory, in bytes */
  struct timespec  st_atim;    /* Time of last access */
  struct timespec  st_mtim;    /* Time of last modification */
  struct timespec  st_ctim;    /* Time of last status change */
  blksize_t        st_blksize; /* Block size used for filesystem I/O */
  blkcnt_t         st_blocks;  /* Number of blocks allocated */
};

/* Extended file status, Linux struct statx layout.  Fixed-width fields
 * keep the ABI stable across toolchains and permit a future kernel-space
 * statx() implementation without a user-space ABI break.
 */

struct statx_timestamp
{
  int64_t  tv_sec;    /* Seconds since the Epoch */
  uint32_t tv_nsec;   /* Nanoseconds since tv_sec */
  int32_t  reserved;
};

struct statx
{
  uint32_t stx_mask;                /* What results were actually filled in */
  uint32_t stx_blksize;             /* Preferred I/O block size */
  uint64_t stx_attributes;          /* STATX_ATTR_* flags supported+set */
  uint32_t stx_nlink;               /* Number of hard links */
  uint32_t stx_uid;                 /* User ID of owner */
  uint32_t stx_gid;                 /* Group ID of owner */
  uint16_t stx_mode;                /* File mode, S_IFMT | permissions */
  uint16_t spare0[1];
  uint64_t stx_ino;                 /* Inode number */
  uint64_t stx_size;                /* File size in bytes */
  uint64_t stx_blocks;              /* Number of 512B blocks allocated */
  uint64_t stx_attributes_mask;     /* STATX_ATTR_* flags supported */

  struct statx_timestamp stx_atime; /* Last access time */
  struct statx_timestamp stx_btime; /* File creation time */
  struct statx_timestamp stx_ctime; /* Last status change time */
  struct statx_timestamp stx_mtime; /* Last data modification time */

  uint32_t stx_rdev_major;          /* Device ID of special file */
  uint32_t stx_rdev_minor;
  uint32_t stx_dev_major;           /* ID of device containing file */
  uint32_t stx_dev_minor;
  uint64_t stx_mnt_id;              /* Mount ID */

  /* Memory/file offset alignment for direct I/O */

  uint32_t stx_dio_mem_align;
  uint32_t stx_dio_offset_align;

  uint64_t spare3[12];              /* Spare space for future expansion */
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

int mkdir(FAR const char *pathname, mode_t mode);
int mkdirat(int dirfd, FAR const char *pathname, mode_t mode);
int mkfifo(FAR const char *pathname, mode_t mode);
int mkfifoat(int dirfd, FAR const char *pathname, mode_t mode);
int mknod(FAR const char *path, mode_t mode, dev_t dev);
int mknodat(int dirfd, FAR const char *path, mode_t mode, dev_t dev);
int stat(FAR const char *path, FAR struct stat *buf);
int lstat(FAR const char *path, FAR struct stat *buf);
int fstat(int fd, FAR struct stat *buf);
int fstatat(int dirfd, FAR const char *path, FAR struct stat *buf,
            int flags);
int statx(int dirfd, FAR const char *path, int flags, unsigned int mask,
          FAR struct statx *buf);
int chmod(FAR const char *path, mode_t mode);
int lchmod(FAR const char *path, mode_t mode);
int fchmod(int fd, mode_t mode);
int fchmodat(int dirfd, FAR const char *path, mode_t mode, int flags);
int utimens(FAR const char *path, const struct timespec times[2]);
int utimensat(int dirfd, FAR const char *path,
              const struct timespec times[2], int flags);
int lutimens(FAR const char *path, const struct timespec times[2]);
int futimens(int fd, const struct timespec times[2]);

mode_t umask(mode_t mask);
mode_t getumask(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_STAT_H */
