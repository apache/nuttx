/****************************************************************************
 * include/sys/stat.h
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

/* The following macros are required by POSIX to achieve backward
 * compatibility with earlier versions of struct stat.
 */

#define st_atime     st_atim.tv_sec
#define st_ctime     st_ctim.tv_sec
#define st_mtime     st_mtim.tv_sec

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
int mkfifo(FAR const char *pathname, mode_t mode);
int stat(FAR const char *path, FAR struct stat *buf);
int lstat(FAR const char *path, FAR struct stat *buf);
int fstat(int fd, FAR struct stat *buf);
int chmod(FAR const char *path, mode_t mode);
int fchmod(int fd, mode_t mode);
int futimens(int fd, const struct timespec times[2]);

mode_t umask(mode_t mask);
mode_t getumask(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_STAT_H */
