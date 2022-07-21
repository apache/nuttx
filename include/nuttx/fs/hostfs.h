/****************************************************************************
 * include/nuttx/fs/hostfs.h
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

#ifndef __INCLUDE_NUTTX_FS_HOSTFS_H
#define __INCLUDE_NUTTX_FS_HOSTFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __SIM__
#  include <sys/types.h>
#  include <sys/stat.h>
#  include <sys/statfs.h>
#  include <dirent.h>
#  include <time.h>
#else
#  include <config.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef __SIM__

/* These must exactly match the definitions from include/dirent.h: */

#define NUTTX_DTYPE_UNKNOWN     0
#define NUTTX_DTYPE_FIFO        1
#define NUTTX_DTYPE_CHR         2
#define NUTTX_DTYPE_SEM         3
#define NUTTX_DTYPE_DIRECTORY   4
#define NUTTX_DTYPE_MQ          5
#define NUTTX_DTYPE_BLK         6
#define NUTTX_DTYPE_SHM         7
#define NUTTX_DTYPE_FILE        8
#define NUTTX_DTYPE_MTD         9
#define NUTTX_DTYPE_LINK        10
#define NUTTX_DTYPE_SOCK        12

/* These must exactly match the definitions from include/sys/stat.h: */

#define NUTTX_S_IFIFO           (1 << 12)
#define NUTTX_S_IFCHR           (2 << 12)
#define NUTTX_S_IFSEM           (3 << 12)
#define NUTTX_S_IFDIR           (4 << 12)
#define NUTTX_S_IFMQ            (5 << 12)
#define NUTTX_S_IFBLK           (6 << 12)
#define NUTTX_S_IFSHM           (7 << 12)
#define NUTTX_S_IFREG           (8 << 12)
#define NUTTX_S_IFMTD           (9 << 12)
#define NUTTX_S_IFLNK           (10 << 12)
#define NUTTX_S_IFSOCK          (12 << 12)
#define NUTTX_S_IFMT            (15 << 12)

/* These must exactly match the definitions from include/fcntl.h: */

#define NUTTX_O_RDONLY          (1 << 0)  /* Open for read access (only) */
#define NUTTX_O_WRONLY          (1 << 1)  /* Open for write access (only) */
#define NUTTX_O_CREAT           (1 << 2)  /* Create file/sem/mq object */
#define NUTTX_O_EXCL            (1 << 3)  /* Name must not exist when opened  */
#define NUTTX_O_APPEND          (1 << 4)  /* Keep contents, append to end */
#define NUTTX_O_TRUNC           (1 << 5)  /* Delete contents */
#define NUTTX_O_NONBLOCK        (1 << 6)  /* Don't wait for data */
#define NUTTX_O_SYNC            (1 << 7)  /* Synchronize output on write */
#define NUTTX_O_BINARY          (1 << 8)  /* Open the file in binary mode. */
#define NUTTX_O_DIRECT          (1 << 9)  /* Avoid caching, write directly to hardware */

#define NUTTX_O_RDWR            (NUTTX_O_RDONLY | NUTTX_O_WRONLY)

/* Should match definition in include/nuttx/fs/fs.h */

#define NUTTX_CH_STAT_MODE      (1 << 0)
#define NUTTX_CH_STAT_UID       (1 << 1)
#define NUTTX_CH_STAT_GID       (1 << 2)
#define NUTTX_CH_STAT_ATIME     (1 << 3)
#define NUTTX_CH_STAT_MTIME     (1 << 4)

#endif /* __SIM__ */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#ifdef __SIM__

/* These must match the definitions in include/sys/types.h */

typedef int16_t      nuttx_blksize_t;
typedef int16_t      nuttx_gid_t;
typedef int16_t      nuttx_uid_t;
typedef uint32_t     nuttx_dev_t;
typedef uint16_t     nuttx_ino_t;
typedef uint16_t     nuttx_nlink_t;
#ifdef CONFIG_FS_LARGEFILE
typedef int64_t      nuttx_off_t;
typedef uint64_t     nuttx_blkcnt_t;
#else
typedef int32_t      nuttx_off_t;
typedef uint32_t     nuttx_blkcnt_t;
#endif
typedef unsigned int nuttx_mode_t;
typedef uintptr_t    nuttx_size_t;
typedef intptr_t     nuttx_ssize_t;

/* These must match the definition in include/time.h */

typedef uint32_t     nuttx_time_t;

struct nuttx_timespec
{
    nuttx_time_t tv_sec;
    long tv_nsec;
};

/* These must exactly match the definition from include/dirent.h: */

struct nuttx_dirent_s
{
  uint8_t      d_type;                      /* type of file */
  char         d_name[CONFIG_NAME_MAX + 1]; /* filename */
};

/* These must exactly match the definition from include/sys/statfs.h: */

struct nuttx_statfs_s
{
  uint32_t     f_type;     /* Type of filesystem */
  nuttx_size_t f_namelen;  /* Maximum length of filenames */
  nuttx_size_t f_bsize;    /* Optimal block size for transfers */
  nuttx_off_t  f_blocks;   /* Total data blocks in the file system of this size */
  nuttx_off_t  f_bfree;    /* Free blocks in the file system */
  nuttx_off_t  f_bavail;   /* Free blocks avail to non-superuser */
  nuttx_off_t  f_files;    /* Total file nodes in the file system */
  nuttx_off_t  f_ffree;    /* Free file nodes in the file system */
};

/* These must exactly match the definition from include/sys/stat.h: */

struct nuttx_stat_s
{
  nuttx_dev_t           st_dev;     /* Device ID of device containing file */
  nuttx_ino_t           st_ino;     /* File serial number */
  nuttx_mode_t          st_mode;    /* File type, attributes, and access mode bits */
  nuttx_nlink_t         st_nlink;   /* Number of hard links to the file */
  nuttx_uid_t           st_uid;     /* User ID of file */
  nuttx_gid_t           st_gid;     /* Group ID of file */
  nuttx_dev_t           st_rdev;    /* Device ID (if file is character or block special) */
  nuttx_off_t           st_size;    /* Size of file/directory, in bytes */
  struct nuttx_timespec st_atim;    /* Time of last access */
  struct nuttx_timespec st_mtim;    /* Time of last modification */
  struct nuttx_timespec st_ctim;    /* Time of last status change */
  nuttx_blksize_t       st_blksize; /* Block size used for filesystem I/O */
  nuttx_blkcnt_t        st_blocks;  /* Number of blocks allocated */
};

#endif /* __SIM__ */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __SIM__
int           host_open(const char *pathname, int flags, nuttx_mode_t mode);
int           host_close(int fd);
nuttx_ssize_t host_read(int fd, void *buf, nuttx_size_t count);
nuttx_ssize_t host_write(int fd, const void *buf, nuttx_size_t count);
nuttx_off_t   host_lseek(int fd, nuttx_off_t offset, int whence);
int           host_ioctl(int fd, int request, unsigned long arg);
void          host_sync(int fd);
int           host_dup(int fd);
int           host_fstat(int fd, struct nuttx_stat_s *buf);
int           host_fchstat(int fd, const struct nuttx_stat_s *buf,
                           int flags);
int           host_ftruncate(int fd, nuttx_off_t length);
void         *host_opendir(const char *name);
int           host_readdir(void *dirp, struct nuttx_dirent_s *entry);
void          host_rewinddir(void *dirp);
int           host_closedir(void *dirp);
int           host_statfs(const char *path, struct nuttx_statfs_s *buf);
int           host_unlink(const char *pathname);
int           host_mkdir(const char *pathname, mode_t mode);
int           host_rmdir(const char *pathname);
int           host_rename(const char *oldpath, const char *newpath);
int           host_stat(const char *path, struct nuttx_stat_s *buf);
int           host_chstat(const char *path,
                          const struct nuttx_stat_s *buf, int flags);
#else
int           host_open(const char *pathname, int flags, int mode);
int           host_close(int fd);
ssize_t       host_read(int fd, void *buf, size_t count);
ssize_t       host_write(int fd, const void *buf, size_t count);
off_t         host_lseek(int fd, off_t offset, int whence);
int           host_ioctl(int fd, int request, unsigned long arg);
void          host_sync(int fd);
int           host_dup(int fd);
int           host_fstat(int fd, struct stat *buf);
int           host_fchstat(int fd, const struct stat *buf, int flags);
int           host_ftruncate(int fd, off_t length);
void         *host_opendir(const char *name);
int           host_readdir(void *dirp, struct dirent *entry);
void          host_rewinddir(void *dirp);
int           host_closedir(void *dirp);
int           host_statfs(const char *path, struct statfs *buf);
int           host_unlink(const char *pathname);
int           host_mkdir(const char *pathname, mode_t mode);
int           host_rmdir(const char *pathname);
int           host_rename(const char *oldpath, const char *newpath);
int           host_stat(const char *path, struct stat *buf);
int           host_chstat(const char *path,
                          const struct stat *buf, int flags);
#endif /* __SIM__ */

#endif /* __INCLUDE_NUTTX_FS_HOSTFS_H */
