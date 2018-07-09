/****************************************************************************
 * include/nuttx/fs/hostfs.h
 *
 *   Copyright (C) 2015 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef __SIM__

/* These must exactly match the definitions from include/dirent.h: */

#define NUTTX_DTYPE_FILE        0x01
#define NUTTX_DTYPE_CHR         0x02
#define NUTTX_DTYPE_BLK         0x04
#define NUTTX_DTYPE_DIRECTORY   0x08

/* These must exactly match the definitions from include/sys/stat.h: */

#define NUTTX_S_IFIFO           0
#define NUTTX_S_IFCHR           (1 << 11)
#define NUTTX_S_IFDIR           (2 << 11)
#define NUTTX_S_IFBLK           (3 << 11)
#define NUTTX_S_IFREG           (4 << 11)
#define NUTTX_S_IFLNK           (1 << 15)

/* These must exactly match the definitions from include/fcntl.h: */

#define NUTTX_O_RDONLY   (1 << 0)  /* Open for read access (only) */
#define NUTTX_O_WRONLY   (1 << 1)  /* Open for write access (only) */
#define NUTTX_O_CREAT    (1 << 2)  /* Create file/sem/mq object */
#define NUTTX_O_EXCL     (1 << 3)  /* Name must not exist when opened  */
#define NUTTX_O_APPEND   (1 << 4)  /* Keep contents, append to end */
#define NUTTX_O_TRUNC    (1 << 5)  /* Delete contents */
#define NUTTX_O_NONBLOCK (1 << 6)  /* Don't wait for data */
#define NUTTX_O_SYNC     (1 << 7)  /* Synchronize output on write */
#define NUTTX_O_BINARY   (1 << 8)  /* Open the file in binary mode. */

#define NUTTX_O_RDWR     (NUTTX_O_RDONLY | NUTTX_O_WRONLY)

/* Should match definition in include/limits.h */

#define NUTTX_NAME_MAX   32

#endif /* __SIM__ */

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#ifdef __SIM__

/* These must match the definitions in include/sys/types.h */

typedef uintptr_t    nuttx_size_t;
typedef int32_t      nuttx_off_t;
typedef unsigned int nuttx_mode_t;
typedef int16_t      nuttx_blksize_t;
typedef uint32_t     nuttx_blkcnt_t;

/* These must match the definition in include/time.h */

typedef uint32_t     nuttx_time_t;

/* These must exactly match the definition from include/dirent.h: */

struct nuttx_dirent_s
{
  uint8_t      d_type;             /* type of file */
  char         d_name[NUTTX_NAME_MAX+1]; /* filename */
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
  nuttx_mode_t    st_mode;    /* File type, atributes, and access mode bits */
  nuttx_off_t     st_size;    /* Size of file/directory, in bytes */
  nuttx_blksize_t st_blksize; /* Blocksize used for filesystem I/O */
  nuttx_blkcnt_t  st_blocks;  /* Number of blocks allocated */
  nuttx_time_t    st_atim;    /* Time of last access */
  nuttx_time_t    st_mtim;    /* Time of last modification */
  nuttx_time_t    st_ctim;    /* Time of last status change */
};

#endif /* __SIM__ */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __SIM__
int           host_open(const char *pathname, int flags, int mode);
int           host_close(int fd);
ssize_t       host_read(int fd, void *buf, nuttx_size_t count);
ssize_t       host_write(int fd, const void *buf, nuttx_size_t count);
off_t         host_lseek(int fd, off_t offset, int whence);
int           host_ioctl(int fd, int request, unsigned long arg);
void          host_sync(int fd);
int           host_dup(int fd);
int           host_fstat(int fd, struct nuttx_stat_s *buf);
int           host_ftruncate(int fd, off_t length);
void         *host_opendir(const char *name);
int           host_readdir(void* dirp, struct nuttx_dirent_s* entry);
void          host_rewinddir(void* dirp);
int           host_closedir(void* dirp);
int           host_statfs(const char *path, struct nuttx_statfs_s *buf);
int           host_unlink(const char *pathname);
int           host_mkdir(const char *pathname, mode_t mode);
int           host_rmdir(const char *pathname);
int           host_rename(const char *oldpath, const char *newpath);
int           host_stat(const char *path, struct nuttx_stat_s *buf);
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
int           host_ftruncate(int fd, off_t length);
void         *host_opendir(const char *name);
int           host_readdir(void* dirp, struct dirent *entry);
void          host_rewinddir(void* dirp);
int           host_closedir(void* dirp);
int           host_statfs(const char *path, struct statfs *buf);
int           host_unlink(const char *pathname);
int           host_mkdir(const char *pathname, mode_t mode);
int           host_rmdir(const char *pathname);
int           host_rename(const char *oldpath, const char *newpath);
int           host_stat(const char *path, struct stat *buf);

#endif /* __SIM__ */

#endif /* __INCLUDE_NUTTX_FS_HOSTFS_H */
