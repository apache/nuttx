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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HOST_ST_MODE_REG        0x01000
#define HOST_ST_MODE_DIR        0x02000
#define HOST_ST_MODE_CHR        0x04000
#define HOST_ST_MODE_BLK        0x08000
#define HOST_ST_MODE_PIPE       0x10000
#define HOST_ST_MODE_LINK       0x20000

#define HOSTFS_FLAG_RDOK        0x0001
#define HOSTFS_FLAG_WROK        0x0002
#define HOSTFS_FLAG_CREAT       0x0004
#define HOSTFS_FLAG_EXCL        0x0008
#define HOSTFS_FLAG_APPEND      0x0010
#define HOSTFS_FLAG_TRUNC       0x0020

#define HOSTFS_DTYPE_FILE       0x0001
#define HOSTFS_DTYPE_CHR        0x0002
#define HOSTFS_DTYPE_BLK        0x0004
#define HOSTFS_DTYPE_DIRECTORY  0x0008

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

struct host_dirent_s
{
  size_t            d_ino;
  size_t            d_off;
  unsigned short    d_reclen;
  unsigned char     d_type;
  char              d_name[256];
};

struct host_statfs_s 
{
  size_t            f_type;       /* Type of file system */
  size_t            f_bsize;      /* Optimal transfer block size */
  size_t            f_blocks;     /* Total data blocks in the file system */
  size_t            f_bfree;      /* Free blocks */
  size_t            f_bavail;     /* Free blocks available */
  size_t            f_files;      /* Total file nodes in file system */
  size_t            f_ffree;      /* Free file nodes in fs */
  size_t            f_fsid;       /* File Systme ID */
  size_t            f_namelen;    /* Max length of filenames */
  size_t            f_frsize;     /* Fragment size */
};

struct host_stat_s 
{
  int               st_dev;       /* ID of the device containing file */
  size_t            st_ino;       /* inode number */
  size_t            st_mode;      /* protection */
  size_t            st_nlink;     /* number of hard links */
  size_t            st_uid;       /* user ID of owner */
  size_t            st_gid;       /* group ID of owner */
  size_t            st_rdev;      /* device ID */
  size_t            st_size;      /* total size, in bytes */
  size_t            st_blksize;   /* blocksize for file system I/O */
  size_t            st_blocks;    /* number of 512B blocks allocated */
  size_t            st_atim;      /* time of last access */
  size_t            st_mtim;      /* time of last modification */
  size_t            st_ctim;      /* time of last status change */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int           host_open(const char *pathname, int flags, int mode);
int           host_close(int fd);
ssize_t       host_read(int fd, void* buf, size_t count);
ssize_t       host_write(int fd, const void *buf, size_t count);
off_t         host_lseek(int fd, off_t offset, int whence);
int           host_ioctl(int fd, int request, unsigned long arg);
void          host_sync(int fd);
int           host_dup(int fd);
void         *host_opendir(const char *name);
int           host_readdir(void* dirp, struct host_dirent_s* entry);
void          host_rewinddir(void* dirp);
int           host_closedir(void* dirp);
int           host_statfs(const char *path, struct host_statfs_s *buf);
int           host_unlink(const char *pathname);
int           host_mkdir(const char *pathname, mode_t mode);
int           host_rmdir(const char *pathname);
int           host_rename(const char *oldpath, const char *newpath);
int           host_stat(const char *path, struct host_stat_s *buf);

#endif /* __INCLUDE_NUTTX_FS_HOSTFS_H */
