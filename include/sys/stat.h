/****************************************************************************
 * include/sys/stat.h
 *
 *   Copyright (C) 2007-2009, 2012, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

/* mode_t bit settings (most of these do not apply to Nuttx). This assumes
 * that the full size of a mode_t is 16-bits. (However, mode_t must be size
 * 'int' because it is promoted to size int when passed in varargs).
 *
 *   LTTT ...U UUGG GOOO
 *
 *   Bits 0-2:   Permissions for others
 *   Bits 3-5:   Group permissions
 *   Bits 6-8:   Owner permissions
 *   Bits 9-10:  Not used
 *   Bits 11-14: File type bits
 *   Bit 15:     Symbolic link
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

#define S_ISVTX     0          /* "Sticky" bit (not used) */
#define S_ISGID     0          /* Set group ID bit (not used)*/
#define S_ISUID     0          /* Set UID bit (not used) */

#define S_IFIFO     0          /* Bits 11-14: File type bits (not all used) */
#define S_IFCHR     (1 << 11)
#define S_IFDIR     (2 << 11)
#define S_IFBLK     (3 << 11)
#define S_IFREG     (4 << 11)
#define S_IFMQ      (5 << 11)
#define S_IFSEM     (6 << 11)
#define S_IFSHM     (7 << 11)
#define S_IFSOCK    (8 << 11)
#define S_IFMTD     (9 << 11)
#define s_IFTGT     (15 << 11) /* May be the target of a symbolic link */

#define S_IFLNK     (1 << 15)  /* Bit 15: Symbolic link */
#define S_IFMT      (31 << 11) /* Bits 11-15: Full file type */

/* File type macros that operate on an instance of mode_t */

#define S_ISFIFO(m) (0)
#define S_ISCHR(m)  (((m) & s_IFTGT) == S_IFCHR)
#define S_ISDIR(m)  (((m) & s_IFTGT) == S_IFDIR)
#define S_ISBLK(m)  (((m) & s_IFTGT) == S_IFBLK)
#define S_ISREG(m)  (((m) & s_IFTGT) == S_IFREG)
#define S_ISMQ(m)   (((m) & s_IFTGT) == S_IFMQ)
#define S_ISSEM(m)  (((m) & s_IFTGT) == S_IFSEM)
#define S_ISSHM(m)  (((m) & s_IFTGT) == S_IFSHM)
#define S_ISSOCK(m) (((m) & s_IFTGT) == S_IFSOCK)
#define S_ISMTD(m)  (((m) & s_IFTGT) == S_IFMTD)
#define S_ISLNK(m)  (((m) & S_IFLNK)   != 0)

/* The following macros are required by POSIX to acheive backward
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

  /* Internal fields.  These are part this specific implementation and
   * should not referenced by application code for portability reasons.
   */

#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  uint8_t   st_count;   /* Used internally to limit traversal of links */
#endif
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
int fstat(int fd, FAR struct stat *buf);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_STAT_H */
