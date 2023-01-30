/********************************************************************************
 * include/fcntl.h
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
 ********************************************************************************/

#ifndef __INCLUDE_FCNTL_H
#define __INCLUDE_FCNTL_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

/* open flag settings for open() (and related APIs) */

#define O_RDONLY    (1 << 0)        /* Open for read access (only) */
#define O_RDOK      O_RDONLY        /* Read access is permitted (non-standard) */
#define O_WRONLY    (1 << 1)        /* Open for write access (only) */
#define O_WROK      O_WRONLY        /* Write access is permitted (non-standard) */
#define O_RDWR      (O_RDOK|O_WROK) /* Open for both read & write access */
#define O_CREAT     (1 << 2)        /* Create file/sem/mq object */
#define O_EXCL      (1 << 3)        /* Name must not exist when opened  */
#define O_APPEND    (1 << 4)        /* Keep contents, append to end */
#define O_TRUNC     (1 << 5)        /* Delete contents */
#define O_NONBLOCK  (1 << 6)        /* Don't wait for data */
#define O_NDELAY    O_NONBLOCK      /* Synonym for O_NONBLOCK */
#define O_SYNC      (1 << 7)        /* Synchronize output on write */
#define O_DSYNC     O_SYNC          /* Equivalent to OSYNC in NuttX */
#define O_TEXT      (1 << 8)        /* Open the file in text (translated) mode. */
#define O_DIRECT    (1 << 9)        /* Avoid caching, write directly to hardware */
#define O_CLOEXEC   (1 << 10)       /* Close on execute */
#define O_DIRECTORY (1 << 11)       /* Must be a directory */
#define O_NOFOLLOW  (1 << 12)       /* Don't follow links */

/* Unsupported, but required open flags */

#define O_RSYNC     0               /* Synchronize input on read */
#define O_ACCMODE   O_RDWR          /* Mask for access mode */
#define O_NOCTTY    0               /* Required by POSIX */
#define O_BINARY    0               /* Open the file in binary (untranslated) mode. */

/* This is the highest bit number used in the open flags bitset.  Bits above
 * this bit number may be used within NuttX for other, internal purposes.
 */

#define _O_MAXBIT   8

/* Synonyms historically used as F_SETFL flags (BSD). */

#define FNDELAY     O_NONBLOCK        /* Don't wait for data */
#define FNONBLOCK   O_NONBLOCK        /* Don't wait for data */
#define FAPPEND     O_APPEND          /* Keep contents, append to end */
#define FSYNC       O_SYNC            /* Synchronize output on write */
#define FASYNC      0                 /* No counterpart in NuttX */

/* FFCNTL is all the bits that may be set via fcntl. */

#define FFCNTL      (FNONBLOCK | FNDELAY | FAPPEND | FSYNC | FASYNC)

/* fcntl() commands */

#define F_DUPFD         0  /* Duplicate a file descriptor */
#define F_GETFD         1  /* Read the file descriptor flags */
#define F_GETFL         2  /* Read the file status flags */
#define F_GETLEASE      3  /* Indicates what type of lease is held on fd (linux) */
#define F_GETLK         4  /* Check if we could place a lock */
#define F_GETOWN        5  /* Get the pid receiving  SIGIO and SIGURG signals for fd */
#define F_GETSIG        6  /* Get the signal sent */
#define F_NOTIFY        7  /* Provide notification when directory referred to by fd changes (linux)*/
#define F_SETFD         8  /* Set the file descriptor flags to value */
#define F_SETFL         9  /* Set the file status flags to the value */
#define F_SETLEASE      10 /* Set or remove file lease (linux) */
#define F_SETLK         11 /* Acquire or release a lock on range of bytes */
#define F_SETLKW        12 /* Like F_SETLK, but wait for lock to become available */
#define F_SETOWN        13 /* Set pid that will receive SIGIO and SIGURG signals for fd */
#define F_SETSIG        14 /* Set the signal to be sent */
#define F_GETPATH       15 /* Get the path of the file descriptor(BSD/macOS) */
#define F_ADD_SEALS     16 /* Add the bit-mask argument arg to the set of seals of the inode */
#define F_GET_SEALS     17 /* Get (as the function result) the current set of seals of the inode */
#define F_DUPFD_CLOEXEC 18 /* Duplicate file descriptor with close-on-exit set.  */

/* For posix fcntl() and lockf() */

#define F_RDLCK     0  /* Take out a read lease */
#define F_WRLCK     1  /* Take out a write lease */
#define F_UNLCK     2  /* Remove a lease */

/* close-on-exec flag for F_GETFD and F_SETFD */

#define FD_CLOEXEC  1

/* These are the notifications that can be received from F_NOTIFY (linux) */

#define DN_ACCESS   0  /* A file was accessed */
#define DN_MODIFY   1  /* A file was modified */
#define DN_CREATE   2  /* A file was created */
#define DN_DELETE   3  /* A file was unlinked */
#define DN_RENAME   4  /* A file was renamed */
#define DN_ATTRIB   5  /* Attributes of a file were changed */

/* Types of seals */

#define F_SEAL_SEAL         0x0001 /* Prevent further seals from being set */
#define F_SEAL_SHRINK       0x0002 /* Prevent file from shrinking */
#define F_SEAL_GROW         0x0004 /* Prevent file from growing */
#define F_SEAL_WRITE        0x0008 /* Prevent writes */
#define F_SEAL_FUTURE_WRITE 0x0010 /* Prevent future writes while mapped */

/* int creat(const char *path, mode_t mode);
 *
 * is equivalent to open with O_WRONLY|O_CREAT|O_TRUNC.
 */

#define creat(path, mode) open(path, O_WRONLY|O_CREAT|O_TRUNC, mode)

#if defined(CONFIG_FS_LARGEFILE) && defined(CONFIG_HAVE_LONG_LONG)
#  define F_GETLK64         F_GETLK
#  define F_SETLK64         F_SETLK
#  define F_SETLKW64        F_SETLKW

#  define flock64           flock
#  define open64            open
#  define openat64          openat
#  define creat64           creat
#  define fallocate64       fallocate
#  define posix_fadvise64   posix_fadvise
#  define posix_fallocate64 posix_fallocate
#endif

/********************************************************************************
 * Public Type Definitions
 ********************************************************************************/

/* struct flock is the third argument for F_GETLK, F_SETLK and F_SETLKW */

struct flock
{
  int16_t l_type;    /* Type of lock: F_RDLCK, F_WRLCK, F_UNLCK */
  int16_t l_whence;  /* How to interpret l_start: SEEK_SET, SEEK_CUR, SEEK_END */
  off_t   l_start;   /* Starting offset for lock */
  off_t   l_len;     /* Number of bytes to lock */
  pid_t   l_pid;     /* PID of process blocking our lock (F_GETLK only) */
};

/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/********************************************************************************
 * Public Data
 ********************************************************************************/

/* POSIX-like File System Interfaces */

int open(FAR const char *path, int oflag, ...);
int fcntl(int fd, int cmd, ...);

int posix_fallocate(int fd, off_t offset, off_t len);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_FCNTL_H */
