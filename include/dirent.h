/****************************************************************************
 * include/dirent.h
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

#ifndef __INCLUDE_DIRENT_H
#define __INCLUDE_DIRENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <limits.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The d_type field of the dirent structure is not specified by POSIX.  It
 * is a non-standard, 4.5BSD extension that is implemented by most OSs.  A
 * POSIX compliant OS may not implement the d_type field at all.  Many OS's
 * (including glibc) may use the following alternative naming for the file
 * type names:
 */

#define DT_UNKNOWN                0
#define DT_FIFO                   1
#define DT_CHR                    2
#define DT_SEM                    3
#define DT_DIR                    4
#define DT_MQ                     5
#define DT_BLK                    6
#define DT_SHM                    7
#define DT_REG                    8
#define DT_MTD                    9
#define DT_LNK                    10
#define DT_SOCK                   12

/* File type code for the d_type field in dirent structure.
 * Note that because of the simplified filesystem organization of the NuttX,
 * top-level, pseudo-file system, an inode can be BOTH a file and a directory
 */

#define DTYPE_UNKNOWN             DT_UNKNOWN
#define DTYPE_FIFO                DT_FIFO
#define DTYPE_CHR                 DT_CHR
#define DTYPE_SEM                 DT_SEM
#define DTYPE_DIRECTORY           DT_DIR
#define DTYPE_MQ                  DT_MQ
#define DTYPE_BLK                 DT_BLK
#define DTYPE_SHM                 DT_SHM
#define DTYPE_FILE                DT_REG
#define DTYPE_MTD                 DT_MTD
#define DTYPE_LINK                DT_LNK
#define DTYPE_SOCK                DT_SOCK

#define DIRENT_ISUNKNOWN(dtype)   ((dtype) == DTYPE_UNKNOWN)
#define DIRENT_ISFIFO(dtype)      ((dtype) == DTYPE_FIFO)
#define DIRENT_ISCHR(dtype)       ((dtype) == DTYPE_CHR)
#define DIRENT_ISSEM(dtype)       ((dtype) == DTYPE_SEM)
#define DIRENT_ISDIRECTORY(dtype) ((dtype) == DTYPE_DIRECTORY)
#define DIRENT_ISMQ(dtype)        ((dtype) == DTYPE_MQ)
#define DIRENT_ISBLK(dtype)       ((dtype) == DTYPE_BLK)
#define DIRENT_ISSHM(dtype)       ((dtype) == DTYPE_SHM)
#define DIRENT_ISFILE(dtype)      ((dtype) == DTYPE_FILE)
#define DIRENT_ISMTD(dtype)       ((dtype) == DTYPE_MTD)
#define DIRENT_ISLINK(dtype)      ((dtype) == DTYPE_LINK)
#define DIRENT_ISSOCK(dtype)      ((dtype) == DTYPE_SOCK)

#if defined(CONFIG_FS_LARGEFILE) && defined(CONFIG_HAVE_LONG_LONG)
#  define dirent64                dirent
#  define readdir64               readdir
#  define readdir64_r             readdir_r
#  define scandir64               scandir
#  define alphasort64             alphasort
#  define versionsort64           versionsort
#  define getdents64              getdents
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* The POSIX specification requires that the caller of readdir_r provide
 * storage "large enough for a dirent with the d_name member and an array
 * of char containing at least {NAME_MAX} plus one elements.
 *
 * POSIX also requires the field d_ino (type ino_t) that provides the file
 * serial number.  This functionality is not implemented in NuttX.
 */

struct dirent
{
  uint8_t  d_type;                /* Type of file */
  char     d_name[NAME_MAX + 1];  /* File name */
};

typedef struct
{
  int fd;
  struct dirent entry;
} DIR;

/****************************************************************************
 * Public Data
 ****************************************************************************/

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

/* POSIX-like File System Interfaces */

int        closedir(FAR DIR *dirp);
FAR DIR   *opendir(FAR const char *path);
FAR DIR   *fdopendir(int fd);
FAR struct dirent *readdir(FAR DIR *dirp);
int        readdir_r(FAR DIR *dirp, FAR struct dirent *entry,
                     FAR struct dirent **result);
void       rewinddir(FAR DIR *dirp);
void       seekdir(FAR DIR *dirp, off_t loc);
off_t      telldir(FAR DIR *dirp);
int        scandir(FAR const char *path, FAR struct dirent ***namelist,
                   CODE int (*filter)(FAR const struct dirent *),
                   CODE int (*compar)(FAR const struct dirent **,
                                      FAR const struct dirent **));
int        alphasort(FAR const struct dirent **a,
                     FAR const struct dirent **b);
int        versionsort(FAR const struct dirent **a,
                       FAR const struct dirent **b);

int        dirfd(FAR DIR *dirp);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_DIRENT_H */
