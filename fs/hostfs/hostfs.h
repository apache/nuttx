/****************************************************************************
 * nuttx/fs/hostfs/hostfs.h
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

#ifndef __FS_HOSTFS_HOSTFS_H
#define __FS_HOSTFS_HOSTFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Quasi-standard definitions */

#ifndef MIN
#  define MIN(a,b)          (a < b ? a : b)
#endif

#ifndef MAX
#  define MAX(a,b)          (a > b ? a : b)
#endif

#define HOSTFS_MAX_PATH     256

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes the state of one open file.  This structure
 * is protected by the volume semaphore.
 */

struct hostfs_ofile_s
{
  struct hostfs_ofile_s    *fnext;      /* Supports a singly linked list */
  int16_t                   crefs;      /* Reference count */
  mode_t                    oflags;     /* Open mode */
  int                       fd;
};

/* This structure represents the overall mountpoint state.  An instance of this
 * structure is retained as inode private data on each mountpoint that is
 * mounted with a hostfs filesystem.
 */

struct hostfs_mountpt_s
{
  sem_t                      *fs_sem;       /* Used to assure thread-safe access */
  FAR struct hostfs_ofile_s  *fs_head;      /* A singly-linked list of open files */
  char                        fs_root[HOSTFS_MAX_PATH];
};

/****************************************************************************
 * Internal function prototypes
 ****************************************************************************/

/* Semaphore access for internal use */

void hostfs_semtake(struct hostfs_mountpt_s *fs);
void hostfs_semgive(struct hostfs_mountpt_s *fs);

/* Forward references for utility functions */

struct hostfs_mountpt_s;

struct file;        /* Forward references */
struct inode;
struct fs_dirent_s;
struct statfs;
struct stat;

#endif /* __FS_HOSTFS_HOSTFS_H */
