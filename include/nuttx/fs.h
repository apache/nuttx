/************************************************************
 * fs.h
 *
 *   Copyright (C) 2007 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 * 3. Neither the name Gregory Nutt nor the names of its contributors may be
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
 ************************************************************/

#ifndef __FS_H
#define __FS_H

/************************************************************
 * Included Files
 ************************************************************/

#include <sys/types.h>
#include <semaphore.h>
#include <nuttx/compiler.h>

/************************************************************
 * Definitions
 ************************************************************/

/************************************************************
 * Type Definitions
 ************************************************************/

/* This structure is provided by filesystems when they are
 * registered with the system.  It is used to call back to
 * perform fs/device specific operations.
 */

struct file;
struct file_operations
{
  int     (*open)(FAR struct file *);
  int     (*close)(FAR struct file *);
  ssize_t (*read)(FAR struct file *, char *, size_t);
  ssize_t (*write)(FAR struct file *, const char *, size_t);
  int     (*ioctl)(FAR struct file *, int, unsigned long);
};

/* This structure represents one inode in the Nuttx psuedo-file system */

struct inode
{
  FAR struct inode           *i_peer;    /* Pointer to inode at same level */
  FAR struct inode           *i_child;   /* Pointer to inode at lower level */
  struct file_operations     *i_ops;     /* Driver file operations for inode */
  sint16                      i_crefs;   /* References to inode */
  uint16                      i_flags;   /* flags for inode */
#ifdef CONFIG_FILE_MODE
  mode_t                      i_mode;    /* Access mode flags */
#endif
  FAR void                   *i_private; /* Driver private data */
  char                        i_name[1]; /* Name of inode (variable length) */
};
#define FSNODE_SIZE(n) (sizeof(struct inode) + (n))

/* This is the underlying representation of a ropen file.
 * A file descriptor is an index into an array of such types.
 * The type associates the file descriptor to the file state
 * and to a set of inode operations.
 */

struct file
{
  int               f_oflags; /* Open mode flags */
  off_t             f_pos;    /* File position */
  FAR struct inode *f_inode;  /* Driver interface */
};

/* This defines a list of files indexed by the file descriptor */

#if CONFIG_NFILE_DESCRIPTORS > 0
struct filelist
{
  sem_t   fl_sem;         /* Manage access to the file list */
  sint16  fl_crefs;       /* Reference count */
  struct file fl_files[CONFIG_NFILE_DESCRIPTORS];
};
#endif

/* This defines the list of files used for standard C I/O
 * We can support the standard C APIs without or without buffering
 */

/* Buffered file I/O structure */

#if CONFIG_NFILE_STREAMS > 0
struct file_struct
{
  int                fs_filedes;   /* File descriptor associated with stream */
  mode_t             fs_oflags;    /* Open mode flags */
#if CONFIG_NUNGET_CHARS > 0
  uint8              fs_nungotten; /* The number of characters buffered for ungetc */
  unsigned char      fs_ungotten[CONFIG_NUNGET_CHARS];
#endif
#if CONFIG_STDIO_BUFFER_SIZE > 0
  sem_t              fs_sem;       /* For thread safety */
  pid_t              fs_holder;    /* Holder of sem */
  int                fs_counts;    /* Number of times sem is held */
  FAR unsigned char *fs_bufstart;  /* Pointer to start of buffer */
  FAR unsigned char *fs_bufend;    /* Pointer to 1 past end of buffer */
  FAR unsigned char *fs_bufpos;    /* Current position in buffer */
  FAR unsigned char *fs_bufread;   /* Pointer to 1 past last buffered read char. */
#endif
};

struct streamlist
{
  int                 sl_crefs; /* Reference count */
  sem_t               sl_sem;   /* For thread safety */
  struct file_struct sl_streams[CONFIG_NFILE_STREAMS];
};
#endif /* CONFIG_NFILE_STREAMS */

/************************************************************
 * Global Function Prototypes
 ************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* fs_inode.c ***********************************************/

/* These interfaces are used by drivers to register their
 * inodes in the inode tree.
 */

EXTERN void   weak_function fs_initialize(void);
EXTERN STATUS register_inode(const char *path,
                             struct file_operations *fops,
                             mode_t mode, void *private);
EXTERN STATUS unregister_inode(const char *path);

/* fs_open.c ************************************************/

EXTERN int   inode_checkflags(FAR struct inode *inode, int oflags);

/* fs_files.c ***********************************************/

#if CONFIG_NFILE_DESCRIPTORS >0
EXTERN FAR struct filelist *files_alloclist(void);
EXTERN int files_addreflist(FAR struct filelist *list);
EXTERN int files_releaselist(FAR struct filelist *list);
EXTERN int files_dup(FAR struct file *filep1, FAR struct file *filep2);
#endif

/* lib_fopen.c **********************************************/

/* Used by the OS to clone stdin, stdout, stderr */

#if CONFIG_NFILE_STREAMS > 0
EXTERN FAR struct file_struct *lib_fdopen(int fd,
                                          const char *mode,
                                          FAR struct filelist *flist,
                                          FAR struct streamlist *slist);
#endif

/* lib_fflush.c *********************************************/

#if CONFIG_NFILE_STREAMS > 0
EXTERN void lib_flushall(FAR struct streamlist *list);
#endif

/* drivers **************************************************/

/* Call in of these to register the corresponding default 
 * default drivers in the drivers/ subdirectory
 */

EXTERN void devnull_register(void);


#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __FS_H */
