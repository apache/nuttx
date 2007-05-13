/****************************************************************************
 * nutts/fs.h
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
 ****************************************************************************/

#ifndef __NUTTX_FS_H
#define __NUTTX_FS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <semaphore.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* This structure is provided by devices when they are registered with the
 * system.  It is used to call back to perform device specific operations.
 */

struct file;
struct file_operations
{
  /* The device driver open method differs from the mountpoint open method */

  int     (*open)(FAR struct file *filp);

  /* The following methods must be identical in signature and position because
   * the struct file_operations and struct mountp_operations are treated like
   * unions.
   */

  int     (*close)(FAR struct file *filp);
  ssize_t (*read)(FAR struct file *filp, char *buffer, size_t buflen);
  ssize_t (*write)(FAR struct file *filp, const char *buffer, size_t buflen);
  off_t   (*seek)(FAR struct file *filp, off_t offset, int whence);
  int     (*ioctl)(FAR struct file *filp, int cmd, unsigned long arg);

  /* The two structures need not be common after this point */
};

/* This structure provides information about the state of a block driver */

struct geometry
{
  boolean geo_available;    /* TRUE: The device is vailable */
  boolean geo_mediachanged; /* TRUE: The media has changed since last query */
  boolean geo_writeenabled; /* TRUE: It is okay to write to this device */
  size_t  geo_nsectors;     /* Number of sectors on the device */
  size_t  geo_sectorsize;   /* Size of one sector */
};

/* This structure is provided by block devices when they register with the
 * system.  It is used by file systems to perform filesystem transfers.  It
 * differs from the normal driver vtable in several ways -- most notably in
 * that it deals in struct inode vs. struct filep.
 */

struct inode;
struct block_operations
{
  int     (*open)(FAR struct inode *inode);
  int     (*close)(FAR struct inode *inode);
  ssize_t (*read)(FAR struct inode *inode, unsigned char *buffer,
                  size_t start_sector, size_t nsectors);
  ssize_t (*write)(FAR struct inode *inode, const unsigned char *buffer,
                   size_t start_sector, size_t nsectors);
  int     (*geometry)(FAR struct inode *inode, struct geometry *geometry);
  int     (*ioctl)(FAR struct inode *inode, int cmd, unsigned long arg);
};

/* This structure is provided by a filesystem to describe a mount point.
 * Note that this structure differs from file_operations ONLY in the form of
 * the open method.  Once the file is opened, it can be accessed either as a
 * struct file_operations or struct mountpt_operations
 */

struct inode;
struct mountpt_operations
{
  /* The mountpoint open method differs from the driver open method
   * because it receives the relative path into the mountpoint.
   */

  int     (*open)(FAR struct file *filp, const char *rel_path);

  /* The following methods must be identical in signature and position because
   * the struct file_operations and struct mountp_operations are treated like
   * unions.
   */

  int     (*close)(FAR struct file *filp);
  ssize_t (*read)(FAR struct file *filp, char *buffer, size_t buflen);
  ssize_t (*write)(FAR struct file *filp, const char *buffer, size_t buflen);
  off_t   (*seek)(FAR struct file *filp, off_t offset, int whence);
  int     (*ioctl)(FAR struct file *filp, int cmd, unsigned long arg);

  /* The two structures need not be common after this point.  For the
   * case of struct mountpt_operations, additional operations are included
   * that used only for mounting and unmounting the volume.
   */

  int   (*bind)(FAR struct inode *blkdriver, const void *data, void **handle);
  int   (*unbind)(void *handle);

  /* NOTE:  More operations will be needed here to support:  disk usage stats, stat(),
   * sync(), unlink(), mkdir(), chmod(), rename(), etc.
   */
};

/* This structure represents one inode in the Nuttx psuedo-file system */

struct inode
{
  FAR struct inode            *i_peer;       /* Pointer to same level inode */
  FAR struct inode            *i_child;      /* Pointer to lower level inode */
  sint16                       i_crefs;      /* References to inode */
  uint16                       i_flags;      /* flags for inode */
  union
  {
    const struct file_operations    *i_ops;  /* Driver operations for inode */
    const struct block_operations   *i_bops; /* Block driver operations */
    const struct mountpt_operations *i_mops; /* Operations on a mountpoint */
  } u;
#ifdef CONFIG_FILE_MODE
  mode_t                       i_mode;       /* Access mode flags */
#endif
  FAR void                    *i_private;    /* Per inode driver private data */
  char                         i_name[1];    /* Name of inode (variable) */
};
#define FSNODE_SIZE(n) (sizeof(struct inode) + (n))

/* This is the underlying representation of an open file.  A file
 * descriptor is an index into an array of such types. The type associates
 * the file descriptor to the file state and to a set of inode operations.
 */

struct file
{
  int               f_oflags; /* Open mode flags */
  off_t             f_pos;    /* File position */
  FAR struct inode *f_inode;  /* Driver interface */
  void             *f_priv;   /* Per file driver private data */
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

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* fs_inode.c ***************************************************************/

/* These interfaces are used by drivers to register their
 * inodes in the inode tree.
 */

EXTERN void   weak_function fs_initialize(void);

/* fs_registerdriver.c ******************************************************/

EXTERN STATUS register_driver(const char *path,
                              const struct file_operations *fops,
                              mode_t mode, void *private);

/* fs_registerdriver.c ******************************************************/

EXTERN STATUS register_blockdriver(const char *path,
                                   const struct block_operations *bops,
                                   mode_t mode, void *private);

/* fs_unregisterdriver.c ****************************************************/

EXTERN STATUS unregister_driver(const char *path);

/* fs_unregisterblockdriver.c ***********************************************/

EXTERN STATUS unregister_blockdriver(const char *path);

/* fs_open.c ****************************************************************/

EXTERN int   inode_checkflags(FAR struct inode *inode, int oflags);

/* fs_files.c ***************************************************************/

#if CONFIG_NFILE_DESCRIPTORS >0
EXTERN FAR struct filelist *files_alloclist(void);
EXTERN int files_addreflist(FAR struct filelist *list);
EXTERN int files_releaselist(FAR struct filelist *list);
EXTERN int files_dup(FAR struct file *filep1, FAR struct file *filep2);
#endif

/* lib_fopen.c **************************************************************/

/* Used by the OS to clone stdin, stdout, stderr */

#if CONFIG_NFILE_STREAMS > 0
EXTERN FAR struct file_struct *lib_fdopen(int fd,
                                          const char *mode,
                                          FAR struct filelist *flist,
                                          FAR struct streamlist *slist);
#endif

/* lib_fflush.c *************************************************************/

#if CONFIG_NFILE_STREAMS > 0
EXTERN void lib_flushall(FAR struct streamlist *list);
#endif

/* drivers ******************************************************************/

/* Call in of these to register the corresponding default 
 * default drivers in the drivers/ subdirectory
 */

EXTERN void devnull_register(void);


#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __NUTTX_FS_H */
