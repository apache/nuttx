/****************************************************************************
 * include/nuttx/fs/fs.h
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

#ifndef __INCLUDE_NUTTX_FS_FS_H
#define __INCLUDE_NUTTX_FS_FS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <dirent.h>

#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/mm/map.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Most internal OS interfaces are not available in the user space in
 * PROTECTED and KERNEL builds.  In that context, the corresponding
 * application interfaces must be used.  The differences between the two
 * sets of interfaces are:  The internal OS interfaces (1) do not cause
 * cancellation points and (2) they do not modify the errno variable.
 *
 * This is only important when compiling libraries (libc or libnx) that are
 * used both by the OS (libkc.a and libknx.a) or by the applications
 * (libc.a and libnx.a).  In that case, the correct interface must be
 * used for the build context.
 *
 * REVISIT:  In the flat build, the same functions must be used both by
 * the OS and by applications.  We have to use the normal user functions
 * in this case or we will fail to set the errno or fail to create the
 * cancellation point.
 *
 * The interfaces close(), creat(), read(), pread(), write(), pwrite(),
 * poll(), select(), fcntl(), and aio_suspend() are all cancellation
 * points.
 *
 * REVISIT:  These cancellation points are an issue and may cause
 * violations:  It use of these internally will cause the calling function
 * to become a cancellation points!
 */

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)
#  define _NX_OPEN             nx_open
#  define _NX_CLOSE(f)         nx_close(f)
#  define _NX_READ(f,b,s)      nx_read(f,b,s)
#  define _NX_WRITE(f,b,s)     nx_write(f,b,s)
#  define _NX_SEEK(f,o,w)      nx_seek(f,o,w)
#  define _NX_STAT(p,s)        nx_stat(p,s,1)
#  define _NX_GETERRNO(r)      (-(r))
#  define _NX_SETERRNO(r)      set_errno(-(r))
#  define _NX_GETERRVAL(r)     (r)
#else
#  define _NX_OPEN             open
#  define _NX_CLOSE(f)         close(f)
#  define _NX_READ(f,b,s)      read(f,b,s)
#  define _NX_WRITE(f,b,s)     write(f,b,s)
#  define _NX_SEEK(f,o,w)      lseek(f,o,w)
#  define _NX_STAT(p,s)        stat(p,s)
#  define _NX_GETERRNO(r)      errno
#  define _NX_SETERRNO(r)      ((void)(r))
#  define _NX_GETERRVAL(r)     (-errno)
#endif

/* Stream flags for the fs_flags field of in struct file_struct */

#define __FS_FLAG_EOF   (1 << 0) /* EOF detected by a read operation */
#define __FS_FLAG_ERROR (1 << 1) /* Error detected by any operation */
#define __FS_FLAG_LBF   (1 << 2) /* Line buffered */
#define __FS_FLAG_UBF   (1 << 3) /* Buffer allocated by caller of setvbuf */

/* Inode i_flags values:
 *
 *   Bit 0-3: Inode type (Bit 3 indicates internal OS types)
 *   Bit 4:   Set if inode has been unlinked and is pending removal.
 */

#define FSNODEFLAG_TYPE_MASK        0x0000000f /* Isolates type field      */
#define   FSNODEFLAG_TYPE_PSEUDODIR 0x00000000 /*   Pseudo dir (default)   */
#define   FSNODEFLAG_TYPE_DRIVER    0x00000001 /*   Character driver       */
#define   FSNODEFLAG_TYPE_BLOCK     0x00000002 /*   Block driver           */
#define   FSNODEFLAG_TYPE_MOUNTPT   0x00000003 /*   Mount point            */
#define   FSNODEFLAG_TYPE_NAMEDSEM  0x00000004 /*   Named semaphore        */
#define   FSNODEFLAG_TYPE_MQUEUE    0x00000005 /*   Message Queue          */
#define   FSNODEFLAG_TYPE_SHM       0x00000006 /*   Shared memory region   */
#define   FSNODEFLAG_TYPE_MTD       0x00000007 /*   Named MTD driver       */
#define   FSNODEFLAG_TYPE_SOFTLINK  0x00000008 /*   Soft link              */
#define   FSNODEFLAG_TYPE_SOCKET    0x00000009 /*   Socket                 */
#define FSNODEFLAG_DELETED          0x00000010 /* Unlinked                 */

#define INODE_IS_TYPE(i,t) \
  (((i)->i_flags & FSNODEFLAG_TYPE_MASK) == (t))

#define INODE_IS_PSEUDODIR(i) INODE_IS_TYPE(i,FSNODEFLAG_TYPE_PSEUDODIR)
#define INODE_IS_DRIVER(i)    INODE_IS_TYPE(i,FSNODEFLAG_TYPE_DRIVER)
#define INODE_IS_BLOCK(i)     INODE_IS_TYPE(i,FSNODEFLAG_TYPE_BLOCK)
#define INODE_IS_MOUNTPT(i)   INODE_IS_TYPE(i,FSNODEFLAG_TYPE_MOUNTPT)
#define INODE_IS_NAMEDSEM(i)  INODE_IS_TYPE(i,FSNODEFLAG_TYPE_NAMEDSEM)
#define INODE_IS_MQUEUE(i)    INODE_IS_TYPE(i,FSNODEFLAG_TYPE_MQUEUE)
#define INODE_IS_SHM(i)       INODE_IS_TYPE(i,FSNODEFLAG_TYPE_SHM)
#define INODE_IS_MTD(i)       INODE_IS_TYPE(i,FSNODEFLAG_TYPE_MTD)
#define INODE_IS_SOFTLINK(i)  INODE_IS_TYPE(i,FSNODEFLAG_TYPE_SOFTLINK)
#define INODE_IS_SOCKET(i)    INODE_IS_TYPE(i,FSNODEFLAG_TYPE_SOCKET)

#define INODE_GET_TYPE(i)     ((i)->i_flags & FSNODEFLAG_TYPE_MASK)
#define INODE_SET_TYPE(i,t) \
  do \
    { \
      (i)->i_flags = ((i)->i_flags & ~FSNODEFLAG_TYPE_MASK) | (t); \
    } \
  while (0)

#define INODE_SET_DRIVER(i)   INODE_SET_TYPE(i,FSNODEFLAG_TYPE_DRIVER)
#define INODE_SET_BLOCK(i)    INODE_SET_TYPE(i,FSNODEFLAG_TYPE_BLOCK)
#define INODE_SET_MOUNTPT(i)  INODE_SET_TYPE(i,FSNODEFLAG_TYPE_MOUNTPT)
#define INODE_SET_NAMEDSEM(i) INODE_SET_TYPE(i,FSNODEFLAG_TYPE_NAMEDSEM)
#define INODE_SET_MQUEUE(i)   INODE_SET_TYPE(i,FSNODEFLAG_TYPE_MQUEUE)
#define INODE_SET_SHM(i)      INODE_SET_TYPE(i,FSNODEFLAG_TYPE_SHM)
#define INODE_SET_MTD(i)      INODE_SET_TYPE(i,FSNODEFLAG_TYPE_MTD)
#define INODE_SET_SOFTLINK(i) INODE_SET_TYPE(i,FSNODEFLAG_TYPE_SOFTLINK)
#define INODE_SET_SOCKET(i)   INODE_SET_TYPE(i,FSNODEFLAG_TYPE_SOCKET)

/* The status change flags.
 * These should be or-ed together to figure out what want to change.
 */

#define CH_STAT_MODE       (1 << 0)
#define CH_STAT_UID        (1 << 1)
#define CH_STAT_GID        (1 << 2)
#define CH_STAT_ATIME      (1 << 3)
#define CH_STAT_MTIME      (1 << 4)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Forward references */

struct file;
struct inode;
struct stat;
struct statfs;
struct pollfd;
struct mtd_dev_s;
struct tcb_s;

/* The internal representation of type DIR is just a container for an inode
 * reference, and the path of directory.
 */

struct fs_dirent_s
{
  /* This is the node that was opened by opendir.  The type of the inode
   * determines the way that the readdir() operations are performed. For the
   * pseudo root pseudo-file system, it is also used to support rewind.
   *
   * We hold a reference on this inode so we know that it will persist until
   * closedir() is called (although inodes linked to this inode may change).
   */

  FAR struct inode *fd_root;

  /* The path name of current directory for FIOC_FILEPATH */

  FAR char *fd_path;
};

/* This structure is provided by devices when they are registered with the
 * system.  It is used to call back to perform device specific operations.
 */

struct file_operations
{
  /* The device driver open method differs from the mountpoint open method */

  CODE int     (*open)(FAR struct file *filep);

  /* The following methods must be identical in signature and position
   * because the struct file_operations and struct mountpt_operations are
   * treated like unions.
   */

  CODE int     (*close)(FAR struct file *filep);
  CODE ssize_t (*read)(FAR struct file *filep, FAR char *buffer,
                       size_t buflen);
  CODE ssize_t (*write)(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen);
  CODE off_t   (*seek)(FAR struct file *filep, off_t offset, int whence);
  CODE int     (*ioctl)(FAR struct file *filep, int cmd, unsigned long arg);
  CODE int     (*mmap)(FAR struct file *filep,
                       FAR struct mm_map_entry_s *map);
  int     (*truncate)(FAR struct file *filep, off_t length);

  /* The two structures need not be common after this point */

  CODE int     (*poll)(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  CODE int     (*unlink)(FAR struct inode *inode);
#endif
};

/* This structure provides information about the state of a block driver */

#ifndef CONFIG_DISABLE_MOUNTPOINT
struct geometry
{
  bool      geo_available;    /* true: The device is available */
  bool      geo_mediachanged; /* true: The media has changed since last query */
  bool      geo_writeenabled; /* true: It is okay to write to this device */
  blkcnt_t  geo_nsectors;     /* Number of sectors on the device */
  blksize_t geo_sectorsize;   /* Size of one sector */

  /* NULL-terminated string representing the device model */

  char      geo_model[NAME_MAX + 1];
};

struct partition_info_s
{
  size_t    numsectors;   /* Number of sectors in the partition */
  size_t    sectorsize;   /* Size in bytes of a single sector */
  off_t     startsector;  /* Offset to the first section/block of the
                           * managed sub-region */

  /* NULL-terminated string representing the name of the parent node of the
   * partition.
   */

  char      parent[NAME_MAX + 1];
};

/* This structure is provided by block devices when they register with the
 * system.  It is used by file systems to perform filesystem transfers.  It
 * differs from the normal driver vtable in several ways -- most notably in
 * that it deals in struct inode vs. struct filep.
 */

struct inode;
struct block_operations
{
  CODE int     (*open)(FAR struct inode *inode);
  CODE int     (*close)(FAR struct inode *inode);
  CODE ssize_t (*read)(FAR struct inode *inode, FAR unsigned char *buffer,
                       blkcnt_t start_sector, unsigned int nsectors);
  CODE ssize_t (*write)(FAR struct inode *inode,
                        FAR const unsigned char *buffer,
                        blkcnt_t start_sector, unsigned int nsectors);
  CODE int     (*geometry)(FAR struct inode *inode,
                           FAR struct geometry *geometry);
  CODE int     (*ioctl)(FAR struct inode *inode, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  CODE int     (*unlink)(FAR struct inode *inode);
#endif
};

/* This structure is provided by a filesystem to describe a mount point.
 * Note that this structure differs from file_operations ONLY in the form of
 * the open method.  Once the file is opened, it can be accessed either as a
 * struct file_operations or struct mountpt_operations
 */

struct mountpt_operations
{
  /* The mountpoint open method differs from the driver open method
   * because it receives (1) the inode that contains the mountpoint
   * private data, (2) the relative path into the mountpoint, and (3)
   * information to manage privileges.
   */

  CODE int     (*open)(FAR struct file *filep, FAR const char *relpath,
            int oflags, mode_t mode);

  /* The following methods must be identical in signature and position
   * because the struct file_operations and struct mountpt_operations are
   * treated like unions.
   */

  CODE int     (*close)(FAR struct file *filep);
  CODE ssize_t (*read)(FAR struct file *filep, FAR char *buffer,
                       size_t buflen);
  CODE ssize_t (*write)(FAR struct file *filep, FAR const char *buffer,
                        size_t buflen);
  CODE off_t   (*seek)(FAR struct file *filep, off_t offset, int whence);
  CODE int     (*ioctl)(FAR struct file *filep, int cmd, unsigned long arg);
  CODE int     (*mmap)(FAR struct file *filep,
                       FAR struct mm_map_entry_s *map);
  CODE int     (*truncate)(FAR struct file *filep, off_t length);

  /* The two structures need not be common after this point. The following
   * are extended methods needed to deal with the unique needs of mounted
   * file systems.
   *
   * Additional open-file-specific mountpoint operations:
   */

  CODE int     (*sync)(FAR struct file *filep);
  CODE int     (*dup)(FAR const struct file *oldp, FAR struct file *newp);
  CODE int     (*fstat)(FAR const struct file *filep, FAR struct stat *buf);
  CODE int     (*fchstat)(FAR const struct file *filep,
                          FAR const struct stat *buf, int flags);

  /* Directory operations */

  CODE int     (*opendir)(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct fs_dirent_s **dir);
  CODE int     (*closedir)(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir);
  CODE int     (*readdir)(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir,
                          FAR struct dirent *entry);
  CODE int     (*rewinddir)(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir);

  /* General volume-related mountpoint operations: */

  CODE int     (*bind)(FAR struct inode *blkdriver, FAR const void *data,
                       FAR void **handle);
  CODE int     (*unbind)(FAR void *handle, FAR struct inode **blkdriver,
                         unsigned int flags);
  CODE int     (*statfs)(FAR struct inode *mountpt, FAR struct statfs *buf);

  /* Operations on paths */

  CODE int     (*unlink)(FAR struct inode *mountpt, FAR const char *relpath);
  CODE int     (*mkdir)(FAR struct inode *mountpt, FAR const char *relpath,
                        mode_t mode);
  CODE int     (*rmdir)(FAR struct inode *mountpt, FAR const char *relpath);
  CODE int     (*rename)(FAR struct inode *mountpt,
                         FAR const char *oldrelpath,
                         FAR const char *newrelpath);
  CODE int     (*stat)(FAR struct inode *mountpt, FAR const char *relpath,
                       FAR struct stat *buf);
  CODE int     (*chstat)(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR const struct stat *buf, int flags);
};
#endif /* CONFIG_DISABLE_MOUNTPOINT */

/* Named OS resources are also maintained by the VFS.  This includes:
 *
 *   - Named semaphores:     sem_open(), sem_close(), and sem_unlink()
 *   - POSIX Message Queues: mq_open() and mq_close()
 *   - Shared memory:        shm_open() and shm_unlink();
 *
 * These are a special case in that they do not follow quite the same
 * pattern as the other file system types in that they have operations.
 */

/* These are the various kinds of operations that can be associated with
 * an inode.
 */

union inode_ops_u
{
  FAR const struct file_operations     *i_ops;    /* Driver operations for inode */
#ifndef CONFIG_DISABLE_MOUNTPOINT
  FAR const struct block_operations    *i_bops;   /* Block driver operations */
  FAR struct mtd_dev_s                 *i_mtd;    /* MTD device driver */
  FAR const struct mountpt_operations  *i_mops;   /* Operations on a mountpoint */
#endif
#ifdef CONFIG_FS_NAMED_SEMAPHORES
  FAR struct nsem_inode_s              *i_nsem;   /* Named semaphore */
#endif
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
  FAR char                             *i_link;   /* Full path to link target */
#endif
};

/* This structure represents one inode in the NuttX pseudo-file system */

struct inode
{
  FAR struct inode *i_parent;   /* Link to parent level inode */
  FAR struct inode *i_peer;     /* Link to same level inode */
  FAR struct inode *i_child;    /* Link to lower level inode */
  int16_t           i_crefs;    /* References to inode */
  uint16_t          i_flags;    /* Flags for inode */
  union inode_ops_u u;          /* Inode operations */
  ino_t             i_ino;      /* Inode serial number */
#ifdef CONFIG_PSEUDOFS_ATTRIBUTES
  mode_t            i_mode;     /* Access mode flags */
  uid_t             i_owner;    /* Owner */
  gid_t             i_group;    /* Group */
  struct timespec   i_atime;    /* Time of last access */
  struct timespec   i_mtime;    /* Time of last modification */
  struct timespec   i_ctime;    /* Time of last status change */
#endif
  FAR void         *i_private;  /* Per inode driver private data */
  char              i_name[1];  /* Name of inode (variable) */
};

#define FSNODE_SIZE(n) (sizeof(struct inode) + (n))

/* This is the underlying representation of an open file.  A file
 * descriptor is an index into an array of such types. The type associates
 * the file descriptor to the file state and to a set of inode operations.
 */

struct file
{
  int               f_oflags;   /* Open mode flags */
  off_t             f_pos;      /* File position */
  FAR struct inode *f_inode;    /* Driver or file system interface */
  FAR void         *f_priv;     /* Per file driver private data */
};

/* This defines a two layer array of files indexed by the file descriptor.
 * Each row of this array is fixed size: CONFIG_NFILE_DESCRIPTORS_PER_BLOCK.
 * You can get file instance in filelist by the follow methods:
 * (file descriptor / CONFIG_NFILE_DESCRIPTORS_PER_BLOCK) as row index and
 * (file descriptor % CONFIG_NFILE_DESCRIPTORS_PER_BLOCK) as column index.
 */

struct filelist
{
  mutex_t           fl_lock;    /* Manage access to the file list */
  uint8_t           fl_rows;    /* The number of rows of fl_files array */
  FAR struct file **fl_files;   /* The pointer of two layer file descriptors array */
};

/* The following structure defines the list of files used for standard C I/O.
 * Note that NuttX can support the standard C APIs with or without buffering
 *
 * When buffering is used, the following describes the usage of the I/O
 * buffer.
 * The buffer can be used for reading or writing -- but not both at the same
 * time.
 * An fflush is implied between each change in direction of access.
 *
 * The field fs_bufread determines whether the buffer is being used for
 * reading or for writing as follows:
 *
 *              BUFFER
 *     +----------------------+ <- fs_bufstart Points to the beginning of
 *     |                      |    the buffer.
 *     | WR: Buffered data    |                WR: Start of buffered write
 *     |                      |                    data.
 *     | RD: Already read     |                RD: Start of already read
 *     |                      |                    data.
 *     +----------------------+
 *     | WR: Available buffer | <- fs_bufpos   Points to next byte:
 *     | RD: Read-ahead data  |                WR: End+1 of buffered write
 *     |                      |                    data.
 *     |                      |                RD: Points to next char to
 *     |                      |                    return
 *     +----------------------+
 *     | WR: Available        | <- fs_bufread  Top+1 of buffered read data
 *     | RD: Available        |                WR: bufstart buffer used for
 *     |                      |                    writing.
 *     |                      |                RD: Pointer to last buffered
 *     |                      |                    read char+1
 *     +----------------------+
 *                              <- fs_bufend   Points to the end of the
 *                                             buffer+1
 */

#ifdef CONFIG_FILE_STREAM
struct file_struct
{
  FAR struct file_struct *fs_next;      /* Pointer to next file stream */
  rmutex_t                fs_lock;      /* Recursive lock */
  int                     fs_fd;        /* File descriptor associated with stream */
#ifndef CONFIG_STDIO_DISABLE_BUFFERING
  FAR unsigned char      *fs_bufstart;  /* Pointer to start of buffer */
  FAR unsigned char      *fs_bufend;    /* Pointer to 1 past end of buffer */
  FAR unsigned char      *fs_bufpos;    /* Current position in buffer */
  FAR unsigned char      *fs_bufread;   /* Pointer to 1 past last buffered read char. */
#  if CONFIG_STDIO_BUFFER_SIZE > 0
  unsigned char           fs_buffer[CONFIG_STDIO_BUFFER_SIZE];
#  endif
#endif
  uint16_t                fs_oflags;    /* Open mode flags */
  uint8_t                 fs_flags;     /* Stream flags */
#if CONFIG_NUNGET_CHARS > 0
  uint8_t                 fs_nungotten; /* The number of characters buffered for ungetc */
  unsigned char           fs_ungotten[CONFIG_NUNGET_CHARS];
#endif
};

struct streamlist
{
  mutex_t                 sl_lock;   /* For thread safety */
  struct file_struct      sl_std[3];
  FAR struct file_struct *sl_head;
  FAR struct file_struct *sl_tail;
};
#endif /* CONFIG_FILE_STREAM */

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

/****************************************************************************
 * Name: fs_initialize
 *
 * Description:
 *   This is called from the OS initialization logic to configure the file
 *   system.
 *
 ****************************************************************************/

void fs_initialize(void);

/****************************************************************************
 * Name: register_driver
 *
 * Description:
 *   Register a character driver inode the pseudo file system.
 *
 * Input Parameters:
 *   path - The path to the inode to create
 *   fops - The file operations structure
 *   mode - Access privileges
 *   priv - Private, user data that will be associated with the inode.
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on a failure (all error values returned by
 *   inode_reserve):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int register_driver(FAR const char *path,
                    FAR const struct file_operations *fops, mode_t mode,
                    FAR void *priv);

/****************************************************************************
 * Name: register_blockdriver
 *
 * Description:
 *   Register a block driver inode the pseudo file system.
 *
 * Input Parameters:
 *   path - The path to the inode to create
 *   bops - The block driver operations structure
 *   mode - Access privileges
 *   priv - Private, user data that will be associated with the inode.
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on a failure (all error values returned by
 *   inode_reserve):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
int register_blockdriver(FAR const char *path,
                         FAR const struct block_operations *bops,
                         mode_t mode, FAR void *priv);
#endif

/****************************************************************************
 * Name: register_blockpartition
 *
 * Description:
 *   Register a block partition driver inode the pseudo file system.
 *
 * Input Parameters:
 *   partition   - The path to the partition inode
 *   parent      - The path to the parent inode
 *   firstsector - The offset in sectors to the partition
 *   nsectors    - The number of sectors in the partition
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on a failure (all error values returned by
 *   inode_reserve):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
int register_blockpartition(FAR const char *partition,
                            mode_t mode, FAR const char *parent,
                            off_t firstsector, off_t nsectors);
#endif

/****************************************************************************
 * Name: unregister_driver
 *
 * Description:
 *   Remove the character driver inode at 'path' from the pseudo-file system
 *
 ****************************************************************************/

int unregister_driver(FAR const char *path);

/****************************************************************************
 * Name: unregister_blockdriver
 *
 * Description:
 *   Remove the block driver inode at 'path' from the pseudo-file system
 *
 ****************************************************************************/

int unregister_blockdriver(FAR const char *path);

/****************************************************************************
 * Name: register_mtddriver
 *
 * Description:
 *   Register an MTD driver inode the pseudo file system.
 *
 * Input Parameters:
 *   path - The path to the inode to create
 *   mtd  - The MTD driver structure
 *   mode - inode privileges
 *   priv - Private, user data that will be associated with the inode.
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on a failure (all error values returned by
 *   inode_reserve):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

#ifdef CONFIG_MTD
int register_mtddriver(FAR const char *path, FAR struct mtd_dev_s *mtd,
                       mode_t mode, FAR void *priv);
#endif

/****************************************************************************
 * Name: register_mtdpartition
 *
 * Description:
 *   Register a mtd partition driver inode the pseudo file system.
 *
 * Input Parameters:
 *   partition  - The path to the partition inode
 *   parent     - The path to the parent inode
 *   firstblock - The offset in block to the partition
 *   nblocks    - The number of block in the partition
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on a failure (all error values returned by
 *   inode_reserve):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

#ifdef CONFIG_MTD
int register_mtdpartition(FAR const char *partition,
                          mode_t mode, FAR const char *parent,
                          off_t firstblock, off_t nblocks);
#endif

/****************************************************************************
 * Name: unregister_mtddriver
 *
 * Description:
 *   Remove the named TMD driver inode at 'path' from the pseudo-file system
 *
 ****************************************************************************/

#ifdef CONFIG_MTD
int unregister_mtddriver(FAR const char *path);
#endif

/****************************************************************************
 * Name: nx_mount
 *
 * Description:
 *   nx_mount() is similar to the standard 'mount' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_mount() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   Zero is returned on success; a negated value is returned on any failure.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
int nx_mount(FAR const char *source, FAR const char *target,
             FAR const char *filesystemtype, unsigned long mountflags,
             FAR const void *data);
#endif

/****************************************************************************
 * Name: nx_umount2
 *
 * Description:
 *   nx_umount2() is similar to the standard 'umount2' interface except that
 *   is not a cancellation point and it does not modify the errno variable.
 *
 *   nx_umount2() is an internal NuttX interface and should not be called
 *   from applications.
 *
 * Returned Value:
 *   Zero is returned on success; a negated value is returned on any failure.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
int nx_umount2(FAR const char *target, unsigned int flags);
#endif

/****************************************************************************
 * Name: files_initlist
 *
 * Description:
 *   Initializes the list of files for a new task
 *
 ****************************************************************************/

void files_initlist(FAR struct filelist *list);

/****************************************************************************
 * Name: files_releaselist
 *
 * Description:
 *   Release a reference to the file list
 *
 ****************************************************************************/

void files_releaselist(FAR struct filelist *list);

/****************************************************************************
 * Name: files_duplist
 *
 * Description:
 *   Duplicate parent task's file descriptors.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int files_duplist(FAR struct filelist *plist, FAR struct filelist *clist);

/****************************************************************************
 * Name: file_allocate_from_tcb
 *
 * Description:
 *   Allocate a struct files instance and associate it with an inode
 *   instance.
 *
 * Returned Value:
 *     Returns the file descriptor == index into the files array on success;
 *     a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int file_allocate_from_tcb(FAR struct tcb_s *tcb, FAR struct inode *inode,
                           int oflags, off_t pos, FAR void *priv, int minfd,
                           bool addref);

/****************************************************************************
 * Name: file_allocate
 *
 * Description:
 *   Allocate a struct files instance and associate it with an inode
 *   instance.
 *
 * Returned Value:
 *     Returns the file descriptor == index into the files array on success;
 *     a negated errno value is returned on any failure.
 *
 ****************************************************************************/

int file_allocate(FAR struct inode *inode, int oflags, off_t pos,
                  FAR void *priv, int minfd, bool addref);

/****************************************************************************
 * Name: file_dup
 *
 * Description:
 *   Equivalent to the standard dup() function except that it
 *   accepts a struct file instance instead of a file descriptor.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int file_dup(FAR struct file *filep, int minfd, bool cloexec);

/****************************************************************************
 * Name: file_dup2
 *
 * Description:
 *   Assign an inode to a specific files structure.  This is the heart of
 *   dup2.
 *
 *   Equivalent to the non-standard dup2() function except that it
 *   accepts struct file instances instead of file descriptors.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int file_dup2(FAR struct file *filep1, FAR struct file *filep2);

/****************************************************************************
 * Name: nx_dup2_from_tcb
 *
 * Description:
 *   nx_dup2_from_tcb() is similar to the standard 'dup2' interface
 *   except that is not a cancellation point and it does not modify the
 *   errno variable.
 *
 *   nx_dup2_from_tcb() is an internal NuttX interface and should not be
 *   called from applications.
 *
 *   Clone a file descriptor to a specific descriptor number.
 *
 * Returned Value:
 *   fd2 is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int nx_dup2_from_tcb(FAR struct tcb_s *tcb, int fd1, int fd2);

/****************************************************************************
 * Name: nx_dup2
 *
 * Description:
 *   nx_dup2() is similar to the standard 'dup2' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_dup2() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   fd2 is returned on success; a negated errno value is return on
 *   any failure.
 *
 ****************************************************************************/

int nx_dup2(int fd1, int fd2);

/****************************************************************************
 * Name: file_open
 *
 * Description:
 *   file_open() is similar to the standard 'open' interface except that it
 *   returns an instance of 'struct file' rather than a file descriptor.  It
 *   also is not a cancellation point and does not modify the errno variable.
 *
 * Input Parameters:
 *   filep  - The caller provided location in which to return the 'struct
 *            file' instance.
 *   path   - The full path to the file to be open.
 *   oflags - open flags
 *   ...    - Variable number of arguments, may include 'mode_t mode'
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  On failure, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

int file_open(FAR struct file *filep, FAR const char *path, int oflags, ...);

/****************************************************************************
 * Name: nx_open_from_tcb
 *
 * Description:
 *   nx_open_from_tcb() is similar to the standard 'open' interface except
 *   that it is not a cancellation point and it does not modify the errno
 *   variable.
 *
 *   nx_open_from_tcb() is an internal NuttX interface and should not be
 *   called from applications.
 *
 * Input Parameters:
 *   tcb    - Address of the task's TCB
 *   path   - The full path to the file to be opened.
 *   oflags - open flags.
 *   ...    - Variable number of arguments, may include 'mode_t mode'
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int nx_open_from_tcb(FAR struct tcb_s *tcb,
                     FAR const char *path, int oflags, ...);

/****************************************************************************
 * Name: nx_open
 *
 * Description:
 *   nx_open() is similar to the standard 'open' interface except that is is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_open() is an internal NuttX interface and should not be called
 *   from applications.
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int nx_open(FAR const char *path, int oflags, ...);

/****************************************************************************
 * Name: fs_getfilep
 *
 * Description:
 *   Given a file descriptor, return the corresponding instance of struct
 *   file.  NOTE that this function will currently fail if it is provided
 *   with a socket descriptor.
 *
 * Input Parameters:
 *   fd    - The file descriptor
 *   filep - The location to return the struct file instance
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int fs_getfilep(int fd, FAR struct file **filep);

/****************************************************************************
 * Name: file_close
 *
 * Description:
 *   Close a file that was previously opened with file_open().
 *
 * Input Parameters:
 *   filep - A pointer to a user provided memory location containing the
 *           open file data returned by file_open().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int file_close(FAR struct file *filep);

/****************************************************************************
 * Name: nx_close_from_tcb
 *
 * Description:
 *   nx_close_from_tcb() is similar to the standard 'close' interface
 *   except that is not a cancellation point and it does not modify the
 *   errno variable.
 *
 *   nx_close_from_tcb() is an internal NuttX interface and should not
 *   be called from applications.
 *
 *   Close an inode (if open)
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   on any failure.
 *
 * Assumptions:
 *   Caller holds the list mutex because the file descriptor will be
 *   freed.
 *
 ****************************************************************************/

int nx_close_from_tcb(FAR struct tcb_s *tcb, int fd);

/****************************************************************************
 * Name: nx_close
 *
 * Description:
 *   nx_close() is similar to the standard 'close' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_close() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   The new file descriptor is returned on success; a negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

int nx_close(int fd);

/****************************************************************************
 * Name: open_blockdriver
 *
 * Description:
 *   Return the inode of the block driver specified by 'pathname'
 *
 * Input Parameters:
 *   pathname - the full path to the block driver to be opened
 *   mountflags - if MS_RDONLY is not set, then driver must support write
 *     operations (see include/sys/mount.h)
 *   ppinode - address of the location to return the inode reference
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   EINVAL  - pathname or pinode is NULL
 *   ENOENT  - No block driver of this name is registered
 *   ENOTBLK - The inode associated with the pathname is not a block driver
 *   EACCESS - The MS_RDONLY option was not set but this driver does not
 *     support write access
 *
 ****************************************************************************/

int open_blockdriver(FAR const char *pathname, int mountflags,
                     FAR struct inode **ppinode);

/****************************************************************************
 * Name: close_blockdriver
 *
 * Description:
 *   Call the close method and release the inode
 *
 * Input Parameters:
 *   inode - reference to the inode of a block driver opened by
 *           open_blockdriver
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   EINVAL  - inode is NULL
 *   ENOTBLK - The inode is not a block driver
 *
 ****************************************************************************/

int close_blockdriver(FAR struct inode *inode);

/****************************************************************************
 * Name: find_mtddriver
 *
 * Description:
 *   Return the inode of the named MTD driver specified by 'pathname'
 *
 * Input Parameters:
 *   pathname   - the full path to the named MTD driver to be located
 *   ppinode    - address of the location to return the inode reference
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   ENOENT  - No MTD driver of this name is registered
 *   ENOTBLK - The inode associated with the pathname is not an MTD driver
 *
 ****************************************************************************/

int find_mtddriver(FAR const char *pathname, FAR struct inode **ppinode);

/****************************************************************************
 * Name: close_mtddriver
 *
 * Description:
 *   Release the inode got by function find_mtddriver()
 *
 * Input Parameters:
 *   pinode    - pointer to the inode
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   EINVAL  - inode is NULL
 *
 ****************************************************************************/

int close_mtddriver(FAR struct inode *pinode);

/****************************************************************************
 * Name: fs_fdopen
 *
 * Description:
 *   This function does the core operations for fopen and fdopen.  It is
 *   used by the OS to clone stdin, stdout, stderr
 *
 ****************************************************************************/

#ifdef CONFIG_FILE_STREAM
int fs_fdopen(int fd, int oflags, FAR struct tcb_s *tcb,
              FAR struct file_struct **filep);
#endif

/****************************************************************************
 * Name: lib_flushall
 *
 * Description:
 *   Called either (1) by the OS when a task exits, or (2) from fflush()
 *   when a NULL stream argument is provided.
 *
 ****************************************************************************/

#ifdef CONFIG_FILE_STREAM
int lib_flushall(FAR struct streamlist *list);
#endif

/****************************************************************************
 * Name: file_read
 *
 * Description:
 *   file_read() is an internal OS interface.  It is functionally similar to
 *   the standard read() interface except:
 *
 *    - It does not modify the errno variable,
 *    - It is not a cancellation point,
 *    - It does not handle socket descriptors, and
 *    - It accepts a file structure instance instead of file descriptor.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *   buf    - User-provided to save the data
 *   nbytes - The maximum size of the user-provided buffer
 *
 * Returned Value:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or a negated errno value on any failure.
 *
 ****************************************************************************/

ssize_t file_read(FAR struct file *filep, FAR void *buf, size_t nbytes);

/****************************************************************************
 * Name: nx_read
 *
 * Description:
 *   nx_read() is an internal OS interface.  It is functionally similar to
 *   the standard read() interface except:
 *
 *    - It does not modify the errno variable, and
 *    - It is not a cancellation point.
 *
 * Input Parameters:
 *   fd     - File descriptor to read from
 *   buf    - User-provided to save the data
 *   nbytes - The maximum size of the user-provided buffer
 *
 * Returned Value:
 *   The positive non-zero number of bytes read on success, 0 on if an
 *   end-of-file condition, or a negated errno value on any failure.
 *
 ****************************************************************************/

ssize_t nx_read(int fd, FAR void *buf, size_t nbytes);

/****************************************************************************
 * Name: file_write
 *
 * Description:
 *   Equivalent to the standard write() function except that is accepts a
 *   struct file instance instead of a file descriptor.  It is functionally
 *   equivalent to write() except that in addition to the differences in
 *   input parameters:
 *
 *  - It does not modify the errno variable,
 *  - It is not a cancellation point, and
 *  - It does not handle socket descriptors.
 *
 * Input Parameters:
 *   filep  - Instance of struct file to use with the write
 *   buf    - Data to write
 *   nbytes - Length of data to write
 *
 * Returned Value:
 *  On success, the number of bytes written are returned (zero indicates
 *  nothing was written).  On any failure, a negated errno value is returned
 *  (see comments withwrite() for a description of the appropriate errno
 *  values).
 *
 ****************************************************************************/

ssize_t file_write(FAR struct file *filep, FAR const void *buf,
                   size_t nbytes);

/****************************************************************************
 * Name: nx_write
 *
 * Description:
 *  nx_write() writes up to nbytes bytes to the file referenced by the file
 *  descriptor fd from the buffer starting at buf.  nx_write() is an
 *  internal OS function.  It is functionally equivalent to write() except
 *  that:
 *
 *  - It does not modify the errno variable, and
 *  - It is not a cancellation point.
 *
 * Input Parameters:
 *   fd     - file descriptor (or socket descriptor) to write to
 *   buf    - Data to write
 *   nbytes - Length of data to write
 *
 * Returned Value:
 *  On success, the number of bytes written are returned (zero indicates
 *  nothing was written).  On any failure, a negated errno value is returned
 *  (see comments with write() for a description of the appropriate errno
 *   values).
 *
 ****************************************************************************/

ssize_t nx_write(int fd, FAR const void *buf, size_t nbytes);

/****************************************************************************
 * Name: file_pread
 *
 * Description:
 *   Equivalent to the standard pread function except that is accepts a
 *   struct file instance instead of a file descriptor.  Currently used
 *   only by aio_read();
 *
 ****************************************************************************/

ssize_t file_pread(FAR struct file *filep, FAR void *buf, size_t nbytes,
                   off_t offset);

/****************************************************************************
 * Name: file_pwrite
 *
 * Description:
 *   Equivalent to the standard pwrite function except that is accepts a
 *   struct file instance instead of a file descriptor.  Currently used
 *   only by aio_write();
 *
 ****************************************************************************/

ssize_t file_pwrite(FAR struct file *filep, FAR const void *buf,
                    size_t nbytes, off_t offset);

/****************************************************************************
 * Name: file_sendfile
 *
 * Description:
 *   Equivalent to the standard sendfile function except that is accepts a
 *   struct file instance instead of a file descriptor.
 *
 ****************************************************************************/

ssize_t file_sendfile(FAR struct file *outfile, FAR struct file *infile,
                      FAR off_t *offset, size_t count);

/****************************************************************************
 * Name: file_seek
 *
 * Description:
 *   Equivalent to the standard lseek() function except that is accepts a
 *   struct file instance instead of a file descriptor.  Currently used
 *   only by net_sendfile()
 *
 ****************************************************************************/

off_t file_seek(FAR struct file *filep, off_t offset, int whence);

/****************************************************************************
 * Name: nx_seek
 *
 * Description:
 *  nx_seek() function repositions the offset of the open file associated
 *  with the file descriptor fd to the argument 'offset' according to the
 *  directive 'whence'.  nx_seek() is an internal OS function. It is
 *  functionally equivalent to lseek() except that:
 *
 *  - It does not modify the errno variable, and
 *  - It is not a cancellation point.
 *
 ****************************************************************************/

off_t nx_seek(int fd, off_t offset, int whence);

/****************************************************************************
 * Name: file_fsync
 *
 * Description:
 *   Equivalent to the standard fsync() function except that is accepts a
 *   struct file instance instead of a file descriptor and it does not set
 *   the errno variable.
 *
 ****************************************************************************/

int file_fsync(FAR struct file *filep);

/****************************************************************************
 * Name: file_truncate
 *
 * Description:
 *   Equivalent to the standard ftruncate() function except that is accepts
 *   a struct file instance instead of a file descriptor and it does not set
 *   the errno variable.
 *
 ****************************************************************************/

int file_truncate(FAR struct file *filep, off_t length);

/****************************************************************************
 * Name: file_mmap
 *
 * Description:
 *   Equivalent to the standard mmap() function except that is accepts
 *   a struct file instance instead of a file descriptor and it does not set
 *   the errno variable.
 *
 ****************************************************************************/

int file_mmap(FAR struct file *filep, FAR void *start, size_t length,
              int prot, int flags, off_t offset, FAR void **mapped);

/****************************************************************************
 * Name: file_mummap
 *
 * Description:
 *   Equivalent to the standard mummap() function except it does not set
 *   the errno variable.
 *
 ****************************************************************************/

int file_munmap(FAR void *start, size_t length);

/****************************************************************************
 * Name: file_ioctl
 *
 * Description:
 *   Perform device specific operations.
 *
 * Input Parameters:
 *   file     File structure instance
 *   req      The ioctl command
 *   ap       The argument of the ioctl cmd
 *
 * Returned Value:
 *   Returns a non-negative number on success;  A negated errno value is
 *   returned on any failure (see comments ioctl() for a list of appropriate
 *   errno values).
 *
 ****************************************************************************/

int file_ioctl(FAR struct file *filep, int req, ...);

/****************************************************************************
 * Name: file_fcntl
 *
 * Description:
 *   Similar to the standard fcntl function except that is accepts a struct
 *   struct file instance instead of a file descriptor.
 *
 * Input Parameters:
 *   filep - Instance for struct file for the opened file.
 *   cmd   - Identifies the operation to be performed.  Command specific
 *           arguments may follow.
 *
 * Returned Value:
 *   The nature of the return value depends on the command.  Non-negative
 *   values indicate success.  Failures are reported as negated errno
 *   values.
 *
 ****************************************************************************/

int file_fcntl(FAR struct file *filep, int cmd, ...);

/****************************************************************************
 * Name: file_poll
 *
 * Description:
 *   Low-level poll operation based on struct file.  This is used both to (1)
 *   support detached file, and also (2) by poll_fdsetup() to perform all
 *   normal operations on file descriptors.
 *
 * Input Parameters:
 *   file     File structure instance
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

int file_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup);

/****************************************************************************
 * Name: file_fstat
 *
 * Description:
 *   file_fstat() is an internal OS interface.  It is functionally similar to
 *   the standard fstat() interface except:
 *
 *    - It does not modify the errno variable,
 *    - It is not a cancellation point,
 *    - It does not handle socket descriptors, and
 *    - It accepts a file structure instance instead of file descriptor.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *   buf    - The caller provide location in which to return information
 *            about the open file.
 *
 * Returned Value:
 *   Upon successful completion, 0 shall be returned. Otherwise, -1 shall be
 *   returned and errno set to indicate the error.
 *
 ****************************************************************************/

int file_fstat(FAR struct file *filep, FAR struct stat *buf);

/****************************************************************************
 * Name: nx_stat
 *
 * Description:
 *   nx_stat() is similar to the standard 'stat' interface except that is
 *   not a cancellation point and it does not modify the errno variable.
 *
 *   nx_stat() is an internal NuttX interface and should not be called from
 *   applications.
 *
 * Returned Value:
 *   Zero is returned on success; a negated value is returned on any failure.
 *
 ****************************************************************************/

int nx_stat(FAR const char *path, FAR struct stat *buf, int resolve);

/****************************************************************************
 * Name: file_fchstat
 *
 * Description:
 *   file_fchstat() is an internal OS interface. It is functionally similar
 *   to the combination of fchmod/fchown/futimens standard interface except:
 *
 *    - It does not modify the errno variable,
 *    - It is not a cancellation point,
 *    - It does not handle socket descriptors, and
 *    - It accepts a file structure instance instead of file descriptor.
 *
 * Input Parameters:
 *   filep  - File structure instance
 *   buf    - The stat to be modified
 *   flags  - The valid field in buf
 *
 * Returned Value:
 *   Upon successful completion, 0 shall be returned. Otherwise, the
 *   negative errno shall be returned to indicate the error.
 *
 ****************************************************************************/

int file_fchstat(FAR struct file *filep, FAR struct stat *buf, int flags);

/****************************************************************************
 * Name: nx_unlink
 *
 * Description:
 *   nx_unlink() is similar to the standard 'unlink' interface except that
 *   is not a cancellation point and it does not modify the errno variable.
 *
 *   nx_unlink() is an internal NuttX interface and should not be called
 *   from applications.
 *
 * Returned Value:
 *   Zero is returned on success; a negated value is returned on any failure.
 *
 ****************************************************************************/

int nx_unlink(FAR const char *pathname);

/****************************************************************************
 * Name: file_pipe
 *
 * Description:
 *   file_pipe() creates a pair of file descriptors, pointing to a pipe
 *   inode, and places them in the array pointed to by 'filep'. filep[0]
 *   is for reading, filep[1] is for writing.
 *
 * Input Parameters:
 *   filep[2] - The user provided array in which to catch the pipe file
 *   descriptors
 *   bufsize - The size of the in-memory, circular buffer in bytes.
 *   flags - The file status flags.
 *
 * Returned Value:
 *   0 is returned on success; a negated errno value is returned on a
 *   failure.
 *
 ****************************************************************************/

#if defined(CONFIG_PIPES) && CONFIG_DEV_PIPE_SIZE > 0
int file_pipe(FAR struct file *filep[2], size_t bufsize, int flags);
#endif

/****************************************************************************
 * Name: nx_mkfifo
 *
 * Description:
 *   nx_mkfifo() makes a FIFO device driver file with name 'pathname.' Unlike
 *   Linux, a NuttX FIFO is not a special file type but simply a device
 *   driver instance.  'mode' specifies the FIFO's permissions.
 *
 *   Once the FIFO has been created by nx_mkfifo(), any thread can open it
 *   for reading or writing, in the same way as an ordinary file. However, it
 *   must have been opened from both reading and writing before input or
 *   output can be performed.  This FIFO implementation will block all
 *   attempts to open a FIFO read-only until at least one thread has opened
 *   the FIFO for  writing.
 *
 *   If all threads that write to the FIFO have closed, subsequent calls to
 *   read() on the FIFO will return 0 (end-of-file).
 *
 *   NOTE: nx_mkfifo is a special, non-standard, NuttX-only interface.  Since
 *   the NuttX FIFOs are based in in-memory, circular buffers, the ability
 *   to control the size of those buffers is critical for system tuning.
 *
 * Input Parameters:
 *   pathname - The full path to the FIFO instance to attach to or to create
 *     (if not already created).
 *   mode - Ignored for now
 *   bufsize - The size of the in-memory, circular buffer in bytes.
 *
 * Returned Value:
 *   0 is returned on success; a negated errno value is returned on a
 *   failure.
 *
 ****************************************************************************/

#if defined(CONFIG_PIPES) && CONFIG_DEV_FIFO_SIZE > 0
int nx_mkfifo(FAR const char *pathname, mode_t mode, size_t bufsize);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_FS_FS_H */
