/****************************************************************************
 * fs/smartfs/smartfs_procfs.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/fs/dirent.h>

#include <arch/irq.h>

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_EXCLUDE_SMARTFS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* This enumeration identifies all of the thread attributes that can be
 * accessed via the procfs file system.
 */

/* This structure describes one open "file" */

struct smartfs_file_s
{
  struct procfs_file_s  base;        /* Base open file structure */

  /* Add context specific data types for managing an open file here */
};

/* Level 1 is the directory of attributes */

struct smartfs_level1_s
{
  struct procfs_dir_priv_s  base;    /* Base directory private data */

  /* Add context specific data types here for managing the directory
   * open / read / stat, etc.
   */

};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* File system methods */

static int     smartfs_open(FAR struct file *filep, FAR const char *relpath,
                 int oflags, mode_t mode);
static int     smartfs_close(FAR struct file *filep);
static ssize_t smartfs_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);

static int     smartfs_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

static int     smartfs_opendir(const char *relpath, FAR struct fs_dirent_s *dir);
static int     smartfs_closedir(FAR struct fs_dirent_s *dir);
static int     smartfs_readdir(FAR struct fs_dirent_s *dir);
static int     smartfs_rewinddir(FAR struct fs_dirent_s *dir);

static int     smartfs_stat(FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* See include/nutts/fs/procfs.h
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct procfs_operations smartfs_procfsoperations =
{
  smartfs_open,       /* open */
  smartfs_close,      /* close */
  smartfs_read,       /* read */

  /* TODO:  Decide if this deiver supports write */
  NULL,            /* write */

  smartfs_dup,        /* dup */

  smartfs_opendir,    /* opendir */
  smartfs_closedir,   /* closedir */
  smartfs_readdir,    /* readdir */
  smartfs_rewinddir,  /* rewinddir */

  smartfs_stat        /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: smartfs_open
 ****************************************************************************/

static int smartfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct smartfs_file_s *priv;

  fvdbg("Open '%s'\n", relpath);

  /* PROCFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   *
   * REVISIT:  Write-able proc files could be quite useful.
   */

  if (((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0) &&
      (smartfs_procfsoperations.write == NULL))
    {
      fdbg("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* Allocate a container to hold the task and attribute selection */

  priv = (FAR struct smartfs_file_s *)kzalloc(sizeof(struct smartfs_file_s));
  if (!priv)
    {
      fdbg("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* TODO: Initialize the context specific data here */


  /* Save the index as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)priv;
  return OK;
}

/****************************************************************************
 * Name: smartfs_close
 ****************************************************************************/

static int smartfs_close(FAR struct file *filep)
{
  FAR struct smartfs_file_s *priv;

  /* Recover our private data from the struct file instance */

  priv = (FAR struct smartfs_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* Release the file attributes structure */

  kfree(priv);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: smartfs_read
 ****************************************************************************/

static ssize_t smartfs_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct smartfs_file_s *priv;
  ssize_t ret;

  fvdbg("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  priv = (FAR struct smartfs_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* TODO: Provide the requested data */

  ret = 0;

  /* Update the file offset */

  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: smartfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int smartfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct smartfs_file_s *oldpriv;
  FAR struct smartfs_file_s *newpriv;

  fvdbg("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldpriv = (FAR struct smartfs_file_s *)oldp->f_priv;
  DEBUGASSERT(oldpriv);

  /* Allocate a new container to hold the task and attribute selection */

  newpriv = (FAR struct smartfs_file_s *)kzalloc(sizeof(struct smartfs_file_s));
  if (!newpriv)
    {
      fdbg("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attribtes from the old attributes to the new */

  memcpy(newpriv, oldpriv, sizeof(struct smartfs_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (FAR void *)newpriv;
  return OK;
}

/****************************************************************************
 * Name: smartfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int smartfs_opendir(FAR const char *relpath, FAR struct fs_dirent_s *dir)
{
  FAR struct smartfs_level1_s *level1;

  fvdbg("relpath: \"%s\"\n", relpath ? relpath : "NULL");
  DEBUGASSERT(relpath && dir && !dir->u.procfs);

  /* The path refers to the 1st level sbdirectory.  Allocate the level1
   * dirent structure.
   */

  level1 = (FAR struct smartfs_level1_s *)
     kzalloc(sizeof(struct smartfs_level1_s));

  if (!level1)
    {
      fdbg("ERROR: Failed to allocate the level1 directory structure\n");
      return -ENOMEM;
    }

  /* TODO:  Initialze context specific data */


  /* Initialze base structure components */

  level1->base.level    = 1;
  level1->base.nentries = 0;
  level1->base.index    = 0;

  dir->u.procfs = (FAR void *) level1;
  return OK;
}

/****************************************************************************
 * Name: smartfs_closedir
 *
 * Description: Close the directory listing
 *
 ****************************************************************************/

static int smartfs_closedir(FAR struct fs_dirent_s *dir)
{
  FAR struct smartfs_level1_s *priv;

  DEBUGASSERT(dir && dir->u.procfs);
  priv = dir->u.procfs;

  if (priv)
    {
      kfree(priv);
    }

  dir->u.procfs = NULL;
  return OK;
}

/****************************************************************************
 * Name: smartfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int smartfs_readdir(struct fs_dirent_s *dir)
{
  FAR struct smartfs_level1_s *level1;
  char  filename[16];
  int ret, index;

  DEBUGASSERT(dir && dir->u.procfs);
  level1 = dir->u.procfs;

  /* TODO: Perform device specific readdir function here.  This may
   *       or may not involve validating the nentries variable
   *       in the base depending on the implementation.
   */

  /* Have we reached the end of the directory */

  index = level1->base.index;
  if (index >= level1->base.nentries)
    {
      /* We signal the end of the directory by returning the special
       * error -ENOENT
       */

      fvdbg("Entry %d: End of directory\n", index);
      ret = -ENOENT;
    }

  /* We are tranversing a subdirectory of task attributes */

  else
    {
      DEBUGASSERT(level1->base.level == 1);

      /* TODO: Add device specific entries */

      strcpy(filename, "dummy");

      /* TODO:  Specify the type of entry */

      dir->fd_dir.d_type = DTYPE_FILE;
      strncpy(dir->fd_dir.d_name, filename, NAME_MAX+1);

      /* Set up the next directory entry offset.  NOTE that we could use the
       * standard f_pos instead of our own private index.
       */

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: smartfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int smartfs_rewinddir(struct fs_dirent_s *dir)
{
  FAR struct smartfs_level1_s *priv;

  DEBUGASSERT(dir && dir->u.procfs);
  priv = dir->u.procfs;

  priv->base.index = 0;
  return OK;
}

/****************************************************************************
 * Name: smartfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int smartfs_stat(const char *relpath, struct stat *buf)
{
  /* TODO:  Decide if the relpath is valid and if it is a file
   *        or a directory and set it's permissions.
   */

  buf->st_mode = S_IFDIR|S_IROTH|S_IRGRP|S_IRUSR;

  /* File/directory size, access block size */

  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* CONFIG_FS_PROCFS && !CONFIG_FS_PROCFS_EXCLUDE_SMARTFS */
