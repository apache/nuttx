/****************************************************************************
 * fs/procfs/fs_skeleton.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
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

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_PROCFS)

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

struct skel_file_s
{
  struct procfs_file_s  base;        /* Base open file structure */

  /* Add context specific data types for managing an open file here */
};

/* Level 1 is the directory of attributes */

struct skel_level1_s
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

static int     skel_open(FAR struct file *filep, FAR const char *relpath,
                 int oflags, mode_t mode);
static int     skel_close(FAR struct file *filep);
static ssize_t skel_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);

/* TODO:  Should not support skel_write if read-only */

static ssize_t skel_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     skel_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

static int     skel_opendir(FAR const char *relpath,
                 FAR struct fs_dirent_s *dir);
static int     skel_closedir(FAR struct fs_dirent_s *dir);
static int     skel_readdir(FAR struct fs_dirent_s *dir,
                            FAR struct dirent *entry);
static int     skel_rewinddir(FAR struct fs_dirent_s *dir);

static int     skel_stat(FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See include/nutts/fs/procfs.h
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct procfs_operations skel_procfsoperations =
{
  skel_open,       /* open */
  skel_close,      /* close */
  skel_read,       /* read */

  /* TODO:  Decide if this procfs entry supports write access */

#if 0 /* NULL if the procfs entry does not support write access. */
  NULL,            /* write */
#else
  skel_write,      /* write */
#endif

  skel_dup,        /* dup */

  skel_opendir,    /* opendir */
  skel_closedir,   /* closedir */
  skel_readdir,    /* readdir */
  skel_rewinddir,  /* rewinddir */

  skel_stat        /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: skel_open
 ****************************************************************************/

static int skel_open(FAR struct file *filep, FAR const char *relpath,
                     int oflags, mode_t mode)
{
  FAR struct skel_file_s *priv;

  finfo("Open '%s'\n", relpath);

  /* If PROCFS is read-only, then the (1) the skel_write() method must not
   * be provided and (2) any attempt to open with any kind of write access
   * can not be permitted.
   */

  if (((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0) &&
      (skel_procfsoperations.write == NULL))
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* Allocate the open file structure */

  priv = (FAR struct skel_file_s *)kmm_zalloc(sizeof(struct skel_file_s));
  if (!priv)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* TODO:  Initialize the context specific data here */

  /* Save the open file structure as the open-specific state in
   * filep->f_priv.
   */

  filep->f_priv = (FAR void *)priv;
  return OK;
}

/****************************************************************************
 * Name: skel_close
 ****************************************************************************/

static int skel_close(FAR struct file *filep)
{
  FAR struct skel_file_s *priv;

  /* Recover our private data from the struct file instance */

  priv = (FAR struct skel_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* Release the file attributes structure */

  kmm_free(priv);
  filep->f_priv = NULL;
  return OK;
}

/****************************************************************************
 * Name: skel_read
 *
 * Description:
 *   Handle read from procfs file.
 *
 ****************************************************************************/

static ssize_t skel_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  FAR struct skel_file_s *priv;
  ssize_t ret;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  priv = (FAR struct skel_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* TODO:  Provide the requested data.
   *        Take into account current filep->f_pos and 'buflen'.  The read
   *        could require several calls to skel_read().
   */

  ret = 0;

  /* Update the file offset */

  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: skel_write
 *
 * Description:
 *   Handle write3 to procfs file.
 *
 ****************************************************************************/

/* TODO: Should not support skel_write if read-only */

static ssize_t skel_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct skel_file_s *priv;
  ssize_t ret;

  finfo("buffer=%p buflen=%d\n", buffer, (int)buflen);

  /* Recover our private data from the struct file instance */

  priv = (FAR struct skel_file_s *)filep->f_priv;
  DEBUGASSERT(priv);

  /* TODO:  Verify that the write is within range */

  /* TODO:  Handle the write data as appropriate to function of file.
   *        Take into account current filep->f_pos and 'buflen' since the
   *        write may require several calls to skel_write().
   */

  ret = 0;

  /* Update the file offset */

  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  return ret;
}

/****************************************************************************
 * Name: skel_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int skel_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct skel_file_s *oldpriv;
  FAR struct skel_file_s *newpriv;

  finfo("Dup %p->%p\n", oldp, newp);

  /* Recover our private data from the old struct file instance */

  oldpriv = (FAR struct skel_file_s *)oldp->f_priv;
  DEBUGASSERT(oldpriv);

  /* Allocate a new container to hold the task and attribute selection */

  newpriv = (FAR struct skel_file_s *)kmm_zalloc(sizeof(struct skel_file_s));
  if (!newpriv)
    {
      ferr("ERROR: Failed to allocate file attributes\n");
      return -ENOMEM;
    }

  /* The copy the file attribute from the old attributes to the new */

  memcpy(newpriv, oldpriv, sizeof(struct skel_file_s));

  /* Save the new attributes in the new file structure */

  newp->f_priv = (FAR void *)newpriv;
  return OK;
}

/****************************************************************************
 * Name: skel_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int skel_opendir(FAR const char *relpath, FAR struct fs_dirent_s *dir)
{
  FAR struct skel_level1_s *level1;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");
  DEBUGASSERT(relpath && dir && !dir->u.procfs);

  /* The path refers to the 1st level sbdirectory.  Allocate the level1
   * dirent structure.
   */

  level1 = (FAR struct skel_level1_s *)
     kmm_zalloc(sizeof(struct skel_level1_s));

  if (!level1)
    {
      ferr("ERROR: Failed to allocate the level1 directory structure\n");
      return -ENOMEM;
    }

  /* TODO:  Initialize context specific data */

  /* Initialize base structure components */

  level1->base.level    = 1;
  level1->base.nentries = 0;
  level1->base.index    = 0;

  dir->u.procfs = (FAR void *) level1;
  return OK;
}

/****************************************************************************
 * Name: skel_closedir
 *
 * Description: Close the directory listing
 *
 ****************************************************************************/

static int skel_closedir(FAR struct fs_dirent_s *dir)
{
  FAR struct skel_level1_s *priv;

  DEBUGASSERT(dir && dir->u.procfs);
  priv = dir->u.procfs;

  if (priv)
    {
      kmm_free(priv);
    }

  dir->u.procfs = NULL;
  return OK;
}

/****************************************************************************
 * Name: skel_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int skel_readdir(FAR struct fs_dirent_s *dir,
                        FAR struct dirent *entry)
{
  FAR struct skel_level1_s *level1;
  char  filename[16];
  int index;
  int ret;

  DEBUGASSERT(dir && dir->u.procfs);
  level1 = dir->u.procfs;

  /* TODO:  Perform device specific readdir function here.  This may
   *        or may not involve validating the nentries variable
   *        in the base depending on the implementation.
   */

  /* Have we reached the end of the directory */

  index = level1->base.index;
  if (index >= level1->base.nentries)
    {
      /* We signal the end of the directory by returning the special
       * error -ENOENT
       */

      finfo("Entry %d: End of directory\n", index);
      ret = -ENOENT;
    }

  /* We are traversing a subdirectory of task attributes */

  else
    {
      DEBUGASSERT(level1->base.level == 1);

      /* TODO:  Add device specific entries */

      strcpy(filename, "dummy");

      /* TODO:  Specify the type of entry */

      entry->d_type = DTYPE_FILE;
      strlcpy(entry->d_name, filename, sizeof(entry->d_name));

      /* Set up the next directory entry offset.  NOTE that we could use the
       * standard f_pos instead of our own private index.
       */

      level1->base.index = index + 1;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: skel_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int skel_rewinddir(FAR struct fs_dirent_s *dir)
{
  FAR struct skel_level1_s *priv;

  DEBUGASSERT(dir && dir->u.procfs);
  priv = dir->u.procfs;

  priv->base.index = 0;
  return OK;
}

/****************************************************************************
 * Name: skel_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int skel_stat(FAR const char *relpath, FAR struct stat *buf)
{
  int ret = -ENOENT;

  /* TODO:  Decide if the relpath is valid and if it is a file
   *        or a directory and set it's permissions.
   */

  memset(buf, 0, sizeof(struct stat));
  buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;

  /* TODO:  Set S_IFREG if the relpath refers to a file.
   * TODO:  If the skel_write() method is supported, then stat must also
   *        report S_IWOTH | S_IWGRP | S_IWUSR for files (but not for
   *        directories) as well.
   * TODO:  Other 'struct buf' settings may be appropriate (optional)
   */

  ret = OK;

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_PROCFS */
