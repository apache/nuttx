/****************************************************************************
 * fs/binfs/fs_binfs.c
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
#include <sys/statfs.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/lib/builtin.h>

#include "inode/inode.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_BINFS)

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct binfs_dir_s
{
  struct fs_dirent_s base; /* VFS directory structure */
  unsigned int index;      /* Index to the next named entry point */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     binfs_open(FAR struct file *filep, const char *relpath,
                          int oflags, mode_t mode);
static int     binfs_close(FAR struct file *filep);
static ssize_t binfs_read(FAR struct file *filep,
                          char *buffer, size_t buflen);
static int     binfs_ioctl(FAR struct file *filep,
                           int cmd, unsigned long arg);

static int     binfs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int     binfs_fstat(FAR const struct file *filep,
                           FAR struct stat *buf);

static int     binfs_opendir(FAR struct inode *mountpt,
                             FAR const char *relpath,
                             FAR struct fs_dirent_s **dir);
static int     binfs_closedir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir);
static int     binfs_readdir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir,
                             FAR struct dirent *entry);
static int     binfs_rewinddir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir);

static int     binfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                          FAR void **handle);
static int     binfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                            unsigned int flags);
static int     binfs_statfs(FAR struct inode *mountpt,
                            FAR struct statfs *buf);

static int     binfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations g_binfs_operations =
{
  binfs_open,        /* open */
  binfs_close,       /* close */
  binfs_read,        /* read */
  NULL,              /* write */
  NULL,              /* seek */
  binfs_ioctl,       /* ioctl */
  NULL,              /* mmap */
  NULL,              /* truncate */

  NULL,              /* sync */
  binfs_dup,         /* dup */
  binfs_fstat,       /* fstat */
  NULL,              /* fchstat */

  binfs_opendir,     /* opendir */
  binfs_closedir,    /* closedir */
  binfs_readdir,     /* readdir */
  binfs_rewinddir,   /* rewinddir */

  binfs_bind,        /* bind */
  binfs_unbind,      /* unbind */
  binfs_statfs,      /* statfs */

  NULL,              /* unlink */
  NULL,              /* mkdir */
  NULL,              /* rmdir */
  NULL,              /* rename */
  binfs_stat,        /* stat */
  NULL               /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: binfs_open
 ****************************************************************************/

static int binfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  int index;

  finfo("Open '%s'\n", relpath);

  /* BINFS is read-only.  Any attempt to open with any kind of write
   * access is not permitted.
   */

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDONLY) == 0)
    {
      ferr("ERROR: Only O_RDONLY supported\n");
      return -EACCES;
    }

  /* Check if the entry exists with this name in the root directory.
   * so the 'relpath' must be the name of the builtin function.
   */

  index = builtin_isavail(relpath);
  if (index < 0)
    {
      ferr("ERROR: Built-in %s does not exist\n", relpath);
      return -ENOENT;
    }

  /* Save the index as the open-specific state in filep->f_priv */

  filep->f_priv = (FAR void *)((uintptr_t)index);
  return OK;
}

/****************************************************************************
 * Name: binfs_close
 ****************************************************************************/

static int binfs_close(FAR struct file *filep)
{
  finfo("Closing\n");
  return OK;
}

/****************************************************************************
 * Name: binfs_read
 ****************************************************************************/

static ssize_t binfs_read(FAR struct file *filep,
                          char *buffer, size_t buflen)
{
  /* Reading is not supported.  Just return end-of-file */

  finfo("Read %zu bytes from offset %jd\n",
        buflen, (intmax_t)filep->f_pos);
  return 0;
}

/****************************************************************************
 * Name: binfs_ioctl
 ****************************************************************************/

static int binfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret;

  finfo("cmd: %d arg: %08lx\n", cmd, arg);

  /* Only one IOCTL command is supported */

  if (cmd == FIOC_FILEPATH)
    {
      /* IN:  FAR char *(length >= PATH_MAX)
       * OUT: The full file path
       */

      FAR char *ptr = (FAR char *)((uintptr_t)arg);
      if (ptr == NULL)
        {
          ret = -EINVAL;
        }
      else
        {
          ret = inode_getpath(filep->f_inode, ptr);
          if (ret < 0)
            {
              return ret;
            }

          strcat(ptr, builtin_getname((int)((uintptr_t)filep->f_priv)));
        }
    }
  else
    {
      ret = -ENOTTY;
    }

  return ret;
}

/****************************************************************************
 * Name: binfs_dup
 *
 * Description:
 *   Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int binfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  finfo("Dup %p->%p\n", oldp, newp);

  /* Copy the index from the old to the new file structure */

  newp->f_priv = oldp->f_priv;
  return OK;
}

/****************************************************************************
 * Name: binfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int binfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  DEBUGASSERT(filep != NULL && buf != NULL);

  /* It's a execute-only file system */

  buf->st_mode    = S_IFREG | S_IXOTH | S_IXGRP | S_IXUSR;
  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}

/****************************************************************************
 * Name: binfs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

static int binfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct fs_dirent_s **dir)
{
  FAR struct binfs_dir_s *bdir;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  /* The requested directory must be the volume-relative "root" directory */

  if (relpath && relpath[0] != '\0')
    {
      return -ENOENT;
    }

  bdir = kmm_zalloc(sizeof(*bdir));
  if (bdir == NULL)
    {
      return -ENOMEM;
    }

  /* Set the index to the first entry */

  bdir->index = 0;
  *dir = (FAR struct fs_dirent_s *)bdir;
  return OK;
}

/****************************************************************************
 * Name: binfs_closedir
 *
 * Description:
 *   Close a directory
 *
 ****************************************************************************/

static int binfs_closedir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir)
{
  DEBUGASSERT(dir);
  kmm_free(dir);
  return 0;
}

/****************************************************************************
 * Name: binfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int binfs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir,
                         FAR struct dirent *entry)
{
  FAR struct binfs_dir_s *bdir;
  FAR const char *name;
  unsigned int index;
  int ret;

  /* Have we reached the end of the directory */

  bdir = (FAR struct binfs_dir_s *)dir;
  index = bdir->index;
  name = builtin_getname(index);
  if (name == NULL)
    {
      /* We signal the end of the directory by returning the
       * special error -ENOENT
       */

      finfo("Entry %d: End of directory\n", index);
      ret = -ENOENT;
    }
  else
    {
      /* Save the filename and file type */

      finfo("Entry %d: \"%s\"\n", index, name);
      entry->d_type = DTYPE_FILE;
      strlcpy(entry->d_name, name, sizeof(entry->d_name));

      /* The application list is terminated by an entry with a NULL name.
       * Therefore, there is at least one more entry in the list.
       */

      index++;

      /* Set up the next directory entry offset.  NOTE that we could use the
       * standard f_pos instead of our own private index.
       */

      bdir->index = index;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: binfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int binfs_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  finfo("Entry\n");

  ((FAR struct binfs_dir_s *)dir)->index = 0;
  return OK;
}

/****************************************************************************
 * Name: binfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int binfs_bind(FAR struct inode *blkdriver, const void *data,
                      void **handle)
{
  finfo("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: binfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int binfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                        unsigned int flags)
{
  finfo("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: binfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int binfs_statfs(struct inode *mountpt, struct statfs *buf)
{
  finfo("Entry\n");

  /* Fill in the statfs info */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = BINFS_MAGIC;
  buf->f_bsize   = 0;
  buf->f_blocks  = 0;
  buf->f_bfree   = 0;
  buf->f_bavail  = 0;
  buf->f_namelen = NAME_MAX;
  return OK;
}

/****************************************************************************
 * Name: binfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int binfs_stat(struct inode *mountpt,
                      const char *relpath, struct stat *buf)
{
  finfo("Entry\n");

  /* The requested directory must be the volume-relative "root" directory */

  if (relpath && relpath[0] != '\0')
    {
      /* Check if there is a file with this name. */

      if (builtin_isavail(relpath) < 0)
        {
          return -ENOENT;
        }

      /* It's a execute-only file name */

      buf->st_mode = S_IFREG | S_IXOTH | S_IXGRP | S_IXUSR;
    }
  else
    {
      /* It's a read/execute-only directory name */

      buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR | S_IXOTH |
                     S_IXGRP | S_IXUSR;
    }

  /* File/directory size, access block size */

  buf->st_size    = 0;
  buf->st_blksize = 0;
  buf->st_blocks  = 0;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_BINFS */
