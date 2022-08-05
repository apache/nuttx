/****************************************************************************
 * fs/exfat/exfat_vfs.c
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

#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

#include <sys/stat.h>
#include <sys/statfs.h>

#include <exfatfs.h>
#include "mkexfat.h"

#include "inode/inode.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct exfatfs_file_s
{
  FAR struct exfat_node *node;
};

struct exfatfs_dir_s
{
  struct fs_dirent_s     base;
  FAR struct exfat_node *entry;
  struct exfat_iterator  it;
};

struct exfatfs_mountpt_s
{
  FAR struct inode      *drv;
  sem_t                  sem;
  struct exfat           ef;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    exfatfs_semgive(FAR struct exfatfs_mountpt_s *fs);
static int     exfatfs_semtake(FAR struct exfatfs_mountpt_s *fs);

static int     exfatfs_open(FAR struct file *filep, FAR const char *relpath,
                            int oflags, mode_t mode);
static int     exfatfs_close(FAR struct file *filep);
static ssize_t exfatfs_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t exfatfs_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static off_t   exfatfs_seek(FAR struct file *filep, off_t offset,
                            int whence);
static int     exfatfs_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

static int     exfatfs_sync(FAR struct file *filep);
static int     exfatfs_dup(FAR const struct file *oldp,
                           FAR struct file *newp);
static int     exfatfs_fstat(FAR const struct file *filep,
                             FAR struct stat *buf);
static int     exfatfs_fchstat(FAR const struct file *filep,
                               FAR const struct stat *buf, int flags);
static int     exfatfs_truncate(FAR struct file *filep,
                                off_t length);

static int     exfatfs_opendir(FAR struct inode *mountpt,
                               FAR const char *relpath,
                               FAR struct fs_dirent_s **dir);
static int     exfatfs_closedir(FAR struct inode *mountpt,
                                FAR struct fs_dirent_s *dir);
static int     exfatfs_readdir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir,
                               FAR struct dirent *entry);
static int     exfatfs_rewinddir(FAR struct inode *mountpt,
                                 FAR struct fs_dirent_s *dir);

static int     exfatfs_bind(FAR struct inode *driver,
                            FAR const void *data, FAR void **handle);
static int     exfatfs_unbind(FAR void *handle, FAR struct inode **driver,
                              unsigned int flags);
static int     exfatfs_statfs(FAR struct inode *mountpt,
                              FAR struct statfs *buf);

static int     exfatfs_unlink(FAR struct inode *mountpt,
                              FAR const char *relpath);
static int     exfatfs_mkdir(FAR struct inode *mountpt,
                             FAR const char *relpath, mode_t mode);
static int     exfatfs_rmdir(FAR struct inode *mountpt,
                              FAR const char *relpath);
static int     exfatfs_rename(FAR struct inode *mountpt,
                              FAR const char *oldrelpath,
                              FAR const char *newrelpath);
static int     exfatfs_stat(FAR struct inode *mountpt,
                            FAR const char *relpath, FAR struct stat *buf);
static int     exfatfs_chstat(FAR struct inode *mountpt,
                              FAR const char *relpath,
                              FAR const struct stat *buf, int flags);
static int     exfatfs_format(FAR struct exfat *ef, FAR const char *spec,
                              int sector_bits, int spc_bits,
                              FAR const char *volume_label,
                              uint32_t volume_serial, uint64_t first_sector);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations exfatfs_operations =
{
  exfatfs_open,          /* open */
  exfatfs_close,         /* close */
  exfatfs_read,          /* read */
  exfatfs_write,         /* write */
  exfatfs_seek,          /* seek */
  exfatfs_ioctl,         /* ioctl */

  exfatfs_sync,          /* sync */
  exfatfs_dup,           /* dup */
  exfatfs_fstat,         /* fstat */
  exfatfs_fchstat,       /* fchstat */
  exfatfs_truncate,      /* truncate */

  exfatfs_opendir,       /* opendir */
  exfatfs_closedir,      /* closedir */
  exfatfs_readdir,       /* readdir */
  exfatfs_rewinddir,     /* rewinddir */

  exfatfs_bind,          /* bind */
  exfatfs_unbind,        /* unbind */
  exfatfs_statfs,        /* statfs */

  exfatfs_unlink,        /* unlink */
  exfatfs_mkdir,         /* mkdir */
  exfatfs_rmdir,         /* rmdir */
  exfatfs_rename,        /* rename */
  exfatfs_stat,          /* stat */
  exfatfs_chstat         /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exfatfs_semtake
 ****************************************************************************/

static int exfatfs_semtake(FAR struct exfatfs_mountpt_s *fs)
{
  return nxsem_wait_uninterruptible(&fs->sem);
}

/****************************************************************************
 * Name: exfatfs_semgive
 ****************************************************************************/

static void exfatfs_semgive(FAR struct exfatfs_mountpt_s *fs)
{
  nxsem_post(&fs->sem);
}

static void exfatfs_release_node(FAR struct exfat *ef,
                                 FAR struct exfat_node *node)
{
  exfat_put_node(ef, node);
  if (node->references == 0)
    {
      exfat_cleanup_node(ef, node);
    }
}

static int exfatfs_getpath(FAR struct inode *inode,
                           FAR struct exfat_node *node,
                           FAR char *path)
{
  int ret;

  if (node == NULL)
    {
      return inode_getpath(inode, path);
    }

  ret = exfatfs_getpath(inode, node->parent, path);
  if (ret < 0)
    {
      return ret;
    }

  path += strlen(path);
  strcpy(path, node->name);
  if (node->child)
    {
      strcat(path, "/");
    }

  return OK;
}

/****************************************************************************
 * Name: exfatfs_open
 ****************************************************************************/

static int exfatfs_open(FAR struct file *filep, FAR const char *relpath,
                        int oflags, mode_t mode)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_file_s *priv;
  FAR struct exfat_node *node;
  FAR struct inode *inode;
  int ret;

  priv = kmm_malloc(sizeof(struct exfatfs_file_s));
  if (!priv)
    {
      return -ENOMEM;
    }

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      goto semout;
    }

  ret = exfat_lookup(&fs->ef, &node, relpath);

  /* Three possibilities: (1) a node exists for the relpath and
   * node describes the file position (2) the node does not
   * exist, or (3) some error occurred.
   */

  if (ret == OK)
    {
      bool readonly;

      if (node->attrib & EXFAT_ATTRIB_DIR)
        {
          ret = -EISDIR;
          goto nodeout;
        }

      if ((oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
        {
          /* Already exists -- can't create it exclusively */

          ret = -EEXIST;
          goto nodeout;
        }

      /* Check if the caller has sufficient privileges to open the file */

      readonly = (node->attrib & EXFAT_ATTRIB_RO) != 0;
      if (((oflags & O_WRONLY) != 0) && readonly)
        {
          ret = -EACCES;
          goto nodeout;
        }

      /* In append mode, we need to set the file pointer to the end of the
       * file.
       */

      if (oflags & O_APPEND)
        {
          filep->f_pos = node->size;
        }

      /* If O_TRUNC is specified and the file is opened for writing,
       * then truncate the file.  This operation requires that the file is
       * writeable, but we have already checked that. O_TRUNC without write
       * access is ignored.
       */

      if ((oflags & (O_TRUNC | O_WRONLY)) == (O_TRUNC | O_WRONLY))
        {
          /* Truncate the file to zero length */

          ret = exfat_truncate(&fs->ef, node, 0, true);
          if (ret < 0)
            {
              goto nodeout;
            }
        }
    }
  else if (ret == -ENOENT)
    {
      /* The file does not exist.  Were we asked to create it? */

      if ((oflags & O_CREAT) == 0)
        {
          /* No.. then we fail with -ENOENT */

          goto errout;
        }

      /* Yes.. create the file */

      ret = exfat_mknod(&fs->ef, relpath);
      if (ret < 0)
        {
          goto errout;
        }

      ret = exfat_lookup(&fs->ef, &node, relpath);
      if (ret < 0)
        {
          goto errout;
        }
    }
  else
    {
      goto errout;
    }

  priv->node = node;
  filep->f_priv = priv;
  exfatfs_semgive(fs);
  return OK;

nodeout:
  exfatfs_release_node(&fs->ef, node);
errout:
  exfatfs_semgive(fs);
semout:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: exfatfs_close
 ****************************************************************************/

static int exfatfs_close(FAR struct file *filep)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_file_s *priv;
  FAR struct inode *inode;
  int ret;

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  exfatfs_release_node(&fs->ef, priv->node);
  kmm_free(filep->f_priv);
  exfatfs_semgive(fs);
  return OK;
}

/****************************************************************************
 * Name: exfatfs_read
 ****************************************************************************/

static ssize_t exfatfs_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_file_s *priv;
  FAR struct inode *inode;
  ssize_t ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = exfat_generic_pread(&fs->ef, priv->node, buffer,
                            buflen, filep->f_pos);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  /* update atime, so we need to flush this node */

  exfat_flush_node(&fs->ef, priv->node);
  exfatfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: exfatfs_write
 ****************************************************************************/

static ssize_t exfatfs_write(FAR struct file *filep, const char *buffer,
                             size_t buflen)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_file_s *priv;
  FAR struct inode *inode;
  ssize_t ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  if ((filep->f_oflags & O_APPEND) && filep->f_pos < priv->node->size)
    {
      filep->f_pos = priv->node->size;
    }

  ret = exfat_generic_pwrite(&fs->ef, priv->node, buffer,
                             buflen, filep->f_pos);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  exfat_flush_node(&fs->ef, priv->node);
  exfatfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: exfatfs_seek
 ****************************************************************************/

static off_t exfatfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_file_s *priv;
  FAR struct inode *inode;
  off_t ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Call LFS to perform the seek */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Map the offset according to the whence option */

  switch (whence)
    {
      case SEEK_CUR:
          offset += filep->f_pos;

      case SEEK_SET:
          if (offset >= 0)
            {
              filep->f_pos = offset;
            }
          else
            {
              ret = -EINVAL;
            }

          break;

      case SEEK_END:
          filep->f_pos = priv->node->size + offset;
          break;

      default:
          ret = -EINVAL;
    }

  exfatfs_semgive(fs);
  return !ret ? filep->f_pos : ret;
}

/****************************************************************************
 * Name: exfatfs_ioctl
 ****************************************************************************/

static int exfatfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_file_s *file;
  FAR struct inode *inode;
  FAR struct inode *drv;

  /* Recover our private data from the struct file instance */

  file  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;
  drv   = fs->drv;

  if (cmd == FIOC_FILEPATH)
    {
      return exfatfs_getpath(inode, file->node, (FAR char *)(uintptr_t)arg);
    }

  return drv->u.i_bops->ioctl(drv, cmd, arg);
}

/****************************************************************************
 * Name: exfatfs_sync
 *
 * Description: Synchronize the file state on disk to match internal, in-
 *   memory state.
 *
 ****************************************************************************/

static int exfatfs_sync(FAR struct file *filep)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_file_s *file;
  FAR struct inode *inode;
  int ret;

  /* Recover our private data from the struct file instance */

  file  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = exfat_flush_node(&fs->ef, file->node);
  exfatfs_semgive(fs);

  return ret;
}

/****************************************************************************
 * Name: exfatfs_dup
 *
 * Description: Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int exfatfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_file_s *priv;
  FAR struct exfatfs_file_s *newpriv;
  FAR struct inode *inode;
  int ret;

  /* Recover our private data from the struct file instance */

  priv  = oldp->f_priv;
  inode = oldp->f_inode;
  fs    = inode->i_private;

  newpriv = kmm_malloc(sizeof(*newpriv));
  if (newpriv == NULL)
    {
      return -ENOMEM;
    }

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      kmm_free(newpriv);
      return ret;
    }

  exfat_get_node(priv->node);
  newpriv->node = priv->node;
  newp->f_priv = newpriv;
  exfatfs_semgive(fs);

  return ret;
}

/****************************************************************************
 * Name: exfatfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int exfatfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_file_s *priv;
  FAR struct inode *inode;
  int ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Call LFS to get file size */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  exfat_stat(&fs->ef, priv->node, buf);
  exfatfs_semgive(fs);

  return OK;
}

/****************************************************************************
 * Name: exfatfs_chstat_i
 ****************************************************************************/

static int exfatfs_chstat_i(FAR struct exfatfs_mountpt_s *fs,
                            FAR struct exfat_node *node,
                            FAR const struct stat *buf, int flags)
{
  if (flags & CH_STAT_MODE)
    {
      const mode_t VALID_MODE_MASK = S_IFREG | S_IFDIR |
                                     S_IRWXU | S_IRWXG | S_IRWXO;
      if (buf->st_mode & ~VALID_MODE_MASK)
        {
          return -EPERM;
        }
    }

  if ((flags & CH_STAT_GID) && buf->st_gid != fs->ef.gid)
    {
      return -EPERM;
    }

  if ((flags & CH_STAT_UID) && buf->st_uid != fs->ef.uid)
    {
      return -EPERM;
    }

  if (flags & CH_STAT_ATIME || flags & CH_STAT_MTIME)
    {
      struct timespec tv[2];

      memset(tv, 0, sizeof(tv));
      if (flags & CH_STAT_ATIME)
        {
          tv[0] = buf->st_atim;
        }
      else
        {
          tv[0].tv_sec = node->atime;
        }

      if (flags & CH_STAT_MTIME)
        {
          tv[1] = buf->st_mtim;
        }
      else
        {
          tv[1].tv_sec = node->mtime;
        }

      exfat_utimes(node, (const struct timespec *)&tv);
      return exfat_flush_node(&fs->ef, node);
    }

  return OK;
}

/****************************************************************************
 * Name: exfatfs_fchstat
 ****************************************************************************/

static int exfatfs_fchstat(FAR const struct file *filep,
                           FAR const struct stat *buf, int flags)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_file_s *priv;
  FAR struct inode *inode;
  int ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Call LFS to get file size */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = exfatfs_chstat_i(fs, priv->node, buf, flags);
  exfatfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: exfatfs_truncate
 *
 * Description:
 *   Set the length of the open, regular file associated with the file
 *   structure 'filep' to 'length'.
 *
 ****************************************************************************/

static int exfatfs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_file_s *priv;
  FAR struct inode *inode;
  int ret;

  /* Recover our private data from the struct file instance */

  priv  = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  /* Call LFS to perform the truncate */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = exfat_truncate(&fs->ef, priv->node, length, true);
  exfatfs_semgive(fs);

  return ret;
}

/****************************************************************************
 * Name: exfatfs_opendir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int exfatfs_opendir(FAR struct inode *mountpt,
                           FAR const char *relpath,
                           FAR struct fs_dirent_s **dir)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_dir_s *priv;
  FAR struct exfat_node *node;
  int ret;

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = exfat_lookup(&fs->ef, &node, relpath);
  if (ret != 0)
    {
      goto lookout;
    }

  if (!(node->attrib & EXFAT_ATTRIB_DIR))
    {
      ret = -ENOTDIR;
      goto errout;
    }

  /* Allocate memory for the open directory */

  priv = kmm_malloc(sizeof(*priv));
  if (priv == NULL)
    {
      ret = -ENOMEM;
      goto errout;
    }

  ret = exfat_opendir(&fs->ef, node, &priv->it);
  if (ret != 0)
    {
      goto openerr;
    }

  priv->entry = exfat_readdir(&priv->it);
  exfatfs_release_node(&fs->ef, node);
  exfatfs_semgive(fs);

  *dir = (FAR struct fs_dirent_s *)priv;
  return OK;

openerr:
  kmm_free(priv);
errout:
  exfatfs_release_node(&fs->ef, node);
lookout:
  exfatfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: exfatfs_closedir
 *
 * Description: Close a directory
 *
 ****************************************************************************/

static int exfatfs_closedir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_dir_s *priv;
  int ret;

  /* Recover our private data from the inode instance */

  priv = (FAR struct exfatfs_dir_s *)dir;
  fs   = mountpt->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->entry)
    {
      exfatfs_release_node(&fs->ef, priv->entry);
    }

  exfat_closedir(&fs->ef, &priv->it);
  kmm_free(priv);
  exfatfs_semgive(fs);

  return OK;
}

/****************************************************************************
 * Name: exfatfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int exfatfs_readdir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir,
                           FAR struct dirent *entry)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_dir_s *priv;
  FAR struct exfat_node *node;
  int ret;

  /* Recover our private data from the inode instance */

  priv = (FAR struct exfatfs_dir_s *)dir;
  fs   = mountpt->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->entry)
    {
      if (priv->entry->attrib == EXFAT_ATTRIB_DIR)
        {
          entry->d_type = DTYPE_DIRECTORY;
        }
      else
        {
          entry->d_type = DTYPE_FILE;
        }

      strlcpy(entry->d_name, priv->entry->name, sizeof(entry->d_name));
      node = priv->entry;
      priv->entry = exfat_readdir(&priv->it);
      exfatfs_release_node(&fs->ef, node);
    }
  else
    {
      ret = -ENOENT;
    }

  exfatfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: exfatfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int exfatfs_rewinddir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfatfs_dir_s *priv;
  int ret;

  /* Recover our private data from the inode instance */

  priv = (FAR struct exfatfs_dir_s *)dir;
  fs   = mountpt->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->entry)
    {
      exfatfs_release_node(&fs->ef, priv->entry);
    }

  priv->it.current = NULL;
  priv->entry = exfat_readdir(&priv->it);
  exfatfs_semgive(fs);

  return OK;
}

/****************************************************************************
 * Name: exfatfs_bind
 ****************************************************************************/

static int exfatfs_bind(FAR struct inode *driver, FAR const void *data,
                        FAR void **handle)
{
  FAR struct exfatfs_mountpt_s *fs;
  struct geometry geo;
  char spec[PATH_MAX];
  int sector_bits;
  int sector_size;
  int ret;

  /* Create an instance of the mountpt state structure */

  fs = kmm_zalloc(sizeof(*fs));
  if (!fs)
    {
      ret = -ENOMEM;
      goto errout_with_block;
    }

  /* Initialize the allocated mountpt state structure. The filesystem is
   * responsible for one reference on the driver inode and does not
   * have to addref() here (but does have to release in unbind().
   */

  fs->drv = driver;           /* Save the driver reference */
  nxsem_init(&fs->sem, 0, 0); /* Initialize the access control semaphore */

  ret = driver->u.i_bops->geometry(driver, &geo);
  if (ret < 0)
    {
      goto errout_with_fs;
    }

  ret = inode_getpath(fs->drv, spec);
  if (ret < 0)
    {
      goto errout_with_fs;
    }

  sector_size = ROUND_UP(geo.geo_sectorsize,
                         sizeof(struct exfat_super_block));
  sector_bits = flsl(sector_size) - 1;

  /* Force format the device if -o forceformat/audoformat  */

  if (data && exfat_match_option(data, "forceformat"))
    {
      ret = exfatfs_format(&fs->ef, spec, sector_bits, -1, NULL, 0, 0);
      if (ret < 0)
        {
          goto errout_with_fs;
        }
    }

  ret = exfat_mount(&fs->ef, spec, data);
  if (ret < 0)
    {
      /* Auto format the device if -o autoformat */

      if (ret != -EIO ||
          !data || !exfat_match_option(data, "autoformat"))
        {
          goto errout_with_fs;
        }

      ret = exfatfs_format(&fs->ef, spec, sector_bits, -1, NULL, 0, 0);
      if (ret < 0)
        {
          goto errout_with_fs;
        }

      /* Try to mount the device again */

      ret = exfat_mount(&fs->ef, spec, data);
      if (ret < 0)
        {
          goto errout_with_fs;
        }
    }

  *handle = fs;
  exfatfs_semgive(fs);
  return OK;

errout_with_fs:
  nxsem_destroy(&fs->sem);
  kmm_free(fs);
errout_with_block:
  return ret;
}

/****************************************************************************
 * Name: exfatfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *  operation.
 *
 ****************************************************************************/

static int exfatfs_unbind(FAR void *handle, FAR struct inode **driver,
                          unsigned int flags)
{
  FAR struct exfatfs_mountpt_s *fs = handle;
  FAR struct inode *drv = fs->drv;
  int ret;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  exfat_unmount(&fs->ef);
  exfatfs_semgive(fs);

  /* We hold a reference to the driver but should not but
   * mucking with inodes in this context. So, we will just return
   * our contained reference to the driver inode and let the
   * umount logic dispose of it.
   */

  if (driver)
    {
      *driver = drv;
    }

  /* Release the mountpoint private data */

  nxsem_destroy(&fs->sem);
  kmm_free(fs);

  return ret;
}

/****************************************************************************
 * Name: exfatfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int exfatfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct exfatfs_mountpt_s *fs;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Return something for the file system description */

  memset(buf, 0, sizeof(*buf));
  buf->f_type    = EXFAT_SUPER_MAGIC;
  buf->f_namelen = EXFAT_ENAME_MAX;
  buf->f_bsize   = CLUSTER_SIZE(*(fs->ef.sb));
  buf->f_blocks  = le32_to_cpu(fs->ef.sb->cluster_count);
  buf->f_bfree   = exfat_count_free_clusters(&fs->ef);
  buf->f_bavail  = exfat_count_free_clusters(&fs->ef);

  return OK;
}

/****************************************************************************
 * Name: exfatfs_unlink
 *
 * Description: Remove a file
 *
 ****************************************************************************/

static int exfatfs_unlink(FAR struct inode *mountpt,
                           FAR const char *relpath)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfat_node *node;
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = exfat_lookup(&fs->ef, &node, relpath);
  if (ret != 0)
    {
      goto errout;
    }

  ret = exfat_unlink(&fs->ef, node);
  exfatfs_release_node(&fs->ef, node);
  if (ret != 0)
    {
      goto errout;
    }

errout:
  exfatfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: exfatfs_mkdir
 *
 * Description: Create a directory
 *
 ****************************************************************************/

static int exfatfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                         mode_t mode)
{
  FAR struct exfatfs_mountpt_s *fs;
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = exfat_mkdir(&fs->ef, relpath);
  exfatfs_semgive(fs);

  return ret;
}

/****************************************************************************
 * Name: exfatfs_rmdir
 *
 * Description: Remove a directory
 *
 ****************************************************************************/

static int exfatfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfat_node *node;
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = exfat_lookup(&fs->ef, &node, relpath);
  if (ret != 0)
    {
      goto errout;
    }

  ret = exfat_rmdir(&fs->ef, node);
  exfatfs_release_node(&fs->ef, node);
  if (ret != 0)
    {
      goto errout;
    }

errout:
  exfatfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: exfatfs_rename
 *
 * Description: Rename a file or directory
 *
 ****************************************************************************/

static int exfatfs_rename(FAR struct inode *mountpt,
                          FAR const char *oldrelpath,
                          FAR const char *newrelpath)
{
  FAR struct exfatfs_mountpt_s *fs;
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Take the semaphore */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = exfat_rename(&fs->ef, oldrelpath, newrelpath);
  exfatfs_semgive(fs);

  return ret;
}

/****************************************************************************
 * Name: exfatfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int exfatfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                        FAR struct stat *buf)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfat_node *node;
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Call LFS to get file size */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = exfat_lookup(&fs->ef, &node, relpath);
  if (ret != 0)
    {
      goto errout;
    }

  exfat_stat(&fs->ef, node, buf);
  exfatfs_release_node(&fs->ef, node);

errout:
  exfatfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: exfatfs_chstat
 ****************************************************************************/

static int exfatfs_chstat(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR const struct stat *buf, int flags)
{
  FAR struct exfatfs_mountpt_s *fs;
  FAR struct exfat_node *node;
  int ret;

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Call LFS to get file size */

  ret = exfatfs_semtake(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = exfat_lookup(&fs->ef, &node, relpath);
  if (ret != 0)
    {
      goto errout;
    }

  ret = exfatfs_chstat_i(fs, node, buf, flags);
  exfatfs_release_node(&fs->ef, node);

errout:
  exfatfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: exfatfs_format
 ****************************************************************************/

static int exfatfs_format(FAR struct exfat *ef, FAR const char *spec,
                          int sector_bits, int spc_bits,
                          FAR const char *volume_label,
                          uint32_t volume_serial, uint64_t first_sector)
{
  FAR struct exfat_dev *dev;
  int ret;

  dev = exfat_open(spec, EXFAT_MODE_RW);
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  ret = exfat_mkfs(dev, sector_bits, spc_bits, volume_label,
                   volume_serial, first_sector);
  if (ret != 0)
    {
      exfat_close(dev);
      return ret;
    }

  return exfat_close(dev);
}

#ifndef CONFIG_LIBC_LOCALTIME
void tzset(void)
{
}
#endif
