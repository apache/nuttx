/****************************************************************************
 * fs/unionfs/fs_unionfs.c
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
#include <sys/mount.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <fixedmath.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/unionfs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mutex.h>

#include "inode/inode.h"

#if !defined(CONFIG_DISABLE_MOUNTPOINT) && defined(CONFIG_FS_UNIONFS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef MIN
#undef MAX
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define MAX(a,b) (((a) > (b)) ? (a) : (b))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct unionfs_dir_s
{
  struct fs_dirent_s fu_base;          /* Vfs directory structure */
  uint8_t fu_ndx;                      /* Index of file system being enumerated */
  bool fu_eod;                         /* True: At end of directory */
  bool fu_prefix[2];                   /* True: Fake directory in prefix */
  FAR char *fu_relpath;                /* Path being enumerated */
  FAR struct fs_dirent_s *fu_lower[2]; /* dirent struct used by contained file system */
};

/* This structure describes one contained file system mountpoint */

struct unionfs_mountpt_s
{
  FAR struct inode *um_node;         /* Filesystem inode */
  FAR char *um_prefix;               /* Path prefix to filesystem */
};

/* This structure describes the union file system */

struct unionfs_inode_s
{
  struct unionfs_mountpt_s ui_fs[2]; /* Contained file systems */
  mutex_t ui_lock;                   /* Enforces mutually exclusive access */
  int16_t ui_nopen;                  /* Number of open references */
  bool ui_unmounted;                 /* File system has been unmounted */
};

/* This structure descries one opened file */

struct unionfs_file_s
{
  uint8_t uf_ndx;                   /* Filesystem index */
  FAR struct file uf_file;          /* Filesystem open file description */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helper functions */

static FAR const char *unionfs_offsetpath(FAR const char *relpath,
                 FAR const char *prefix);
static bool    unionfs_ispartprefix(FAR const char *partprefix,
                 FAR const char *prefix);
static int     unionfs_tryopen(FAR struct file *filep,
                 FAR const char *relpath, FAR const char *prefix, int oflags,
                 mode_t mode);
static int     unionfs_tryopendir(FAR struct inode *inode,
                 FAR const char *relpath, FAR const char *prefix,
                 FAR struct fs_dirent_s **dir);
static int     unionfs_trymkdir(FAR struct inode *inode,
                 FAR const char *relpath, FAR const char *prefix,
                 mode_t mode);
static int     unionfs_tryrmdir(FAR struct inode *inode,
                 FAR const char *relpath, FAR const char *prefix);
static int     unionfs_tryunlink(FAR struct inode *inode,
                 FAR const char *relpath, FAR const char *prefix);
static int     unionfs_tryrename(FAR struct inode *mountpt,
                 FAR const char *oldrelpath, FAR const char *newrelpath,
                 FAR const char *prefix);
static int     unionfs_trystat(FAR struct inode *inode,
                 FAR const char *relpath, FAR const char *prefix,
                 FAR struct stat *buf);
static int     unionfs_trychstat(FAR struct inode *inode,
                 FAR const char *relpath, FAR const char *prefix,
                 FAR const struct stat *buf, int flags);
static int     unionfs_trystatdir(FAR struct inode *inode,
                 FAR const char *relpath, FAR const char *prefix);
static int     unionfs_trystatfile(FAR struct inode *inode,
                 FAR const char *relpath, FAR const char *prefix);
static FAR char *unionfs_relpath(FAR const char *path,
                 FAR const char *name);

static int     unionfs_unbind_child(FAR struct unionfs_mountpt_s *um);
static void    unionfs_destroy(FAR struct unionfs_inode_s *ui);

/* Operations on opened files (with struct file) */

static int     unionfs_open(FAR struct file *filep, const char *relpath,
                 int oflags, mode_t mode);
static int     unionfs_close(FAR struct file *filep);
static ssize_t unionfs_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t unionfs_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static off_t   unionfs_seek(FAR struct file *filep, off_t offset,
                 int whence);
static int     unionfs_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
static int     unionfs_sync(FAR struct file *filep);
static int     unionfs_dup(FAR const struct file *oldp,
                 FAR struct file *newp);
static int     unionfs_fstat(FAR const struct file *filep,
                 FAR struct stat *buf);
static int     unionfs_fchstat(FAR const struct file *filep,
                 FAR const struct stat *buf, int flags);
static int     unionfs_truncate(FAR struct file *filep, off_t length);

/* Operations on directories */

static int     unionfs_opendir(struct inode *mountpt, const char *relpath,
                 FAR struct fs_dirent_s **dir);
static int     unionfs_closedir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);
static int     unionfs_readdir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir,
                 FAR struct dirent *entry);
static int     unionfs_rewinddir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);

static int     unionfs_bind(FAR struct inode *blkdriver,
                 FAR const void *data, FAR void **handle);
static int     unionfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                 unsigned int flags);
static int     unionfs_statfs(FAR struct inode *mountpt,
                 FAR struct statfs *buf);

  /* Operations on paths */

static int     unionfs_unlink(FAR struct inode *mountpt,
                 FAR const char *relpath);
static int     unionfs_mkdir(FAR struct inode *mountpt,
                 FAR const char *relpath, mode_t mode);
static int     unionfs_rmdir(FAR struct inode *mountpt,
                 FAR const char *relpath);
static int     unionfs_rename(FAR struct inode *mountpt,
                 FAR const char *oldrelpath, FAR const char *newrelpath);
static int     unionfs_stat(FAR struct inode *mountpt,
                 FAR const char *relpath, FAR struct stat *buf);
static int     unionfs_chstat(FAR struct inode *mountpt,
                 FAR const char *relpath,
                 FAR const struct stat *buf, int flags);

/* Initialization */

static int     unionfs_getmount(FAR const char *path,
                 FAR struct inode **inode);
static int     unionfs_dobind(FAR const char *fspath1,
                 FAR const char *prefix1, FAR const char *fspath2,
                 FAR const char *prefix2, FAR void **handle);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations unionfs_operations =
{
  unionfs_open,        /* open */
  unionfs_close,       /* close */
  unionfs_read,        /* read */
  unionfs_write,       /* write */
  unionfs_seek,        /* seek */
  unionfs_ioctl,       /* ioctl */
  unionfs_truncate,    /* truncate */
  NULL,                /* mmap */

  unionfs_sync,        /* sync */
  unionfs_dup,         /* dup */
  unionfs_fstat,       /* fstat */
  unionfs_fchstat,     /* fchstat */

  unionfs_opendir,     /* opendir */
  unionfs_closedir,    /* closedir */
  unionfs_readdir,     /* readdir */
  unionfs_rewinddir,   /* rewinddir */

  unionfs_bind,        /* bind */
  unionfs_unbind,      /* unbind */
  unionfs_statfs,      /* statfs */

  unionfs_unlink,      /* unlink */
  unionfs_mkdir,       /* mkdir */
  unionfs_rmdir,       /* rmdir */
  unionfs_rename,      /* rename */
  unionfs_stat,        /* stat */
  unionfs_chstat       /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unionfs_offsetpath
 ****************************************************************************/

static FAR const char *unionfs_offsetpath(FAR const char *relpath,
                                          FAR const char *prefix)
{
  FAR const char *trypath;
  int pfxlen;

  /* Is there a prefix on the path to this file system? */

  if (prefix && (pfxlen = strlen(prefix)) > 0)
    {
      /* Does the prefix match? */

      if (strncmp(prefix, relpath, pfxlen) != 0)
        {
          /* No, then this relative cannot be within this file system */

          return NULL;
        }

      /* Skip over the prefix */

      trypath = relpath + pfxlen;

      /* Make sure that what is left is a valid, relative path */

      for (; *trypath == '/'; trypath++);
    }
  else
    {
      /* No.. use the full, relative path */

      trypath = relpath;
    }

  return trypath;
}

/****************************************************************************
 * Name: unionfs_ispartprefix
 ****************************************************************************/

static bool unionfs_ispartprefix(FAR const char *partprefix,
                                 FAR const char *prefix)
{
  int partlen = 0;
  int pfxlen  = 0;

  /* Trim any '/' characters in the partial prefix */

  if (partprefix != NULL)
    {
      /* Skip over any leading '/' */

      for (; *partprefix == '/'; partprefix++);

      /* Skip over any tailing '/' */

      partlen = strlen(partprefix);
      for (; partlen > 1 && partprefix[partlen - 1]  == '/'; partlen--);
    }

  /* Check for NUL or empty partial prefix */

  if (partprefix == NULL || *partprefix == '\0')
    {
      /* A NUL partial prefix is always contained in the full prefix, even
       * if there is no prefix.
       */

      return true;
    }

  /* Trim any '/' characters in the partial prefix */

  if (prefix != NULL)
    {
      /* Skip over any leading '/' */

      for (; *prefix == '/'; prefix++);

      /* Skip over any tailing '/' */

      pfxlen = strlen(prefix);
      for (; pfxlen > 1 && prefix[pfxlen - 1]  == '/'; pfxlen--);
    }

  /* Check for NUL or empty full prefix */

  if (prefix == NULL || *prefix == '\0')
    {
      /* A non-NUL partial path cannot be a contained in a NUL prefix */

      return false;
    }

#if 0 /* Only whole offset is currently supported */
  /* Both the partial path and the prefix are non-NULL.  Check if the partial
   * path is contained in the prefix.
   */

  if (partlen > pfxlen)
    {
      return false;
    }
#else
  /* Check if the trimmed offsets are identical */

  if (partlen != pfxlen)
    {
      return false;
    }
#endif

  if (strncmp(partprefix, prefix, partlen) == 0)
    {
      return true;
    }
  else
    {
      return false;
    }
}

/****************************************************************************
 * Name: unionfs_tryopen
 ****************************************************************************/

static int unionfs_tryopen(FAR struct file *filep, FAR const char *relpath,
                           FAR const char *prefix, int oflags, mode_t mode)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. try to open this directory */

  DEBUGASSERT(filep->f_inode != NULL && filep->f_inode->u.i_mops != NULL);
  ops = filep->f_inode->u.i_mops;

  if (!ops->open)
    {
      return -ENOSYS;
    }

  return ops->open(filep, trypath, oflags, mode);
}

/****************************************************************************
 * Name: unionfs_tryopendir
 ****************************************************************************/

static int unionfs_tryopendir(FAR struct inode *inode,
                              FAR const char *relpath,
                              FAR const char *prefix,
                              FAR struct fs_dirent_s **dir)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to open this directory */

  ops = inode->u.i_mops;
  DEBUGASSERT(ops && ops->opendir);

  if (!ops->opendir)
    {
      return -ENOSYS;
    }

  return ops->opendir(inode, trypath, dir);
}

/****************************************************************************
 * Name: unionfs_trymkdir
 ****************************************************************************/

static int unionfs_trymkdir(FAR struct inode *inode, FAR const char *relpath,
                            FAR const char *prefix, mode_t mode)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to create the directory */

  ops = inode->u.i_mops;
  if (!ops->mkdir)
    {
      return -ENOSYS;
    }

  return ops->mkdir(inode, trypath, mode);
}

/****************************************************************************
 * Name: unionfs_tryrename
 ****************************************************************************/

static int unionfs_tryrename(FAR struct inode *mountpt,
                             FAR const char *oldrelpath,
                             FAR const char *newrelpath,
                             FAR const char *prefix)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *tryoldpath;
  FAR const char *trynewpath;

  /* Is source path valid on this file system? */

  tryoldpath = unionfs_offsetpath(oldrelpath, prefix);
  if (tryoldpath == NULL)
    {
      /* No.. return -ENOENT.  This should not fail because the existence
       * of the file has already been verified.
       */

      return -ENOENT;
    }

  /* Is source path valid on this file system?
   * REVISIT:  There is no logic currently to rename the file by copying i
   * to a different file system.  So we just fail if the destination name
   * is not within the same file system.  I might, however, be on the other
   * file system and that rename should be supported as a file copy and
   * delete.
   */

  trynewpath = unionfs_offsetpath(newrelpath, prefix);
  if (trynewpath == NULL)
    {
      /* No.. return -ENOSYS.  We should be able to do this, but we can't
       * yet.
       */

      return -ENOSYS;
    }

  /* Yes.. Try to rename the file */

  ops = mountpt->u.i_mops;
  if (!ops->rename)
    {
      return -ENOSYS;
    }

  return ops->rename(mountpt, tryoldpath, trynewpath);
}

/****************************************************************************
 * Name: unionfs_trystat
 ****************************************************************************/

static int unionfs_trystat(FAR struct inode *inode, FAR const char *relpath,
                           FAR const char *prefix, FAR struct stat *buf)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to create the directory */

  ops = inode->u.i_mops;
  if (!ops->stat)
    {
      return -ENOSYS;
    }

  return ops->stat(inode, trypath, buf);
}

/****************************************************************************
 * Name: unionfs_trychstat
 ****************************************************************************/

static int unionfs_trychstat(FAR struct inode *inode,
                             FAR const char *relpath, FAR const char *prefix,
                             FAR const struct stat *buf, int flags)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to change the status */

  ops = inode->u.i_mops;
  if (!ops->chstat)
    {
      return -ENOSYS;
    }

  return ops->chstat(inode, trypath, buf, flags);
}

/****************************************************************************
 * Name: unionfs_trystatdir
 ****************************************************************************/

static int unionfs_trystatdir(FAR struct inode *inode,
                              FAR const char *relpath,
                              FAR const char *prefix)
{
  FAR struct stat buf;
  int ret;

  /* Check if relative path refers to a directory. */

  ret = unionfs_trystat(inode, relpath, prefix, &buf);
  if (ret >= 0 && !S_ISDIR(buf.st_mode))
    {
      return -ENOTDIR;
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_trystatfile
 ****************************************************************************/

static int unionfs_trystatfile(FAR struct inode *inode,
                               FAR const char *relpath,
                               FAR const char *prefix)
{
  FAR struct stat buf;
  int ret;

  /* Check if relative path refers to a regular file.  We specifically
   * exclude directories but neither do we expect any kind of special file
   * to reside on the mounted filesystem.
   */

  ret = unionfs_trystat(inode, relpath, prefix, &buf);
  if (ret >= 0 && !S_ISREG(buf.st_mode))
    {
      return -EISDIR;
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_tryrmdir
 ****************************************************************************/

static int unionfs_tryrmdir(FAR struct inode *inode, FAR const char *relpath,
                            FAR const char *prefix)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to remove the directory */

  ops = inode->u.i_mops;
  if (!ops->rmdir)
    {
      return -ENOSYS;
    }

  return ops->rmdir(inode, trypath);
}

/****************************************************************************
 * Name: unionfs_tryunlink
 ****************************************************************************/

static int unionfs_tryunlink(FAR struct inode *inode,
                             FAR const char *relpath,
                             FAR const char *prefix)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_offsetpath(relpath, prefix);
  if (trypath == NULL)
    {
      /* No.. return -ENOENT */

      return -ENOENT;
    }

  /* Yes.. Try to unlink the file */

  ops = inode->u.i_mops;
  if (!ops->unlink)
    {
      return -ENOSYS;
    }

  return ops->unlink(inode, trypath);
}

/****************************************************************************
 * Name: unionfs_relpath
 ****************************************************************************/

static FAR char *unionfs_relpath(FAR const char *path, FAR const char *name)
{
  FAR char *relpath;
  int pathlen;
  int ret;

  /* Check if there is a valid, non-zero-legnth path */

  if (path && (pathlen = strlen(path)) > 0)
    {
      /* Yes.. extend the file name by prepending the path */

      if (path[pathlen - 1] == '/')
        {
          ret = asprintf(&relpath, "%s%s", path, name);
        }
      else
        {
          ret = asprintf(&relpath, "%s/%s", path, name);
        }

      /* Handle errors */

      if (ret < 0)
        {
          return NULL;
        }
      else
        {
          return relpath;
        }
    }
  else
    {
      /* There is no path... just duplicate the name (so that kmm_free()
       * will work later).
       */

      return strdup(name);
    }
}

/****************************************************************************
 * Name: unionfs_unbind_child
 ****************************************************************************/

static int unionfs_unbind_child(FAR struct unionfs_mountpt_s *um)
{
  FAR struct inode *mpinode = um->um_node;
  FAR struct inode *bdinode = NULL;
  int ret;

  /* Unbind the block driver from the file system (destroying any fs
   * private data).  This logic is essentially the same as the logic in
   * nuttx/fs/mount/fs_umount2.c.
   */

  if (!mpinode->u.i_mops->unbind)
    {
      /* The filesystem does not support the unbind operation ??? */

      return -EINVAL;
    }

  /* The unbind method returns the number of references to the file system
   * (i.e., open files), zero if the unbind was performed, or a negated
   * error code on a failure.
   */

  ret = mpinode->u.i_mops->unbind(mpinode->i_private, &bdinode, MNT_FORCE);
  if (ret < 0)
    {
      /* Some failure occurred */

      return ret;
    }
  else if (ret > 0)
    {
      /* REVISIT: This is bad if the file system cannot support a deferred
       * unmount.  Ideally it would perform the unmount when the last file
       * is closed.  But I don't think any file system do that.
       */

      return -EBUSY;
    }

  /* Successfully unbound */

  mpinode->i_private = NULL;

  /* Release the mountpoint inode and any block driver inode
   * returned by the file system unbind above.  This should cause
   * the inode to be deleted (unless there are other references)
   */

  inode_release(mpinode);

  /* Did the unbind method return a contained block driver */

  if (bdinode)
    {
      inode_release(bdinode);
    }

  return OK;
}

/****************************************************************************
 * Name: unionfs_destroy
 ****************************************************************************/

static void unionfs_destroy(FAR struct unionfs_inode_s *ui)
{
  DEBUGASSERT(ui != NULL && ui->ui_fs[0].um_node != NULL &&
              ui->ui_fs[1].um_node != NULL && ui->ui_nopen == 0);

  /* Unbind the contained file systems */

  unionfs_unbind_child(&ui->ui_fs[0]);
  unionfs_unbind_child(&ui->ui_fs[1]);

  /* Free any allocated prefix strings */

  if (ui->ui_fs[0].um_prefix)
    {
      kmm_free(ui->ui_fs[0].um_prefix);
    }

  if (ui->ui_fs[1].um_prefix)
    {
      kmm_free(ui->ui_fs[1].um_prefix);
    }

  /* And finally free the allocated unionfs state structure as well */

  nxmutex_destroy(&ui->ui_lock);
  kmm_free(ui);
}

/****************************************************************************
 * Name: unionfs_open
 ****************************************************************************/

static int unionfs_open(FAR struct file *filep, FAR const char *relpath,
                        int oflags, mode_t mode)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  int ret;

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  finfo("Opening: ui_nopen=%d\n", ui->ui_nopen);

  /* Get exclusive access to the file system data structures */

  ret = nxmutex_lock(&ui->ui_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate a container to hold the open file system information */

  uf = (FAR struct unionfs_file_s *)
    kmm_malloc(sizeof(struct unionfs_file_s));
  if (uf == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Try to open the file on file system 1 */

  um = &ui->ui_fs[0];
  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);

  uf->uf_file.f_oflags = filep->f_oflags;
  uf->uf_file.f_pos    = 0;
  uf->uf_file.f_inode  = um->um_node;
  uf->uf_file.f_priv   = NULL;

  ret = unionfs_tryopen(&uf->uf_file, relpath, um->um_prefix, oflags, mode);
  if (ret >= 0)
    {
      /* Successfully opened on file system 1 */

      uf->uf_ndx = 0;
    }
  else
    {
      /* Try to open the file on file system 1 */

      um  = &ui->ui_fs[1];

      uf->uf_file.f_oflags = filep->f_oflags;
      uf->uf_file.f_pos    = 0;
      uf->uf_file.f_inode  = um->um_node;
      uf->uf_file.f_priv   = NULL;

      ret = unionfs_tryopen(&uf->uf_file, relpath, um->um_prefix, oflags,
                            mode);
      if (ret < 0)
        {
          goto errout_with_lock;
        }

      /* Successfully opened on file system 1 */

      uf->uf_ndx = 1;
    }

  /* Increment the open reference count */

  ui->ui_nopen++;
  DEBUGASSERT(ui->ui_nopen > 0);

  /* Save our private data in the file structure */

  filep->f_priv = (FAR void *)uf;
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&ui->ui_lock);
  return ret;
}

/****************************************************************************
 * Name: unionfs_close
 ****************************************************************************/

static int unionfs_close(FAR struct file *filep)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;
  int ret = OK;

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  /* Get exclusive access to the file system data structures */

  ret = nxmutex_lock(&ui->ui_lock);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  finfo("Closing: ui_nopen=%d\n", ui->ui_nopen);

  /* Perform the lower level close operation */

  if (ops->close != NULL)
    {
      ret = ops->close(&uf->uf_file);
    }

  /* Decrement the count of open reference.  If that count would go to zero
   * and if the file system has been unmounted, then destroy the file system
   * now.
   */

  if (--ui->ui_nopen <= 0 && ui->ui_unmounted)
    {
      unionfs_destroy(ui);
    }
  else
    {
      nxmutex_unlock(&ui->ui_lock);
    }

  /* Free the open file container */

  kmm_free(uf);
  filep->f_priv = NULL;
  return ret;
}

/****************************************************************************
 * Name: unionfs_read
 ****************************************************************************/

static ssize_t unionfs_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("buflen: %lu\n", (unsigned long)buflen);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level read operation */

  return ops->read ? ops->read(&uf->uf_file, buffer, buflen) : -EPERM;
}

/****************************************************************************
 * Name: unionfs_write
 ****************************************************************************/

static ssize_t unionfs_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("buflen: %lu\n", (unsigned long)buflen);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level write operation */

  return ops->write ? ops->write(&uf->uf_file, buffer, buflen) : -EPERM;
}

/****************************************************************************
 * Name: unionfs_seek
 ****************************************************************************/

static off_t unionfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("offset: %lu whence: %d\n", (unsigned long)offset, whence);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Invoke the file seek method if available */

  if (ops->seek != NULL)
    {
      offset = ops->seek(&uf->uf_file, offset, whence);
    }
  else
    {
      int ret;

      /* Get exclusive access to the file system data structures */

      ret = nxmutex_lock(&ui->ui_lock);
      if (ret < 0)
        {
          return ret;
        }

      /* No... Just set the common file position value */

      switch (whence)
        {
          case SEEK_CUR:
            offset += filep->f_pos;

          case SEEK_SET:
            if (offset >= 0)
              {
                filep->f_pos = offset; /* Might be beyond the end-of-file */
              }
            else
              {
                offset = (off_t)-EINVAL;
              }
            break;

          case SEEK_END:
            offset = (off_t)-ENOSYS;
            break;

          default:
            offset = (off_t)-EINVAL;
            break;
        }

      nxmutex_unlock(&ui->ui_lock);
    }

  return offset;
}

/****************************************************************************
 * Name: unionfs_ioctl
 ****************************************************************************/

static int unionfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("cmd: %d arg: %lu\n", cmd, arg);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level ioctl operation */

  return ops->ioctl ? ops->ioctl(&uf->uf_file, cmd, arg) : -ENOTTY;
}

/****************************************************************************
 * Name: unionfs_sync
 ****************************************************************************/

static int unionfs_sync(FAR struct file *filep)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("filep=%p\n", filep);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level sync operation */

  return ops->sync ? ops->sync(&uf->uf_file) : -EINVAL;
}

/****************************************************************************
 * Name: unionfs_dup
 ****************************************************************************/

static int unionfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct unionfs_file_s *oldpriv;
  FAR struct unionfs_file_s *newpriv;
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;
  int ret = -ENOMEM;

  finfo("oldp=%p newp=%p\n", oldp, newp);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(oldp != NULL && oldp->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)oldp->f_inode->i_private;

  DEBUGASSERT(ui != NULL && oldp->f_priv != NULL);
  oldpriv = (FAR struct unionfs_file_s *)oldp->f_priv;

  DEBUGASSERT(oldpriv->uf_ndx == 0 || oldpriv->uf_ndx == 1);
  um = &ui->ui_fs[oldpriv->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  DEBUGASSERT(newp != NULL && newp->f_priv == NULL);

  /* Allocate a new container for the union FS open file */

  newpriv = (FAR struct unionfs_file_s *)
    kmm_malloc(sizeof(struct unionfs_file_s));
  if (newpriv != NULL)
    {
      /* Clone the old file structure into the newly allocated one */

      memcpy(newpriv, oldpriv, sizeof(struct unionfs_file_s));
      newpriv->uf_file.f_priv = NULL;

      /* Then perform the lower lowel dup operation */

      ret = OK;
      if (ops->dup != NULL)
        {
          ret = ops->dup(&oldpriv->uf_file, &newpriv->uf_file);
          if (ret < 0)
            {
              kmm_free(newpriv);
              newpriv = NULL;
            }
        }

      /* Save the new container in the new file structure */

      newp->f_priv = newpriv;
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int unionfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("filep=%p buf=%p\n", filep, buf);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level write operation */

  return ops->fstat ? ops->fstat(&uf->uf_file, buf) : -EPERM;
}

/****************************************************************************
 * Name: unionfs_fchstat
 *
 * Description:
 *   Change information about an open file associated with the file
 *   descriptor 'filep'.
 *
 ****************************************************************************/

static int unionfs_fchstat(FAR const struct file *filep,
                           FAR const struct stat *buf, int flags)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("filep=%p buf=%p\n", filep, buf);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level change operation */

  return ops->fchstat ? ops->fchstat(&uf->uf_file, buf, flags) : -EPERM;
}

/****************************************************************************
 * Name: unionfs_truncate
 *
 * Description:
 *   Set the size of the file references by 'filep' to 'length'.
 *
 ****************************************************************************/

static int unionfs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_file_s *uf;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;

  finfo("filep=%p length=%ld\n", filep, (long)length);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL &&
              um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level write operation */

  return ops->truncate ? ops->truncate(&uf->uf_file, length) : -EPERM;
}

/****************************************************************************
 * Name: unionfs_opendir
 ****************************************************************************/

static int unionfs_opendir(FAR struct inode *mountpt,
                           FAR const char *relpath,
                           FAR struct fs_dirent_s **dir)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR struct unionfs_dir_s *udir;
  int ret;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  if (!relpath)
    {
      return -EINVAL;
    }

  /* Recover the filesystem data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  udir = kmm_zalloc(sizeof(*udir));
  if (udir == NULL)
    {
      return -ENOMEM;
    }

  /* Get exclusive access to the file system data structures */

  ret = nxmutex_lock(&ui->ui_lock);
  if (ret < 0)
    {
      goto errout_with_udir;
    }

  DEBUGASSERT(dir);

  /* Clone the path.  We will need this when we traverse file system 2 to
   * omit duplicates on file system 1.
   */

  if (strlen(relpath) > 0)
    {
      udir->fu_relpath = strdup(relpath);
      if (!udir->fu_relpath)
        {
          goto errout_with_lock;
        }
    }

  /* Check file system 2 first. */

  um = &ui->ui_fs[1];
  ret = unionfs_tryopendir(um->um_node, relpath, um->um_prefix,
                           &udir->fu_lower[1]);
  if (ret >= 0)
    {
      /* Save the file system 2 access info */

      udir->fu_ndx = 1;
      udir->fu_lower[1]->fd_root = um->um_node;
    }

  /* Check if the user is stat'ing some "fake" node between the unionfs root
   * and the file system 1/2 root directory.
   */

  else if (unionfs_ispartprefix(relpath, ui->ui_fs[1].um_prefix))
    {
      /* File system 2 prefix includes this relpath */

      udir->fu_ndx = 1;
      udir->fu_prefix[1] = true;
    }

  /* Check file system 1 last, possibly overwriting fu_ndx */

  um = &ui->ui_fs[0];
  ret = unionfs_tryopendir(um->um_node, relpath, um->um_prefix,
                           &udir->fu_lower[0]);
  if (ret >= 0)
    {
      /* Save the file system 1 access info */

      udir->fu_ndx = 0;
      udir->fu_lower[0]->fd_root = um->um_node;
    }
  else
    {
      /* Check if the user is stat'ing some "fake" node between the unionfs
       * root and the file system 1 root directory.
       */

      if (unionfs_ispartprefix(relpath, ui->ui_fs[0].um_prefix))
        {
          /* File system 1 offset includes this relpath.  Make sure that only
           * one
           */

          udir->fu_ndx = 0;
          udir->fu_prefix[0] = true;
          udir->fu_prefix[1] = false;
        }

      /* If the directory was not found on either file system, then we have
       * failed to open this path on either file system.
       */

      else if (udir->fu_lower[1] == NULL && !udir->fu_prefix[1])
        {
          /* Neither of the two path file systems include this relpath */

          ret = -ENOENT;
          goto errout_with_relpath;
        }
    }

  /* Increment the number of open references and return success */

  ui->ui_nopen++;
  DEBUGASSERT(ui->ui_nopen > 0);

  nxmutex_unlock(&ui->ui_lock);
  *dir = &udir->fu_base;
  return OK;

errout_with_relpath:
  if (udir->fu_relpath != NULL)
    {
      kmm_free(udir->fu_relpath);
    }

errout_with_lock:
  nxmutex_unlock(&ui->ui_lock);

errout_with_udir:
  kmm_free(udir);
  return ret;
}

/****************************************************************************
 * Name: unionfs_closedir
 ****************************************************************************/

static int unionfs_closedir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;
  FAR struct unionfs_dir_s *udir;
  int ret = OK;
  int i;

  finfo("mountpt=%p dir=%p\n", mountpt, dir);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* Get exclusive access to the file system data structures */

  ret = nxmutex_lock(&ui->ui_lock);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(dir);
  udir = (FAR struct unionfs_dir_s *)dir;

  /* Close both contained file systems */

  for (i = 0; i < 2; i++)
    {
      /* Was this file system opened? */

      if (udir->fu_lower[i] != NULL)
        {
          um = &ui->ui_fs[i];

          DEBUGASSERT(um != NULL && um->um_node != NULL &&
                      um->um_node->u.i_mops != NULL);
          ops = um->um_node->u.i_mops;

          /* Perform the lower level closedir operation */

          if (ops->closedir != NULL)
            {
              ret = ops->closedir(um->um_node, udir->fu_lower[i]);
            }
        }
    }

  /* Free any allocated path */

  if (udir->fu_relpath != NULL)
    {
      kmm_free(udir->fu_relpath);
    }

  kmm_free(udir);

  /* Decrement the count of open reference.  If that count would go to zero
   * and if the file system has been unmounted, then destroy the file system
   * now.
   */

  if (--ui->ui_nopen <= 0 && ui->ui_unmounted)
    {
      unionfs_destroy(ui);
    }
  else
    {
      nxmutex_unlock(&ui->ui_lock);
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_readdir
 ****************************************************************************/

static int unionfs_readdir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir,
                           FAR struct dirent *entry)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR struct unionfs_mountpt_s *um0;
  FAR const struct mountpt_operations *ops;
  FAR struct unionfs_dir_s *udir;
  FAR char *relpath;
  struct stat buf;
  bool duplicate;
  int ret = -ENOSYS;

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  DEBUGASSERT(dir);
  udir = (FAR struct unionfs_dir_s *)dir;

  /* Check if we are at the end of the directory listing. */

  if (udir->fu_eod)
    {
      /* End of file and error conditions are not distinguishable
       * with readdir.  Here we return -ENOENT to signal the end
       * of the directory.
       */

      return -ENOENT;
    }

  DEBUGASSERT(udir->fu_ndx == 0 || udir->fu_ndx == 1);
  um = &ui->ui_fs[udir->fu_ndx];

  /* Special case: If the open directory is a 'fake' node in the prefix on
   * one of the mounted file system, then we must also fake the return value.
   */

  if (udir->fu_prefix[udir->fu_ndx])
    {
      DEBUGASSERT(udir->fu_lower[udir->fu_ndx] == NULL &&
                  um->um_prefix != NULL);

      /* Copy the file system offset into the dirent structure.
       * REVISIT:  This will not handle the case where the prefix contains
       * the '/' character the so the offset appears to be multiple
       * directories.
       */

      strlcpy(entry->d_name, um->um_prefix, sizeof(entry->d_name));

      /* Describe this as a read only directory */

      entry->d_type = DTYPE_DIRECTORY;

      /* Increment the index to file system 2 (maybe) */

      if (udir->fu_ndx == 0 && (udir->fu_prefix[1] ||
                                udir->fu_lower[1] != NULL))
        {
          /* Yes.. set up to do file system 2 next time */

          udir->fu_ndx++;
        }
      else
        {
          /* No.. we are finished */

          udir->fu_eod = true;
        }

      return OK;
    }

  /* This is a normal, mediated file system readdir() */

  DEBUGASSERT(udir->fu_lower[udir->fu_ndx] != NULL);
  DEBUGASSERT(um->um_node != NULL && um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  finfo("fu_ndx: %d\n", udir->fu_ndx);

  /* Perform the lower level readdir operation */

  if (ops->readdir != NULL)
    {
      /* Loop if we discard duplicate directory entries in filey system 2 */

      do
        {
          /* Read the directory entry */

          ret = ops->readdir(um->um_node, udir->fu_lower[udir->fu_ndx],
                             entry);

          /* Did the read operation fail because we reached the end of the
           * directory?  In that case, the error would be -ENOENT.  If we
           * hit the end-of-directory on file system, we need to seamlessly
           * move to the second file system (if there is one).
           */

          if (ret == -ENOENT && udir->fu_ndx == 0)
            {
              /* Special case: If the open directory is a 'fake' node in the
               * prefix on file system2, then we must also fake the return
               * value.
               */

              if (udir->fu_prefix[1])
                {
                  DEBUGASSERT(udir->fu_lower[1] == NULL);

                  /* Switch to the second file system */

                  udir->fu_ndx = 1;
                  um = &ui->ui_fs[1];

                  DEBUGASSERT(um != NULL && um->um_prefix != NULL);

                  /* Copy the file system offset into the dirent structure.
                   * REVISIT:  This will not handle the case where the prefix
                   * contains the '/' character the so the offset appears to
                   * be multiple directories.
                   */

                  strlcpy(entry->d_name, um->um_prefix,
                          sizeof(entry->d_name));

                  /* Describe this as a read only directory */

                  entry->d_type = DTYPE_DIRECTORY;

                  /* Mark the end of the directory listing */

                  udir->fu_eod = true;

                  /* Check if have already reported something of this name
                   * in file system 1.
                   */

                  relpath = unionfs_relpath(udir->fu_relpath, um->um_prefix);
                  if (relpath)
                    {
                      int tmp;

                      /* Check if anything exists at this path on file
                       * system 1
                       */

                      um0 = &ui->ui_fs[0];
                      tmp = unionfs_trystat(um0->um_node, relpath,
                                            um0->um_prefix, &buf);

                      /* Free the allocated relpath */

                      kmm_free(relpath);

                      /* Check for a duplicate */

                      if (tmp >= 0)
                        {
                          /* There is something there!
                           * REVISIT: We could allow files and directories to
                           * have duplicate names.
                           */

                          return -ENOENT;
                        }
                    }

                  return OK;
                }

              /* No.. check for a normal directory access */

              else if (udir->fu_lower[1] != NULL)
                {
                  /* Switch to the second file system */

                  udir->fu_ndx = 1;
                  um = &ui->ui_fs[1];

                  DEBUGASSERT(um != NULL && um->um_node != NULL &&
                              um->um_node->u.i_mops != NULL);
                  ops = um->um_node->u.i_mops;

                  /* Make sure that the second file system directory
                   * enumeration is rewound to the beginning of the
                   * directory.
                   */

                  if (ops->rewinddir != NULL)
                    {
                      ret = ops->rewinddir(um->um_node, udir->fu_lower[1]);
                      if (ret < 0)
                        {
                          return ret;
                        }
                    }

                  /* Then try the read operation again */

                  ret = ops->readdir(um->um_node, udir->fu_lower[1], entry);
                }
            }

          /* Did we successfully read a directory from file system 2?  If
           * so, we need to omit an duplicates that should be occluded by
           * the matching file on file system 1 (if we are enumerating
           * file system 1).
           */

          duplicate = false;
          if (ret >= 0 && udir->fu_ndx == 1 && udir->fu_lower[0] != NULL)
            {
              /* Get the relative path to the same file on file system 1.
               * NOTE: the on any failures we just assume that the filep
               * is not a duplicate.
               */

              relpath = unionfs_relpath(udir->fu_relpath, entry->d_name);
              if (relpath)
                {
                  int tmp;

                  /* Check if anything exists at this path on file system 1 */

                  um0 = &ui->ui_fs[0];
                  tmp = unionfs_trystat(um0->um_node, relpath,
                                        um0->um_prefix, &buf);
                  if (tmp >= 0)
                    {
                      /* There is something there!
                       * REVISIT: We could allow files and directories to
                       * have duplicate names.
                       */

                      duplicate = true;
                    }

                  /* Free the allocated relpath */

                  kmm_free(relpath);
                }
            }
        }
      while (duplicate);
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_rewindir
 ****************************************************************************/

static int unionfs_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR const struct mountpt_operations *ops;
  FAR struct unionfs_dir_s *udir;
  int ret = -EINVAL;

  finfo("mountpt=%p dir=%p\n", mountpt, dir);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  DEBUGASSERT(dir);
  udir = (FAR struct unionfs_dir_s *)dir;

  /* Were we currently enumerating on file system 1?  If not, is an
   * enumeration possible on file system 1?
   */

  DEBUGASSERT(udir->fu_ndx == 0 || udir->fu_ndx == 1);
  if (/* udir->fu_ndx != 0 && */ udir->fu_prefix[0] || udir->fu_lower[0] != NULL)
    {
      /* Yes.. switch to file system 1 */

      udir->fu_ndx = 0;
    }

  if (!udir->fu_prefix[udir->fu_ndx])
    {
      DEBUGASSERT(udir->fu_lower[udir->fu_ndx] != NULL);
      um = &ui->ui_fs[udir->fu_ndx];

      DEBUGASSERT(um != NULL && um->um_node != NULL &&
                  um->um_node->u.i_mops != NULL);
      ops = um->um_node->u.i_mops;

      /* Perform the file system rewind operation */

      if (ops->rewinddir != NULL)
        {
          ret = ops->rewinddir(um->um_node, udir->fu_lower[udir->fu_ndx]);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_bind
 ****************************************************************************/

static int unionfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                        FAR void **handle)
{
  FAR const char *fspath1 = "";
  FAR const char *prefix1 = "";
  FAR const char *fspath2 = "";
  FAR const char *prefix2 = "";
  FAR char *dup;
  FAR char *tmp;
  FAR char *tok;
  int ret;

  /* Parse options from mount syscall */

  dup = tmp = strdup(data);
  if (!dup)
    {
      return -ENOMEM;
    }

  while ((tok = strsep(&tmp, ",")))
    {
      if (tok == strstr(tok, "fspath1="))
        {
          fspath1 = tok + 8;
        }
      else if (tok == strstr(tok, "prefix1="))
        {
          prefix1 = tok + 8;
        }
      else if (tok == strstr(tok, "fspath2="))
        {
          fspath2 = tok + 8;
        }
      else if (tok == strstr(tok, "prefix2="))
        {
          prefix2 = tok + 8;
        }
    }

  /* Call unionfs_dobind to do the real work. */

  ret = unionfs_dobind(fspath1, prefix1, fspath2, prefix2, handle);
  kmm_free(dup);

  return ret;
}

/****************************************************************************
 * Name: unionfs_unbind
 ****************************************************************************/

static int unionfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                          unsigned int flags)
{
  FAR struct unionfs_inode_s *ui;
  int ret;

  finfo("handle=%p blkdriver=%p flags=%x\n", handle, blkdriver, flags);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(handle != NULL);
  ui = (FAR struct unionfs_inode_s *)handle;

  /* Get exclusive access to the file system data structures */

  ret = nxmutex_lock(&ui->ui_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Mark the file system as unmounted. */

  ui->ui_unmounted = true;

  /* If there are no open references, then we can destroy the file system
   * now.
   */

  if (ui->ui_nopen <= 0)
    {
      nxmutex_unlock(&ui->ui_lock);
      unionfs_destroy(ui);
    }
  else
    {
      nxmutex_unlock(&ui->ui_lock);
    }

  return OK;
}

/****************************************************************************
 * Name: unionfs_statfs
 ****************************************************************************/

static int unionfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um1;
  FAR struct unionfs_mountpt_s *um2;
  FAR const struct mountpt_operations *ops1;
  FAR const struct mountpt_operations *ops2;
  FAR struct statfs *adj;
  struct statfs buf1;
  struct statfs buf2;
  uint64_t tmp;
  uint32_t ratiob16;
  int ret;

  finfo("mountpt=%p buf=%p\n", mountpt, buf);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && buf != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  memset(buf, 0, sizeof(struct statfs));

  /* Get statfs info from file system 1.
   *
   * REVISIT: What would it mean if one file system did not support statfs?
   * Perhaps we could simplify the following by simply insisting that both
   * file systems support the statfs method.
   */

  um1 = &ui->ui_fs[0];
  DEBUGASSERT(um1 != NULL && um1->um_node != NULL &&
              um1->um_node->u.i_mops != NULL);
  ops1 = um1->um_node->u.i_mops;

  um2 = &ui->ui_fs[1];
  DEBUGASSERT(um2 != NULL && um2->um_node != NULL &&
              um2->um_node->u.i_mops != NULL);
  ops2 = um2->um_node->u.i_mops;

  if (ops1->statfs != NULL && ops2->statfs != NULL)
    {
      ret = ops1->statfs(um1->um_node, &buf1);
      if (ret < 0)
        {
          return ret;
        }

      /* Get stafs info from file system 2 */

      ret = ops2->statfs(um2->um_node, &buf2);
      if (ret < 0)
        {
          return ret;
        }
    }
  else if (ops1->statfs != NULL)
    {
      /* We have statfs for file system 1 only */

      return ops1->statfs(um1->um_node, buf);
    }
  else if (ops2->statfs != NULL)
    {
      /* We have statfs for file system 2 only */

      return ops2->statfs(um2->um_node, buf);
    }
  else
    {
      /* We could not get stafs info from either file system */

      return -ENOSYS;
    }

  /* We get here is we successfully obtained statfs info from both file
   * systems.  Now combine those results into one statfs report, trying to
   * reconcile any conflicts between the file system geometries.
   */

  buf->f_type    = UNIONFS_MAGIC;
  buf->f_namelen = MIN(buf1.f_namelen, buf2.f_namelen);
  buf->f_files   = buf1.f_files + buf2.f_files;
  buf->f_ffree   = buf1.f_ffree + buf2.f_ffree;

  /* Things expressed in units of blocks are the only tricky ones.  We will
   * depend on a uint64_t * temporary to avoid arithmetic overflow.
   */

  buf->f_bsize           = buf1.f_bsize;
  if (buf1.f_bsize != buf2.f_bsize)
    {
      if (buf1.f_bsize < buf2.f_bsize)
        {
          tmp           = (((uint64_t)buf2.f_blocks *
                            (uint64_t)buf2.f_bsize) << 16);
          ratiob16      = (uint32_t)(tmp / buf1.f_bsize);
          adj           = &buf2;
        }
      else
        {
          buf->f_bsize  = buf2.f_bsize;
          tmp           = (((uint64_t)buf1.f_blocks *
                            (uint64_t)buf1.f_bsize) << 16);
          ratiob16      = (uint32_t)(tmp / buf2.f_bsize);
          adj           = &buf1;
        }

      tmp               = (uint64_t)adj->f_blocks * ratiob16;
      adj->f_blocks     = (off_t)(tmp >> 16);

      tmp               = (uint64_t)adj->f_bfree  * ratiob16;
      adj->f_bfree      = (off_t)(tmp >> 16);

      tmp               = (uint64_t)adj->f_bavail * ratiob16;
      adj->f_bavail     = (off_t)(tmp >> 16);
    }

  /* Then we can just sum the adjusted sizes */

  buf->f_blocks         = buf1.f_blocks + buf2.f_blocks;
  buf->f_bfree          = buf1.f_bfree  + buf2.f_bfree;
  buf->f_bavail         = buf1.f_bavail + buf2.f_bavail;

  return OK;
}

/****************************************************************************
 * Name: unionfs_unlink
 ****************************************************************************/

static int unionfs_unlink(FAR struct inode *mountpt,
                          FAR const char *relpath)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  struct stat buf;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL &&
              relpath != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* Check if some exists at this path on file system 1.  This might be
   * a file or a directory
   */

  um  = &ui->ui_fs[0];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, &buf);
  if (ret >= 0)
    {
      /* Yes.. Try to unlink the file on file system 1 (perhaps exposing
       * a file of the same name on file system 2).  This would fail
       * with -ENOSYS if file system 1 is a read-only only file system or
       * -EISDIR if the path is not a file.
       */

      ret = unionfs_tryunlink(um->um_node, relpath, um->um_prefix);
    }

  /* There is nothing at this path on file system 1 */

  else
    {
      /* Check if the file exists with name on file system 2.  The only
       * reason that we check here is so that we can return the more
       * meaningful -ENOSYS if file system 2 is a read-only file system.
       */

      um  = &ui->ui_fs[1];
      ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, &buf);
      if (ret >= 0)
        {
          /* Yes.. Try to unlink the file on file system 1.  This would fail
           * with -ENOSYS if file system 2 is a read-only only file system or
           * -EISDIR if the path is not a file.
           * */

          ret = unionfs_tryunlink(um->um_node, relpath, um->um_prefix);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_mkdir
 ****************************************************************************/

static int unionfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                         mode_t mode)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  struct stat buf;
  int ret1;
  int ret2;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL &&
              relpath != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* Is there anything with this name on either file system? */

  um  = &ui->ui_fs[0];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, &buf);
  if (ret >= 0)
    {
      return -EEXIST;
    }

  um  = &ui->ui_fs[1];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, &buf);
  if (ret >= 0)
    {
      return -EEXIST;
    }

  /* Try to create the directory on both file systems. */

  um  = &ui->ui_fs[0];
  ret1 = unionfs_trymkdir(um->um_node, relpath, um->um_prefix, mode);

  um  = &ui->ui_fs[1];
  ret2 = unionfs_trymkdir(um->um_node, relpath, um->um_prefix, mode);

  /* We will say we were successful if we were able to create the
   * directory on either file system.  Perhaps one file system is
   * read-only and the other is write-able?
   */

  return (ret1 >= 0 || ret2 >= 0) ? OK : ret1;
}

/****************************************************************************
 * Name: unionfs_rmdir
 ****************************************************************************/

static int unionfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  int ret = -ENOENT;
  int tmp;

  finfo("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL &&
              relpath != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* We really don't know any better so we will try to remove the directory
   * from both file systems.
   */

  /* Is there a directory with this name on file system 1 */

  um   = &ui->ui_fs[0];
  tmp = unionfs_trystatdir(um->um_node, relpath, um->um_prefix);
  if (tmp >= 0)
    {
      /* Yes.. remove it.  Since we know that the directory exists, any
       * failure to remove it is a showstopper.
       */

      ret = unionfs_tryrmdir(um->um_node, relpath, um->um_prefix);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* Either the directory does not exist on file system 1, or we
   * successfully removed it.  Try again on file system 2.
   */

  um   = &ui->ui_fs[1];
  tmp = unionfs_trystatdir(um->um_node, relpath, um->um_prefix);
  if (tmp >= 0)
    {
      /* Yes.. remove it.  Since we know that the directory exists, any
       * failure to remove it is a showstopper.
       */

      ret = unionfs_tryrmdir(um->um_node, relpath, um->um_prefix);

      /* REVISIT:  Should we try to restore the directory on file system 1
       * if we failure to removed the directory on file system 2?
       */
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_rename
 ****************************************************************************/

static int unionfs_rename(FAR struct inode *mountpt,
                         FAR const char *oldrelpath,
                         FAR const char *newrelpath)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  int ret = -ENOENT;
  int tmp;

  finfo("oldrelpath: %s newrelpath: %s\n", oldrelpath, newrelpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  DEBUGASSERT(oldrelpath != NULL && oldrelpath != NULL);

  /* Is there a file with this name on file system 1 */

  um   = &ui->ui_fs[0];
  tmp = unionfs_trystatfile(um->um_node, oldrelpath, um->um_prefix);
  if (tmp >= 0)
    {
      /* Yes.. rename it.  Since we know that the directory exists, any
       * failure to remove it is a showstopper.
       */

      ret = unionfs_tryrename(um->um_node, oldrelpath, newrelpath,
                              um->um_prefix);
      if (ret >= 0)
        {
          /* Return immediately on success.  In the event that the file
           * exists in both file systems, this will produce the odd behavior
           * that one file on file system 1 was renamed but another obscured
           * file of the same relative path will become visible.
           */

          return OK;
        }
    }

  /* Either the file does not exist on file system 1, or we failed to rename
   * it (perhaps because the file system was read-only).  Try again on file
   * system 2.
   */

  um   = &ui->ui_fs[1];
  tmp = unionfs_trystatfile(um->um_node, oldrelpath, um->um_prefix);
  if (tmp >= 0)
    {
      /* Yes.. remove it.  Since we know that the directory exists, any
       * failure to remove it is a showstopper.
       */

      ret = unionfs_tryrename(um->um_node, oldrelpath, newrelpath,
                              um->um_prefix);
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_stat
 ****************************************************************************/

static int unionfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                       FAR struct stat *buf)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL &&
              relpath != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* stat this path on file system 1 */

  um  = &ui->ui_fs[0];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, buf);
  if (ret >= 0)
    {
      /* Return on the first success.  The first instance of the file will
       * shadow the second anyway.
       */

      return OK;
    }

  /* stat failed on the file system 1.  Try again on file system 2. */

  um  = &ui->ui_fs[1];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, buf);
  if (ret >= 0)
    {
      /* Return on the first success.  The first instance of the file will
       * shadow the second anyway.
       */

      return OK;
    }

  /* Special case the unionfs root directory when both file systems are
   * offset.  In that case, both of the above trystat calls will fail.
   */

  if (ui->ui_fs[0].um_prefix != NULL && ui->ui_fs[1].um_prefix != NULL)
    {
      /* Most of the stat entries just do not apply */

      memset(buf, 0, sizeof(struct stat));

      /* Claim that this is a read-only directory */

      buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;

      /* Check if the user is stat'ing some "fake" node between the unionfs
       * root and the file system 1 root directory.
       */

      if (unionfs_ispartprefix(relpath, ui->ui_fs[0].um_prefix) ||
          unionfs_ispartprefix(relpath, ui->ui_fs[1].um_prefix))
        {
          ret = OK;
        }
      else
        {
          ret = -ENOENT;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_chstat
 ****************************************************************************/

static int unionfs_chstat(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR const struct stat *buf, int flags)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  int ret;

  finfo("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL &&
              relpath != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* chstat this path on file system 1 */

  um  = &ui->ui_fs[0];
  ret = unionfs_trychstat(um->um_node, relpath, um->um_prefix, buf, flags);
  if (ret >= 0)
    {
      /* Return on the first success.  The first instance of the file will
       * shadow the second anyway.
       */

      return OK;
    }

  /* chstat failed on the file system 1.  Try again on file system 2. */

  um  = &ui->ui_fs[1];
  ret = unionfs_trychstat(um->um_node, relpath, um->um_prefix, buf, flags);
  if (ret >= 0)
    {
      return OK;
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_getmount
 ****************************************************************************/

static int unionfs_getmount(FAR const char *path, FAR struct inode **inode)
{
  FAR struct inode *minode;
  struct inode_search_s desc;
  int ret;

  /* Find the mountpt */

  SETUP_SEARCH(&desc, path, false);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* Mountpoint inode not found */

      goto errout_with_search;
    }

  /* Get the search results */

  minode = desc.node;
  DEBUGASSERT(minode != NULL);

  /* Verify that the inode is a mountpoint.
   *
   * REVISIT: If desc.relpath points to a non-empty string, then the path
   * does not really refer to a mountpoint but, rather, to a some entity
   * within the mounted volume.
   */

  if (!INODE_IS_MOUNTPT(minode))
    {
      /* Inode was found, but is it is not a mounpoint */

      ret = -EINVAL;
      goto errout_with_inode;
    }

  /* Success! */

  *inode = minode;
  RELEASE_SEARCH(&desc);
  return OK;

errout_with_inode:
  inode_release(minode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}

/****************************************************************************
 * Name: unionfs_dobind
 ****************************************************************************/

static int unionfs_dobind(FAR const char *fspath1, FAR const char *prefix1,
                          FAR const char *fspath2, FAR const char *prefix2,
                          FAR void **handle)
{
  FAR struct unionfs_inode_s *ui;
  int ret;

  DEBUGASSERT(fspath1 != NULL && fspath2 != NULL && handle != NULL);

  /* Allocate a structure a structure that will describe the union file
   * system.
   */

  ui = (FAR struct unionfs_inode_s *)
    kmm_zalloc(sizeof(struct unionfs_inode_s));
  if (!ui)
    {
      ferr("ERROR: Failed to allocated union FS state structure\n");
      return -ENOMEM;
    }

  nxmutex_init(&ui->ui_lock);

  /* Get the inodes associated with fspath1 and fspath2 */

  ret = unionfs_getmount(fspath1, &ui->ui_fs[0].um_node);
  if (ret < 0)
    {
      ferr("ERROR: unionfs_getmount(fspath1) failed: %d\n", ret);
      goto errout_with_uinode;
    }

  ret = unionfs_getmount(fspath2, &ui->ui_fs[1].um_node);
  if (ret < 0)
    {
      ferr("ERROR: unionfs_getmount(fspath2) failed: %d\n", ret);
      goto errout_with_fs1;
    }

  /* Duplicate the prefix strings */

  if (prefix1 && strlen(prefix1) > 0)
    {
      ui->ui_fs[0].um_prefix = strdup(prefix1);
      if (ui->ui_fs[0].um_prefix == NULL)
        {
          ferr("ERROR: strdup(prefix1) failed\n");
          ret = -ENOMEM;
          goto errout_with_fs2;
        }
    }

  if (prefix2 && strlen(prefix2) > 0)
    {
      ui->ui_fs[1].um_prefix = strdup(prefix2);
      if (ui->ui_fs[1].um_prefix == NULL)
        {
          ferr("ERROR: strdup(prefix2) failed\n");
          ret = -ENOMEM;
          goto errout_with_prefix1;
        }
    }

  /* Unlink the contained mountpoint inodes from the pseudo file system.
   * The inodes will be marked as deleted so that they will be removed when
   * the reference count decrements to zero in inode_release().  Because we
   * hold a reference count on the inodes, they will not be deleted until
   * this logic calls inode_release() in the unionfs_destroy() function.
   */

  inode_remove(fspath1);
  inode_remove(fspath2);

  *handle = ui;
  return OK;

errout_with_prefix1:
  if (ui->ui_fs[0].um_prefix != NULL)
    {
      kmm_free(ui->ui_fs[0].um_prefix);
    }

errout_with_fs2:
  inode_release(ui->ui_fs[1].um_node);

errout_with_fs1:
  inode_release(ui->ui_fs[0].um_node);

errout_with_uinode:
  nxmutex_lock(&ui->ui_lock);
  kmm_free(ui);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unionfs_mount
 *
 * Description:
 *   Create and mount a union file system
 *
 * Input Parameters:
 *   fspath1 - The full path to the first file system mountpoint
 *   prefix1 - An optiona prefix that may be applied to make the first
 *             file system appear a some path below the unionfs mountpoint,
 *   fspath2 - The full path to the second file system mountpoint
 *   prefix2 - An optiona prefix that may be applied to make the first
 *             file system appear a some path below the unionfs mountpoint,
 *   mountpt - The full path to the mountpoint for the union file system
 *
 * Returned Value:
 *   Zero (OK) is returned if the union file system was correctly created and
 *   mounted.  On any failure, a negated error value will be returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

int unionfs_mount(FAR const char *fspath1, FAR const char *prefix1,
                  FAR const char *fspath2, FAR const char *prefix2,
                  FAR const char *mountpt)
{
  FAR struct inode *mpinode;
  int ret;

  DEBUGASSERT(mountpt != NULL);

  /* Mount the union FS.  We should adapt the standard mount to do
   * this using optional parameters.  This custom mount should do the job
   * for now, however.
   */

  ret = inode_reserve(mountpt, 0777, &mpinode);
  if (ret < 0)
    {
      /* inode_reserve can fail for a couple of reasons, but the most likely
       * one is that the inode already exists. inode_reserve may return:
       *
       *  -EINVAL - 'path' is invalid for this operation
       *  -EEXIST - An inode already exists at 'path'
       *  -ENOMEM - Failed to allocate in-memory resources for the operation
       */

      ferr("ERROR: Failed to reserve inode\n");
      return ret;
    }

  /* Populate the inode with driver specific information. */

  INODE_SET_MOUNTPT(mpinode);

  mpinode->u.i_mops = &unionfs_operations;

  /* Call unionfs_dobind to do the real work. */

  ret = unionfs_dobind(fspath1, prefix1, fspath2, prefix2,
                       &mpinode->i_private);
  if (ret < 0)
    {
      goto errout_with_mountpt;
    }

  return OK;

errout_with_mountpt:
  inode_release(mpinode);
  return ret;
}
#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_UNIONFS */
