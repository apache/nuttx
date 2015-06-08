/****************************************************************************
 * fs/unionfs/fs_unionfs.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <sys/mount.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <fixedmath.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/unionfs.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/binfmt/builtin.h>

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
  sem_t ui_exclsem;                  /* Enforces mutually exclusive access */
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

static int     unionfs_semtake(FAR struct unionfs_inode_s *ui, bool noint);
#define        unionfs_semgive(ui) (void)sem_post(&(ui)->ui_exclsem)

static FAR const char *unionfs_trypath(FAR const char *relpath,
                 FAR const char *prefix);
static int     unionfs_tryopen(FAR struct file *filep,
                 FAR const char *relpath, FAR const char *prefix, int oflags,
                 mode_t mode);
static int     unionfs_tryopendir(FAR struct inode *inode,
                 FAR const char *relpath, FAR const char *prefix,
                 FAR struct fs_dirent_s *dir);
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
static off_t   unionfs_seek(FAR struct file *filep, off_t offset, int whence);
static int     unionfs_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
static int     unionfs_sync(FAR struct file *filep);
static int     unionfs_dup(FAR const struct file *oldp,
                 FAR struct file *newp);

/* Operations on directories */

static int     unionfs_opendir(struct inode *mountpt, const char *relpath,
                 FAR struct fs_dirent_s *dir);
static int     unionfs_closedir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);
static int     unionfs_readdir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);
static int     unionfs_rewinddir(FAR struct inode *mountpt,
                 FAR struct fs_dirent_s *dir);

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

/* Initialization */

static int     unionfs_getmount(FAR const char *path,
                 FAR struct inode **inode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly extern'ed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

static const struct mountpt_operations g_unionfs_mops =
{
  unionfs_open,        /* open */
  unionfs_close,       /* close */
  unionfs_read,        /* read */
  unionfs_write,       /* write */
  unionfs_seek,        /* seek */
  unionfs_ioctl,       /* ioctl */

  unionfs_sync,        /* sync */
  unionfs_dup,         /* dup */

  unionfs_opendir,     /* opendir */
  unionfs_closedir,    /* closedir */
  unionfs_readdir,     /* readdir */
  unionfs_rewinddir,   /* rewinddir */

  NULL,                /* bind */
  unionfs_unbind,      /* unbind */
  unionfs_statfs,      /* statfs */

  unionfs_unlink,      /* unlink */
  unionfs_mkdir,       /* mkdir */
  unionfs_rmdir,       /* rmdir */
  unionfs_rename,      /* rename */
  unionfs_stat         /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unionfs_semtake
 ****************************************************************************/

static int unionfs_semtake(FAR struct unionfs_inode_s *ui, bool noint)
{
  int ret;

  do
    {
      ret = sem_wait(&ui->ui_exclsem);
      if (ret < 0)
        {
          int errcode = errno;
          DEBUGASSERT(errcode == EINTR);
          if (!noint)
            {
              return -errcode;
            }
        }
    }
  while (ret < 0);

  return OK;
}

/****************************************************************************
 * Name: unionfs_trypath
 ****************************************************************************/

static FAR const char *unionfs_trypath(FAR const char *relpath,
                                       FAR const char *prefix)
{
  FAR const char *trypath;
  int pfxlen;

  /* Is there a prefix on the the path to this file system? */

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
 * Name: unionfs_tryopen
 ****************************************************************************/

static int unionfs_tryopen(FAR struct file *filep, FAR const char *relpath,
                           FAR const char *prefix, int oflags, mode_t mode)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_trypath(relpath, prefix);
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
                              FAR const char *relpath, FAR const char *prefix,
                              FAR struct fs_dirent_s *dir)
{
  FAR const struct mountpt_operations *ops;
  FAR const char *trypath;

  /* Is this path valid on this file system? */

  trypath = unionfs_trypath(relpath, prefix);
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

  trypath = unionfs_trypath(relpath, prefix);
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
 * Name: unionfs_trystat
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

  tryoldpath = unionfs_trypath(oldrelpath, prefix);
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

  trynewpath = unionfs_trypath(newrelpath, prefix);
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

  trypath = unionfs_trypath(relpath, prefix);
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
 * Name: unionfs_trystatdir
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

  trypath = unionfs_trypath(relpath, prefix);
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

  trypath = unionfs_trypath(relpath, prefix);
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

      if (path[pathlen-1] == '/')
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
      /* REVISIT: This is bad if the file sysem cannot support a deferred
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

  (void)unionfs_unbind_child(&ui->ui_fs[0]);
  (void)unionfs_unbind_child(&ui->ui_fs[1]);

  /* Free any allocated prefix strings */

  if (ui->ui_fs[1].um_prefix)
    {
      kmm_free(ui->ui_fs[0].um_prefix);
    }

  if (ui->ui_fs[1].um_prefix)
    {
      kmm_free(ui->ui_fs[1].um_prefix);
    }

  /* And finally free the allocated unionfs state structure as well */

  sem_destroy(&ui->ui_exclsem);
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

  fvdbg("Opening: ui_nopen=%d\n", ui->ui_nopen);

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate a container to hold the open file system information */

  uf = (FAR struct unionfs_file_s *)kmm_malloc(sizeof(struct unionfs_file_s));
  if (uf == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_semaphore;
    }

  /* Try to open the file on file system 1 */

  um = &ui->ui_fs[0];
  DEBUGASSERT(um != NULL && um->um_node != NULL && um->um_node->u.i_mops != NULL);

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

      ret = unionfs_tryopen(&uf->uf_file, relpath, um->um_prefix, oflags, mode);
      if (ret < 0)
        {
          goto errout_with_semaphore;
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

errout_with_semaphore:
  unionfs_semgive(ui);
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

  (void)unionfs_semtake(ui, false);

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL && um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  fvdbg("Closing: ui_nopen=%d\n", ui->ui_nopen);

  /* Perform the lower level close operation */

  if (ops->close)
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

  /* Free the open file container */

  kmm_free(uf);
  filep->f_priv = NULL;
  unionfs_semgive(ui);
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
  int ret = -EPERM;

  fvdbg("buflen: %lu\n", (unsigned long)buflen);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL && um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level write operation */

  if (ops->read)
    {
      ret = ops->read(&uf->uf_file, buffer, buflen);
    }

  unionfs_semgive(ui);
  return ret;
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
  int ret = -EPERM;

  fvdbg("buflen: %lu\n", (unsigned long)buflen);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL && um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level write operation */

  if (ops->write)
    {
      ret = ops->write(&uf->uf_file, buffer, buflen);
    }

  unionfs_semgive(ui);
  return ret;
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
  int ret;

  fvdbg("offset: %lu whence: %d\n", (unsigned long)offset, whence);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL && um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Invoke the file seek method if available */

  if (ops->seek)
    {
      offset = ops->seek(&uf->uf_file, offset, whence);
    }
  else
    {
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
    }

  unionfs_semgive(ui);
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
  int ret = -ENOTTY;

  fvdbg("cmd: %d arg: %lu\n", cmd, arg);

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL && um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level ioctl operation */

  if (ops->ioctl)
    {
      ret = ops->ioctl(&uf->uf_file, cmd, arg);
    }

  unionfs_semgive(ui);
  return ret;
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
  int ret = -EINVAL;

  fvdbg("Entry\n");

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)filep->f_inode->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(ui != NULL && filep->f_priv != NULL);
  uf = (FAR struct unionfs_file_s *)filep->f_priv;

  DEBUGASSERT(uf->uf_ndx == 0 || uf->uf_ndx == 1);
  um = &ui->ui_fs[uf->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL && um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the lower level sync operation */

  if (ops->sync)
    {
      ret = ops->sync(&uf->uf_file);
    }

  unionfs_semgive(ui);
  return ret;
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

  fvdbg("Entry\n");

  /* Recover the open file data from the struct file instance */

  DEBUGASSERT(oldp != NULL && oldp->f_inode != NULL);
  ui = (FAR struct unionfs_inode_s *)oldp->f_inode->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(ui != NULL && oldp->f_priv != NULL);
  oldpriv = (FAR struct unionfs_file_s *)oldp->f_priv;

  DEBUGASSERT(oldpriv->uf_ndx == 0 || oldpriv->uf_ndx == 1);
  um = &ui->ui_fs[oldpriv->uf_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL && um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  DEBUGASSERT(newp != NULL && newp->f_priv == NULL);

  /* Allocate a new container for the union FS open file */

  newpriv = (FAR struct unionfs_file_s *)kmm_malloc(sizeof(struct unionfs_file_s));
  if (newpriv != NULL)
    {
      /* Clone the old file structure into the newly allocated one */

      memcpy(newpriv, oldpriv, sizeof(struct unionfs_file_s));
      newpriv->uf_file.f_priv = NULL;

      /* Then perform the lower lowel dup operation */

      ret = OK;
      if (ops->dup)
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
 * Name: unionfs_opendir
 ****************************************************************************/

static int unionfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                           FAR struct fs_dirent_s *dir)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR struct fs_unionfsdir_s *fu;
  FAR const struct mountpt_operations *ops;
  FAR struct fs_dirent_s *lowerdir;
  int ret;

  fvdbg("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  /* Recover the filesystem data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(dir);
  fu = &dir->u.unionfs;

  /* Clone the path.  We will need this when we traverse file system 2 to
   * omit duplicates on file system 1.
   */

  if (relpath && strlen(relpath) > 0)
    {
      fu->fu_relpath = strdup(relpath);
      if (!fu->fu_relpath)
        {
          goto errout_with_semaphore;
        }
    }

  /* Allocate another dirent structure for the lower file system */

  lowerdir = (FAR struct fs_dirent_s *)kmm_zalloc(sizeof(struct fs_dirent_s));
  if (lowerdir == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_relpath;
    }

  /* Check file system 2 first. */

  fu->fu_ndx      = 0;
  fu->fu_lower[0] = NULL;
  fu->fu_lower[1] = NULL;

  um = &ui->ui_fs[1];
  lowerdir->fd_root = um->um_node;
  ret = unionfs_tryopendir(um->um_node, relpath, um->um_prefix, lowerdir);
  if (ret >= 0)
    {
      /* Save the file system 2 access info */

      fu->fu_ndx = 1;
      fu->fu_lower[1] = lowerdir;

      /* Allocate yet another dirent structure for the lower file system 1 */

      lowerdir = (FAR struct fs_dirent_s *)kmm_zalloc(sizeof(struct fs_dirent_s));
      if (lowerdir == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_fs2open;
        }
    }

  /* Check file system 1 last, possibly overwriting fu_ndx */

  um = &ui->ui_fs[0];
  lowerdir->fd_root = um->um_node;
  ret = unionfs_tryopendir(um->um_node, relpath, um->um_prefix, lowerdir);
  if (ret >= 0)
    {
      /* Save the file system 1 access info */

      fu->fu_ndx = 0;
      fu->fu_lower[0] = lowerdir;
    }
  else
    {
      /* File system 1 was not opened... then we won't be needing that last
       * localdir allocation after all.
       */

      kmm_free(lowerdir);

      /* If the directory was not found on either file system, then we have
       * failed.
       */

      if (fu->fu_lower[1] == NULL)
        {
          /* Neither file system was opened! */

          goto errout_with_relpath;
        }
    }

  /* Increment the number of open references and return success */

  ui->ui_nopen++;
  DEBUGASSERT(ui->ui_nopen > 0);

  unionfs_semgive(ui);
  return OK;

errout_with_fs2open:
  ops = ui->ui_fs[1].um_node->u.i_mops;
  DEBUGASSERT(ops != NULL);
  if (ops->closedir)
    {
      ret = ops->closedir(um->um_node, fu->fu_lower[1]);
    }

  kmm_free(fu->fu_lower[1]);

errout_with_relpath:
  if (fu->fu_relpath != NULL)
    {
      kmm_free(fu->fu_relpath);
    }

errout_with_semaphore:
  unionfs_semgive(ui);
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
  FAR struct fs_unionfsdir_s *fu;
  int ret = OK;
  int i;

  fvdbg("Entry\n");

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* Get exclusive access to the file system data structures */

  (void)unionfs_semtake(ui, true);

  DEBUGASSERT(dir);
  fu = &dir->u.unionfs;

  /* Close both contained file systems */

  for (i = 0; i < 2; i++)
    {
      /* Was this file system opened? */

      if (fu->fu_lower[i] != NULL);
        {
          um = &ui->ui_fs[i];

          DEBUGASSERT(um != NULL && um->um_node != NULL && um->um_node->u.i_mops != NULL);
          ops = um->um_node->u.i_mops;

          /* Perform the lower level closedir operation */

          if (ops->closedir)
            {
              ret = ops->closedir(um->um_node, fu->fu_lower[i]);
            }

          /* Free the lower dirent structure */

          kmm_free(fu->fu_lower[i]);
        }
    }

  /* Free any allocated path */

  if (fu->fu_relpath != NULL)
    {
      kmm_free(fu->fu_relpath);
    }

  fu->fu_ndx      = 0;
  fu->fu_relpath  = NULL;
  fu->fu_lower[0] = NULL;
  fu->fu_lower[1] = NULL;

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
      unionfs_semgive(ui);
    }

  return ret;
}

/****************************************************************************
 * Name: unionfs_readdir
 ****************************************************************************/

static int unionfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  FAR struct unionfs_mountpt_s *um0;
  FAR const struct mountpt_operations *ops;
  FAR struct fs_unionfsdir_s *fu;
  FAR char *relpath;
  struct stat buf;
  bool duplicate;
  int ret = -ENOSYS;

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  DEBUGASSERT(dir);
  fu = &dir->u.unionfs;

  DEBUGASSERT(fu->fu_ndx == 0 || fu->fu_ndx == 1);
  um = &ui->ui_fs[fu->fu_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL && um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  fvdbg("fu_ndx: %d\n", fu->fu_ndx);

  /* Perform the lower level readdir operation */

  if (ops->readdir)
    {
      /* Loop if we discard duplicate directory entries in filey system 2 */

      do
        {
          /* Read the directory entry */

          ret = ops->readdir(um->um_node, fu->fu_lower[fu->fu_ndx]);

          /* Did the read operation fail because we reached the end of the
           * directory?  In that case, the error would be -ENOENT.  If we
           * hit the end-of-directory on file system, we need to seamlessly
           * move to the second file system (if there is one).
           */

          if (ret == -ENOENT && fu->fu_ndx == 0 && fu->fu_lower[1] != NULL)
            {
              /* Switch to the second file system */

              fu->fu_ndx = 1;
              um = &ui->ui_fs[1];

              DEBUGASSERT(um != NULL && um->um_node != NULL &&
                          um->um_node->u.i_mops != NULL);
              ops = um->um_node->u.i_mops;

              /* Make sure that the second file system directory enumeration
               * is rewound to the beginning of the directory.
               */

              if (ops->rewinddir != NULL)
                {
                  ret = ops->rewinddir(um->um_node, fu->fu_lower[1]);
                }

              /* Then try the read operation again */

              ret = ops->readdir(um->um_node, fu->fu_lower[1]);
            }

          /* Did we successfully read a directory from file system 2?  If
           * so, we need to omit an duplicates that should be occluded by
           * the matching file on file system 1 (if we are enumerating
           * file system 1).
           */

          duplicate = false;
          if (ret >= 0 && fu->fu_ndx == 1 && fu->fu_lower[0] != NULL)
            {
              /* Get the relative path to the same file on file system 1.
               * NOTE: the on any failures we just assume that the filep
               * is not a duplicate.
               */

              relpath = unionfs_relpath(fu->fu_relpath,
                                        fu->fu_lower[1]->fd_dir.d_name);
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
                       * have duplicat names.
                       */

                      duplicate = true;
                    }

                  /* Free the allocated relpath */

                  kmm_free(relpath);
                }
            }
        }
      while (duplicate);

      /* Copy the return information into the dirent structure that the
       * application will see.
       */

      dir->fd_position = fu->fu_lower[fu->fu_ndx]->fd_position;
      memcpy(&dir->fd_dir, &fu->fu_lower[fu->fu_ndx]->fd_dir, sizeof(struct dirent));
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
  FAR struct fs_unionfsdir_s *fu;
  int ret;

  fvdbg("Entry\n");

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(dir);
  fu = &dir->u.unionfs;

  /* Were we currently enumerating on file system 1?  If not, is an
   * enumeration possible on file system 1?
   */

  DEBUGASSERT(fu->fu_ndx == 0 || fu->fu_ndx == 1);
  if (/* fu->fu_ndx != 0 && */ fu->fu_lower[0] != 0)
    {
      /* Yes.. switch to file system 1 */

      fu->fu_ndx = 0;
    }

  um = &ui->ui_fs[fu->fu_ndx];

  DEBUGASSERT(um != NULL && um->um_node != NULL && um->um_node->u.i_mops != NULL);
  ops = um->um_node->u.i_mops;

  /* Perform the file system rewind operation */

  if (ops->rewinddir != NULL)
    {
      ret = ops->rewinddir(um->um_node, fu->fu_lower[fu->fu_ndx]);
      dir->fd_position = fu->fu_lower[fu->fu_ndx]->fd_position;
    }
  else
    {
      ret = -EINVAL;
    }

  unionfs_semgive(ui);
  return ret;
}

/****************************************************************************
 * Name: unionfs_unbind
 ****************************************************************************/

static int unionfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                          unsigned int flags)
{
  FAR struct unionfs_inode_s *ui;

  fvdbg("Entry\n");

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(handle != NULL);
  ui = (FAR struct unionfs_inode_s *)handle;

  /* Get exclusive access to the file system data structures */

  (void)unionfs_semtake(ui, true);

  /* Mark the file system as unmounted. */

  ui->ui_unmounted = true;

  /* If there are no open references, then we can destroy the file system
   * now.
   */

  if (ui->ui_nopen <= 0)
    {
      unionfs_destroy(ui);
    }

  unionfs_semgive(ui);
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

  fvdbg("Entry\n");

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && buf != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  memset(buf, 0, sizeof(struct statfs));

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  /* Get statfs info from file system 1.
   *
   * REVISIT: What would it mean if one file system did not support statfs?
   * Perhaps we could simplify the following by simply insisting that both
   * file systems support the statfs method.
   */

  um1 = &ui->ui_fs[0];
  DEBUGASSERT(um1 != NULL && um1->um_node != NULL && um1->um_node->u.i_mops != NULL);
  ops1 = um1->um_node->u.i_mops;

  um2 = &ui->ui_fs[1];
  DEBUGASSERT(um2 != NULL && um2->um_node != NULL && um2->um_node->u.i_mops != NULL);
  ops2 = um2->um_node->u.i_mops;

  if (ops1->statfs != NULL && ops2->statfs != NULL)
    {
      ret = ops1->statfs(um1->um_node, &buf1);
      if (ret < 0)
        {
          goto errout_with_semaphore;
        }

      /* Get stafs info from file system 2 */

      ret = ops2->statfs(um2->um_node, &buf2);
      if (ret < 0)
        {
          goto errout_with_semaphore;
        }
    }
  else if (ops1->statfs != NULL)
    {
      /* We have statfs for file system 1 only */

      ret = ops1->statfs(um1->um_node, buf);
      goto errout_with_semaphore;
    }
  else if (ops2->statfs != NULL)
    {
      /* We have statfs for file system 2 only */

      ret = ops2->statfs(um2->um_node, buf);
      goto errout_with_semaphore;
    }
  else
    {
      /* We could not get stafs info from either file system */

      ret = -ENOSYS;
      goto errout_with_semaphore;
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
          tmp           = (((uint64_t)buf2.f_blocks * (uint64_t)buf2.f_bsize) << 16);
          ratiob16      = (uint32_t)(tmp / buf1.f_bsize);
          adj           = &buf2;
        }
      else
        {
          buf->f_bsize  = buf2.f_bsize;
          tmp           = (((uint64_t)buf1.f_blocks * (uint64_t)buf1.f_bsize) << 16);
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

  ret = OK;

errout_with_semaphore:
  unionfs_semgive(ui);
  return ret;
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

  fdbg("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && relpath != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if some exists at this path on file system 1.  This might be
   * a file or a directory*/

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
      /* Check if the file exists with with name on file system 2.  The only
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

  unionfs_semgive(ui);
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

  fdbg("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && relpath != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  /* Is there anything with this name on either file system? */

  um  = &ui->ui_fs[0];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, &buf);
  if (ret >= 0)
    {
      ret = -EEXIST;
      goto errout_with_semaphore;
    }

  um  = &ui->ui_fs[1];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, &buf);
  if (ret >= 0)
    {
      ret = -EEXIST;
      goto errout_with_semaphore;
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

  if (ret1 >= 0 || ret2 >= 0)
    {
      ret = OK;
    }
  else
    {
      /* Otherwise, pick one */

      ret = ret1;
    }

errout_with_semaphore:
  unionfs_semgive(ui);
  return ret;
}

/****************************************************************************
 * Name: unionfs_rmdir
 ****************************************************************************/

static int unionfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct unionfs_mountpt_s *um;
  int tmp;
  int ret;

  fdbg("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && relpath != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  ret = -ENOENT;

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
          unionfs_semgive(ui);
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

  unionfs_semgive(ui);
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
  int tmp;
  int ret = -ENOENT;

  fdbg("oldrelpath: %s newrelpath\n", oldrelpath, newrelpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

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

          unionfs_semgive(ui);
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

  unionfs_semgive(ui);
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

  fdbg("relpath: %s\n", relpath);

  /* Recover the union file system data from the struct inode instance */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL && relpath != NULL);
  ui = (FAR struct unionfs_inode_s *)mountpt->i_private;

  /* Get exclusive access to the file system data structures */

  ret = unionfs_semtake(ui, false);
  if (ret < 0)
    {
      return ret;
    }

  /* stat this path on file system 1 */

  um  = &ui->ui_fs[0];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, buf);
  if (ret >= 0)
    {
      /* Return on the first success.  The first instance of the file will
       * shadow the second anyway.
       */

      unionfs_semgive(ui);
      return OK;
    }

  /* stat failed on the file system 1.  Try again on file system 2. */

  um  = &ui->ui_fs[1];
  ret = unionfs_trystat(um->um_node, relpath, um->um_prefix, buf);

  unionfs_semgive(ui);
  return ret;
}

/****************************************************************************
 * Name: unionfs_getmount
 ****************************************************************************/

static int unionfs_getmount(FAR const char *path, FAR struct inode **inode)
{
 FAR struct inode *minode;

  /* Find the mountpt */

  minode = inode_find(path, NULL);
  if (!minode)
    {
      /* Mountpoint inode not found */

      return -ENOENT;
    }

  /* Verify that the inode is a mountpoint */

  if (!INODE_IS_MOUNTPT(minode))
    {
      /* Inode was found, but is it is a mounpoint */

      inode_release(minode);
      return -EINVAL;
    }

  /* Success! */

  *inode = minode;
  return OK;
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
 * Returned value:
 *   Zero (OK) is returned if the union file system was correctly created and
 *   mounted.  On any failure, a negated error value will be returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

int unionfs_mount(FAR const char *fspath1, FAR const char *prefix1,
                  FAR const char *fspath2, FAR const char *prefix2,
                  FAR const char *mountpt)
{
  FAR struct unionfs_inode_s *ui;
  FAR struct inode *mpinode;
  int ret;

  DEBUGASSERT(fspath1 != NULL && fspath2 != NULL && mountpt != NULL);

  /* Allocate a structure a structure that will describe the union file
   * system.
   */

  ui = (FAR struct unionfs_inode_s *)kmm_zalloc(sizeof(struct unionfs_inode_s));
  if (!ui)
    {
      fdbg("ERROR: Failed to allocated union FS state structure\n");
      return -ENOMEM;
    }

  sem_init(&ui->ui_exclsem, 0, 1);

  /* Get the inodes associated with fspath1 and fspath2 */

  ret = unionfs_getmount(fspath1, &ui->ui_fs[0].um_node);
  if (ret < 0)
    {
      fdbg("ERROR: unionfs_getmount(fspath1) failed: %d\n", ret);
      goto errout_with_uinode;
    }

  ret = unionfs_getmount(fspath2, &ui->ui_fs[1].um_node);
  if (ret < 0)
    {
      fdbg("ERROR: unionfs_getmount(fspath2) failed: %d\n", ret);
      goto errout_with_fs1;
    }

  /* Duplicate the prefix strings */

  if (prefix1 && strlen(prefix1) > 0)
    {
      ui->ui_fs[0].um_prefix = strdup(prefix1);
      if (ui->ui_fs[0].um_prefix == NULL)
        {
          fdbg("ERROR: strdup(prefix1) failed\n");
          ret = -ENOMEM;
          goto errout_with_fs2;
        }
    }

  if (prefix2 && strlen(prefix2) > 0)
    {
      ui->ui_fs[1].um_prefix = strdup(prefix2);
      if (ui->ui_fs[1].um_prefix == NULL)
        {
          fdbg("ERROR: strdup(prefix2) failed\n");
          ret = -ENOMEM;
          goto errout_with_prefix1;
        }
    }

  /* Finally, mount the union FS.  We should adapt the standard mount to do
   * this using optional parameters.  This custom mount should do the job
   * for now, however.
   */

  /* Insert a dummy node -- we need to hold the inode semaphore
   * to do this because we will have a momentarily bad structure.
   */

  inode_semtake();
  ret = inode_reserve(mountpt, &mpinode);
  if (ret < 0)
    {
      /* inode_reserve can fail for a couple of reasons, but the most likely
       * one is that the inode already exists. inode_reserve may return:
       *
       *  -EINVAL - 'path' is invalid for this operation
       *  -EEXIST - An inode already exists at 'path'
       *  -ENOMEM - Failed to allocate in-memory resources for the operation
       */

      fdbg("ERROR: Failed to reserve inode\n");
      goto errout_with_semaphore;
    }

  /* Populate the inode with driver specific information. */

  INODE_SET_MOUNTPT(mpinode);

  mpinode->u.i_mops  = &g_unionfs_mops;
#ifdef CONFIG_FILE_MODE
  mpinode->i_mode    = 0755;
#endif
  mpinode->i_private = ui;

  /* Unlink the contained mountpoint inodes from the pseudo file system.
   * The inodes will be marked as deleted so that they will be removed when
   * the reference count decrements to zero in inode_release().  Because we
   * hold a reference count on the inodes, they will not be deleted until
   * this logic calls inode_release() in the unionfs_destroy() function.
   */

  (void)inode_remove(fspath1);
  (void)inode_remove(fspath2);
  inode_semgive();
  return OK;

errout_with_semaphore:
  inode_semgive();

//errout_with_prefix2:
  if (ui->ui_fs[1].um_prefix != NULL)
    {
      kmm_free(ui->ui_fs[1].um_prefix);
    }

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
  sem_destroy(&ui->ui_exclsem);
  kmm_free(ui);
  return ret;
}
#endif /* !CONFIG_DISABLE_MOUNTPOINT && CONFIG_FS_UNIONFS */
