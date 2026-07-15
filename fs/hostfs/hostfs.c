/****************************************************************************
 * fs/hostfs/hostfs.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <sys/statfs.h>

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/lib/lib.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/fat.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/hostfs.h>

#include "inode/inode.h"
#include "hostfs.h"
#include "fs_heap.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hostfs_dir_s
{
  struct fs_dirent_s base;
  FAR void *dir;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     hostfs_open(FAR struct file *filep, FAR const char *relpath,
                           int oflags, mode_t mode);
static int     hostfs_close(FAR struct file *filep);
static ssize_t hostfs_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t hostfs_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static off_t   hostfs_seek(FAR struct file *filep, off_t offset,
                           int whence);
static int     hostfs_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

static int     hostfs_sync(FAR struct file *filep);
static int     hostfs_dup(FAR const struct file *oldp,
                          FAR struct file *newp);
static int     hostfs_fstat(FAR const struct file *filep,
                            FAR struct stat *buf);
static int     hostfs_fchstat(FAR const struct file *filep,
                              FAR const struct stat *buf, int flags);
static int     hostfs_ftruncate(FAR struct file *filep,
                                off_t length);

static int     hostfs_opendir(FAR struct inode *mountpt,
                              FAR const char *relpath,
                          FAR struct fs_dirent_s **dir);
static int     hostfs_closedir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir);
static int     hostfs_readdir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir,
                          FAR struct dirent *entry);
static int     hostfs_rewinddir(FAR struct inode *mountpt,
                                FAR struct fs_dirent_s *dir);

static int     hostfs_bind(FAR struct inode *blkdriver,
                           FAR const void *data, FAR void **handle);
static int     hostfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                             unsigned int flags);
static int     hostfs_statfs(FAR struct inode *mountpt,
                             FAR struct statfs *buf);

static int     hostfs_unlink(FAR struct inode *mountpt,
                             FAR const char *relpath);
static int     hostfs_mkdir(FAR struct inode *mountpt,
                            FAR const char *relpath, mode_t mode);
static int     hostfs_rmdir(FAR struct inode *mountpt,
                            FAR const char *relpath);
static int     hostfs_rename(FAR struct inode *mountpt,
                             FAR const char *oldrelpath,
                             FAR const char *newrelpath);
static int     hostfs_stat(FAR struct inode *mountpt,
                           FAR const char *relpath, FAR struct stat *buf);
static int     hostfs_chstat(FAR struct inode *mountpt,
                             FAR const char *relpath,
                             FAR const struct stat *buf, int flags);

#ifdef CONFIG_FS_LINKS
static int     hostfs_link(FAR struct inode *mountpt,
                           FAR const char *relpath1,
                           FAR const char *relpath2);
static int     hostfs_symlink(FAR struct inode *mountpt,
                              FAR const char *path1,
                              FAR const char *relpath2);
static ssize_t hostfs_readlink(FAR struct inode *mountpt,
                               FAR const char *relpath,
                               FAR char *buf, size_t bufsize);
static int     hostfs_lstat(FAR struct inode *mountpt,
                            FAR const char *relpath,
                            FAR struct stat *buf);
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations g_hostfs_operations =
{
  hostfs_open,          /* open */
  hostfs_close,         /* close */
  hostfs_read,          /* read */
  hostfs_write,         /* write */
  hostfs_seek,          /* seek */
  hostfs_ioctl,         /* ioctl */
  NULL,                 /* mmap */
  hostfs_ftruncate,     /* ftruncate */
  NULL,                 /* poll */
  NULL,                 /* readv */
  NULL,                 /* writev */

  hostfs_sync,          /* sync */
  hostfs_dup,           /* dup */
  hostfs_fstat,         /* fstat */
  hostfs_fchstat,       /* fchstat */

  hostfs_opendir,       /* opendir */
  hostfs_closedir,      /* closedir */
  hostfs_readdir,       /* readdir */
  hostfs_rewinddir,     /* rewinddir */

  hostfs_bind,          /* bind */
  hostfs_unbind,        /* unbind */
  hostfs_statfs,        /* statfs */

  hostfs_unlink,        /* unlink */
  hostfs_mkdir,         /* mkdir */
  hostfs_rmdir,         /* rmdir */
  hostfs_rename,        /* rename */
  hostfs_stat,          /* stat */
  hostfs_chstat,        /* chstat */
  NULL,                 /* syncfs */
  NULL,                 /* ioctldir */
  NULL,                 /* permission */
#ifdef CONFIG_FS_LINKS
  hostfs_link,          /* link */
  hostfs_symlink,       /* symlink */
  hostfs_readlink,      /* readlink */
  hostfs_lstat,         /* lstat */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hostfs_mkpath
 *
 * Description: Build absolute host path from relative NuttX path.
 *
 ****************************************************************************/

static void hostfs_mkpath(FAR struct hostfs_mountpt_s  *fs,
                          FAR const char *relpath,
                          FAR char *path, int pathlen)
{
  /* Copy base host path to output and append relative path directly.
   * Note: Both ".." segments and leading slashes are already resolved
   * by the VFS layer (_inode_canonicalize + inode_nextname) before
   * relpath reaches here.
   */

  strlcpy(path, fs->fs_root, pathlen);
  strlcat(path, relpath, pathlen);
}

/****************************************************************************
 * Name: hostfs_open
 ****************************************************************************/

static int hostfs_open(FAR struct file *filep, FAR const char *relpath,
                       int oflags, mode_t mode)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s  *hf;
  FAR char *path;
  size_t len;
  int ret;

  /* Sanity checks */

  DEBUGASSERT((filep->f_priv == NULL) && (filep->f_inode != NULL));

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  path = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (path == NULL)
    {
      return -ENOMEM;
    }

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_path;
    }

  /* Allocate memory for the open file */

  len = strlen(relpath);
  hf = fs_heap_malloc(sizeof(*hf) + len);
  if (hf == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, HOSTFS_MAX_PATH);

  /* Try to open the file in the host file system */

  hf->fd = host_open(path, oflags, mode);
  if (hf->fd < 0)
    {
      /* Error opening file */

      ret = hf->fd;
      goto errout_with_buffer;
    }

  /* In write/append mode, we need to set the file pointer to the end of the
   * file.
   */

  if ((oflags & O_APPEND) && (oflags & O_ACCMODE) != O_RDONLY)
    {
      ret = host_lseek(hf->fd, 0, 0, SEEK_END);
      if (ret >= 0)
        {
          filep->f_pos = ret;
        }
      else
        {
          goto errout_with_buffer;
        }
    }

  /* Attach the private date to the struct file instance */

  filep->f_priv = hf;

  /* Then insert the new instance into the mountpoint structure.
   * It needs to be there (1) to handle error conditions that effect
   * all files, and (2) to inform the umount logic that we are busy
   * (but a simple reference count could have done that).
   */

  hf->fnext = fs->fs_head;
  hf->crefs = 1;
  hf->oflags = oflags;
  memcpy(hf->relpath, relpath, len + 1);
  fs->fs_head = hf;

  ret = OK;
  goto errout_with_lock;

errout_with_buffer:
  fs_heap_free(hf);

errout_with_lock:
  nxmutex_unlock(&fs->fs_lock);

errout_with_path:
  lib_put_tempbuffer(path);

  if (ret == -EINVAL)
    {
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: hostfs_close
 ****************************************************************************/

static int hostfs_close(FAR struct file *filep)
{
  FAR struct inode            *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s   *hf;
  FAR struct hostfs_ofile_s   *nextfile;
  FAR struct hostfs_ofile_s   *prevfile;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL);

  /* Recover our private data from the struct file instance */

  inode = filep->f_inode;
  fs    = inode->i_private;
  hf    = filep->f_priv;

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if we are the last one with a reference to the file and
   * only close if we are.
   */

  if (hf->crefs > 1)
    {
      /* The file is opened more than once.  Just decrement the
       * reference count and return.
       */

      hf->crefs--;
      goto okout;
    }

  /* Remove ourselves from the linked list */

  nextfile = fs->fs_head;
  prevfile = nextfile;
  while ((nextfile != hf) && (nextfile != NULL))
    {
      /* Save the previous file pointer too */

      prevfile = nextfile;
      nextfile = nextfile->fnext;
    }

  if (nextfile != NULL)
    {
      /* Test if we were the first entry */

      if (nextfile == fs->fs_head)
        {
          /* Assign a new head */

          fs->fs_head = nextfile->fnext;
        }
      else
        {
          /* Take ourselves out of the list */

          prevfile->fnext = nextfile->fnext;
        }
    }

  /* Close the host file */

  host_close(hf->fd);

  /* Now free the pointer */

  filep->f_priv = NULL;
  fs_heap_free(hf);

okout:
  nxmutex_unlock(&fs->fs_lock);
  return OK;
}

/****************************************************************************
 * Name: hostfs_read
 ****************************************************************************/

static ssize_t hostfs_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  ssize_t ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the host to perform the read */

  ret = host_read(hf->fd, buffer, buflen);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: hostfs_write
 ****************************************************************************/

static ssize_t hostfs_write(FAR struct file *filep, const char *buffer,
                         size_t buflen)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  ssize_t ret;

  DEBUGASSERT(filep->f_priv != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Test the permissions.  Only allow write if the file was opened with
   * write flags.
   */

  if ((hf->oflags & O_ACCMODE) == O_RDONLY)
    {
      ret = -EACCES;
      goto errout_with_lock;
    }

  /* Call the host to perform the write */

  ret = host_write(hf->fd, buffer, buflen);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

errout_with_lock:
  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: hostfs_seek
 ****************************************************************************/

static off_t hostfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  off_t ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call our internal routine to perform the seek */

  ret = host_lseek(hf->fd, filep->f_pos, offset, whence);
  if (ret >= 0)
    {
      filep->f_pos = ret;
    }

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: hostfs_ioctl
 ****************************************************************************/

static int hostfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call our internal routine to perform the ioctl */

  ret = host_ioctl(hf->fd, cmd, arg);
  if (ret < 0)
    {
      switch (cmd)
        {
          case FIOC_FILEPATH:
            {
              FAR char *path = (FAR char *)(uintptr_t)arg;

              ret = inode_getpath(filep->f_inode, path, PATH_MAX);
              if (ret >= 0)
                {
                  strlcat(path, hf->relpath, PATH_MAX);
                }
            }

            break;
          default:
            ret = -ENOTTY;
        }
    }

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: hostfs_sync
 *
 * Description: Synchronize the file state on disk to match internal, in-
 *   memory state.
 *
 ****************************************************************************/

static int hostfs_sync(FAR struct file *filep)
{
  FAR struct inode            *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s   *hf;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  host_sync(hf->fd);

  nxmutex_unlock(&fs->fs_lock);
  return OK;
}

/****************************************************************************
 * Name: hostfs_dup
 *
 * Description: Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int hostfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct hostfs_ofile_s *sf;

  /* Sanity checks */

  DEBUGASSERT(oldp->f_priv != NULL &&
              newp->f_priv == NULL &&
              newp->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  sf = oldp->f_priv;

  DEBUGASSERT(sf != NULL);

  /* Just increment the reference count on the ofile */

  sf->crefs++;
  newp->f_priv = (FAR void *)sf;

  return OK;
}

/****************************************************************************
 * Name: hostfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int hostfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  int ret = OK;

  /* Sanity checks */

  DEBUGASSERT(buf != NULL);

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL);
  hf    = filep->f_priv;
  inode = filep->f_inode;

  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the host to perform the read */

  ret = host_fstat(hf->fd, buf);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: hostfs_fchstat
 *
 * Description:
 *   Change information about an open file associated with the file
 *   descriptor 'fd'.
 *
 ****************************************************************************/

static int hostfs_fchstat(FAR const struct file *filep,
                          FAR const struct stat *buf, int flags)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  int ret = OK;

  /* Sanity checks */

  DEBUGASSERT(buf != NULL);

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL);
  hf    = filep->f_priv;
  inode = filep->f_inode;

  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the host to perform the change */

  ret = host_fchstat(hf->fd, buf, flags);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: hostfs_ftruncate
 *
 * Description:
 *   Set the length of the open, regular file associated with the file
 *   structure 'filep' to 'length'.
 *
 ****************************************************************************/

static int hostfs_ftruncate(FAR struct file *filep, off_t length)
{
  FAR struct inode *inode;
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_ofile_s *hf;
  int ret = OK;

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL);
  hf    = filep->f_priv;
  inode = filep->f_inode;

  fs    = inode->i_private;
  DEBUGASSERT(fs != NULL);

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the host to perform the truncate */

  ret = host_ftruncate(hf->fd, length);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: hostfs_opendir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int hostfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR struct fs_dirent_s **dir)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_dir_s *hdir;
  FAR char *path;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  path = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (path == NULL)
    {
      return -ENOMEM;
    }

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;
  hdir = fs_heap_zalloc(sizeof(struct hostfs_dir_s));
  if (hdir == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_path;
    }

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_hdir;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, HOSTFS_MAX_PATH);

  /* Call the host's opendir function */

  hdir->dir = host_opendir(path);
  if (hdir->dir == NULL)
    {
      ret = -ENOENT;
      goto errout_with_lock;
    }

  *dir = (FAR struct fs_dirent_s *)hdir;
  nxmutex_unlock(&fs->fs_lock);
  lib_put_tempbuffer(path);
  return OK;

errout_with_lock:
  nxmutex_unlock(&fs->fs_lock);

errout_with_hdir:
  fs_heap_free(hdir);

errout_with_path:
  lib_put_tempbuffer(path);
  return ret;
}

/****************************************************************************
 * Name: hostfs_closedir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int hostfs_closedir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_dir_s *hdir;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs   = mountpt->i_private;
  hdir = (FAR struct hostfs_dir_s *)dir;

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the host's closedir function */

  host_closedir(hdir->dir);

  nxmutex_unlock(&fs->fs_lock);
  fs_heap_free(hdir);
  return OK;
}

/****************************************************************************
 * Name: hostfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int hostfs_readdir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir,
                          FAR struct dirent *entry)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_dir_s *hdir;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs   = mountpt->i_private;
  hdir = (FAR struct hostfs_dir_s *)dir;

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the host OS's readdir function */

  ret = host_readdir(hdir->dir, entry);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: hostfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int hostfs_rewinddir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR struct hostfs_dir_s *hdir;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs   = mountpt->i_private;
  hdir = (FAR struct hostfs_dir_s *)dir;

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the host and let it do all the work */

  host_rewinddir(hdir->dir);

  nxmutex_unlock(&fs->fs_lock);
  return OK;
}

/****************************************************************************
 * Name: hostfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int hostfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                       FAR void **handle)
{
  size_t data_size = strlen(data) + 1;
  FAR struct hostfs_mountpt_s *fs;
  FAR char *options;
  FAR char *saveptr;
  FAR char *ptr;
  int len;
  int ret;

  /* Validate the block driver is NULL */

  if (blkdriver || !data)
    {
      return -ENODEV;
    }

  /* Create an instance of the mountpt state structure */

  fs = (FAR struct hostfs_mountpt_s *)
    fs_heap_zalloc(sizeof(struct hostfs_mountpt_s));

  if (fs == NULL)
    {
      return -ENOMEM;
    }

  /* Hostfs by instance lock initialize */

  nxmutex_init(&fs->fs_lock);

  /* The options we support are:
   *  "fs=whatever", remote dir
   */

  options = lib_get_tempbuffer(data_size);
  if (!options)
    {
      fs_heap_free(fs);
      return -ENOMEM;
    }

  memcpy(options, data, data_size);
  ptr = strtok_r(options, ",", &saveptr);
  while (ptr != NULL)
    {
      if ((strncmp(ptr, "fs=", 3) == 0))
        {
          strlcpy(fs->fs_root, &ptr[3], sizeof(fs->fs_root));
        }

      ptr = strtok_r(NULL, ",", &saveptr);
    }

  lib_put_tempbuffer(options);

  /* Take the lock for the mount */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      fs_heap_free(fs);
      return ret;
    }

  /* Initialize the allocated mountpt state structure.  The filesystem is
   * responsible for one reference ont the blkdriver inode and does not
   * have to addref() here (but does have to release in ubind().
   */

  fs->fs_head = NULL;

  /* Now perform the mount.  */

  len = strlen(fs->fs_root);
  if (len > 1 && fs->fs_root[len - 1] == '/')
    {
      /* Remove trailing '/' */

      fs->fs_root[len - 1] = '\0';
    }

  /* Append a '/' to the name now */

  if (fs->fs_root[len - 1] != '/')
    {
      strlcat(fs->fs_root, "/", sizeof(fs->fs_root));
    }

  *handle = (FAR void *)fs;
  nxmutex_unlock(&fs->fs_lock);
  return OK;
}

/****************************************************************************
 * Name: hostfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int hostfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                         unsigned int flags)
{
  FAR struct hostfs_mountpt_s *fs = (FAR struct hostfs_mountpt_s *)handle;
  int ret;

  if (!fs)
    {
      return -EINVAL;
    }

  /* Check if there are sill any files opened on the filesystem. */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (fs->fs_head != NULL)
    {
      /* We cannot unmount now.. there are open files */

      nxmutex_unlock(&fs->fs_lock);

      /* This implementation currently only supports unmounting if there are
       * no open file references.
       */

      return (flags != 0) ? -ENOSYS : -EBUSY;
    }

  nxmutex_unlock(&fs->fs_lock);
  nxmutex_destroy(&fs->fs_lock);
  fs_heap_free(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int hostfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct hostfs_mountpt_s *fs;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the host fs to perform the statfs */

  ret = host_statfs(fs->fs_root, buf);
  buf->f_type = HOSTFS_MAGIC;

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: hostfs_unlink
 *
 * Description: Remove a file
 *
 ****************************************************************************/

static int hostfs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR char *path;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  path = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (path == NULL)
    {
      return -ENOMEM;
    }

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_path;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, HOSTFS_MAX_PATH);

  /* Call the host fs to perform the unlink */

  ret = host_unlink(path);

  nxmutex_unlock(&fs->fs_lock);

errout_with_path:
  lib_put_tempbuffer(path);
  return ret;
}

/****************************************************************************
 * Name: hostfs_mkdir
 *
 * Description: Create a directory
 *
 ****************************************************************************/

static int hostfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                        mode_t mode)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR char *path;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  path = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (path == NULL)
    {
      return -ENOMEM;
    }

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_path;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, HOSTFS_MAX_PATH);

  /* Call the host FS to do the mkdir */

  ret = host_mkdir(path, mode);

  nxmutex_unlock(&fs->fs_lock);

errout_with_path:
  lib_put_tempbuffer(path);
  return ret;
}

/****************************************************************************
 * Name: hostfs_rmdir
 *
 * Description: Remove a directory
 *
 ****************************************************************************/

int hostfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR char *path;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  path = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (path == NULL)
    {
      return -ENOMEM;
    }

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_path;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, HOSTFS_MAX_PATH);

  /* Call the host FS to do the mkdir */

  ret = host_rmdir(path);

  nxmutex_unlock(&fs->fs_lock);

errout_with_path:
  lib_put_tempbuffer(path);
  return ret;
}

/****************************************************************************
 * Name: hostfs_rename
 *
 * Description: Rename a file or directory
 *
 ****************************************************************************/

int hostfs_rename(FAR struct inode *mountpt, FAR const char *oldrelpath,
                  FAR const char *newrelpath)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR char *oldpath;
  FAR char *newpath;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  oldpath = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (oldpath == NULL)
    {
      return -ENOMEM;
    }

  newpath = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (newpath == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_oldpath;
    }

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_newpath;
    }

  /* Append to the host's root directory */

  strlcpy(oldpath, fs->fs_root, sizeof(oldpath));
  strlcat(oldpath, oldrelpath, sizeof(oldpath));
  strlcpy(newpath, fs->fs_root, sizeof(newpath));
  strlcat(newpath, newrelpath, sizeof(newpath));

  /* Call the host FS to do the mkdir */

  ret = host_rename(oldpath, newpath);

  nxmutex_unlock(&fs->fs_lock);

errout_with_newpath:
  lib_put_tempbuffer(newpath);

errout_with_oldpath:
  lib_put_tempbuffer(oldpath);
  return ret;
}

/****************************************************************************
 * Name: hostfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int hostfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                       FAR struct stat *buf)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR char *path;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  path = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (path == NULL)
    {
      return -ENOMEM;
    }

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_path;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, HOSTFS_MAX_PATH);

  /* Call the host FS to do the stat operation */

  ret = host_stat(path, buf);

  nxmutex_unlock(&fs->fs_lock);

errout_with_path:
  lib_put_tempbuffer(path);
  return ret;
}

/****************************************************************************
 * Name: hostfs_chstat
 *
 * Description: Change information about a file or directory
 *
 ****************************************************************************/

static int hostfs_chstat(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR const struct stat *buf, int flags)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR char *path;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  path = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (path == NULL)
    {
      return -ENOMEM;
    }

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_path;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, HOSTFS_MAX_PATH);

  /* Call the host FS to do the chstat operation */

  ret = host_chstat(path, buf, flags);

  nxmutex_unlock(&fs->fs_lock);

errout_with_path:
  lib_put_tempbuffer(path);
  return ret;
}

#ifdef CONFIG_FS_LINKS
/****************************************************************************
 * Name: hostfs_link
 *
 * Description: Create a hard link
 *
 ****************************************************************************/

static int hostfs_link(FAR struct inode *mountpt, FAR const char *relpath1,
                       FAR const char *relpath2)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR char *path1;
  FAR char *path2;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  path1 = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (path1 == NULL)
    {
      return -ENOMEM;
    }

  path2 = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (path2 == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_path1;
    }

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_path2;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath1, path1, HOSTFS_MAX_PATH);
  hostfs_mkpath(fs, relpath2, path2, HOSTFS_MAX_PATH);

  /* Call the host FS to create the hard link */

  ret = host_link(path1, path2);

  nxmutex_unlock(&fs->fs_lock);

errout_with_path2:
  lib_put_tempbuffer(path2);

errout_with_path1:
  lib_put_tempbuffer(path1);
  return ret;
}

/****************************************************************************
 * Name: hostfs_symlink
 *
 * Description: Create a symbolic link
 *
 ****************************************************************************/

static int hostfs_symlink(FAR struct inode *mountpt,
                          FAR const char *path1,
                          FAR const char *relpath2)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR char *fullpath2;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  fullpath2 = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (fullpath2 == NULL)
    {
      return -ENOMEM;
    }

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_path;
    }

  /* Build the full path for the link location (relpath2)
   * The target (path1) is used as-is, allowing relative or absolute paths
   */

  hostfs_mkpath(fs, relpath2, fullpath2, HOSTFS_MAX_PATH);

  /* Call the host FS to create the symbolic link */

  ret = host_symlink(path1, fullpath2);

  nxmutex_unlock(&fs->fs_lock);

errout_with_path:
  lib_put_tempbuffer(fullpath2);
  return ret;
}

/****************************************************************************
 * Name: hostfs_readlink
 *
 * Description: Read the target of a symbolic link
 *
 ****************************************************************************/

static ssize_t hostfs_readlink(FAR struct inode *mountpt,
                               FAR const char *relpath,
                               FAR char *buf, size_t bufsize)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR char *path;
  ssize_t ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  path = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (path == NULL)
    {
      return -ENOMEM;
    }

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_path;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, HOSTFS_MAX_PATH);

  /* Call the host FS to read the symbolic link */

  ret = host_readlink(path, buf, bufsize);

  nxmutex_unlock(&fs->fs_lock);

errout_with_path:
  lib_put_tempbuffer(path);
  return ret;
}

/****************************************************************************
 * Name: hostfs_lstat
 *
 * Description: Return information about a file or directory (don't follow
 *              symbolic links)
 *
 ****************************************************************************/

static int hostfs_lstat(FAR struct inode *mountpt, FAR const char *relpath,
                        FAR struct stat *buf)
{
  FAR struct hostfs_mountpt_s *fs;
  FAR char *path;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  path = lib_get_tempbuffer(HOSTFS_MAX_PATH);
  if (path == NULL)
    {
      return -ENOMEM;
    }

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_path;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, HOSTFS_MAX_PATH);

  /* Call the host FS to do the lstat operation */

  ret = host_lstat(path, buf);

  nxmutex_unlock(&fs->fs_lock);

errout_with_path:
  lib_put_tempbuffer(path);
  return ret;
}
#endif /* CONFIG_FS_LINKS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/
