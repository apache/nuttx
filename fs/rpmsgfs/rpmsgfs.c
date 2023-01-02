/****************************************************************************
 * fs/rpmsgfs/rpmsgfs.c
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
#include <debug.h>
#include <limits.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include "rpmsgfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RPMSGFS_RETRY_DELAY_MS       10

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsgfs_dir_s
{
  struct fs_dirent_s base;
  FAR void *dir;
};

/* This structure describes the state of one open file.  This structure
 * is protected by the volume semaphore.
 */

struct rpmsgfs_ofile_s
{
  struct rpmsgfs_ofile_s     *fnext;   /* Supports a singly linked list */
  int16_t                    crefs;    /* Reference count */
  mode_t                     oflags;   /* Open mode */
  int                        fd;
};

/* This structure represents the overall mountpoint state.  An instance of
 * this structure is retained as inode private data on each mountpoint that
 * is mounted with a rpmsgfs filesystem.
 */

struct rpmsgfs_mountpt_s
{
  mutex_t                    fs_lock;  /* Assure thread-safe access */
  FAR struct rpmsgfs_ofile_s *fs_head; /* Singly-linked list of open files */
  char                       fs_root[PATH_MAX];
  void                       *handle;
  int                        timeout;  /* Connect timeout */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     rpmsgfs_open(FAR struct file *filep, FAR const char *relpath,
                            int oflags, mode_t mode);
static int     rpmsgfs_close(FAR struct file *filep);
static ssize_t rpmsgfs_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t rpmsgfs_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static off_t   rpmsgfs_seek(FAR struct file *filep, off_t offset,
                            int whence);
static int     rpmsgfs_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

static int     rpmsgfs_sync(FAR struct file *filep);
static int     rpmsgfs_dup(FAR const struct file *oldp,
                           FAR struct file *newp);
static int     rpmsgfs_fstat(FAR const struct file *filep,
                             FAR struct stat *buf);
static int     rpmsgfs_fchstat(FAR const struct file *filep,
                               FAR const struct stat *buf, int flags);
static int     rpmsgfs_ftruncate(FAR struct file *filep,
                                 off_t length);

static int     rpmsgfs_opendir(FAR struct inode *mountpt,
                               FAR const char *relpath,
                               FAR struct fs_dirent_s **dir);
static int     rpmsgfs_closedir(FAR struct inode *mountpt,
                                FAR struct fs_dirent_s *dir);
static int     rpmsgfs_readdir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir,
                               FAR struct dirent *entry);
static int     rpmsgfs_rewinddir(FAR struct inode *mountpt,
                                 FAR struct fs_dirent_s *dir);

static int     rpmsgfs_bind(FAR struct inode *blkdriver,
                            FAR const void *data, FAR void **handle);
static int     rpmsgfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                              unsigned int flags);
static int     rpmsgfs_statfs(FAR struct inode *mountpt,
                              FAR struct statfs *buf);

static int     rpmsgfs_unlink(FAR struct inode *mountpt,
                              FAR const char *relpath);
static int     rpmsgfs_mkdir(FAR struct inode *mountpt,
                             FAR const char *relpath, mode_t mode);
static int     rpmsgfs_rmdir(FAR struct inode *mountpt, const char *relpath);
static int     rpmsgfs_rename(FAR struct inode *mountpt,
                              FAR const char *oldrelpath,
                              FAR const char *newrelpath);
static int     rpmsgfs_stat(FAR struct inode *mountpt,
                            FAR const char *relpath, FAR struct stat *buf);
static int     rpmsgfs_chstat(FAR struct inode *mountpt,
                              FAR const char *relpath,
                              FAR const struct stat *buf, int flags);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations rpmsgfs_operations =
{
  rpmsgfs_open,          /* open */
  rpmsgfs_close,         /* close */
  rpmsgfs_read,          /* read */
  rpmsgfs_write,         /* write */
  rpmsgfs_seek,          /* seek */
  rpmsgfs_ioctl,         /* ioctl */
  NULL,                  /* mmap */
  rpmsgfs_ftruncate,     /* ftruncate */

  rpmsgfs_sync,          /* sync */
  rpmsgfs_dup,           /* dup */
  rpmsgfs_fstat,         /* fstat */
  rpmsgfs_fchstat,       /* fchstat */

  rpmsgfs_opendir,       /* opendir */
  rpmsgfs_closedir,      /* closedir */
  rpmsgfs_readdir,       /* readdir */
  rpmsgfs_rewinddir,     /* rewinddir */

  rpmsgfs_bind,          /* bind */
  rpmsgfs_unbind,        /* unbind */
  rpmsgfs_statfs,        /* statfs */

  rpmsgfs_unlink,        /* unlink */
  rpmsgfs_mkdir,         /* mkdir */
  rpmsgfs_rmdir,         /* rmdir */
  rpmsgfs_rename,        /* rename */
  rpmsgfs_stat,          /* stat */
  rpmsgfs_chstat,        /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpmsgfs_mkpath
 *
 * Description: Build absolute host path from relative NuttX path.
 *
 ****************************************************************************/

static void rpmsgfs_mkpath(FAR struct rpmsgfs_mountpt_s *fs,
                           FAR const char *relpath,
                           FAR char *path, int pathlen)
{
  int depth = 0;
  int first;
  int x;

  /* Copy base host path to output */

  strlcpy(path, fs->fs_root, pathlen);

  /* Be sure we aren't trying to use ".." to display outside of our
   * mounted path.
   */

  x = 0;
  while (relpath[x] == '/')
    {
      x++;
    }

  first = x;

  while (relpath[x] != '\0')
    {
      /* Test for ".." occurrence */

      if (strncmp(&relpath[x], "..", 2) == 0)
        {
          /* Reduce depth by 1 */

          depth--;
          x += 2;
        }

      else if (relpath[x] == '/' && relpath[x + 1] != '/' &&
               relpath[x + 1] != '\0')
        {
          depth++;
          x++;
        }
      else
        {
          x++;
        }
    }

  if (depth >= 0)
    {
      strncat(path, &relpath[first], pathlen - strlen(path) - 1);
    }

  while (fs->timeout > 0)
    {
      struct stat buf;
      int ret;

      ret = rpmsgfs_client_stat(fs->handle, fs->fs_root, &buf);
      if (ret == 0)
        {
          break;
        }

      usleep(RPMSGFS_RETRY_DELAY_MS * USEC_PER_MSEC);
      fs->timeout -= RPMSGFS_RETRY_DELAY_MS;
    }
}

/****************************************************************************
 * Name: rpmsgfs_open
 ****************************************************************************/

static int rpmsgfs_open(FAR struct file *filep, FAR const char *relpath,
                        int oflags, mode_t mode)
{
  FAR struct inode *inode;
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_ofile_s  *hf;
  char path[PATH_MAX];
  int ret;

  /* Sanity checks */

  DEBUGASSERT((filep->f_priv == NULL) && (filep->f_inode != NULL));

  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate memory for the open file */

  hf = (struct rpmsgfs_ofile_s *) kmm_malloc(sizeof *hf);
  if (hf == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  /* Append to the host's root directory */

  rpmsgfs_mkpath(fs, relpath, path, sizeof(path));

  /* Try to open the file in the host file system */

  hf->fd = rpmsgfs_client_open(fs->handle, path, oflags, mode);
  if (hf->fd < 0)
    {
      /* Error opening file */

      ret = -EBADF;
      goto errout_with_buffer;
    }

  /* In write/append mode, we need to set the file pointer to the end of the
   * file.
   */

  if ((oflags & (O_APPEND | O_WRONLY)) == (O_APPEND | O_WRONLY))
    {
      ret = rpmsgfs_client_lseek(fs->handle, hf->fd, 0, SEEK_END);
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
  fs->fs_head = hf;

  ret = OK;
  goto errout_with_lock;

errout_with_buffer:
  kmm_free(hf);

errout_with_lock:
  nxmutex_unlock(&fs->fs_lock);
  if (ret == -EINVAL)
    {
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_close
 ****************************************************************************/

static int rpmsgfs_close(FAR struct file *filep)
{
  FAR struct inode            *inode;
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_ofile_s   *hf;
  FAR struct rpmsgfs_ofile_s   *nextfile;
  FAR struct rpmsgfs_ofile_s   *prevfile;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

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

  rpmsgfs_client_close(fs->handle, hf->fd);

  /* Now free the pointer */

  filep->f_priv = NULL;
  kmm_free(hf);

okout:
  nxmutex_unlock(&fs->fs_lock);
  return OK;
}

/****************************************************************************
 * Name: rpmsgfs_read
 ****************************************************************************/

static ssize_t rpmsgfs_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode;
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_ofile_s *hf;
  ssize_t ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

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

  ret = rpmsgfs_client_read(fs->handle, hf->fd, buffer, buflen);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_write
 ****************************************************************************/

static ssize_t rpmsgfs_write(FAR struct file *filep, const char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode;
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_ofile_s *hf;
  ssize_t ret;

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

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

  if ((hf->oflags & O_WROK) == 0)
    {
      ret = -EACCES;
      goto errout_with_lock;
    }

  /* Call the host to perform the write */

  ret = rpmsgfs_client_write(fs->handle, hf->fd, buffer, buflen);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

errout_with_lock:
  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_seek
 ****************************************************************************/

static off_t rpmsgfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct inode *inode;
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_ofile_s *hf;
  off_t ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

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

  ret = rpmsgfs_client_lseek(fs->handle, hf->fd, offset, whence);
  if (ret >= 0)
    {
      filep->f_pos = ret;
    }

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_ioctl
 ****************************************************************************/

static int rpmsgfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_ofile_s *hf;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

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

  ret = rpmsgfs_client_ioctl(fs->handle, hf->fd, cmd, arg);
  if (ret == 0 && (cmd == FIONBIO || cmd == FIOCLEX || cmd == FIONCLEX))
    {
      ret = -ENOTTY;
    }

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_sync
 *
 * Description: Synchronize the file state on disk to match internal, in-
 *   memory state.
 *
 ****************************************************************************/

static int rpmsgfs_sync(FAR struct file *filep)
{
  FAR struct inode            *inode;
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_ofile_s   *hf;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

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

  rpmsgfs_client_sync(fs->handle, hf->fd);

  nxmutex_unlock(&fs->fs_lock);
  return OK;
}

/****************************************************************************
 * Name: rpmsgfs_dup
 *
 * Description: Duplicate open file data in the new file structure.
 *
 ****************************************************************************/

static int rpmsgfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct rpmsgfs_ofile_s *sf;

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
 * Name: rpmsgfs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

static int rpmsgfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct inode *inode;
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_ofile_s *hf;
  int ret = OK;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL && buf != NULL);

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
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

  ret = rpmsgfs_client_fstat(fs->handle, hf->fd, buf);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_fchstat
 *
 * Description:
 *   Change information about an open file associated with the file
 *   descriptor 'fd'.
 *
 ****************************************************************************/

static int rpmsgfs_fchstat(FAR const struct file *filep,
                           FAR const struct stat *buf, int flags)
{
  FAR struct inode *inode;
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_ofile_s *hf;
  int ret = OK;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL && buf != NULL);

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
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

  ret = rpmsgfs_client_fchstat(fs->handle, hf->fd, buf, flags);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_ftruncate
 *
 * Description:
 *   Set the length of the open, regular file associated with the file
 *   structure 'filep' to 'length'.
 *
 ****************************************************************************/

static int rpmsgfs_ftruncate(FAR struct file *filep, off_t length)
{
  FAR struct inode *inode;
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_ofile_s *hf;
  int ret = OK;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL);

  /* Recover our private data from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
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

  ret = rpmsgfs_client_ftruncate(fs->handle, hf->fd, length);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_opendir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int rpmsgfs_opendir(FAR struct inode *mountpt,
                           FAR const char *relpath,
                           FAR struct fs_dirent_s **dir)
{
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_dir_s *rdir;
  char path[PATH_MAX];
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;
  rdir = kmm_zalloc(sizeof(struct rpmsgfs_dir_s));
  if (rdir == NULL)
    {
      return -ENOMEM;
    }

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      goto errout_with_rdir;
    }

  /* Append to the host's root directory */

  rpmsgfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host's opendir function */

  rdir->dir = rpmsgfs_client_opendir(fs->handle, path);
  if (rdir->dir == NULL)
    {
      ret = -ENOENT;
      goto errout_with_lock;
    }

  *dir = (FAR struct fs_dirent_s *)rdir;
  nxmutex_unlock(&fs->fs_lock);
  return OK;

errout_with_lock:
  nxmutex_unlock(&fs->fs_lock);

errout_with_rdir:
  kmm_free(rdir);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_closedir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int rpmsgfs_closedir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_dir_s *rdir;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;
  rdir = (FAR struct rpmsgfs_dir_s *)dir;

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the host's closedir function */

  rpmsgfs_client_closedir(fs->handle, rdir->dir);

  nxmutex_unlock(&fs->fs_lock);
  kmm_free(rdir);
  return OK;
}

/****************************************************************************
 * Name: rpmsgfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int rpmsgfs_readdir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir,
                           FAR struct dirent *entry)
{
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_dir_s *rdir;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;
  rdir = (FAR struct rpmsgfs_dir_s *)dir;

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the host OS's readdir function */

  ret = rpmsgfs_client_readdir(fs->handle, rdir->dir, entry);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int rpmsgfs_rewinddir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir)
{
  FAR struct rpmsgfs_mountpt_s *fs;
  FAR struct rpmsgfs_dir_s *rdir;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;
  rdir = (FAR struct rpmsgfs_dir_s *)dir;

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Call the host and let it do all the work */

  rpmsgfs_client_rewinddir(fs->handle, rdir->dir);

  nxmutex_unlock(&fs->fs_lock);
  return OK;
}

/****************************************************************************
 * Name: rpmsgfs_bind
 *
 * Description: This implements a portion of the mount operation. This
 *  function allocates and initializes the mountpoint private data and
 *  binds the blockdriver inode to the filesystem private data.  The final
 *  binding of the private data (containing the blockdriver) to the
 *  mountpoint is performed by mount().
 *
 ****************************************************************************/

static int rpmsgfs_bind(FAR struct inode *blkdriver, FAR const void *data,
                        FAR void **handle)
{
  FAR struct rpmsgfs_mountpt_s  *fs;
  FAR const char *cpuname = NULL;
  FAR char *options;
  char *saveptr;
  char *ptr;
  int len;
  int ret;

  /* Validate the block driver is NULL */

  if (blkdriver || !data)
    {
      return -ENODEV;
    }

  /* Create an instance of the mountpt state structure */

  fs = (FAR struct rpmsgfs_mountpt_s *)
    kmm_zalloc(sizeof(struct rpmsgfs_mountpt_s));

  if (fs == NULL)
    {
      return -ENOMEM;
    }

  /* The options we support are:
   *  "fs=whatever,cpu=cpuname", remote dir
   *  "timeout=xx", connect timeout, unit (ms)
   */

  options = strdup(data);
  if (!options)
    {
      kmm_free(fs);
      return -ENOMEM;
    }

  /* Set timeout default value */

  fs->timeout = INT_MAX;

  ptr = strtok_r(options, ",", &saveptr);
  while (ptr != NULL)
    {
      if ((strncmp(ptr, "fs=", 3) == 0))
        {
          strlcpy(fs->fs_root, &ptr[3], sizeof(fs->fs_root));
        }
      else if ((strncmp(ptr, "cpu=", 4) == 0))
        {
          cpuname = &ptr[4];
        }
      else if ((strncmp(ptr, "timeout=", 8) == 0))
        {
          fs->timeout = atoi(&ptr[8]);
        }

      ptr = strtok_r(NULL, ",", &saveptr);
    }

  ret = rpmsgfs_client_bind(&fs->handle, cpuname);
  kmm_free(options);
  if (ret < 0)
    {
      kmm_free(fs);
      return ret;
    }

  /* Initialize the mutex that controls access */

  nxmutex_init(&fs->fs_lock);

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
      strcat(fs->fs_root, "/");
    }

  *handle = (FAR void *)fs;
  return OK;
}

/****************************************************************************
 * Name: rpmsgfs_unbind
 *
 * Description: This implements the filesystem portion of the umount
 *   operation.
 *
 ****************************************************************************/

static int rpmsgfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                          unsigned int flags)
{
  FAR struct rpmsgfs_mountpt_s *fs = (FAR struct rpmsgfs_mountpt_s *)handle;
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

  ret = rpmsgfs_client_unbind(fs->handle);
  nxmutex_unlock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  nxmutex_destroy(&fs->fs_lock);
  kmm_free(fs);
  return 0;
}

/****************************************************************************
 * Name: rpmsgfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int rpmsgfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct rpmsgfs_mountpt_s *fs;
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

  memset(buf, 0, sizeof(struct statfs));
  ret = rpmsgfs_client_statfs(fs->handle, fs->fs_root, buf);
  buf->f_type = RPMSGFS_MAGIC;

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_unlink
 *
 * Description: Remove a file
 *
 ****************************************************************************/

static int rpmsgfs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct rpmsgfs_mountpt_s *fs;
  char path[PATH_MAX];
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

  /* Append to the host's root directory */

  rpmsgfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host fs to perform the unlink */

  ret = rpmsgfs_client_unlink(fs->handle, path);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_mkdir
 *
 * Description: Create a directory
 *
 ****************************************************************************/

static int rpmsgfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                         mode_t mode)
{
  FAR struct rpmsgfs_mountpt_s *fs;
  char path[PATH_MAX];
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

  /* Append to the host's root directory */

  rpmsgfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host FS to do the mkdir */

  ret = rpmsgfs_client_mkdir(fs->handle, path, mode);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_rmdir
 *
 * Description: Remove a directory
 *
 ****************************************************************************/

int rpmsgfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct rpmsgfs_mountpt_s *fs;
  char path[PATH_MAX];
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Take the lock */

  ret = nxmutex_lock(&fs->fs_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Append to the host's root directory */

  rpmsgfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host FS to do the mkdir */

  ret = rpmsgfs_client_rmdir(fs->handle, path);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_rename
 *
 * Description: Rename a file or directory
 *
 ****************************************************************************/

int rpmsgfs_rename(FAR struct inode *mountpt, FAR const char *oldrelpath,
                   FAR const char *newrelpath)
{
  FAR struct rpmsgfs_mountpt_s *fs;
  char oldpath[PATH_MAX];
  char newpath[PATH_MAX];
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

  /* Append to the host's root directory */

  strlcpy(oldpath, fs->fs_root, sizeof(oldpath));
  strlcat(oldpath, oldrelpath, sizeof(oldpath));
  strlcpy(newpath, fs->fs_root, sizeof(newpath));
  strlcat(newpath, newrelpath, sizeof(newpath));

  /* Call the host FS to do the mkdir */

  ret = rpmsgfs_client_rename(fs->handle, oldpath, newpath);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int rpmsgfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                        FAR struct stat *buf)
{
  FAR struct rpmsgfs_mountpt_s *fs;
  char path[PATH_MAX];
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

  /* Append to the host's root directory */

  rpmsgfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host FS to do the stat operation */

  ret = rpmsgfs_client_stat(fs->handle, path, buf);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}

/****************************************************************************
 * Name: rpmsgfs_chstat
 *
 * Description: Change information about a file or directory
 *
 ****************************************************************************/

static int rpmsgfs_chstat(FAR struct inode *mountpt, FAR const char *relpath,
                          FAR const struct stat *buf, int flags)
{
  FAR struct rpmsgfs_mountpt_s *fs;
  char path[PATH_MAX];
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

  /* Append to the host's root directory */

  rpmsgfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host FS to do the chstat operation */

  ret = rpmsgfs_client_chstat(fs->handle, path, buf, flags);

  nxmutex_unlock(&fs->fs_lock);
  return ret;
}
