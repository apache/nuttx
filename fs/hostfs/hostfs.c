/****************************************************************************
 * nuttx/fs/hostfs/hostfs.c
 *
 *   Copyright (C) 2015 Ken Pettit. All rights reserved.
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
#include <sys/stat.h>
#include <sys/statfs.h>

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/fat.h>
#include <nuttx/fs/dirent.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/hostfs.h>

#include "hostfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

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

static int     hostfs_opendir(FAR struct inode *mountpt,
                        FAR const char *relpath,
                        FAR struct fs_dirent_s *dir);
static int     hostfs_closedir(FAR struct inode *mountpt,
                        FAR struct fs_dirent_s *dir);
static int     hostfs_readdir(FAR struct inode *mountpt,
                        FAR struct fs_dirent_s *dir);
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
static int     hostfs_rmdir(FAR struct inode *mountpt, const char *relpath);
static int     hostfs_rename(FAR struct inode *mountpt,
                        FAR const char *oldrelpath,
                        FAR const char *newrelpath);
static int     hostfs_stat(FAR struct inode *mountpt,
                        FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t  g_seminitialized = FALSE;
static sem_t    g_sem;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* See fs_mount.c -- this structure is explicitly externed there.
 * We use the old-fashioned kind of initializers so that this will compile
 * with any compiler.
 */

const struct mountpt_operations hostfs_operations =
{
  hostfs_open,          /* open */
  hostfs_close,         /* close */
  hostfs_read,          /* read */
  hostfs_write,         /* write */
  hostfs_seek,          /* seek */
  hostfs_ioctl,         /* ioctl */

  hostfs_sync,          /* sync */
  hostfs_dup,           /* dup */

  hostfs_opendir,       /* opendir */
  hostfs_closedir,      /* closedir */
  hostfs_readdir,       /* readdir */
  hostfs_rewinddir,     /* rewinddir */

  hostfs_bind,          /* bind */
  hostfs_unbind,        /* unbind */
  hostfs_statfs,        /* statfs */

  hostfs_unlink,        /* unlinke */
  hostfs_mkdir,         /* mkdir */
  hostfs_rmdir,         /* rmdir */
  hostfs_rename,        /* rename */
  hostfs_stat           /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hostfs_semtake
 ****************************************************************************/

void hostfs_semtake(struct hostfs_mountpt_s *fs)
{
  /* Take the semaphore (perhaps waiting) */

  while (sem_wait(fs->fs_sem) != 0)
    {
      /* The only case that an error should occur here is if
       * the wait was awakened by a signal.
       */

      ASSERT(*get_errno_ptr() == EINTR);
    }
}

/****************************************************************************
 * Name: hostfs_semgive
 ****************************************************************************/

void hostfs_semgive(struct hostfs_mountpt_s *fs)
{
  sem_post(fs->fs_sem);
}

/****************************************************************************
 * Name: hostfs_mkpath
 *
 * Description: Build absolute host path from relative NuttX path.
 *
 ****************************************************************************/

static void hostfs_mkpath(struct hostfs_mountpt_s  *fs, const char *relpath,
                          char *path, int pathlen)
{
  int   depth = 0;
  int   first;
  int   x;

  /* Copy base host path to output */

  strncpy(path, fs->fs_root, pathlen);

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
      /* Test for ".." occurance */

      if (strncmp(&relpath[x], "..", 2) == 0)
        {
          /* Reduce depth by 1 */

          depth--;
          x += 2;
        }

      else if (relpath[x] == '/' && relpath[x+1] != '/' && relpath[x+1] != '\0')
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
      strncat(path, &relpath[first], pathlen-strlen(path)-1);
    }
}

/****************************************************************************
 * Name: hostfs_open
 ****************************************************************************/

static int hostfs_open(FAR struct file *filep, const char *relpath,
                       int oflags, mode_t mode)
{
  struct inode             *inode;
  struct hostfs_mountpt_s  *fs;
  int                       ret;
  struct hostfs_ofile_s    *hf;
  char                      path[HOSTFS_MAX_PATH];

  /* Sanity checks */

  DEBUGASSERT((filep->f_priv == NULL) && (filep->f_inode != NULL));


  /* Get the mountpoint inode reference from the file structure and the
   * mountpoint private data from the inode structure
   */

  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Allocate memory for the open file */

  hf = (struct hostfs_ofile_s *) kmm_malloc(sizeof *hf);
  if (hf == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_semaphore;
    }

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Try to open the file in the host file system */

  hf->fd = host_open(path, oflags, mode);
  if (hf->fd == -1)
    {
      /* Error opening file */

      ret = -EBADF;
      goto errout_with_buffer;
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
  goto errout_with_semaphore;

errout_with_buffer:
  kmm_free(hf);

errout_with_semaphore:
  hostfs_semgive(fs);
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
  struct inode             *inode;
  struct hostfs_mountpt_s  *fs;
  struct hostfs_ofile_s    *hf;
  struct hostfs_ofile_s    *nextfile;
  struct hostfs_ofile_s    *prevfile;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  inode = filep->f_inode;
  fs    = inode->i_private;
  hf    = filep->f_priv;

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Check if we are the last one with a reference to the file and
   * only close if we are. */

  if (hf->crefs > 1)
    {
      /* The file is opened more than once.  Just decrement the
       * reference count and return. */

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
  kmm_free(hf);

okout:
  hostfs_semgive(fs);
  return OK;
}

/****************************************************************************
 * Name: hostfs_read
 ****************************************************************************/

static ssize_t hostfs_read(FAR struct file *filep, char *buffer, size_t buflen)
{
  struct inode             *inode;
  struct hostfs_mountpt_s  *fs;
  struct hostfs_ofile_s    *hf;
  int                       ret = OK;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call the host to perform the read */

  ret = host_read(hf->fd, buffer, buflen);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_write
 ****************************************************************************/

static ssize_t hostfs_write(FAR struct file *filep, const char *buffer,
                         size_t buflen)
{
  struct inode             *inode;
  struct hostfs_mountpt_s  *fs;
  struct hostfs_ofile_s    *hf;
  int                       ret;

  /* Sanity checks.  I have seen the following assertion misfire if
   * CONFIG_DEBUG_MM is enabled while re-directing output to a
   * file.  In this case, the debug output can get generated while
   * the file is being opened,  FAT data structures are being allocated,
   * and things are generally in a perverse state.
   */

#ifdef CONFIG_DEBUG_MM
  if (filep->f_priv == NULL || filep->f_inode == NULL)
    {
      return -ENXIO;
    }
#else
  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
#endif

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Test the permissions.  Only allow write if the file was opened with
   * write flags.
   */

  if ((hf->oflags & O_WROK) == 0)
    {
      ret = -EACCES;
      goto errout_with_semaphore;
    }

  /* Call the host to perform the write */

  ret = host_write(hf->fd, buffer, buflen);

errout_with_semaphore:
  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_seek
 ****************************************************************************/

static off_t hostfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  struct inode             *inode;
  struct hostfs_mountpt_s  *fs;
  struct hostfs_ofile_s    *hf;
  int                       ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call our internal routine to perform the seek */

  ret = host_lseek(hf->fd, offset, whence);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_ioctl
 ****************************************************************************/

static int hostfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  struct inode             *inode;
  struct hostfs_mountpt_s  *fs;
  struct hostfs_ofile_s    *hf;
  int                       ret;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call our internal routine to perform the ioctl */

  ret = host_ioctl(hf->fd, cmd, arg);

  hostfs_semgive(fs);
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
  struct inode             *inode;
  struct hostfs_mountpt_s  *fs;
  struct hostfs_ofile_s    *hf;

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover our private data from the struct file instance */

  hf    = filep->f_priv;
  inode = filep->f_inode;
  fs    = inode->i_private;

  DEBUGASSERT(fs != NULL);

  /* Take the semaphore */

  hostfs_semtake(fs);

  host_sync(hf->fd);

  hostfs_semgive(fs);
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
  struct hostfs_ofile_s   *sf;

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
 * Name: hostfs_opendir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int hostfs_opendir(struct inode *mountpt, const char *relpath, struct fs_dirent_s *dir)
{
  struct hostfs_mountpt_s  *fs;
  int                       ret;
  char                      path[HOSTFS_MAX_PATH];

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host's opendir function */

  dir->u.hostfs.fs_dir = host_opendir(path);

  if (dir->u.hostfs.fs_dir == NULL)
    {
      ret = -ENOENT;
      goto errout_with_semaphore;
    }

  ret = OK;

errout_with_semaphore:

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_closedir
 *
 * Description: Open a directory for read access
 *
 ****************************************************************************/

static int hostfs_closedir(FAR struct inode *mountpt, FAR struct fs_dirent_s *dir)
{
  struct hostfs_mountpt_s  *fs;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call the host's closedir function */

  host_closedir(dir->u.hostfs.fs_dir);

  hostfs_semgive(fs);
  return OK;
}

/****************************************************************************
 * Name: hostfs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

static int hostfs_readdir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  struct hostfs_mountpt_s  *fs;
  int                       ret;
  struct host_dirent_s      entry;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover our private data from the inode instance */

  fs = mountpt->i_private;

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Call the host OS's readdir function */

  ret = host_readdir(dir->u.hostfs.fs_dir, &entry);

  /* Save the entry name when successful */

  if (ret == OK)
    {
      /* Copy the entry name */
      memset(dir->fd_dir.d_name, 0, sizeof(dir->fd_dir.d_name));
      strncpy(dir->fd_dir.d_name, entry.d_name, sizeof(dir->fd_dir.d_name));

      /* Copy the entry type */

      /* TODO:  May need to do some type mapping */

      dir->fd_dir.d_type = entry.d_type;
    }

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_rewindir
 *
 * Description: Reset directory read to the first entry
 *
 ****************************************************************************/

static int hostfs_rewinddir(struct inode *mountpt, struct fs_dirent_s *dir)
{
  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Call the host and let it do all the work */

  host_rewinddir(dir->u.hostfs.fs_dir);

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

static int hostfs_bind(FAR struct inode *blkdriver, const void *data,
                        void **handle)
{
  struct hostfs_mountpt_s  *fs;
  struct host_stat_s        buf;
  int                       ret, len;
  const char *              options;

  /* Validate the block driver is NULL */

  if (blkdriver || !data)
    {
      return -ENODEV;
    }

  /* The only options we suppor are "-o fs=whatever", so search
   * for the 'dir=' portion 
   */

  options = (const char *) data;
  if ((strncmp(options, "fs=", 3) != 0) || (strlen(options) < 5))
    {
      return -ENODEV;
    }

  /* Create an instance of the mountpt state structure */

  fs = (struct hostfs_mountpt_s *)kmm_zalloc(sizeof(struct hostfs_mountpt_s));
  if (!fs)
    {
      return -ENOMEM;
    }

  /* If the global semaphore hasn't been initialized, then
   * initialized it now. */

  fs->fs_sem = &g_sem;
  if (!g_seminitialized)
    {
      /* Initialize the semaphore that controls access */

      sem_init(&g_sem, 0, 0);   
      g_seminitialized = TRUE;
    }
  else
    {
      /* Take the semaphore for the mount */

      hostfs_semtake(fs);
    }

  /* Initialize the allocated mountpt state structure.  The filesystem is
   * responsible for one reference ont the blkdriver inode and does not
   * have to addref() here (but does have to release in ubind().
   */

  fs->fs_head = NULL;

  /* Now perform the mount.  */

  strncpy(fs->fs_root, &options[3], sizeof(fs->fs_root));
  len = strlen(fs->fs_root);
  if (len && fs->fs_root[len-1] == '/')
    {
      /* Remove trailing '/' */

      fs->fs_root[len-1] = '\0';
    }

  /* Try to stat the file in the host FS */

  ret = host_stat(fs->fs_root, &buf);
  if ((ret != 0) || ((buf.st_mode & HOST_ST_MODE_DIR) == 0))
    {
      hostfs_semgive(fs);
      kmm_free(fs);
      return -ENOENT;
    }

  /* Append a '/' to the name now */

  strcat(fs->fs_root, "/");

  *handle = (FAR void *)fs;
  hostfs_semgive(fs);
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

  ret = OK; /* Assume success */
  hostfs_semtake(fs);
  if (fs->fs_head != NULL)
    {
      /* We cannot unmount now.. there are open files */

      hostfs_semgive(fs);

      /* This implementation currently only supports unmounting if there are
       * no open file references.
       */

      return (flags != 0) ? -ENOSYS : -EBUSY;
    }

  hostfs_semgive(fs);
  kmm_free(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

static int hostfs_statfs(struct inode *mountpt, struct statfs *buf)
{
  struct hostfs_mountpt_s *fs;
  struct host_statfs_s      host_buf;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  hostfs_semtake(fs);

  /* Implement the logic!! */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type = HOSTFS_MAGIC;

  /* Call the host fs to perform the statfs */

  ret = host_statfs(fs->fs_root, &host_buf);

  buf->f_namelen = host_buf.f_namelen;
  buf->f_bsize = host_buf.f_bsize;
  buf->f_blocks = host_buf.f_blocks;
  buf->f_bfree = host_buf.f_bfree;
  buf->f_bavail = host_buf.f_bavail;
  buf->f_files = host_buf.f_files;
  buf->f_ffree = host_buf.f_ffree;

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_unlink
 *
 * Description: Remove a file
 *
 ****************************************************************************/

static int hostfs_unlink(struct inode *mountpt, const char *relpath)
{
  struct hostfs_mountpt_s  *fs;
  int                       ret;
  char                      path[HOSTFS_MAX_PATH];

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host fs to perform the unlink */

  ret = host_unlink(path);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_mkdir
 *
 * Description: Create a directory
 *
 ****************************************************************************/

static int hostfs_mkdir(struct inode *mountpt, const char *relpath, mode_t mode)
{
  struct hostfs_mountpt_s  *fs;
  int                       ret;
  char                      path[HOSTFS_MAX_PATH];

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host FS to do the mkdir */

  ret = host_mkdir(path, mode);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_rmdir
 *
 * Description: Remove a directory
 *
 ****************************************************************************/

int hostfs_rmdir(struct inode *mountpt, const char *relpath)
{
  struct hostfs_mountpt_s  *fs;
  int                       ret;
  char                      path[HOSTFS_MAX_PATH];

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  /* Take the semaphore */

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host FS to do the mkdir */

  ret = host_rmdir(path);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_rename
 *
 * Description: Rename a file or directory
 *
 ****************************************************************************/

int hostfs_rename(struct inode *mountpt, const char *oldrelpath,
               const char *newrelpath)
{
  struct hostfs_mountpt_s  *fs;
  int                       ret;
  char                      oldpath[HOSTFS_MAX_PATH];
  char                      newpath[HOSTFS_MAX_PATH];

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  strncpy(oldpath, fs->fs_root, sizeof(oldpath));
  strncat(oldpath, oldrelpath, sizeof(oldpath)-strlen(oldpath)-1);
  strncpy(newpath, fs->fs_root, sizeof(newpath));
  strncat(newpath, newrelpath, sizeof(newpath)-strlen(newpath)-1);

  /* Call the host FS to do the mkdir */

  ret = host_rename(oldpath, newpath);

  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Name: hostfs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

static int hostfs_stat(struct inode *mountpt, const char *relpath, struct stat *buf)
{
  struct hostfs_mountpt_s  *fs;
  int                       ret;
  struct host_stat_s        host_buf;
  char                      path[HOSTFS_MAX_PATH];

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the inode structure */

  fs = mountpt->i_private;

  hostfs_semtake(fs);

  /* Append to the host's root directory */

  hostfs_mkpath(fs, relpath, path, sizeof(path));

  /* Call the host FS to do the mkdir */

  ret = host_stat(path, &host_buf);

  if (ret != 0)
    {
      goto errout_with_semaphore;
    }

  /* Initialize the stat structure */

  memset(buf, 0, sizeof(struct stat));

  buf->st_mode = host_buf.st_mode & 0xFFF;

  if (host_buf.st_mode & HOST_ST_MODE_DIR)
    {
      buf->st_mode |= S_IFDIR;
    }

  if (host_buf.st_mode & HOST_ST_MODE_REG)
    {
      buf->st_mode |= S_IFREG;
    }

  if (host_buf.st_mode & HOST_ST_MODE_CHR)
    {
      buf->st_mode |= S_IFCHR;
    }

  if (host_buf.st_mode & HOST_ST_MODE_BLK)
    {
      buf->st_mode |= S_IFBLK;
    }

  if (host_buf.st_mode & HOST_ST_MODE_LINK)
    {
      buf->st_mode |= S_IFLNK;
    }

  if (host_buf.st_mode & HOST_ST_MODE_PIPE)
    {
      buf->st_mode |= S_IFIFO;
    }

  buf->st_size      = host_buf.st_size;
  buf->st_blksize   = host_buf.st_blksize;
  buf->st_blocks    = host_buf.st_blocks;
  buf->st_atime     = host_buf.st_atim;
  buf->st_ctime     = host_buf.st_ctim;

  ret = OK;

errout_with_semaphore:
  hostfs_semgive(fs);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
